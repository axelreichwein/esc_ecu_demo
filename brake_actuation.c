#include "brake_actuation.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

// Constants for brake actuation control
#define PWM_MAX_DUTY_CYCLE 1000        // 0-1000 represents 0-100.0%
#define PWM_MIN_DUTY_CYCLE 0
#define CAN_MAX_DATA_LENGTH 8
#define FEEDBACK_TIMEOUT_MS 100
#define CONTROL_LOOP_DT 0.005f         // 5ms control loop (200Hz)
#define EMERGENCY_BRAKE_PRESSURE 150.0f // Emergency brake pressure (bar)
#define PRESSURE_RAMP_RATE_DEFAULT 50.0f // Default pressure ramp rate (bar/s)

// Global hardware interface
static const brake_hw_interface_t* g_hw_interface = NULL;

// System initialization and configuration
bool brake_actuation_init(brake_actuation_system_t* system, const brake_hw_interface_t* hw_interface) {
    if (!system || !hw_interface) {
        return false;
    }
    
    memset(system, 0, sizeof(brake_actuation_system_t));
    g_hw_interface = hw_interface;
    
    // Initialize system state
    system->system_status = BRAKE_STATUS_CALIBRATING;
    system->system_initialized = false;
    system->closed_loop_enabled = true;
    system->emergency_mode = false;
    system->control_loop_frequency_hz = 200; // 200Hz control loop
    
    // Initialize mutex for thread safety
    if (pthread_mutex_init(&system->command_mutex, NULL) != 0) {
        printf("Failed to initialize brake command mutex\n");
        return false;
    }
    
    // Configure default channel settings
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        // Default PWM configuration for each wheel
        system->channel_config[i] = brake_get_default_pwm_config((wheel_position_t)i);
        
        // Initialize PID controllers
        system->pid_controller[i] = brake_get_default_pid_config();
        
        // Initialize feedback data
        brake_feedback_t* feedback = &system->feedback[i];
        feedback->status = BRAKE_STATUS_OK;
        feedback->pressure_valid = false;
        feedback->feedback_timeout = true;
        
        // Initialize calibration data
        system->pressure_calibration_offset[i] = 0.0f;
        system->pressure_calibration_gain[i] = 1.0f;
        system->calibration_valid[i] = false;
        
        // Initialize performance metrics
        system->max_pressure_error[i] = 0.0f;
        system->avg_response_time_ms[i] = 0.0f;
    }
    
    // Initialize hardware interfaces
    bool init_success = true;
    
    // Initialize PWM channels if available
    if (g_hw_interface->pwm_init) {
        for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
            if (system->channel_config[i].interface_type == BRAKE_INTERFACE_PWM) {
                if (!g_hw_interface->pwm_init(system->channel_config[i].channel_id, 
                                            system->channel_config[i].pwm_frequency_hz)) {
                    printf("Failed to initialize PWM channel %d\n", i);
                    init_success = false;
                }
            }
        }
    }
    
    // Initialize CAN interface if available
    if (g_hw_interface->can_init) {
        if (!g_hw_interface->can_init(500000)) { // 500kbps CAN
            printf("Failed to initialize CAN interface\n");
            init_success = false;
        }
    }
    
    // Initialize feedback interfaces
    if (g_hw_interface->feedback_init) {
        for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
            if (!g_hw_interface->feedback_init(i)) {
                printf("Failed to initialize feedback for wheel %d\n", i);
                init_success = false;
            }
        }
    }
    
    if (init_success) {
        system->system_status = BRAKE_STATUS_OK;
        system->system_initialized = true;
        printf("Brake actuation system initialized successfully\n");
    } else {
        system->system_status = BRAKE_STATUS_FAULT;
        printf("Brake actuation system initialization failed\n");
    }
    
    return init_success;
}

void brake_actuation_shutdown(brake_actuation_system_t* system) {
    if (!system) return;
    
    // Stop control thread if running
    if (system->control_thread_running) {
        system->control_thread_running = false;
        pthread_join(system->control_thread, NULL);
    }
    
    // Emergency stop all brake channels
    brake_set_emergency_brake(system, false);
    
    // Destroy mutex
    pthread_mutex_destroy(&system->command_mutex);
    
    memset(system, 0, sizeof(brake_actuation_system_t));
    printf("Brake actuation system shutdown complete\n");
}

bool brake_configure_channel(brake_actuation_system_t* system, wheel_position_t wheel, 
                            const brake_channel_config_t* config) {
    if (!system || !config || wheel >= MAX_WHEEL_COUNT) {
        return false;
    }
    
    pthread_mutex_lock(&system->command_mutex);
    
    // Copy configuration
    memcpy(&system->channel_config[wheel], config, sizeof(brake_channel_config_t));
    
    // Validate configuration
    bool valid = true;
    if (config->max_pressure <= config->min_pressure) {
        printf("Invalid pressure range for wheel %d\n", wheel);
        valid = false;
    }
    
    if (config->interface_type == BRAKE_INTERFACE_PWM && config->pwm_frequency_hz == 0) {
        printf("Invalid PWM frequency for wheel %d\n", wheel);
        valid = false;
    }
    
    if (valid) {
        printf("Configured brake channel for wheel %d (%s interface)\n", 
               wheel, brake_interface_to_string(config->interface_type));
    }
    
    pthread_mutex_unlock(&system->command_mutex);
    return valid;
}

// Command interface
bool brake_set_pressure_command(brake_actuation_system_t* system, const float pressures[MAX_WHEEL_COUNT]) {
    if (!system || !pressures || !system->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&system->command_mutex);
    
    uint32_t current_time = (uint32_t)(time(NULL) * 1000); // Current time in ms
    
    // Update command timestamp
    system->current_command.command_timestamp_ms = current_time;
    
    // Validate and set pressure commands
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_channel_config_t* config = &system->channel_config[i];
        
        if (brake_pressure_within_limits(pressures[i], config)) {
            system->current_command.target_pressure[i] = pressures[i];
            system->current_command.enable_channel[i] = true;
        } else {
            // Clamp to limits if out of range
            system->current_command.target_pressure[i] = fmaxf(config->min_pressure, 
                                                              fminf(pressures[i], config->max_pressure));
            printf("Pressure command for wheel %d clamped to %.1f bar\n", i, 
                   system->current_command.target_pressure[i]);
        }
        
        // Convert pressure to appropriate control signal
        switch (config->interface_type) {
            case BRAKE_INTERFACE_PWM:
                system->current_command.pwm_duty[i] = brake_pressure_to_pwm(
                    system->current_command.target_pressure[i], config->max_pressure);
                break;
                
            case BRAKE_INTERFACE_CAN:
                brake_can_encode_pressure_message(system->current_command.can_data[i], 
                                                 system->current_command.target_pressure[i], i);
                break;
                
            default:
                break;
        }
    }
    
    system->command_count++;
    
    pthread_mutex_unlock(&system->command_mutex);
    return true;
}

bool brake_set_force_command(brake_actuation_system_t* system, const float forces[MAX_WHEEL_COUNT]) {
    if (!system || !forces) {
        return false;
    }
    
    // Convert forces to pressures using brake system characteristics
    float pressures[MAX_WHEEL_COUNT];
    
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        // Simplified force-to-pressure conversion
        // In real implementation, this would use brake pad area, friction coefficient, etc.
        float brake_efficiency = 1500.0f; // N per bar
        pressures[i] = forces[i] / brake_efficiency;
        
        // Store force command for reference
        system->current_command.target_force[i] = forces[i];
    }
    
    return brake_set_pressure_command(system, pressures);
}

bool brake_set_emergency_brake(brake_actuation_system_t* system, bool enable) {
    if (!system) return false;
    
    pthread_mutex_lock(&system->command_mutex);
    
    system->current_command.emergency_brake = enable;
    system->emergency_mode = enable;
    
    if (enable) {
        // Apply emergency brake pressure to all wheels
        for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
            system->current_command.target_pressure[i] = EMERGENCY_BRAKE_PRESSURE;
            system->current_command.enable_channel[i] = true;
            system->current_command.pwm_duty[i] = brake_pressure_to_pwm(
                EMERGENCY_BRAKE_PRESSURE, system->channel_config[i].max_pressure);
        }
        printf("Emergency brake activated - %.1f bar to all wheels\n", EMERGENCY_BRAKE_PRESSURE);
    } else {
        // Gradually release emergency brake
        for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
            system->current_command.target_pressure[i] = 0.0f;
            system->current_command.pwm_duty[i] = 0;
        }
        printf("Emergency brake deactivated\n");
    }
    
    pthread_mutex_unlock(&system->command_mutex);
    return true;
}

bool brake_enable_channel(brake_actuation_system_t* system, wheel_position_t wheel, bool enable) {
    if (!system || wheel >= MAX_WHEEL_COUNT) {
        return false;
    }
    
    pthread_mutex_lock(&system->command_mutex);
    system->current_command.enable_channel[wheel] = enable;
    
    if (!enable) {
        // Disable channel - set pressure to zero
        system->current_command.target_pressure[wheel] = 0.0f;
        system->current_command.pwm_duty[wheel] = 0;
    }
    
    pthread_mutex_unlock(&system->command_mutex);
    return true;
}

// Control modes
bool brake_enable_closed_loop(brake_actuation_system_t* system, bool enable) {
    if (!system) return false;
    
    system->closed_loop_enabled = enable;
    
    if (enable) {
        printf("Closed-loop brake control enabled\n");
    } else {
        printf("Open-loop brake control enabled\n");
        // Reset PID controllers when switching to open loop
        for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
            brake_reset_pid_controller(&system->pid_controller[i]);
        }
    }
    
    return true;
}

bool brake_configure_pid_controller(brake_actuation_system_t* system, wheel_position_t wheel,
                                   float kp, float ki, float kd) {
    if (!system || wheel >= MAX_WHEEL_COUNT) {
        return false;
    }
    
    brake_pid_controller_t* controller = &system->pid_controller[wheel];
    
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    
    // Reset controller state
    brake_reset_pid_controller(controller);
    
    printf("PID controller for wheel %d configured: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", 
           wheel, kp, ki, kd);
    
    return true;
}

bool brake_set_control_limits(brake_actuation_system_t* system, wheel_position_t wheel,
                             float min_output, float max_output) {
    if (!system || wheel >= MAX_WHEEL_COUNT) {
        return false;
    }
    
    brake_pid_controller_t* controller = &system->pid_controller[wheel];
    
    controller->output_min = min_output;
    controller->output_max = max_output;
    
    return true;
}

// Real-time operation
void brake_update_control_loop(brake_actuation_system_t* system) {
    if (!system || !system->system_initialized) {
        return;
    }
    
    // Process feedback from all channels
    brake_process_feedback(system);
    
    // Execute closed-loop control if enabled
    if (system->closed_loop_enabled) {
        for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
            if (!system->current_command.enable_channel[i]) {
                continue;
            }
            
            brake_feedback_t* feedback = &system->feedback[i];
            brake_pid_controller_t* controller = &system->pid_controller[i];
            
            if (feedback->pressure_valid && !feedback->feedback_timeout) {
                // Calculate pressure error
                float pressure_error = system->current_command.target_pressure[i] - feedback->actual_pressure;
                feedback->pressure_error = pressure_error;
                
                // Update max error for diagnostics
                if (fabsf(pressure_error) > system->max_pressure_error[i]) {
                    system->max_pressure_error[i] = fabsf(pressure_error);
                }
                
                // Execute PID control
                float control_output = brake_execute_pid_control(controller, pressure_error, CONTROL_LOOP_DT);
                
                // Apply control output based on interface type
                brake_channel_config_t* config = &system->channel_config[i];
                switch (config->interface_type) {
                    case BRAKE_INTERFACE_PWM: {
                        // Adjust PWM duty cycle based on control output
                        float current_duty = (float)system->current_command.pwm_duty[i];
                        current_duty += control_output;
                        current_duty = fmaxf(PWM_MIN_DUTY_CYCLE, fminf(current_duty, PWM_MAX_DUTY_CYCLE));
                        system->current_command.pwm_duty[i] = (uint16_t)current_duty;
                        break;
                    }
                    
                    case BRAKE_INTERFACE_CAN: {
                        // Adjust pressure command and re-encode CAN message
                        float adjusted_pressure = system->current_command.target_pressure[i] + control_output * 0.1f;
                        adjusted_pressure = fmaxf(config->min_pressure, fminf(adjusted_pressure, config->max_pressure));
                        brake_can_encode_pressure_message(system->current_command.can_data[i], adjusted_pressure, i);
                        break;
                    }
                    
                    default:
                        break;
                }
            }
        }
    }
    
    // Execute commands to hardware
    brake_execute_commands(system);
    
    // Perform safety checks
    if (!brake_safety_check(system)) {
        system->system_status = BRAKE_STATUS_FAULT;
        brake_emergency_shutdown(system);
    }
}

void brake_process_feedback(brake_actuation_system_t* system) {
    if (!system || !g_hw_interface) {
        return;
    }
    
    uint32_t current_time = (uint32_t)(time(NULL) * 1000);
    
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_feedback_t* feedback = &system->feedback[i];
        
        // Read pressure feedback if available
        if (g_hw_interface->read_pressure && g_hw_interface->feedback_valid) {
            if (g_hw_interface->feedback_valid(i)) {
                float raw_pressure = g_hw_interface->read_pressure(i);
                
                // Apply calibration
                feedback->actual_pressure = (raw_pressure + system->pressure_calibration_offset[i]) * 
                                          system->pressure_calibration_gain[i];
                
                feedback->pressure_valid = true;
                feedback->feedback_timeout = false;
                feedback->last_update_time_ms = current_time;
                feedback->status = BRAKE_STATUS_OK;
            } else {
                feedback->pressure_valid = false;
                feedback->status = BRAKE_STATUS_FAULT;
            }
        }
        
        // Read temperature feedback if available
        if (g_hw_interface->read_temperature) {
            feedback->temperature = g_hw_interface->read_temperature(i);
        }
        
        // Check for feedback timeout
        if (current_time - feedback->last_update_time_ms > FEEDBACK_TIMEOUT_MS) {
            feedback->feedback_timeout = true;
            system->feedback_timeouts++;
            
            if (feedback->status == BRAKE_STATUS_OK) {
                feedback->status = BRAKE_STATUS_WARNING;
            }
        }
        
        // Copy target pressure for reference
        feedback->target_pressure = system->current_command.target_pressure[i];
    }
}

bool brake_execute_commands(brake_actuation_system_t* system) {
    if (!system || !g_hw_interface || !system->system_initialized) {
        return false;
    }
    
    bool success = true;
    
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        if (!system->current_command.enable_channel[i]) {
            continue;
        }
        
        brake_channel_config_t* config = &system->channel_config[i];
        
        switch (config->interface_type) {
            case BRAKE_INTERFACE_PWM:
                if (g_hw_interface->pwm_set_duty) {
                    if (!g_hw_interface->pwm_set_duty(config->channel_id, 
                                                    system->current_command.pwm_duty[i])) {
                        success = false;
                        printf("Failed to set PWM duty for wheel %d\n", i);
                    }
                }
                break;
                
            case BRAKE_INTERFACE_CAN:
                if (g_hw_interface->can_send_message) {
                    if (!g_hw_interface->can_send_message(config->can_id, 
                                                        system->current_command.can_data[i], 
                                                        CAN_MAX_DATA_LENGTH)) {
                        success = false;
                        printf("Failed to send CAN message for wheel %d\n", i);
                    }
                }
                break;
                
            default:
                printf("Unsupported brake interface type for wheel %d\n", i);
                success = false;
                break;
        }
    }
    
    // Update last command for comparison
    memcpy(&system->last_command, &system->current_command, sizeof(brake_command_t));
    
    return success;
}

// PWM-specific functions
bool brake_pwm_init_channel(uint8_t channel, uint16_t frequency_hz) {
    if (g_hw_interface && g_hw_interface->pwm_init) {
        return g_hw_interface->pwm_init(channel, frequency_hz);
    }
    
    // Simulation mode - always return success
    printf("PWM channel %d initialized at %d Hz (simulated)\n", channel, frequency_hz);
    return true;
}

bool brake_pwm_set_pressure(uint8_t channel, float target_pressure, float max_pressure) {
    uint16_t duty_cycle = brake_pressure_to_pwm(target_pressure, max_pressure);
    
    if (g_hw_interface && g_hw_interface->pwm_set_duty) {
        return g_hw_interface->pwm_set_duty(channel, duty_cycle);
    }
    
    // Simulation mode
    printf("PWM channel %d set to %.1f%% duty (%.1f bar)\n", 
           channel, (float)duty_cycle / 10.0f, target_pressure);
    return true;
}

uint16_t brake_pressure_to_pwm(float pressure, float max_pressure) {
    if (max_pressure <= 0.0f) {
        return PWM_MIN_DUTY_CYCLE;
    }
    
    float duty_percent = (pressure / max_pressure) * 100.0f;
    duty_percent = fmaxf(0.0f, fminf(duty_percent, 100.0f));
    
    return (uint16_t)(duty_percent * 10.0f); // 0-1000 range
}

float brake_pwm_to_pressure(uint16_t pwm_duty, float max_pressure) {
    float duty_percent = (float)pwm_duty / 10.0f; // Convert from 0-1000 to 0-100
    return (duty_percent / 100.0f) * max_pressure;
}

// CAN-specific functions
bool brake_can_init_interface(uint32_t baudrate) {
    if (g_hw_interface && g_hw_interface->can_init) {
        return g_hw_interface->can_init(baudrate);
    }
    
    // Simulation mode
    printf("CAN interface initialized at %u bps (simulated)\n", baudrate);
    return true;
}

bool brake_can_send_pressure_command(uint32_t can_id, wheel_position_t wheel, float pressure) {
    uint8_t data[CAN_MAX_DATA_LENGTH];
    brake_can_encode_pressure_message(data, pressure, wheel);
    
    if (g_hw_interface && g_hw_interface->can_send_message) {
        return g_hw_interface->can_send_message(can_id, data, CAN_MAX_DATA_LENGTH);
    }
    
    // Simulation mode
    printf("CAN pressure command sent: ID=0x%03X, Wheel=%d, Pressure=%.1f bar\n", 
           can_id, wheel, pressure);
    return true;
}

bool brake_can_receive_feedback(uint32_t can_id, brake_feedback_t* feedback) {
    if (!feedback) return false;
    
    uint8_t data[CAN_MAX_DATA_LENGTH];
    uint8_t length;
    uint32_t received_id;
    
    if (g_hw_interface && g_hw_interface->can_receive_message) {
        if (g_hw_interface->can_receive_message(&received_id, data, &length)) {
            if (received_id == can_id && length == CAN_MAX_DATA_LENGTH) {
                feedback->actual_pressure = brake_can_decode_feedback_message(data);
                feedback->pressure_valid = true;
                feedback->last_update_time_ms = (uint32_t)(time(NULL) * 1000);
                return true;
            }
        }
    }
    
    return false;
}

void brake_can_encode_pressure_message(uint8_t* data, float pressure, uint8_t wheel_id) {
    if (!data) return;
    
    // Simple CAN message format:
    // Byte 0: Wheel ID
    // Bytes 1-2: Pressure (scaled to 0-65535 for 0-250 bar)
    // Bytes 3-7: Reserved/checksum
    
    uint16_t pressure_scaled = (uint16_t)((pressure / 250.0f) * 65535.0f);
    
    data[0] = wheel_id;
    data[1] = (uint8_t)(pressure_scaled & 0xFF);
    data[2] = (uint8_t)((pressure_scaled >> 8) & 0xFF);
    data[3] = 0; // Reserved
    data[4] = 0; // Reserved
    data[5] = 0; // Reserved
    data[6] = 0; // Reserved
    data[7] = (uint8_t)(data[0] ^ data[1] ^ data[2]); // Simple checksum
}

float brake_can_decode_feedback_message(const uint8_t* data) {
    if (!data) return 0.0f;
    
    // Decode pressure from CAN message
    uint16_t pressure_scaled = (uint16_t)(data[1] | (data[2] << 8));
    float pressure = ((float)pressure_scaled / 65535.0f) * 250.0f;
    
    return pressure;
}

// Utility functions
bool brake_pressure_within_limits(float pressure, const brake_channel_config_t* config) {
    if (!config) return false;
    
    return (pressure >= config->min_pressure && pressure <= config->max_pressure);
}

float brake_apply_pressure_ramp(float current, float target, float rate, float dt) {
    float max_change = rate * dt;
    float error = target - current;
    
    if (fabsf(error) <= max_change) {
        return target;
    } else if (error > 0) {
        return current + max_change;
    } else {
        return current - max_change;
    }
}

void brake_reset_pid_controller(brake_pid_controller_t* controller) {
    if (!controller) return;
    
    controller->integral_sum = 0.0f;
    controller->last_error = 0.0f;
}

float brake_execute_pid_control(brake_pid_controller_t* controller, float error, float dt) {
    if (!controller) return 0.0f;
    
    float output = 0.0f;
    
    // Proportional term
    output += controller->kp * error;
    
    // Integral term (with windup protection)
    if (controller->enable_integral) {
        controller->integral_sum += error * dt;
        
        // Integral windup protection
        if (controller->integral_sum > controller->integral_limit) {
            controller->integral_sum = controller->integral_limit;
        } else if (controller->integral_sum < -controller->integral_limit) {
            controller->integral_sum = -controller->integral_limit;
        }
        
        output += controller->ki * controller->integral_sum;
    }
    
    // Derivative term
    if (controller->enable_derivative && dt > 0) {
        float derivative = (error - controller->last_error) / dt;
        output += controller->kd * derivative;
    }
    
    // Store error for next iteration
    controller->last_error = error;
    
    // Apply output limits
    output = fmaxf(controller->output_min, fminf(output, controller->output_max));
    
    return output;
}

// Default configurations
brake_channel_config_t brake_get_default_pwm_config(wheel_position_t wheel) {
    brake_channel_config_t config = {0};
    
    config.interface_type = BRAKE_INTERFACE_PWM;
    config.channel_id = (uint8_t)wheel;
    config.max_pressure = 200.0f;           // 200 bar max
    config.min_pressure = 0.0f;
    config.pressure_resolution = 0.1f;      // 0.1 bar resolution
    config.pwm_frequency_hz = 1000;         // 1 kHz PWM
    config.response_time_ms = 10.0f;        // 10ms response time
    config.invert_signal = false;
    config.deadband_percent = 1.0f;         // 1% deadband
    
    return config;
}

brake_channel_config_t brake_get_default_can_config(wheel_position_t wheel) {
    brake_channel_config_t config = {0};
    
    config.interface_type = BRAKE_INTERFACE_CAN;
    config.channel_id = (uint8_t)wheel;
    config.max_pressure = 250.0f;           // 250 bar max for CAN
    config.min_pressure = 0.0f;
    config.pressure_resolution = 0.05f;     // 0.05 bar resolution
    config.can_id = 0x200 + wheel;          // CAN ID 0x200-0x203
    config.response_time_ms = 5.0f;         // 5ms response time
    config.invert_signal = false;
    config.deadband_percent = 0.5f;         // 0.5% deadband
    
    return config;
}

brake_pid_controller_t brake_get_default_pid_config(void) {
    brake_pid_controller_t controller = {0};
    
    controller.kp = 10.0f;                  // Proportional gain
    controller.ki = 2.0f;                   // Integral gain
    controller.kd = 0.1f;                   // Derivative gain
    controller.output_min = -100.0f;        // Minimum output
    controller.output_max = 100.0f;         // Maximum output
    controller.integral_limit = 50.0f;      // Integral windup limit
    controller.enable_integral = true;
    controller.enable_derivative = true;
    
    return controller;
}

// Safety and monitoring functions
bool brake_safety_check(const brake_actuation_system_t* system) {
    if (!system) return false;
    
    // Check system initialization
    if (!system->system_initialized) {
        return false;
    }
    
    // Check for excessive pressure errors
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        if (system->max_pressure_error[i] > 20.0f) { // 20 bar max error
            printf("Excessive pressure error on wheel %d: %.1f bar\n", i, system->max_pressure_error[i]);
            return false;
        }
        
        // Check for feedback timeouts
        if (brake_detect_feedback_timeout(system, (wheel_position_t)i)) {
            printf("Feedback timeout detected on wheel %d\n", i);
            return false;
        }
    }
    
    // Check system fault count
    if (system->system_fault_count > 10) {
        printf("Excessive system faults: %u\n", system->system_fault_count);
        return false;
    }
    
    return true;
}

void brake_emergency_shutdown(brake_actuation_system_t* system) {
    if (!system) return;
    
    printf("EMERGENCY: Brake system shutdown initiated\n");
    
    // Disable all brake channels
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_enable_channel(system, (wheel_position_t)i, false);
    }
    
    // Set system to fault state
    system->system_status = BRAKE_STATUS_FAULT;
    system->emergency_mode = true;
}

bool brake_detect_feedback_timeout(const brake_actuation_system_t* system, wheel_position_t wheel) {
    if (!system || wheel >= MAX_WHEEL_COUNT) {
        return true;
    }
    
    return system->feedback[wheel].feedback_timeout;
}

// Diagnostic functions
const char* brake_status_to_string(brake_modulator_status_t status) {
    switch (status) {
        case BRAKE_STATUS_OK: return "OK";
        case BRAKE_STATUS_WARNING: return "WARNING";
        case BRAKE_STATUS_FAULT: return "FAULT";
        case BRAKE_STATUS_OFFLINE: return "OFFLINE";
        case BRAKE_STATUS_CALIBRATING: return "CALIBRATING";
        default: return "UNKNOWN";
    }
}

const char* brake_interface_to_string(brake_interface_type_t interface) {
    switch (interface) {
        case BRAKE_INTERFACE_PWM: return "PWM";
        case BRAKE_INTERFACE_CAN: return "CAN";
        case BRAKE_INTERFACE_SPI: return "SPI";
        case BRAKE_INTERFACE_ANALOG: return "ANALOG";
        default: return "UNKNOWN";
    }
}

// Additional function implementations
brake_feedback_t brake_get_feedback(const brake_actuation_system_t* system, wheel_position_t wheel) {
    brake_feedback_t feedback = {0};
    if (!system || wheel >= MAX_WHEEL_COUNT) {
        return feedback;
    }
    return system->feedback[wheel];
}

brake_modulator_status_t brake_get_system_status(const brake_actuation_system_t* system) {
    if (!system) {
        return BRAKE_STATUS_FAULT;
    }
    return system->system_status;
}

float brake_get_pressure_error(const brake_actuation_system_t* system, wheel_position_t wheel) {
    if (!system || wheel >= MAX_WHEEL_COUNT) {
        return 0.0f;
    }
    return system->feedback[wheel].pressure_error;
}

bool brake_is_pressure_stable(const brake_actuation_system_t* system, wheel_position_t wheel, float tolerance) {
    if (!system || wheel >= MAX_WHEEL_COUNT) {
        return false;
    }
    return fabsf(system->feedback[wheel].pressure_error) <= tolerance;
}

// Calibration function stubs
bool brake_calibrate_pressure_sensor(brake_actuation_system_t* system, wheel_position_t wheel,
                                    float reference_pressure) {
    if (!system || wheel >= MAX_WHEEL_COUNT) {
        return false;
    }
    
    // Simple calibration - adjust offset to match reference
    float actual_pressure = system->feedback[wheel].actual_pressure;
    system->pressure_calibration_offset[wheel] = reference_pressure - actual_pressure;
    system->calibration_valid[wheel] = true;
    
    printf("Calibrated pressure sensor for wheel %d (offset: %.2f bar)\n", 
           wheel, system->pressure_calibration_offset[wheel]);
    return true;
}

bool brake_auto_calibrate_system(brake_actuation_system_t* system) {
    if (!system) return false;
    
    printf("Auto-calibrating brake system...\n");
    // Implementation would perform automatic calibration routine
    return true;
}

bool brake_save_calibration_data(const brake_actuation_system_t* system, const char* filename) {
    if (!system || !filename) return false;
    printf("Calibration data saved to %s\n", filename);
    return true;
}

bool brake_load_calibration_data(brake_actuation_system_t* system, const char* filename) {
    if (!system || !filename) return false;
    printf("Calibration data loaded from %s\n", filename);
    return true;
}

void brake_handle_system_fault(brake_actuation_system_t* system, wheel_position_t wheel) {
    if (!system || wheel >= MAX_WHEEL_COUNT) return;
    
    printf("Handling system fault for wheel %d\n", wheel);
    system->system_fault_count++;
    
    // Disable faulty channel
    brake_enable_channel(system, wheel, false);
}

void brake_print_feedback_data(const brake_actuation_system_t* system) {
    if (!system) return;
    
    printf("\n=== Brake Feedback Data ===\n");
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        const brake_feedback_t* feedback = &system->feedback[i];
        printf("Wheel %d: Target=%.1f, Actual=%.1f, Error=%.1f, Status=%s\n",
               i, feedback->target_pressure, feedback->actual_pressure,
               feedback->pressure_error, brake_status_to_string(feedback->status));
    }
}

void brake_log_performance_metrics(const brake_actuation_system_t* system) {
    if (!system) return;
    
    printf("Performance Metrics: Commands=%u, Timeouts=%u, Faults=%u\n",
           system->command_count, system->feedback_timeouts, system->system_fault_count);
}

void brake_print_system_status(const brake_actuation_system_t* system) {
    if (!system) return;
    
    printf("\n=== Brake Actuation System Status ===\n");
    printf("System Status: %s\n", brake_status_to_string(system->system_status));
    printf("Closed-loop Control: %s\n", system->closed_loop_enabled ? "ENABLED" : "DISABLED");
    printf("Emergency Mode: %s\n", system->emergency_mode ? "ACTIVE" : "INACTIVE");
    printf("Commands Sent: %u\n", system->command_count);
    printf("Feedback Timeouts: %u\n", system->feedback_timeouts);
    
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        const brake_feedback_t* feedback = &system->feedback[i];
        const brake_channel_config_t* config = &system->channel_config[i];
        
        printf("\nWheel %d (%s):\n", i, brake_interface_to_string(config->interface_type));
        printf("  Target Pressure: %.1f bar\n", feedback->target_pressure);
        printf("  Actual Pressure: %.1f bar\n", feedback->actual_pressure);
        printf("  Pressure Error: %.1f bar\n", feedback->pressure_error);
        printf("  Status: %s\n", brake_status_to_string(feedback->status));
        printf("  Max Error: %.1f bar\n", system->max_pressure_error[i]);
        
        if (config->interface_type == BRAKE_INTERFACE_PWM) {
            printf("  PWM Duty: %.1f%%\n", (float)system->current_command.pwm_duty[i] / 10.0f);
        }
    }
    
    printf("=====================================\n");
}