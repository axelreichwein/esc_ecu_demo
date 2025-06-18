#include "esc_ecu.h"
#include "sensor_plausibility.h"
#include "esc_control_algorithm.h"
#include "brake_actuation.h"
#include "engine_torque_interface.h"
#include "watchdog_supervision.h"
#include "dtc_management.h"
#include "post_system.h"
#include "esc_logging.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Global system state
static esc_system_t* g_esc_system = NULL;
static vehicle_parameters_t g_vehicle_params;
static esc_control_system_t g_control_system;

// POST hardware interface function prototypes
static bool esc_post_hw_memory_test(void* address, uint32_t size);
static uint32_t esc_post_hw_calculate_checksum(void* address, uint32_t size);
static bool esc_post_hw_sensor_power_on(esc_sensor_id_t sensor_id);
static bool esc_post_hw_sensor_power_off(esc_sensor_id_t sensor_id);
static float esc_post_hw_sensor_read_value(esc_sensor_id_t sensor_id);
static bool esc_post_hw_sensor_self_test(esc_sensor_id_t sensor_id);
static bool esc_post_hw_communication_test(uint8_t interface_id);
static uint32_t esc_post_hw_get_timestamp(void);
static void esc_post_hw_delay_ms(uint32_t milliseconds);
static void esc_post_hw_system_reset(void);

// Control algorithm constants
#define YAW_RATE_THRESHOLD 0.1f        // rad/s
#define LATERAL_ACCEL_THRESHOLD 4.0f   // m/s^2
#define SLIP_ANGLE_THRESHOLD 0.1f      // radians
#define ROLLOVER_THRESHOLD 0.8f        // lateral acceleration ratio
#define FILTER_ALPHA 0.8f              // Low-pass filter coefficient
#define MIN_VEHICLE_SPEED 2.0f         // m/s (minimum speed for ESC activation)
#define MAX_BRAKE_PRESSURE 250.0f      // bar
#define PI 3.14159265359f

void esc_init(esc_system_t* esc, const vehicle_parameters_t* params) {
    if (!esc || !params) return;
    
    // Initialize system state
    memset(esc, 0, sizeof(esc_system_t));
    esc->status = ESC_STATUS_STANDBY;
    esc->dt = ESC_SAMPLE_TIME_MS / 1000.0f;
    esc->initialization_complete = false;
    esc->system_ready = false;
    
    // Store vehicle parameters
    memcpy(&g_vehicle_params, params, sizeof(vehicle_parameters_t));
    g_esc_system = esc;
    
    printf("ESC: Starting Power-On Self-Test (POST) sequence...\n");
    
    // Configure and execute POST system FIRST
    if (!esc_configure_post_system(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        printf("ESC: POST system configuration failed\n");
        return;
    }
    
    // Execute complete POST sequence
    if (!esc_execute_post_sequence(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        printf("ESC: POST sequence failed - ESC system DISABLED\n");
        return;
    }
    
    // Validate system readiness after POST
    if (!esc_check_system_readiness(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        printf("ESC: System readiness check failed\n");
        return;
    }
    
    printf("ESC: POST completed successfully - proceeding with subsystem initialization\n");
    
    // Initialize sensor interface system
    if (!sensor_interface_init()) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        esc_handle_post_failure(esc, "Sensor interface initialization failed");
        return;
    }
    
    // Configure all sensors
    if (!esc_configure_sensors(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        esc_handle_post_failure(esc, "Sensor configuration failed");
        return;
    }
    
    // Allocate and initialize plausibility checking system
    esc->plausibility = (struct plausibility_system_t*)malloc(sizeof(plausibility_system_t));
    if (!esc->plausibility || !plausibility_init((plausibility_system_t*)esc->plausibility, params)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        if (esc->plausibility) {
            free(esc->plausibility);
            esc->plausibility = NULL;
        }
        return;
    }
    
    // Configure plausibility thresholds (use default for now)
    plausibility_thresholds_t thresholds = plausibility_get_default_thresholds();
    plausibility_configure_thresholds((plausibility_system_t*)esc->plausibility, &thresholds);
    
    // Initialize advanced ESC control algorithm
    if (!esc_control_init(&g_control_system)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        return;
    }
    
    // Initialize brake actuation system
    if (!esc_configure_brake_system(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        return;
    }
    
    // Initialize engine torque interface
    if (!esc_configure_engine_system(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        return;
    }
    
    // Initialize watchdog supervision system
    if (!esc_configure_watchdog_system(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        return;
    }
    
    // Initialize DTC management system
    if (!esc_configure_dtc_system(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        esc_handle_post_failure(esc, "DTC system configuration failed");
        return;
    }
    
    // Initialize logging system
    if (!esc_configure_logging_system(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        esc_handle_post_failure(esc, "Logging system configuration failed");
        return;
    }
    
    // Final validation of initialization complete
    if (!esc_validate_initialization_complete(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        esc_handle_post_failure(esc, "Final initialization validation failed");
        return;
    }
    
    // Mark initialization as complete
    esc->initialization_complete = true;
    esc->system_ready = true;
    
    // Perform self-test
    if (!esc_self_test(esc)) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        esc_handle_post_failure(esc, "Self-test failed");
    } else {
        printf("ESC: System initialization completed successfully - ESC ENABLED\n");
    }
}

void esc_sensor_acquisition(esc_system_t* esc) {
    if (!esc) return;
    
    // Kick watchdog for sensor acquisition task
    WATCHDOG_TASK_START(esc->watchdog_system, WATCHDOG_TASK_SENSOR_ACQUISITION);
    
    // Get sensor data from the asynchronous sensor interface
    sensor_data_t sensor_data;
    
    // Timestamp
    esc->sensors.timestamp_ms = get_microsecond_timestamp() / 1000;
    
    // Wheel speed sensors
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        if (sensor_get_data((sensor_type_t)i, &sensor_data)) {
            if (sensor_data.data_valid && sensor_data.status == SENSOR_STATUS_OK) {
                esc->sensors.wheel_speed[i] = sensor_data.filtered_value;
            }
        }
    }
    
    // Steering angle sensor
    if (sensor_get_data(SENSOR_STEERING_ANGLE, &sensor_data)) {
        if (sensor_data.data_valid && sensor_data.status == SENSOR_STATUS_OK) {
            esc->sensors.steering_angle = sensor_data.filtered_value;
        }
    }
    
    // Yaw rate sensor
    if (sensor_get_data(SENSOR_YAW_RATE, &sensor_data)) {
        if (sensor_data.data_valid && sensor_data.status == SENSOR_STATUS_OK) {
            esc->sensors.yaw_rate = sensor_data.filtered_value;
        }
    }
    
    // Lateral acceleration sensor
    if (sensor_get_data(SENSOR_LATERAL_ACCEL, &sensor_data)) {
        if (sensor_data.data_valid && sensor_data.status == SENSOR_STATUS_OK) {
            esc->sensors.lateral_acceleration = sensor_data.filtered_value;
        }
    }
    
    // Longitudinal acceleration sensor
    if (sensor_get_data(SENSOR_LONGITUDINAL_ACCEL, &sensor_data)) {
        if (sensor_data.data_valid && sensor_data.status == SENSOR_STATUS_OK) {
            esc->sensors.longitudinal_acceleration = sensor_data.filtered_value;
        }
    }
    
    // Calculate vehicle speed from wheel speeds
    esc->sensors.vehicle_speed = esc_calculate_vehicle_speed(esc->sensors.wheel_speed);
    
    // Digital inputs (brake and accelerator pedal status)
    // These would typically come from separate digital inputs or CAN messages
    esc->sensors.brake_pedal_pressed = false;      // Would read from brake switch
    esc->sensors.accelerator_pedal_pressed = false; // Would read from accelerator position
    
    // Complete watchdog kick for sensor acquisition
    WATCHDOG_TASK_END(esc->watchdog_system, WATCHDOG_TASK_SENSOR_ACQUISITION);
}

void esc_vehicle_dynamics_calculation(esc_system_t* esc, const vehicle_parameters_t* params) {
    if (!esc || !params) return;
    
    // Calculate reference yaw rate based on steering input and vehicle speed
    esc->dynamics.reference_yaw_rate = esc_calculate_reference_yaw_rate(
        esc->sensors.steering_angle, 
        esc->sensors.vehicle_speed, 
        params->wheelbase);
    
    // Calculate yaw rate error
    esc->dynamics.yaw_rate_error = esc->sensors.yaw_rate - esc->dynamics.reference_yaw_rate;
    
    // Calculate vehicle sideslip angle (beta)
    if (esc->sensors.vehicle_speed > MIN_VEHICLE_SPEED) {
        esc->dynamics.vehicle_beta = atan2f(
            esc->sensors.lateral_acceleration,
            esc->sensors.vehicle_speed * esc->sensors.vehicle_speed / g_vehicle_params.wheelbase);
    } else {
        esc->dynamics.vehicle_beta = 0.0f;
    }
    
    // Calculate slip angle
    esc->dynamics.slip_angle = esc->dynamics.vehicle_beta;
    
    // Detect understeer condition
    esc->dynamics.understeer_detected = 
        (fabsf(esc->dynamics.yaw_rate_error) > YAW_RATE_THRESHOLD) && 
        (esc->dynamics.yaw_rate_error * esc->sensors.steering_angle > 0) &&
        (esc->sensors.vehicle_speed > MIN_VEHICLE_SPEED);
    
    // Detect oversteer condition
    esc->dynamics.oversteer_detected = 
        (fabsf(esc->dynamics.yaw_rate_error) > YAW_RATE_THRESHOLD) && 
        (esc->dynamics.yaw_rate_error * esc->sensors.steering_angle < 0) &&
        (esc->sensors.vehicle_speed > MIN_VEHICLE_SPEED);
    
    // Rollover detection
    float lateral_accel_threshold = ROLLOVER_THRESHOLD * 9.81f; // Convert to m/s^2
    esc->dynamics.rollover_risk = 
        (fabsf(esc->sensors.lateral_acceleration) > lateral_accel_threshold);
}

void esc_stability_control(esc_system_t* esc, const vehicle_parameters_t* params) {
    if (!esc || !params) return;
    
    // Kick watchdog for control algorithm task
    WATCHDOG_TASK_START(esc->watchdog_system, WATCHDOG_TASK_CONTROL_ALGORITHM);
    
    // Reset actuator commands
    memset(&esc->actuators, 0, sizeof(actuator_commands_t));
    
    // Only activate ESC if vehicle speed is above minimum threshold
    if (esc->sensors.vehicle_speed < MIN_VEHICLE_SPEED) {
        esc->status = ESC_STATUS_STANDBY;
        return;
    }
    
    // Use advanced ESC control algorithm
    uint32_t current_time_ms = get_microsecond_timestamp() / 1000;
    esc_control_update(&g_control_system, &esc->sensors, params, current_time_ms);
    
    // Copy control commands from advanced algorithm to actuator commands
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        esc->actuators.brake_pressure[i] = g_control_system.brake_control.brake_pressure[i];
    }
    esc->actuators.engine_torque_reduction = g_control_system.engine_control.torque_reduction_percent;
    
    // Update system status based on control algorithm state
    bool intervention_required = false;
    switch (g_control_system.current_state) {
        case ESC_STATE_UNDERSTEER:
        case ESC_STATE_OVERSTEER:
        case ESC_STATE_ROLLOVER_PREVENTION:
        case ESC_STATE_BRAKE_ASSIST:
            esc->status = ESC_STATUS_ACTIVE;
            intervention_required = true;
            break;
            
        case ESC_STATE_FAULT:
            esc->status = ESC_STATUS_FAULT;
            break;
            
        case ESC_STATE_STANDBY:
        default:
            esc->status = ESC_STATUS_STANDBY;
            break;
    }
    
    // Update ESC dynamics based on control system assessment
    switch (g_control_system.vehicle_condition) {
        case VEHICLE_UNDERSTEER_MILD:
        case VEHICLE_UNDERSTEER_SEVERE:
            esc->dynamics.understeer_detected = true;
            break;
        case VEHICLE_OVERSTEER_MILD:
        case VEHICLE_OVERSTEER_SEVERE:
            esc->dynamics.oversteer_detected = true;
            break;
        case VEHICLE_ROLLOVER_RISK:
            esc->dynamics.rollover_risk = true;
            break;
        default:
            esc->dynamics.understeer_detected = false;
            esc->dynamics.oversteer_detected = false;
            esc->dynamics.rollover_risk = false;
            break;
    }
    
    // Set warning lights based on intervention
    esc->actuators.esc_warning_light = intervention_required;
    
    // Activate brake lights if any brake pressure is applied
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        if (esc->actuators.brake_pressure[i] > 0) {
            esc->actuators.brake_light_active = true;
            break;
        }
    }
    
    // Complete watchdog kick for control algorithm
    WATCHDOG_TASK_END(esc->watchdog_system, WATCHDOG_TASK_CONTROL_ALGORITHM);
}

void esc_actuator_control(esc_system_t* esc) {
    if (!esc) return;
    
    // Execute brake commands through brake actuation system
    WATCHDOG_TASK_START(esc->watchdog_system, WATCHDOG_TASK_BRAKE_ACTUATION);
    esc_execute_brake_commands(esc);
    WATCHDOG_TASK_END(esc->watchdog_system, WATCHDOG_TASK_BRAKE_ACTUATION);
    
    // Execute engine torque commands
    WATCHDOG_TASK_START(esc->watchdog_system, WATCHDOG_TASK_ENGINE_INTERFACE);
    esc_execute_engine_commands(esc);
    WATCHDOG_TASK_END(esc->watchdog_system, WATCHDOG_TASK_ENGINE_INTERFACE);
    
    // Engine torque reduction (would send CAN message to engine ECU)
    if (esc->actuators.engine_torque_reduction > 0) {
        // send_engine_torque_request(esc->actuators.engine_torque_reduction);
        printf("Engine torque reduction: %.1f%%\n", esc->actuators.engine_torque_reduction);
    }
    
    // Warning lights control
    // set_warning_light(ESC_WARNING, esc->actuators.esc_warning_light);
    // set_brake_light(esc->actuators.brake_light_active);
}

void esc_diagnostics(esc_system_t* esc) {
    if (!esc) return;
    
    // Kick watchdog for diagnostics task
    WATCHDOG_TASK_START(esc->watchdog_system, WATCHDOG_TASK_DIAGNOSTICS);
    
    // Check sensor status from sensor interface
    bool any_sensor_fault = false;
    
    // Check wheel speed sensors
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        sensor_status_t status = sensor_get_status((sensor_type_t)i);
        esc->diagnostics.sensor_fault[i] = (status != SENSOR_STATUS_OK);
        if (status != SENSOR_STATUS_OK && status != SENSOR_STATUS_NO_SIGNAL) {
            any_sensor_fault = true;
        }
    }
    
    // Check other sensors
    sensor_status_t steering_status = sensor_get_status(SENSOR_STEERING_ANGLE);
    esc->diagnostics.sensor_fault[4] = (steering_status != SENSOR_STATUS_OK);
    if (steering_status != SENSOR_STATUS_OK && steering_status != SENSOR_STATUS_NO_SIGNAL) {
        any_sensor_fault = true;
    }
    
    sensor_status_t yaw_status = sensor_get_status(SENSOR_YAW_RATE);
    esc->diagnostics.sensor_fault[5] = (yaw_status != SENSOR_STATUS_OK);
    if (yaw_status != SENSOR_STATUS_OK && yaw_status != SENSOR_STATUS_NO_SIGNAL) {
        any_sensor_fault = true;
    }
    
    sensor_status_t lat_accel_status = sensor_get_status(SENSOR_LATERAL_ACCEL);
    esc->diagnostics.sensor_fault[6] = (lat_accel_status != SENSOR_STATUS_OK);
    if (lat_accel_status != SENSOR_STATUS_OK && lat_accel_status != SENSOR_STATUS_NO_SIGNAL) {
        any_sensor_fault = true;
    }
    
    sensor_status_t long_accel_status = sensor_get_status(SENSOR_LONGITUDINAL_ACCEL);
    esc->diagnostics.sensor_fault[7] = (long_accel_status != SENSOR_STATUS_OK);
    if (long_accel_status != SENSOR_STATUS_OK && long_accel_status != SENSOR_STATUS_NO_SIGNAL) {
        any_sensor_fault = true;
    }
    
    // Run sensor plausibility checks
    if (esc->plausibility) {
        plausibility_update((plausibility_system_t*)esc->plausibility, &esc->sensors, &g_vehicle_params);
    }
    
    // Check plausibility status
    plausibility_status_t plausibility_status = PLAUSIBILITY_OK;
    if (esc->plausibility) {
        plausibility_status = plausibility_get_overall_status((const plausibility_system_t*)esc->plausibility);
    }
    bool plausibility_fault = false;
    
    if (plausibility_status >= PLAUSIBILITY_MAJOR_FAULT) {
        plausibility_fault = true;
        any_sensor_fault = true;
    } else if (plausibility_status == PLAUSIBILITY_MINOR_FAULT) {
        // Minor plausibility faults are warnings but don't cause system fault
        esc->diagnostics.calibration_required = true;
    }
    
    // Update system fault status
    if (any_sensor_fault) {
        esc->diagnostics.system_fault = true;
        esc->status = ESC_STATUS_FAULT;
        
        // Set fault code based on type of fault
        if (plausibility_fault) {
            esc->diagnostics.fault_code = 0x1000; // Plausibility fault base code
            if (plausibility_status == PLAUSIBILITY_CRITICAL_FAULT) {
                esc->diagnostics.fault_code |= 0x0100; // Critical plausibility fault
            }
        }
    } else {
        esc->diagnostics.system_fault = false;
        esc->diagnostics.fault_code = 0;
        if (esc->status == ESC_STATUS_FAULT) {
            esc->status = ESC_STATUS_STANDBY; // Recover from fault if sensors are OK
        }
    }
    
    // Complete watchdog kick for diagnostics
    WATCHDOG_TASK_END(esc->watchdog_system, WATCHDOG_TASK_DIAGNOSTICS);
}

bool esc_self_test(esc_system_t* esc) {
    if (!esc) return false;
    
    // Perform system self-test
    // In real implementation, this would test:
    // - Sensor functionality
    // - Actuator response
    // - Communication links
    // - Memory integrity
    
    // For demonstration, assume self-test passes
    return true;
}

void esc_update(esc_system_t* esc) {
    if (!esc) return;
    
    // Kick watchdog for main loop
    WATCHDOG_TASK_START(esc->watchdog_system, WATCHDOG_TASK_MAIN_LOOP);
    
    // Increment cycle counter
    esc->cycle_count++;
    
    // Check for watchdog safe state requirements
    if (!esc_handle_watchdog_safe_state(esc)) {
        // If in safe state, perform limited operations only
        WATCHDOG_TASK_END(esc->watchdog_system, WATCHDOG_TASK_MAIN_LOOP);
        return;
    }
    
    // Main control loop
    esc_sensor_acquisition(esc);
    esc_vehicle_dynamics_calculation(esc, &g_vehicle_params);
    esc_stability_control(esc, &g_vehicle_params);
    esc_actuator_control(esc);
    esc_diagnostics(esc);
    
    // Update DTC monitoring
    esc_update_dtc_monitoring(esc);
    
    // Update watchdog monitoring
    esc_update_watchdog_monitoring(esc);
    
    // Update logging system
    esc_update_logging(esc);
    
    // Global watchdog kick
    WATCHDOG_GLOBAL_KICK(esc->watchdog_system);
    WATCHDOG_TASK_END(esc->watchdog_system, WATCHDOG_TASK_MAIN_LOOP);
}

// Utility functions
float esc_calculate_vehicle_speed(const float wheel_speeds[MAX_WHEEL_COUNT]) {
    if (!wheel_speeds) return 0.0f;
    
    // Calculate average of non-driven wheels (typically rear wheels)
    // This assumes front-wheel drive vehicle
    float avg_speed = (wheel_speeds[WHEEL_RL] + wheel_speeds[WHEEL_RR]) / 2.0f;
    
    // Convert from angular velocity to linear velocity
    return avg_speed * g_vehicle_params.wheel_radius;
}

// esc_calculate_reference_yaw_rate is now defined in esc_control_algorithm.c

float esc_low_pass_filter(float current_value, float new_value, float alpha) {
    return alpha * current_value + (1.0f - alpha) * new_value;
}

float esc_limit_value(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// Sensor interface integration functions
bool esc_configure_sensors(esc_system_t* esc) {
    if (!esc) return false;
    
    // Configure wheel speed sensors (analog interface)
    sensor_hw_spec_t wheel_speed_spec = {
        .interface_type = SENSOR_IF_ANALOG,
        .sample_rate_hz = 1000,        // High frequency for wheel speed
        .resolution_bits = 12,         // 12-bit ADC
        .voltage_range_min = 0.0f,
        .voltage_range_max = 5.0f,
        .filter_type = FILTER_LOW_PASS,
        .filter_cutoff_hz = 50.0f      // 50 Hz cutoff for wheel speed
    };
    
    sensor_calibration_t wheel_speed_cal = {
        .offset = 0.0f,
        .scale = 1.0f,
        .min_value = 0.0f,
        .max_value = 300.0f,           // 300 rad/s max wheel speed
        .stuck_threshold = 0.1f,
        .noise_threshold = 0.5f,
        .stuck_time_ms = 1000,
        .calibration_valid = true
    };
    
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        if (!sensor_configure((sensor_type_t)i, &wheel_speed_spec, &wheel_speed_cal)) {
            return false;
        }
        sensor_set_callback((sensor_type_t)i, esc_sensor_callback);
        sensor_start_acquisition((sensor_type_t)i);
    }
    
    // Configure steering angle sensor (analog interface)
    sensor_hw_spec_t steering_spec = {
        .interface_type = SENSOR_IF_ANALOG,
        .sample_rate_hz = 100,         // 100 Hz for steering angle
        .resolution_bits = 12,
        .voltage_range_min = 0.0f,
        .voltage_range_max = 5.0f,
        .filter_type = FILTER_COMBINED, // Use combined filtering
        .filter_cutoff_hz = 10.0f,     // 10 Hz cutoff
        .median_window_size = 5        // 5-point median filter
    };
    
    sensor_calibration_t steering_cal = {
        .offset = 0.0f,
        .scale = 1.0f,
        .min_value = -1.57f,           // ±90 degrees in radians
        .max_value = 1.57f,
        .stuck_threshold = 0.01f,
        .noise_threshold = 0.1f,
        .stuck_time_ms = 2000,
        .calibration_valid = true
    };
    
    if (!sensor_configure(SENSOR_STEERING_ANGLE, &steering_spec, &steering_cal)) {
        return false;
    }
    sensor_set_callback(SENSOR_STEERING_ANGLE, esc_sensor_callback);
    sensor_start_acquisition(SENSOR_STEERING_ANGLE);
    
    // Configure yaw rate sensor (SPI interface)
    sensor_hw_spec_t yaw_rate_spec = {
        .interface_type = SENSOR_IF_SPI,
        .sample_rate_hz = 200,         // 200 Hz for yaw rate (IMU)
        .resolution_bits = 16,
        .filter_type = FILTER_LOW_PASS,
        .filter_cutoff_hz = 20.0f      // 20 Hz cutoff for yaw rate
    };
    
    sensor_calibration_t yaw_rate_cal = {
        .offset = 0.0f,
        .scale = 1.0f,
        .min_value = -10.0f,           // ±10 rad/s max yaw rate
        .max_value = 10.0f,
        .stuck_threshold = 0.01f,
        .noise_threshold = 0.1f,
        .stuck_time_ms = 500,
        .calibration_valid = true
    };
    
    if (!sensor_configure(SENSOR_YAW_RATE, &yaw_rate_spec, &yaw_rate_cal)) {
        return false;
    }
    sensor_set_callback(SENSOR_YAW_RATE, esc_sensor_callback);
    sensor_start_acquisition(SENSOR_YAW_RATE);
    
    // Configure acceleration sensors (SPI interface - same IMU)
    sensor_hw_spec_t accel_spec = {
        .interface_type = SENSOR_IF_SPI,
        .sample_rate_hz = 200,         // 200 Hz for accelerometers
        .resolution_bits = 16,
        .filter_type = FILTER_LOW_PASS,
        .filter_cutoff_hz = 30.0f      // 30 Hz cutoff for acceleration
    };
    
    sensor_calibration_t accel_cal = {
        .offset = 0.0f,
        .scale = 1.0f,
        .min_value = -20.0f,           // ±20 m/s² max acceleration
        .max_value = 20.0f,
        .stuck_threshold = 0.1f,
        .noise_threshold = 0.5f,
        .stuck_time_ms = 500,
        .calibration_valid = true
    };
    
    if (!sensor_configure(SENSOR_LATERAL_ACCEL, &accel_spec, &accel_cal)) {
        return false;
    }
    sensor_set_callback(SENSOR_LATERAL_ACCEL, esc_sensor_callback);
    sensor_start_acquisition(SENSOR_LATERAL_ACCEL);
    
    if (!sensor_configure(SENSOR_LONGITUDINAL_ACCEL, &accel_spec, &accel_cal)) {
        return false;
    }
    sensor_set_callback(SENSOR_LONGITUDINAL_ACCEL, esc_sensor_callback);
    sensor_start_acquisition(SENSOR_LONGITUDINAL_ACCEL);
    
    printf("All ESC sensors configured and started\n");
    return true;
}

void esc_sensor_callback(sensor_type_t sensor_id, const sensor_data_t* data) {
    if (!g_esc_system || !data) return;
    
    // This callback is called asynchronously when new sensor data is available
    // For now, we just update a flag - the main loop will read the data
    // In a more advanced implementation, we could implement event-driven processing
    
    // Optional: Log sensor faults or warnings
    if (data->status != SENSOR_STATUS_OK) {
        static uint32_t last_error_log_time[SENSOR_COUNT] = {0};
        uint32_t current_time = get_microsecond_timestamp() / 1000;
        
        // Log errors at most once per second per sensor
        if (current_time - last_error_log_time[sensor_id] > 1000) {
            printf("Sensor %s fault: %s\n", 
                   sensor_type_to_string(sensor_id), 
                   sensor_status_to_string(data->status));
            last_error_log_time[sensor_id] = current_time;
        }
    }
}

// Hardware interface simulation functions for brake actuation
static bool sim_pwm_init(uint8_t channel, uint16_t frequency_hz) {
    printf("Simulated PWM channel %d initialized at %d Hz\n", channel, frequency_hz);
    return true;
}

static bool sim_pwm_set_duty(uint8_t channel, uint16_t duty_cycle) {
    static uint16_t last_duty[MAX_WHEEL_COUNT] = {0};
    
    // Only print if duty cycle changed significantly
    if (abs((int)duty_cycle - (int)last_duty[channel]) > 10) {
        printf("PWM channel %d: %.1f%% duty\n", channel, (float)duty_cycle / 10.0f);
        last_duty[channel] = duty_cycle;
    }
    return true;
}

static uint16_t sim_pwm_get_duty(uint8_t channel) {
    // Simulate reading current duty cycle
    (void)channel;
    return 0;
}

static bool sim_can_init(uint32_t baudrate) {
    printf("Simulated CAN interface initialized at %u bps\n", baudrate);
    return true;
}

static bool sim_can_send_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    // Simulate CAN message transmission
    printf("CAN TX: ID=0x%03X, Data=[", can_id);
    for (int i = 0; i < length; i++) {
        printf("%02X", data[i]);
        if (i < length - 1) printf(" ");
    }
    printf("]\n");
    return true;
}

static bool sim_can_receive_message(uint32_t* can_id, uint8_t* data, uint8_t* length) {
    // Simulate receiving feedback messages
    (void)can_id;
    (void)data;
    (void)length;
    return false; // No messages available in simulation
}

static bool sim_feedback_init(uint8_t channel) {
    printf("Simulated pressure feedback for channel %d initialized\n", channel);
    return true;
}

static float sim_read_pressure(uint8_t channel) {
    // Simulate pressure feedback - return slightly noisy version of commanded pressure
    static float simulated_pressure[MAX_WHEEL_COUNT] = {0};
    
    // Add some noise and response delay simulation
    float noise = ((float)rand() / RAND_MAX - 0.5f) * 0.5f; // ±0.25 bar noise
    simulated_pressure[channel] += noise;
    
    // Clamp to reasonable range
    if (simulated_pressure[channel] < 0) simulated_pressure[channel] = 0;
    if (simulated_pressure[channel] > 200) simulated_pressure[channel] = 200;
    
    return simulated_pressure[channel];
}

static float sim_read_temperature(uint8_t channel) {
    // Simulate brake fluid temperature
    (void)channel;
    return 60.0f; // 60°C typical brake fluid temperature
}

static bool sim_feedback_valid(uint8_t channel) {
    // Simulate valid feedback for all channels
    (void)channel;
    return true;
}

// Hardware interface structure for simulation
static const brake_hw_interface_t g_brake_hw_interface = {
    .pwm_init = sim_pwm_init,
    .pwm_set_duty = sim_pwm_set_duty,
    .pwm_get_duty = sim_pwm_get_duty,
    .can_init = sim_can_init,
    .can_send_message = sim_can_send_message,
    .can_receive_message = sim_can_receive_message,
    .feedback_init = sim_feedback_init,
    .read_pressure = sim_read_pressure,
    .read_temperature = sim_read_temperature,
    .feedback_valid = sim_feedback_valid
};

bool esc_configure_brake_system(esc_system_t* esc) {
    if (!esc) return false;
    
    // Allocate brake actuation system
    esc->brake_system = (struct brake_actuation_system_t*)malloc(sizeof(brake_actuation_system_t));
    if (!esc->brake_system) {
        printf("Failed to allocate brake actuation system\n");
        return false;
    }
    
    // Initialize brake actuation system with simulated hardware interface
    if (!brake_actuation_init((brake_actuation_system_t*)esc->brake_system, &g_brake_hw_interface)) {
        printf("Failed to initialize brake actuation system\n");
        free(esc->brake_system);
        esc->brake_system = NULL;
        return false;
    }
    
    // Configure brake channels
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_channel_config_t config;
        
        // Use PWM interface for front wheels, CAN for rear wheels (demonstration)
        if (i < 2) {
            config = brake_get_default_pwm_config((wheel_position_t)i);
        } else {
            config = brake_get_default_can_config((wheel_position_t)i);
        }
        
        if (!brake_configure_channel((brake_actuation_system_t*)esc->brake_system, 
                                    (wheel_position_t)i, &config)) {
            printf("Failed to configure brake channel %d\n", i);
            return false;
        }
        
        // Configure PID controllers for closed-loop operation
        brake_configure_pid_controller((brake_actuation_system_t*)esc->brake_system, 
                                     (wheel_position_t)i, 15.0f, 3.0f, 0.2f);
    }
    
    // Enable closed-loop control
    brake_enable_closed_loop((brake_actuation_system_t*)esc->brake_system, true);
    
    printf("Brake actuation system configured successfully\n");
    return true;
}

bool esc_execute_brake_commands(esc_system_t* esc) {
    if (!esc || !esc->brake_system) {
        return false;
    }
    
    // Convert ESC actuator commands to brake system commands
    float brake_pressures[MAX_WHEEL_COUNT];
    
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_pressures[i] = esc->actuators.brake_pressure[i];
    }
    
    // Send pressure commands to brake actuation system
    if (!brake_set_pressure_command((brake_actuation_system_t*)esc->brake_system, brake_pressures)) {
        printf("Failed to send brake pressure commands\n");
        return false;
    }
    
    // Update brake control loop (this would normally run in a separate high-frequency thread)
    brake_update_control_loop((brake_actuation_system_t*)esc->brake_system);
    
    // Get feedback for diagnostics
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_feedback_t feedback = brake_get_feedback((brake_actuation_system_t*)esc->brake_system, 
                                                      (wheel_position_t)i);
        
        // Check for brake system faults
        if (feedback.status >= BRAKE_STATUS_FAULT) {
            esc->diagnostics.actuator_fault[i] = true;
            printf("Brake actuator fault on wheel %d\n", i);
        } else {
            esc->diagnostics.actuator_fault[i] = false;
        }
    }
    
    // Check overall brake system status
    brake_modulator_status_t brake_status = brake_get_system_status((brake_actuation_system_t*)esc->brake_system);
    if (brake_status >= BRAKE_STATUS_FAULT) {
        esc->diagnostics.system_fault = true;
        printf("Brake system fault detected\n");
        return false;
    }
    
    return true;
}

// Hardware interface simulation functions for engine torque
static bool sim_engine_can_init(uint32_t baudrate) {
    printf("Simulated engine CAN interface initialized at %u bps\n", baudrate);
    return true;
}

static bool sim_engine_can_send_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    printf("Engine CAN TX: ID=0x%03X, Data=[", can_id);
    for (int i = 0; i < length; i++) {
        printf("%02X", data[i]);
        if (i < length - 1) printf(" ");
    }
    printf("]\n");
    return true;
}

static bool sim_engine_can_receive_message(uint32_t* can_id, uint8_t* data, uint8_t* length) {
    // Simulate receiving engine responses
    static int counter = 0;
    counter++;
    
    if (counter % 20 == 0) { // Send response every 20th call
        *can_id = 0x701; // Engine response ID
        *length = 8;
        
        // Simulate engine accepting torque reduction request
        data[0] = 0x01; // Status: ACCEPTED
        data[1] = 0x64; // 10.0% reduction (scaled)
        data[2] = 0x00;
        data[3] = 0x50; // Max available: 80% (scaled)
        data[4] = 0x00;
        data[5] = 0xE8; // Duration: 1000ms
        data[6] = 0x03;
        data[7] = 0x64 ^ 0x00 ^ 0x50 ^ 0x00 ^ 0xE8 ^ 0x03; // Checksum
        return true;
    }
    
    if (counter % 10 == 0) { // Send heartbeat every 10th call
        *can_id = 0x702; // Engine heartbeat ID
        *length = 8;
        
        // Simulate normal engine operation
        data[0] = 0x08; // Normal mode + torque capable
        data[1] = 0x80; // RPM low byte (2000 RPM / 10)
        data[2] = 0x07; // RPM high byte
        data[3] = 0x32; // 50% throttle
        data[4] = 0x00; // No fault
        data[5] = 0x96; // Torque low byte (150 Nm * 10)
        data[6] = 0x05; // Torque high byte
        data[7] = 0x08 ^ 0x80 ^ 0x07 ^ 0x32 ^ 0x00 ^ 0x96 ^ 0x05; // Checksum
        return true;
    }
    
    return false;
}

static bool sim_engine_can_set_filter(uint32_t can_id, uint32_t mask) {
    (void)can_id;
    (void)mask;
    return true;
}

static uint32_t sim_engine_get_timestamp_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// Engine CAN interface structure for simulation
static const engine_can_interface_t g_engine_can_interface = {
    .can_init = sim_engine_can_init,
    .can_send_message = sim_engine_can_send_message,
    .can_receive_message = sim_engine_can_receive_message,
    .can_set_filter = sim_engine_can_set_filter,
    .get_timestamp_ms = sim_engine_get_timestamp_ms
};

bool esc_configure_engine_system(esc_system_t* esc) {
    if (!esc) return false;
    
    // Allocate engine torque interface system
    esc->engine_system = (struct engine_torque_interface_t*)malloc(sizeof(engine_torque_interface_t));
    if (!esc->engine_system) {
        printf("Failed to allocate engine torque interface system\n");
        return false;
    }
    
    // Initialize engine torque interface with simulated CAN interface
    if (!engine_torque_init((engine_torque_interface_t*)esc->engine_system, &g_engine_can_interface)) {
        printf("Failed to initialize engine torque interface\n");
        free(esc->engine_system);
        esc->engine_system = NULL;
        return false;
    }
    
    // Configure CAN IDs for engine communication
    engine_torque_configure_can_ids((engine_torque_interface_t*)esc->engine_system, 
                                   0x700, 0x701, 0x702);
    
    // Configure fallback settings
    engine_fallback_config_t fallback_config = engine_get_default_fallback_config();
    fallback_config.enable_fallback = true;
    fallback_config.max_brake_compensation = 80.0f; // 80 bar max compensation
    fallback_config.engine_timeout_ms = 300;        // 300ms timeout
    fallback_config.max_retry_attempts = 2;         // 2 retry attempts
    
    engine_torque_configure_fallback((engine_torque_interface_t*)esc->engine_system, &fallback_config);
    
    printf("Engine torque interface configured successfully\n");
    return true;
}

bool esc_execute_engine_commands(esc_system_t* esc) {
    if (!esc || !esc->engine_system) {
        return false;
    }
    
    engine_torque_interface_t* engine_system = (engine_torque_interface_t*)esc->engine_system;
    
    // Check if engine torque reduction is needed
    if (esc->actuators.engine_torque_reduction > 0.1f) {
        // Check if we already have an active reduction
        if (!engine_is_reduction_active(engine_system)) {
            // Request new torque reduction
            uint8_t priority = 7; // High priority for ESC intervention
            uint16_t duration = 2000; // 2 second duration
            
            bool success = engine_request_torque_reduction(engine_system, 
                                                         esc->actuators.engine_torque_reduction,
                                                         duration, priority);
            
            if (!success) {
                printf("Failed to request engine torque reduction\n");
                return false;
            }
        } else {
            // Modify existing reduction if significantly different
            float current_reduction = engine_get_actual_reduction(engine_system);
            float reduction_diff = fabsf(esc->actuators.engine_torque_reduction - current_reduction);
            
            if (reduction_diff > 5.0f) { // 5% threshold for modification
                engine_modify_torque_reduction(engine_system, esc->actuators.engine_torque_reduction);
            }
        }
    } else {
        // Cancel torque reduction if currently active
        if (engine_is_reduction_active(engine_system)) {
            engine_cancel_torque_reduction(engine_system);
        }
    }
    
    // Update engine communication (process responses, handle timeouts)
    engine_update_communication(engine_system);
    
    // Check for fallback activation and update brake compensation
    if (engine_system->fallback_active) {
        // Add fallback brake pressure to ESC brake commands
        for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
            esc->actuators.brake_pressure[i] += engine_system->fallback_brake_pressure * 0.25f; // Distribute evenly
        }
        
        printf("Engine fallback active: Adding %.1f bar brake compensation\n", 
               engine_system->fallback_brake_pressure);
    }
    
    // Update diagnostics based on engine system status
    engine_request_status_t engine_status = engine_get_request_status(engine_system);
    if (engine_status == ENGINE_REQUEST_ECU_FAULT || engine_status == ENGINE_REQUEST_TIMEOUT) {
        esc->diagnostics.system_fault = true;
        printf("Engine torque system fault detected: %s\n", 
               engine_request_status_to_string(engine_status));
    }
    
    return true;
}

// Hardware interface simulation functions for watchdog
static bool sim_watchdog_hw_init(uint32_t timeout_ms) {
    printf("Simulated hardware watchdog initialized with %u ms timeout\n", timeout_ms);
    return true;
}

static void sim_watchdog_hw_kick(void) {
    // Hardware watchdog kick simulation
    static uint32_t kick_counter = 0;
    kick_counter++;
    if (kick_counter % 100 == 0) {
        printf("Hardware watchdog kicked (%u times)\n", kick_counter);
    }
}

static void sim_watchdog_hw_disable(void) {
    printf("Hardware watchdog disabled\n");
}

static void sim_watchdog_system_reset(void) {
    printf("SYSTEM RESET: Hardware reset requested\n");
    exit(2); // Exit with code 2 to indicate hardware reset
}

static uint32_t sim_watchdog_get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void sim_watchdog_enter_safe_mode(safe_state_level_t level) {
    printf("ENTERING HARDWARE SAFE MODE: Level %s\n", safe_state_level_to_string(level));
    
    switch (level) {
        case SAFE_STATE_LEVEL_1:
            printf("Hardware: Minimal safe operation enabled\n");
            break;
        case SAFE_STATE_LEVEL_2:
            printf("Hardware: Reduced functionality mode\n");
            break;
        case SAFE_STATE_LEVEL_3:
            printf("Hardware: Diagnostic mode activated\n");
            break;
        case SAFE_STATE_LEVEL_4:
            printf("Hardware: Shutdown preparation\n");
            break;
    }
}

static void sim_watchdog_emergency_shutdown(void) {
    printf("EMERGENCY SHUTDOWN: System emergency shutdown initiated\n");
    exit(3); // Exit with code 3 to indicate emergency shutdown
}

// Watchdog hardware interface for simulation
static const watchdog_hw_interface_t g_watchdog_hw_interface = {
    .hw_watchdog_init = sim_watchdog_hw_init,
    .hw_watchdog_kick = sim_watchdog_hw_kick,
    .hw_watchdog_disable = sim_watchdog_hw_disable,
    .hw_system_reset = sim_watchdog_system_reset,
    .get_system_time_ms = sim_watchdog_get_time_ms,
    .enter_safe_mode = sim_watchdog_enter_safe_mode,
    .emergency_shutdown = sim_watchdog_emergency_shutdown
};

// Watchdog recovery callback for ESC tasks
static void esc_watchdog_recovery_callback(watchdog_task_id_t task_id) {
    printf("ESC Watchdog Recovery: Attempting recovery for task %s\n", watchdog_task_name(task_id));
    
    switch (task_id) {
        case WATCHDOG_TASK_SENSOR_ACQUISITION:
            printf("Recovery: Reinitializing sensor interface\n");
            // In real system, would reinitialize sensor interface
            break;
            
        case WATCHDOG_TASK_BRAKE_ACTUATION:
            printf("Recovery: Resetting brake actuation system\n");
            // In real system, would reset brake system
            break;
            
        case WATCHDOG_TASK_ENGINE_INTERFACE:
            printf("Recovery: Resetting engine interface\n");
            // In real system, would reset engine communication
            break;
            
        case WATCHDOG_TASK_CONTROL_ALGORITHM:
            printf("Recovery: Resetting control algorithm state\n");
            // In real system, would reset control state
            break;
            
        default:
            printf("Recovery: Generic task reset\n");
            break;
    }
}

// Watchdog safe state callback for ESC system
static void esc_watchdog_safe_state_callback(safe_state_level_t level, const char* reason) {
    printf("ESC Safe State Callback: Level %s, Reason: %s\n", 
           safe_state_level_to_string(level), reason ? reason : "Unknown");
    
    // In real system, would implement specific safe state actions for ESC
}

bool esc_configure_watchdog_system(esc_system_t* esc) {
    if (!esc) return false;
    
    // Allocate watchdog supervision system
    esc->watchdog_system = (struct watchdog_supervision_t*)malloc(sizeof(watchdog_supervision_t));
    if (!esc->watchdog_system) {
        printf("Failed to allocate watchdog supervision system\n");
        return false;
    }
    
    // Initialize watchdog supervision with simulated hardware interface
    if (!watchdog_init((watchdog_supervision_t*)esc->watchdog_system, &g_watchdog_hw_interface)) {
        printf("Failed to initialize watchdog supervision system\n");
        free(esc->watchdog_system);
        esc->watchdog_system = NULL;
        return false;
    }
    
    // Configure recovery settings
    recovery_config_t recovery_config = watchdog_get_default_recovery_config();
    recovery_config.strategy = RESET_STRATEGY_MODULE_RESTART;
    recovery_config.recovery_timeout_ms = 3000;    // 3 second recovery timeout
    recovery_config.max_recovery_attempts = 2;     // 2 attempts before safe state
    recovery_config.recovery_callback = esc_watchdog_recovery_callback;
    recovery_config.safe_state_callback = esc_watchdog_safe_state_callback;
    
    watchdog_configure_recovery((watchdog_supervision_t*)esc->watchdog_system, &recovery_config);
    
    // Configure specific task settings for ESC
    watchdog_task_config_t main_loop_config = watchdog_get_default_task_config(WATCHDOG_TASK_MAIN_LOOP);
    main_loop_config.timeout_ms = 15;  // 15ms timeout for 100Hz main loop
    main_loop_config.reset_strategy = RESET_STRATEGY_SAFE_STATE;
    watchdog_configure_task((watchdog_supervision_t*)esc->watchdog_system, WATCHDOG_TASK_MAIN_LOOP, &main_loop_config);
    
    watchdog_task_config_t sensor_config = watchdog_get_default_task_config(WATCHDOG_TASK_SENSOR_ACQUISITION);
    sensor_config.timeout_ms = 12;  // 12ms timeout for sensor acquisition
    sensor_config.reset_strategy = RESET_STRATEGY_MODULE_RESTART;
    watchdog_configure_task((watchdog_supervision_t*)esc->watchdog_system, WATCHDOG_TASK_SENSOR_ACQUISITION, &sensor_config);
    
    watchdog_task_config_t control_config = watchdog_get_default_task_config(WATCHDOG_TASK_CONTROL_ALGORITHM);
    control_config.timeout_ms = 8;   // 8ms timeout for control algorithm
    control_config.reset_strategy = RESET_STRATEGY_SAFE_STATE;
    watchdog_configure_task((watchdog_supervision_t*)esc->watchdog_system, WATCHDOG_TASK_CONTROL_ALGORITHM, &control_config);
    
    watchdog_task_config_t brake_config = watchdog_get_default_task_config(WATCHDOG_TASK_BRAKE_ACTUATION);
    brake_config.timeout_ms = 20;    // 20ms timeout for brake actuation
    brake_config.reset_strategy = RESET_STRATEGY_MODULE_RESTART;
    watchdog_configure_task((watchdog_supervision_t*)esc->watchdog_system, WATCHDOG_TASK_BRAKE_ACTUATION, &brake_config);
    
    watchdog_task_config_t engine_config = watchdog_get_default_task_config(WATCHDOG_TASK_ENGINE_INTERFACE);
    engine_config.timeout_ms = 50;   // 50ms timeout for engine interface
    engine_config.reset_strategy = RESET_STRATEGY_SOFT_RESET;
    engine_config.critical_task = false;
    watchdog_configure_task((watchdog_supervision_t*)esc->watchdog_system, WATCHDOG_TASK_ENGINE_INTERFACE, &engine_config);
    
    // Configure safe state for different levels
    safe_state_config_t safe_state_level2 = watchdog_get_default_safe_state_config(SAFE_STATE_LEVEL_2);
    safe_state_level2.disable_engine_torque = true;
    safe_state_level2.enable_emergency_brake = true;
    safe_state_level2.emergency_brake_pressure = 40.0f;
    safe_state_level2.enable_limp_mode = true;
    watchdog_configure_safe_state((watchdog_supervision_t*)esc->watchdog_system, &safe_state_level2);
    
    printf("Watchdog supervision system configured successfully\n");
    return true;
}

void esc_update_watchdog_monitoring(esc_system_t* esc) {
    if (!esc || !esc->watchdog_system) {
        return;
    }
    
    watchdog_supervision_t* watchdog = (watchdog_supervision_t*)esc->watchdog_system;
    
    // Check for watchdog system faults
    watchdog_status_t system_status = watchdog_get_system_status(watchdog);
    
    if (system_status == WATCHDOG_STATUS_CRITICAL_FAULT) {
        esc->status = ESC_STATUS_FAULT;
        esc->diagnostics.system_fault = true;
        printf("ESC: Critical watchdog fault detected\n");
    } else if (system_status == WATCHDOG_STATUS_TIMEOUT) {
        esc->status = ESC_STATUS_FAULT;
        printf("ESC: Watchdog timeout detected\n");
    }
    
    // Check individual task timeouts and update ESC diagnostics
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        if (watchdog_is_task_timeout(watchdog, (watchdog_task_id_t)i)) {
            printf("ESC: Task %s timeout detected\n", watchdog_task_name((watchdog_task_id_t)i));
            
            // Map watchdog task timeouts to ESC fault flags
            switch (i) {
                case WATCHDOG_TASK_SENSOR_ACQUISITION:
                    for (int j = 0; j < 4; j++) {
                        esc->diagnostics.sensor_fault[j] = true;
                    }
                    break;
                case WATCHDOG_TASK_BRAKE_ACTUATION:
                    for (int j = 0; j < MAX_WHEEL_COUNT; j++) {
                        esc->diagnostics.actuator_fault[j] = true;
                    }
                    break;
                default:
                    esc->diagnostics.system_fault = true;
                    break;
            }
        }
    }
}

bool esc_handle_watchdog_safe_state(esc_system_t* esc) {
    if (!esc || !esc->watchdog_system) {
        return true; // Continue normal operation if no watchdog
    }
    
    watchdog_supervision_t* watchdog = (watchdog_supervision_t*)esc->watchdog_system;
    
    // Check if system is in safe state
    if (watchdog->safe_state_active) {
        printf("ESC operating in safe state: %s\n", 
               safe_state_level_to_string(watchdog->current_safe_state));
        
        // Apply safe state restrictions based on level
        switch (watchdog->current_safe_state) {
            case SAFE_STATE_LEVEL_1:
                // Minimal restrictions - allow basic operation
                return true;
                
            case SAFE_STATE_LEVEL_2:
                // Reduced functionality - disable engine torque reduction
                esc->actuators.engine_torque_reduction = 0.0f;
                
                // Apply emergency brake if configured
                if (watchdog->safe_state_config.enable_emergency_brake) {
                    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
                        esc->actuators.brake_pressure[i] = 
                            fmaxf(esc->actuators.brake_pressure[i], 
                                  watchdog->safe_state_config.emergency_brake_pressure);
                    }
                }
                return true;
                
            case SAFE_STATE_LEVEL_3:
                // Diagnostic mode - very limited operation
                esc->status = ESC_STATUS_FAULT;
                esc->actuators.engine_torque_reduction = 0.0f;
                
                // Apply strong emergency braking
                for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
                    esc->actuators.brake_pressure[i] = 60.0f; // 60 bar emergency brake
                }
                return false; // Skip normal control loop
                
            case SAFE_STATE_LEVEL_4:
                // Shutdown preparation - emergency braking only
                esc->status = ESC_STATUS_FAULT;
                esc->actuators.engine_torque_reduction = 0.0f;
                
                // Maximum emergency braking
                for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
                    esc->actuators.brake_pressure[i] = 100.0f; // 100 bar maximum brake
                }
                
                // Execute emergency brake commands only
                WATCHDOG_TASK_START(esc->watchdog_system, WATCHDOG_TASK_BRAKE_ACTUATION);
                esc_execute_brake_commands(esc);
                WATCHDOG_TASK_END(esc->watchdog_system, WATCHDOG_TASK_BRAKE_ACTUATION);
                
                return false; // Skip normal control loop
        }
    }
    
    return true; // Continue normal operation
}

// DTC Management Integration Functions
bool esc_configure_dtc_system(esc_system_t* esc) {
    if (!esc) return false;
    
    // Allocate DTC management system
    esc->dtc_manager = (struct dtc_manager_t*)malloc(sizeof(dtc_manager_t));
    if (!esc->dtc_manager) {
        printf("Failed to allocate DTC management system\n");
        return false;
    }
    
    // Initialize DTC management system
    if (!dtc_manager_init((dtc_manager_t*)esc->dtc_manager)) {
        printf("Failed to initialize DTC management system\n");
        free(esc->dtc_manager);
        esc->dtc_manager = NULL;
        return false;
    }
    
    // Start diagnostic session for ESC ECU
    dtc_start_diagnostic_session((dtc_manager_t*)esc->dtc_manager, 0x01); // Default session
    
    printf("DTC management system configured successfully\n");
    return true;
}

void esc_update_dtc_monitoring(esc_system_t* esc) {
    if (!esc || !esc->dtc_manager) {
        return;
    }
    
    dtc_manager_t* dtc_manager = (dtc_manager_t*)esc->dtc_manager;
    
    // Monitor sensor faults and generate DTCs
    for (int i = 0; i < 8; i++) {
        if (i < 4) {
            // Wheel speed sensors
            dtc_process_esc_sensor_fault(dtc_manager, i, esc->diagnostics.sensor_fault[i]);
        } else {
            // Other sensors (steering, yaw, accel)
            dtc_process_esc_sensor_fault(dtc_manager, i, esc->diagnostics.sensor_fault[i]);
        }
    }
    
    // Monitor actuator faults
    for (int i = 0; i < 4; i++) {
        dtc_process_esc_actuator_fault(dtc_manager, i, esc->diagnostics.actuator_fault[i]);
    }
    
    // Monitor system-level faults
    if (esc->diagnostics.system_fault) {
        dtc_process_esc_control_fault(dtc_manager, 0, true); // Control module fault
    }
    
    if (esc->diagnostics.calibration_required) {
        dtc_process_esc_control_fault(dtc_manager, 2, true); // Calibration fault
    }
    
    // Monitor watchdog faults
    if (esc->watchdog_system) {
        watchdog_supervision_t* watchdog = (watchdog_supervision_t*)esc->watchdog_system;
        if (watchdog->system_status == WATCHDOG_STATUS_CRITICAL_FAULT) {
            dtc_process_esc_control_fault(dtc_manager, 4, true); // Watchdog fault
        }
    }
    
    // Monitor communication faults
    if (esc->engine_system) {
        engine_torque_interface_t* engine = (engine_torque_interface_t*)esc->engine_system;
        engine_request_status_t engine_status = engine_get_request_status(engine);
        if (engine_status == ENGINE_REQUEST_ECU_FAULT || engine_status == ENGINE_REQUEST_TIMEOUT) {
            dtc_process_esc_communication_fault(dtc_manager, 1, true); // Engine ECU comm fault
        }
    }
    
    // Update MIL status
    dtc_update_mil_status(dtc_manager);
    
    // Capture freeze frame data for new confirmed DTCs
    static uint16_t last_confirmed_count = 0;
    if (dtc_manager->confirmed_dtc_count > last_confirmed_count) {
        // Find newly confirmed DTCs and capture freeze frame
        for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
            if (dtc_manager->stored_dtcs[i].dtc_code != 0 && 
                dtc_manager->stored_dtcs[i].is_confirmed && 
                !dtc_manager->stored_dtcs[i].has_freeze_frame) {
                esc_capture_freeze_frame_data(esc, dtc_manager->stored_dtcs[i].dtc_code);
            }
        }
        last_confirmed_count = dtc_manager->confirmed_dtc_count;
    }
}

bool esc_process_uds_request(esc_system_t* esc, const uint8_t* request, uint16_t request_length,
                            uint8_t* response, uint16_t* response_length) {
    if (!esc || !esc->dtc_manager || !request || !response || !response_length) {
        return false;
    }
    
    dtc_manager_t* dtc_manager = (dtc_manager_t*)esc->dtc_manager;
    
    // Parse UDS request
    if (request_length < 1) {
        return false;
    }
    
    uds_request_t uds_request = {0};
    uds_response_t uds_response = {0};
    
    uds_request.service_id = request[0];
    
    // Parse based on service
    switch (uds_request.service_id) {
        case UDS_SERVICE_READ_DTC_INFORMATION:
            if (request_length >= 2) {
                uds_request.sub_function = request[1];
                if (request_length >= 3) {
                    uds_request.status_mask = request[2];
                }
                if (request_length >= 6) {
                    uds_request.dtc_mask = (request[2] << 16) | (request[3] << 8) | request[4];
                    uds_request.record_number = request[5];
                }
            }
            break;
            
        case UDS_SERVICE_CLEAR_DTC_INFORMATION:
            if (request_length >= 4) {
                uds_request.request_length = request_length - 1;
                memcpy(uds_request.request_data, &request[1], uds_request.request_length);
            }
            break;
            
        default:
            // Unknown service
            break;
    }
    
    // Process the request
    bool success = uds_process_diagnostic_request(dtc_manager, &uds_request, &uds_response);
    
    // Format response
    if (success && uds_response.is_positive_response) {
        response[0] = uds_response.service_id;
        if (uds_response.response_length > 0) {
            memcpy(&response[1], uds_response.response_data, uds_response.response_length);
            *response_length = 1 + uds_response.response_length;
        } else {
            *response_length = 1;
        }
    } else {
        // Negative response
        response[0] = 0x7F;
        response[1] = uds_request.service_id;
        response[2] = uds_response.response_code;
        *response_length = 3;
    }
    
    printf("UDS Request processed: Service 0x%02X, Response length: %u\n", 
           uds_request.service_id, *response_length);
    
    return success;
}

void esc_capture_freeze_frame_data(esc_system_t* esc, uint32_t dtc_code) {
    if (!esc || !esc->dtc_manager) {
        return;
    }
    
    // Prepare freeze frame data with current system state
    uint8_t freeze_frame_data[128];
    uint16_t data_index = 0;
    
    // Parameter ID 0x01: Vehicle Speed (km/h)
    freeze_frame_data[data_index++] = 0x01;
    freeze_frame_data[data_index++] = 2; // Data length
    uint16_t vehicle_speed_kmh = (uint16_t)(esc->sensors.vehicle_speed * 3.6f);
    freeze_frame_data[data_index++] = (vehicle_speed_kmh >> 8) & 0xFF;
    freeze_frame_data[data_index++] = vehicle_speed_kmh & 0xFF;
    
    // Parameter ID 0x02: Engine Load (%)
    freeze_frame_data[data_index++] = 0x02;
    freeze_frame_data[data_index++] = 1; // Data length
    freeze_frame_data[data_index++] = 50; // Simulated 50% load
    
    // Parameter ID 0x03: Fuel System Status
    freeze_frame_data[data_index++] = 0x03;
    freeze_frame_data[data_index++] = 1; // Data length
    freeze_frame_data[data_index++] = 0x02; // Closed loop
    
    // Parameter ID 0x04: Calculated Engine Load (%)
    freeze_frame_data[data_index++] = 0x04;
    freeze_frame_data[data_index++] = 1; // Data length
    freeze_frame_data[data_index++] = 45; // Simulated 45% load
    
    // Parameter ID 0x05: Engine Coolant Temperature (°C)
    freeze_frame_data[data_index++] = 0x05;
    freeze_frame_data[data_index++] = 1; // Data length
    freeze_frame_data[data_index++] = 85; // 85°C
    
    // ESC-specific parameters
    // Parameter ID 0x80: Yaw Rate (0.01 deg/s)
    freeze_frame_data[data_index++] = 0x80;
    freeze_frame_data[data_index++] = 2; // Data length
    int16_t yaw_rate_scaled = (int16_t)(esc->sensors.yaw_rate * 100.0f * 180.0f / 3.14159f);
    freeze_frame_data[data_index++] = (yaw_rate_scaled >> 8) & 0xFF;
    freeze_frame_data[data_index++] = yaw_rate_scaled & 0xFF;
    
    // Parameter ID 0x81: Lateral Acceleration (0.01 m/s²)
    freeze_frame_data[data_index++] = 0x81;
    freeze_frame_data[data_index++] = 2; // Data length
    int16_t lat_accel_scaled = (int16_t)(esc->sensors.lateral_acceleration * 100.0f);
    freeze_frame_data[data_index++] = (lat_accel_scaled >> 8) & 0xFF;
    freeze_frame_data[data_index++] = lat_accel_scaled & 0xFF;
    
    // Parameter ID 0x82: Steering Angle (0.1 degrees)
    freeze_frame_data[data_index++] = 0x82;
    freeze_frame_data[data_index++] = 2; // Data length
    int16_t steering_angle_scaled = (int16_t)(esc->sensors.steering_angle * 10.0f * 180.0f / 3.14159f);
    freeze_frame_data[data_index++] = (steering_angle_scaled >> 8) & 0xFF;
    freeze_frame_data[data_index++] = steering_angle_scaled & 0xFF;
    
    // Parameter ID 0x83: Wheel Speeds (0.1 km/h each wheel)
    freeze_frame_data[data_index++] = 0x83;
    freeze_frame_data[data_index++] = 8; // Data length (4 wheels * 2 bytes each)
    for (int i = 0; i < 4; i++) {
        uint16_t wheel_speed_scaled = (uint16_t)(esc->sensors.wheel_speed[i] * 36.0f); // Convert rad/s to 0.1 km/h
        freeze_frame_data[data_index++] = (wheel_speed_scaled >> 8) & 0xFF;
        freeze_frame_data[data_index++] = wheel_speed_scaled & 0xFF;
    }
    
    // Parameter ID 0x84: Brake Pressures (0.1 bar each wheel)
    freeze_frame_data[data_index++] = 0x84;
    freeze_frame_data[data_index++] = 8; // Data length (4 wheels * 2 bytes each)
    for (int i = 0; i < 4; i++) {
        uint16_t brake_pressure_scaled = (uint16_t)(esc->actuators.brake_pressure[i] * 10.0f);
        freeze_frame_data[data_index++] = (brake_pressure_scaled >> 8) & 0xFF;
        freeze_frame_data[data_index++] = brake_pressure_scaled & 0xFF;
    }
    
    // Parameter ID 0x85: ESC Status
    freeze_frame_data[data_index++] = 0x85;
    freeze_frame_data[data_index++] = 1; // Data length
    freeze_frame_data[data_index++] = (uint8_t)esc->status;
    
    // Capture the freeze frame
    dtc_capture_freeze_frame((dtc_manager_t*)esc->dtc_manager, dtc_code, freeze_frame_data, data_index);
    
    printf("Freeze frame captured for DTC %s (%u bytes)\n", dtc_code_to_string(dtc_code), data_index);
}

// POST System Integration Functions
bool esc_configure_post_system(esc_system_t* esc) {
    if (!esc) {
        return false;
    }
    
    // Allocate POST system
    esc->post_system = (struct post_system_t*)malloc(sizeof(post_system_t));
    if (!esc->post_system) {
        printf("ESC: Failed to allocate POST system\n");
        return false;
    }
    
    // Create hardware interface for POST system
    static post_hardware_interface_t hw_interface = {
        .hw_memory_test = esc_post_hw_memory_test,
        .hw_calculate_checksum = esc_post_hw_calculate_checksum,
        .hw_sensor_power_on = esc_post_hw_sensor_power_on,
        .hw_sensor_power_off = esc_post_hw_sensor_power_off,
        .hw_sensor_read_value = esc_post_hw_sensor_read_value,
        .hw_sensor_self_test = esc_post_hw_sensor_self_test,
        .hw_communication_test = esc_post_hw_communication_test,
        .hw_get_timestamp = esc_post_hw_get_timestamp,
        .hw_delay_ms = esc_post_hw_delay_ms,
        .hw_system_reset = esc_post_hw_system_reset
    };
    
    // Initialize POST system
    if (!post_system_init((post_system_t*)esc->post_system, &hw_interface)) {
        printf("ESC: Failed to initialize POST system\n");
        free(esc->post_system);
        esc->post_system = NULL;
        return false;
    }
    
    printf("ESC: POST system configured successfully\n");
    return true;
}

bool esc_execute_post_sequence(esc_system_t* esc) {
    if (!esc || !esc->post_system) {
        return false;
    }
    
    printf("ESC: Executing Power-On Self-Test sequence...\n");
    
    // Execute complete POST sequence
    bool post_success = post_execute_all_tests((post_system_t*)esc->post_system);
    
    if (post_success) {
        printf("ESC: POST sequence completed successfully\n");
        
        // Print POST results
        post_print_system_status((post_system_t*)esc->post_system);
        
        return true;
    } else {
        printf("ESC: POST sequence failed\n");
        
        // Print error details
        post_print_system_status((post_system_t*)esc->post_system);
        post_print_error_log((post_system_t*)esc->post_system);
        
        return false;
    }
}

bool esc_check_system_readiness(esc_system_t* esc) {
    if (!esc || !esc->post_system) {
        return false;
    }
    
    post_system_t* post = (post_system_t*)esc->post_system;
    
    // Check POST completion
    if (!post_is_initialization_complete(post)) {
        printf("ESC: POST initialization not complete\n");
        return false;
    }
    
    // Check ESC enable conditions
    if (!post_is_esc_system_enabled(post)) {
        printf("ESC: POST did not enable ESC system\n");
        return false;
    }
    
    // Check for critical failures
    if (post_has_critical_failure(post)) {
        printf("ESC: Critical failures detected during POST\n");
        return false;
    }
    
    printf("ESC: System readiness check passed\n");
    return true;
}

void esc_handle_post_failure(esc_system_t* esc, const char* failure_reason) {
    if (!esc) {
        return;
    }
    
    printf("ESC: POST failure detected - %s\n", failure_reason ? failure_reason : "Unknown failure");
    
    // Disable ESC system
    if (esc->post_system) {
        post_disable_esc_system((post_system_t*)esc->post_system, failure_reason);
    }
    
    // Set system to fault state
    esc->status = ESC_STATUS_FAULT;
    esc->diagnostics.system_fault = true;
    esc->system_ready = false;
    
    // Log DTC if DTC system is available
    if (esc->dtc_manager) {
        dtc_set_code((dtc_manager_t*)esc->dtc_manager, DTC_ESC_CONTROL_MODULE_FAULT, 
                    "POST system failure");
    }
    
    printf("ESC: System placed in safe state due to POST failure\n");
}

bool esc_validate_initialization_complete(esc_system_t* esc) {
    if (!esc) {
        return false;
    }
    
    // Check that all critical subsystems are initialized
    if (!esc->post_system) {
        printf("ESC: POST system not initialized\n");
        return false;
    }
    
    if (!esc->plausibility) {
        printf("ESC: Plausibility system not initialized\n");
        return false;
    }
    
    if (!esc->brake_system) {
        printf("ESC: Brake system not initialized\n");
        return false;
    }
    
    if (!esc->engine_system) {
        printf("ESC: Engine system not initialized\n");
        return false;
    }
    
    if (!esc->watchdog_system) {
        printf("ESC: Watchdog system not initialized\n");
        return false;
    }
    
    if (!esc->dtc_manager) {
        printf("ESC: DTC manager not initialized\n");
        return false;
    }
    
    if (!esc->logging_system) {
        printf("ESC: Logging system not initialized\n");
        return false;
    }
    
    // Verify POST system status
    if (!post_is_initialization_complete((post_system_t*)esc->post_system)) {
        printf("ESC: POST system initialization incomplete\n");
        return false;
    }
    
    printf("ESC: All subsystems initialized successfully\n");
    return true;
}

// POST Hardware Interface Implementations
static bool esc_post_hw_memory_test(void* address, uint32_t size) {
    // Implement actual memory test
    // For demonstration, return true
    return true;
}

static uint32_t esc_post_hw_calculate_checksum(void* address, uint32_t size) {
    // Implement actual checksum calculation
    // For demonstration, return a fixed value
    return 0x12345678;
}

static bool esc_post_hw_sensor_power_on(esc_sensor_id_t sensor_id) {
    // Implement actual sensor power control
    // For demonstration, return true
    return true;
}

static bool esc_post_hw_sensor_power_off(esc_sensor_id_t sensor_id) {
    // Implement actual sensor power control
    // For demonstration, return true
    return true;
}

static float esc_post_hw_sensor_read_value(esc_sensor_id_t sensor_id) {
    // Implement actual sensor reading
    // Return realistic values for demonstration
    switch (sensor_id) {
        case ESC_SENSOR_SUPPLY_VOLTAGE: return 12.3f;
        case ESC_SENSOR_YAW_RATE: return 0.5f;
        case ESC_SENSOR_LATERAL_ACCEL: return 0.1f;
        case ESC_SENSOR_STEERING_ANGLE: return 2.0f;
        default: return 0.0f;
    }
}

static bool esc_post_hw_sensor_self_test(esc_sensor_id_t sensor_id) {
    // Implement actual sensor self-test
    // For demonstration, return true
    return true;
}

static bool esc_post_hw_communication_test(uint8_t interface_id) {
    // Implement actual communication test
    // For demonstration, return true
    return true;
}

static uint32_t esc_post_hw_get_timestamp(void) {
    // Implement actual timestamp
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void esc_post_hw_delay_ms(uint32_t milliseconds) {
    // Implement actual delay
    usleep(milliseconds * 1000);
}

static void esc_post_hw_system_reset(void) {
    // Implement actual system reset
    printf("ESC: SYSTEM RESET TRIGGERED\n");
    exit(1); // For demonstration
}

// Logging System Integration Functions
bool esc_configure_logging_system(esc_system_t* esc) {
    if (!esc) {
        return false;
    }
    
    // Allocate logging system
    esc->logging_system = (struct esc_logging_system_t*)malloc(sizeof(esc_logging_system_t));
    if (!esc->logging_system) {
        printf("ESC: Failed to allocate logging system\n");
        return false;
    }
    
    // Configure logging system for ESC operation
    logging_config_t config = logging_get_default_config();
    config.min_level = LOG_LEVEL_INFO;  // Log INFO and above
    config.enabled_categories = LOG_CATEGORY_ALL;  // Enable all categories
    config.enable_persistent_storage = true;
    config.storage_type = STORAGE_TYPE_EEPROM;
    
    // Initialize logging system
    if (!logging_init((esc_logging_system_t*)esc->logging_system, &config)) {
        printf("ESC: Failed to initialize logging system\n");
        free(esc->logging_system);
        esc->logging_system = NULL;
        return false;
    }
    
    // Set up storage interface
    storage_interface_t* storage = logging_get_eeprom_storage_interface();
    logging_set_storage_interface((esc_logging_system_t*)esc->logging_system, storage);
    
    // Log successful initialization
    LOG_INFO((esc_logging_system_t*)esc->logging_system, LOG_CATEGORY_SYSTEM, ESC_MODULE_MAIN, 0x0001,
             "ESC Logging System initialized successfully");
    
    printf("ESC: Logging system configured successfully\n");
    return true;
}

void esc_log_intervention(esc_system_t* esc, const char* intervention_type) {
    if (!esc || !esc->logging_system || !intervention_type) {
        return;
    }
    
    esc_logging_system_t* logging = (esc_logging_system_t*)esc->logging_system;
    
    // Create intervention log entry
    esc_intervention_log_t intervention = {0};
    intervention.timestamp_ms = get_microsecond_timestamp() / 1000;
    intervention.vehicle_speed = esc->sensors.vehicle_speed;
    intervention.yaw_rate = esc->sensors.yaw_rate;
    intervention.lateral_acceleration = esc->sensors.lateral_acceleration;
    intervention.steering_angle = esc->sensors.steering_angle;
    intervention.engine_torque_reduction = esc->actuators.engine_torque_reduction;
    intervention.sequence_id = logging->sequence_counter++;
    
    // Copy brake pressures
    for (int i = 0; i < 4; i++) {
        intervention.brake_pressures[i] = esc->actuators.brake_pressure[i];
    }
    
    // Determine intervention type
    if (strstr(intervention_type, "understeer")) {
        intervention.type = ESC_INTERVENTION_UNDERSTEER_CORRECTION;
    } else if (strstr(intervention_type, "oversteer")) {
        intervention.type = ESC_INTERVENTION_OVERSTEER_CORRECTION;
    } else if (strstr(intervention_type, "rollover")) {
        intervention.type = ESC_INTERVENTION_ROLLOVER_PREVENTION;
    } else if (strstr(intervention_type, "brake")) {
        intervention.type = ESC_INTERVENTION_BRAKE_ASSIST;
    } else if (strstr(intervention_type, "engine")) {
        intervention.type = ESC_INTERVENTION_ENGINE_TORQUE_REDUCTION;
    } else {
        intervention.type = ESC_INTERVENTION_NONE;
    }
    
    intervention.duration_ms = 100; // Default duration
    
    // Log the intervention
    logging_log_intervention(logging, &intervention);
    
    // Also log as regular entry
    LOG_INTERVENTION(logging, LOG_CATEGORY_INTERVENTION, ESC_MODULE_CONTROL, 0x1000,
                     "ESC Intervention: %s at %.1f km/h", intervention_type,
                     esc->sensors.vehicle_speed * 3.6f);
}

void esc_log_system_anomaly(esc_system_t* esc, const char* anomaly_description) {
    if (!esc || !esc->logging_system || !anomaly_description) {
        return;
    }
    
    esc_logging_system_t* logging = (esc_logging_system_t*)esc->logging_system;
    
    // Determine anomaly type from description
    system_anomaly_type_t anomaly_type = ANOMALY_UNEXPECTED_BEHAVIOR;
    uint8_t affected_module = ESC_MODULE_MAIN;
    
    if (strstr(anomaly_description, "sensor")) {
        anomaly_type = ANOMALY_SENSOR_DRIFT;
        affected_module = ESC_MODULE_SENSOR;
    } else if (strstr(anomaly_description, "actuator")) {
        anomaly_type = ANOMALY_ACTUATOR_DELAY;
        affected_module = ESC_MODULE_ACTUATOR;
    } else if (strstr(anomaly_description, "communication") || strstr(anomaly_description, "timeout")) {
        anomaly_type = ANOMALY_COMMUNICATION_TIMEOUT;
        affected_module = ESC_MODULE_CAN;
    } else if (strstr(anomaly_description, "performance")) {
        anomaly_type = ANOMALY_PERFORMANCE_DEGRADATION;
        affected_module = ESC_MODULE_CONTROL;
    } else if (strstr(anomaly_description, "timing")) {
        anomaly_type = ANOMALY_TIMING_VIOLATION;
        affected_module = ESC_MODULE_WATCHDOG;
    }
    
    // Use the auto-detection function
    logging_detect_and_log_anomaly(logging, anomaly_type, affected_module,
                                  0.0f, 0.0f, anomaly_description);
    
    // Also log as regular entry
    LOG_ANOMALY(logging, LOG_CATEGORY_ANOMALY, affected_module, 0x2000,
                "System Anomaly: %s", anomaly_description);
}

void esc_update_logging(esc_system_t* esc) {
    if (!esc || !esc->logging_system) {
        return;
    }
    
    esc_logging_system_t* logging = (esc_logging_system_t*)esc->logging_system;
    
    // Check for conditions that warrant logging
    
    // Log ESC status changes
    static esc_status_t last_status = ESC_STATUS_INACTIVE;
    if (esc->status != last_status) {
        const char* status_names[] = {"INACTIVE", "STANDBY", "ACTIVE", "FAULT"};
        LOG_INFO(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_MAIN, 0x0010,
                 "ESC status changed: %s -> %s", 
                 status_names[last_status], status_names[esc->status]);
        last_status = esc->status;
    }
    
    // Log active ESC interventions
    if (esc->status == ESC_STATUS_ACTIVE) {
        if (esc->dynamics.understeer_detected) {
            esc_log_intervention(esc, "understeer correction");
        }
        if (esc->dynamics.oversteer_detected) {
            esc_log_intervention(esc, "oversteer correction");
        }
        if (esc->dynamics.rollover_risk) {
            esc_log_intervention(esc, "rollover prevention");
        }
    }
    
    // Log sensor anomalies
    for (int i = 0; i < 8; i++) {
        if (esc->diagnostics.sensor_fault[i]) {
            static bool sensor_fault_logged[8] = {false};
            if (!sensor_fault_logged[i]) {
                char anomaly_desc[128];
                snprintf(anomaly_desc, sizeof(anomaly_desc), 
                         "Sensor fault detected on sensor %d", i);
                esc_log_system_anomaly(esc, anomaly_desc);
                sensor_fault_logged[i] = true;
            }
        }
    }
    
    // Log actuator anomalies
    for (int i = 0; i < 4; i++) {
        if (esc->diagnostics.actuator_fault[i]) {
            static bool actuator_fault_logged[4] = {false};
            if (!actuator_fault_logged[i]) {
                char anomaly_desc[128];
                snprintf(anomaly_desc, sizeof(anomaly_desc),
                         "Actuator fault detected on wheel %d", i);
                esc_log_system_anomaly(esc, anomaly_desc);
                actuator_fault_logged[i] = true;
            }
        }
    }
    
    // Log system faults
    if (esc->diagnostics.system_fault) {
        static bool system_fault_logged = false;
        if (!system_fault_logged) {
            esc_log_system_anomaly(esc, "Critical system fault detected");
            system_fault_logged = true;
        }
    }
    
    // Log performance metrics periodically
    static uint32_t last_perf_log_time = 0;
    uint32_t current_time = get_microsecond_timestamp() / 1000;
    if (current_time - last_perf_log_time > 10000) { // Every 10 seconds
        LOG_DEBUG(logging, LOG_CATEGORY_PERFORMANCE, ESC_MODULE_MAIN, 0x0020,
                  "Performance: Cycle %u, Vehicle speed %.1f km/h, Yaw rate %.2f deg/s",
                  esc->cycle_count, esc->sensors.vehicle_speed * 3.6f,
                  esc->sensors.yaw_rate * 180.0f / 3.14159f);
        last_perf_log_time = current_time;
    }
}