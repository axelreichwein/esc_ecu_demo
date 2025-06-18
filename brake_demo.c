#include "brake_actuation.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

// Demo hardware interface functions (same as in esc_ecu.c)
static bool demo_pwm_init(uint8_t channel, uint16_t frequency_hz) {
    printf("Demo PWM channel %d initialized at %d Hz\n", channel, frequency_hz);
    return true;
}

static bool demo_pwm_set_duty(uint8_t channel, uint16_t duty_cycle) {
    printf("PWM Ch%d: %.1f%% duty (%.1f bar equivalent)\n", 
           channel, (float)duty_cycle / 10.0f, brake_pwm_to_pressure(duty_cycle, 200.0f));
    return true;
}

static uint16_t demo_pwm_get_duty(uint8_t channel) {
    (void)channel;
    return 0;
}

static bool demo_can_init(uint32_t baudrate) {
    printf("Demo CAN interface initialized at %u bps\n", baudrate);
    return true;
}

static bool demo_can_send_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    printf("CAN TX: ID=0x%03X, Pressure=%.1f bar, Wheel=%d\n", 
           can_id, brake_can_decode_feedback_message(data), data[0]);
    return true;
}

static bool demo_can_receive_message(uint32_t* can_id, uint8_t* data, uint8_t* length) {
    // Simulate feedback messages
    static int counter = 0;
    counter++;
    
    if (counter % 10 == 0) { // Send feedback every 10th call
        *can_id = 0x300; // Feedback message ID
        *length = 8;
        // Simulate pressure feedback
        data[0] = 0; // Wheel ID
        data[1] = 0x80; // Pressure low byte
        data[2] = 0x40; // Pressure high byte
        return true;
    }
    return false;
}

static bool demo_feedback_init(uint8_t channel) {
    printf("Demo pressure feedback for channel %d initialized\n", channel);
    return true;
}

static float demo_read_pressure(uint8_t channel) {
    // Simulate realistic pressure feedback with some dynamics
    static float target_pressure[MAX_WHEEL_COUNT] = {0};
    static float actual_pressure[MAX_WHEEL_COUNT] = {0};
    static int first_call = 1;
    
    if (first_call) {
        srand((unsigned int)time(NULL));
        first_call = 0;
    }
    
    // Simulate first-order response dynamics
    float time_constant = 0.05f; // 50ms time constant
    float dt = 0.005f; // 5ms update rate
    float alpha = dt / (time_constant + dt);
    
    actual_pressure[channel] = alpha * target_pressure[channel] + (1.0f - alpha) * actual_pressure[channel];
    
    // Add realistic noise
    float noise = ((float)rand() / RAND_MAX - 0.5f) * 0.2f; // ±0.1 bar noise
    
    return actual_pressure[channel] + noise;
}

static float demo_read_temperature(uint8_t channel) {
    (void)channel;
    return 65.0f + ((float)rand() / RAND_MAX - 0.5f) * 10.0f; // 60-70°C
}

static bool demo_feedback_valid(uint8_t channel) {
    (void)channel;
    return true;
}

static const brake_hw_interface_t demo_hw_interface = {
    .pwm_init = demo_pwm_init,
    .pwm_set_duty = demo_pwm_set_duty,
    .pwm_get_duty = demo_pwm_get_duty,
    .can_init = demo_can_init,
    .can_send_message = demo_can_send_message,
    .can_receive_message = demo_can_receive_message,
    .feedback_init = demo_feedback_init,
    .read_pressure = demo_read_pressure,
    .read_temperature = demo_read_temperature,
    .feedback_valid = demo_feedback_valid
};

void demo_basic_brake_control(void) {
    printf("\n=== Basic Brake Control Demo ===\n");
    
    brake_actuation_system_t brake_system;
    
    // Initialize brake system
    if (!brake_actuation_init(&brake_system, &demo_hw_interface)) {
        printf("Failed to initialize brake system\n");
        return;
    }
    
    // Configure channels - PWM for front, CAN for rear
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_channel_config_t config;
        
        if (i < 2) {
            config = brake_get_default_pwm_config((wheel_position_t)i);
            printf("Configuring wheel %d with PWM interface\n", i);
        } else {
            config = brake_get_default_can_config((wheel_position_t)i);
            printf("Configuring wheel %d with CAN interface\n", i);
        }
        
        brake_configure_channel(&brake_system, (wheel_position_t)i, &config);
        brake_configure_pid_controller(&brake_system, (wheel_position_t)i, 20.0f, 5.0f, 0.5f);
    }
    
    // Enable closed-loop control
    brake_enable_closed_loop(&brake_system, true);
    
    // Test basic pressure commands
    printf("\nTesting basic pressure commands:\n");
    
    float test_pressures[MAX_WHEEL_COUNT] = {50.0f, 50.0f, 30.0f, 30.0f}; // Front higher than rear
    
    if (brake_set_pressure_command(&brake_system, test_pressures)) {
        printf("Pressure commands sent successfully\n");
        
        // Run control loop for a few iterations
        for (int i = 0; i < 10; i++) {
            brake_update_control_loop(&brake_system);
            usleep(5000); // 5ms delay
        }
        
        // Print feedback data
        printf("\nBrake feedback after 10 control cycles:\n");
        for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
            brake_feedback_t feedback = brake_get_feedback(&brake_system, (wheel_position_t)i);
            printf("Wheel %d: Target=%.1f bar, Actual=%.1f bar, Error=%.1f bar, Temp=%.1f°C\n",
                   i, feedback.target_pressure, feedback.actual_pressure, 
                   feedback.pressure_error, feedback.temperature);
        }
    }
    
    brake_actuation_shutdown(&brake_system);
}

void demo_emergency_brake_test(void) {
    printf("\n=== Emergency Brake Test ===\n");
    
    brake_actuation_system_t brake_system;
    
    if (!brake_actuation_init(&brake_system, &demo_hw_interface)) {
        printf("Failed to initialize brake system\n");
        return;
    }
    
    // Configure all channels with PWM for simplicity
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_channel_config_t config = brake_get_default_pwm_config((wheel_position_t)i);
        brake_configure_channel(&brake_system, (wheel_position_t)i, &config);
    }
    
    // Test emergency brake activation
    printf("Activating emergency brake...\n");
    brake_set_emergency_brake(&brake_system, true);
    
    // Run control loop to see emergency brake commands
    for (int i = 0; i < 5; i++) {
        brake_update_control_loop(&brake_system);
        usleep(10000); // 10ms delay
    }
    
    printf("Deactivating emergency brake...\n");
    brake_set_emergency_brake(&brake_system, false);
    
    // Run control loop to see brake release
    for (int i = 0; i < 5; i++) {
        brake_update_control_loop(&brake_system);
        usleep(10000); // 10ms delay
    }
    
    brake_actuation_shutdown(&brake_system);
}

void demo_closed_loop_control(void) {
    printf("\n=== Closed-Loop Control Demo ===\n");
    
    brake_actuation_system_t brake_system;
    
    if (!brake_actuation_init(&brake_system, &demo_hw_interface)) {
        printf("Failed to initialize brake system\n");
        return;
    }
    
    // Configure channels
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_channel_config_t config = brake_get_default_pwm_config((wheel_position_t)i);
        brake_configure_channel(&brake_system, (wheel_position_t)i, &config);
        
        // Configure aggressive PID for demonstration
        brake_configure_pid_controller(&brake_system, (wheel_position_t)i, 25.0f, 8.0f, 1.0f);
    }
    
    brake_enable_closed_loop(&brake_system, true);
    
    printf("Testing closed-loop pressure control with step response:\n");
    
    // Step response test
    float step_pressures[MAX_WHEEL_COUNT] = {0, 0, 0, 0};
    
    printf("Initial pressure command: 0 bar\n");
    brake_set_pressure_command(&brake_system, step_pressures);
    
    // Let system settle
    for (int i = 0; i < 20; i++) {
        brake_update_control_loop(&brake_system);
        usleep(5000);
    }
    
    printf("Step to 100 bar:\n");
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        step_pressures[i] = 100.0f;
    }
    brake_set_pressure_command(&brake_system, step_pressures);
    
    // Monitor step response
    for (int cycle = 0; cycle < 50; cycle++) {
        brake_update_control_loop(&brake_system);
        
        if (cycle % 10 == 0) {
            printf("Cycle %d feedback:\n", cycle);
            for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
                brake_feedback_t feedback = brake_get_feedback(&brake_system, (wheel_position_t)i);
                printf("  Wheel %d: %.1f bar (error: %.1f)\n", 
                       i, feedback.actual_pressure, feedback.pressure_error);
            }
        }
        
        usleep(5000); // 5ms
    }
    
    brake_actuation_shutdown(&brake_system);
}

void demo_mixed_interface_control(void) {
    printf("\n=== Mixed Interface Control Demo ===\n");
    
    brake_actuation_system_t brake_system;
    
    if (!brake_actuation_init(&brake_system, &demo_hw_interface)) {
        printf("Failed to initialize brake system\n");
        return;
    }
    
    // Configure mixed interfaces: PWM for front, CAN for rear
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        brake_channel_config_t config;
        
        if (i < 2) {
            config = brake_get_default_pwm_config((wheel_position_t)i);
            printf("Front wheel %d: PWM interface, max pressure %.0f bar\n", i, config.max_pressure);
        } else {
            config = brake_get_default_can_config((wheel_position_t)i);
            printf("Rear wheel %d: CAN interface, max pressure %.0f bar\n", i, config.max_pressure);
        }
        
        brake_configure_channel(&brake_system, (wheel_position_t)i, &config);
        brake_configure_pid_controller(&brake_system, (wheel_position_t)i, 15.0f, 3.0f, 0.2f);
    }
    
    brake_enable_closed_loop(&brake_system, true);
    
    printf("\nTesting mixed interface operation:\n");
    
    // Test different pressure patterns
    float pressures[MAX_WHEEL_COUNT];
    
    // Test 1: Equal pressure all around
    printf("Test 1: Equal pressure (80 bar) all wheels\n");
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        pressures[i] = 80.0f;
    }
    brake_set_pressure_command(&brake_system, pressures);
    
    for (int i = 0; i < 10; i++) {
        brake_update_control_loop(&brake_system);
        usleep(10000);
    }
    
    // Test 2: Front bias
    printf("Test 2: Front bias (120 bar front, 60 bar rear)\n");
    pressures[WHEEL_FL] = 120.0f;
    pressures[WHEEL_FR] = 120.0f;
    pressures[WHEEL_RL] = 60.0f;
    pressures[WHEEL_RR] = 60.0f;
    brake_set_pressure_command(&brake_system, pressures);
    
    for (int i = 0; i < 10; i++) {
        brake_update_control_loop(&brake_system);
        usleep(10000);
    }
    
    // Test 3: Individual wheel control (simulate ESC intervention)
    printf("Test 3: ESC intervention (individual wheel control)\n");
    pressures[WHEEL_FL] = 40.0f;  // Light front left
    pressures[WHEEL_FR] = 100.0f; // Heavy front right
    pressures[WHEEL_RL] = 20.0f;  // Light rear left
    pressures[WHEEL_RR] = 80.0f;  // Medium rear right
    brake_set_pressure_command(&brake_system, pressures);
    
    for (int i = 0; i < 15; i++) {
        brake_update_control_loop(&brake_system);
        
        if (i % 5 == 0) {
            printf("  Control cycle %d feedback:\n", i);
            for (int j = 0; j < MAX_WHEEL_COUNT; j++) {
                brake_feedback_t feedback = brake_get_feedback(&brake_system, (wheel_position_t)j);
                printf("    Wheel %d: Target=%.1f, Actual=%.1f, Error=%.1f\n", 
                       j, feedback.target_pressure, feedback.actual_pressure, feedback.pressure_error);
            }
        }
        
        usleep(10000);
    }
    
    brake_print_system_status(&brake_system);
    brake_actuation_shutdown(&brake_system);
}

int main(void) {
    printf("Brake Actuation Command Interface Demonstration\n");
    printf("===============================================\n");
    
    demo_basic_brake_control();
    demo_emergency_brake_test();
    demo_closed_loop_control();
    demo_mixed_interface_control();
    
    printf("\nBrake actuation demonstration completed.\n");
    return 0;
}