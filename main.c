#include "esc_ecu.h"
#include "sensor_plausibility.h"
#include "esc_control_algorithm.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <string.h>

// Global variables for main control loop
static volatile bool system_running = true;
static esc_system_t esc_system;
static vehicle_parameters_t vehicle_config;

// Signal handler for graceful shutdown
void signal_handler(int signum) {
    printf("\nReceived signal %d, shutting down ESC system...\n", signum);
    system_running = false;
}

// Initialize vehicle parameters (typically loaded from calibration data)
void init_vehicle_parameters(vehicle_parameters_t* params) {
    if (!params) return;
    
    // Example parameters for a typical passenger car
    params->wheelbase = 2.7f;                    // meters
    params->track_width = 1.5f;                  // meters
    params->mass = 1500.0f;                      // kg
    params->inertia_z = 2500.0f;                // kg*m^2
    params->cg_height = 0.5f;                   // meters
    params->cornering_stiffness_front = 60000.0f; // N/rad
    params->cornering_stiffness_rear = 55000.0f;  // N/rad
    params->wheel_radius = 0.32f;               // meters
}

// Simulate sensor data for demonstration purposes
void simulate_sensor_data(esc_system_t* esc, uint32_t cycle) {
    if (!esc) return;
    
    // Simulate various driving scenarios
    float time_sec = cycle * (ESC_SAMPLE_TIME_MS / 1000.0f);
    
    // Simulate wheel speeds (all wheels rotating at similar speed)
    float base_speed = 20.0f + 5.0f * sinf(time_sec * 0.1f); // 15-25 m/s
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        esc->sensors.wheel_speed[i] = base_speed / vehicle_config.wheel_radius;
        // Add some variation to simulate different wheel conditions
        esc->sensors.wheel_speed[i] += (i * 0.5f) * sinf(time_sec * 0.2f);
    }
    
    // Simulate steering input (sine wave to simulate lane changes)
    esc->sensors.steering_angle = 0.1f * sinf(time_sec * 0.5f); // ±0.1 radians
    
    // Simulate yaw rate (should follow steering with some dynamics)
    float reference_yaw = esc_calculate_reference_yaw_rate(
        esc->sensors.steering_angle, 
        esc->sensors.vehicle_speed, 
        vehicle_config.wheelbase);
    esc->sensors.yaw_rate = reference_yaw + 0.02f * sinf(time_sec * 2.0f);
    
    // Simulate lateral acceleration
    esc->sensors.lateral_acceleration = esc->sensors.vehicle_speed * esc->sensors.yaw_rate;
    
    // Simulate longitudinal acceleration
    esc->sensors.longitudinal_acceleration = 0.5f * sinf(time_sec * 0.3f);
    
    // Simulate emergency scenario every 30 seconds
    if (((int)time_sec % 30) < 5) {
        // Simulate oversteer condition
        esc->sensors.yaw_rate += 0.3f * sinf(time_sec * 5.0f);
        esc->sensors.lateral_acceleration += 2.0f * sinf(time_sec * 5.0f);
    }
    
    // Digital inputs
    esc->sensors.brake_pedal_pressed = (cycle % 500) < 50; // Brake for 0.5s every 5s
    esc->sensors.accelerator_pedal_pressed = !esc->sensors.brake_pedal_pressed;
}

// Print system status for monitoring
void print_system_status(const esc_system_t* esc) {
    if (!esc) return;
    
    printf("\n=== ESC System Status (Cycle: %u) ===\n", esc->cycle_count);
    
    // System status
    const char* status_str[] = {"INACTIVE", "STANDBY", "ACTIVE", "FAULT"};
    printf("Status: %s\n", status_str[esc->status]);
    
    // Sensor data
    printf("Vehicle Speed: %.2f m/s\n", esc->sensors.vehicle_speed);
    printf("Steering Angle: %.3f rad (%.1f deg)\n", 
           esc->sensors.steering_angle, esc->sensors.steering_angle * 180.0f / 3.14159f);
    printf("Yaw Rate: %.3f rad/s\n", esc->sensors.yaw_rate);
    printf("Lateral Accel: %.2f m/s²\n", esc->sensors.lateral_acceleration);
    
    // Vehicle dynamics
    printf("Reference Yaw Rate: %.3f rad/s\n", esc->dynamics.reference_yaw_rate);
    printf("Yaw Rate Error: %.3f rad/s\n", esc->dynamics.yaw_rate_error);
    printf("Understeer: %s, Oversteer: %s, Rollover Risk: %s\n",
           esc->dynamics.understeer_detected ? "YES" : "NO",
           esc->dynamics.oversteer_detected ? "YES" : "NO",
           esc->dynamics.rollover_risk ? "YES" : "NO");
    
    // Actuator commands
    if (esc->status == ESC_STATUS_ACTIVE) {
        printf("Brake Pressures [FL, FR, RL, RR]: [%.1f, %.1f, %.1f, %.1f] bar\n",
               esc->actuators.brake_pressure[0], esc->actuators.brake_pressure[1],
               esc->actuators.brake_pressure[2], esc->actuators.brake_pressure[3]);
        printf("Engine Torque Reduction: %.1f%%\n", esc->actuators.engine_torque_reduction);
    }
    
    // Plausibility status
    if (esc->plausibility) {
        plausibility_status_t plausibility_status = plausibility_get_overall_status((const plausibility_system_t*)esc->plausibility);
        if (plausibility_status != PLAUSIBILITY_OK) {
            printf("Plausibility Status: %s\n", plausibility_status_to_string(plausibility_status));
            
            // Show detailed plausibility results for major faults
            if (plausibility_status >= PLAUSIBILITY_MAJOR_FAULT) {
                printf("Active Plausibility Faults:\n");
                for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
                    plausibility_result_t result = plausibility_get_check_result((const plausibility_system_t*)esc->plausibility, (plausibility_check_type_t)i);
                    if (result.status >= PLAUSIBILITY_MAJOR_FAULT) {
                        printf("  - %s: %s\n", plausibility_check_type_to_string(result.check_type), result.description);
                    }
                }
            }
        }
    }
    
    // Warnings
    if (esc->actuators.esc_warning_light) {
        printf("*** ESC WARNING LIGHT ACTIVE ***\n");
    }
    
    if (esc->diagnostics.system_fault) {
        printf("*** SYSTEM FAULT DETECTED ***\n");
        if (esc->diagnostics.fault_code != 0) {
            printf("*** FAULT CODE: 0x%04X ***\n", esc->diagnostics.fault_code);
        }
    }
    
    printf("=====================================\n");
}

// High-precision timing function
void precise_sleep(int milliseconds) {
    struct timespec req = {0};
    req.tv_sec = milliseconds / 1000;
    req.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&req, NULL);
}

int main(int argc, char* argv[]) {
    printf("Electronic Stability Control (ESC) ECU Simulation\n");
    printf("=================================================\n");
    
    // Install signal handlers for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize vehicle parameters
    init_vehicle_parameters(&vehicle_config);
    
    // Initialize ESC system
    esc_init(&esc_system, &vehicle_config);
    
    if (esc_system.status == ESC_STATUS_FAULT) {
        printf("ESC system initialization failed!\n");
        return EXIT_FAILURE;
    }
    
    printf("ESC system initialized successfully\n");
    printf("Sample rate: %d Hz (%d ms)\n", ESC_SAMPLE_RATE_HZ, ESC_SAMPLE_TIME_MS);
    printf("Press Ctrl+C to stop the simulation\n\n");
    
    // Main control loop
    struct timespec loop_start, loop_end;
    uint32_t cycle_count = 0;
    bool verbose_output = (argc > 1 && strcmp(argv[1], "-v") == 0);
    
    while (system_running) {
        clock_gettime(CLOCK_MONOTONIC, &loop_start);
        
        // Simulate sensor data (in real system, this would be actual sensor readings)
        simulate_sensor_data(&esc_system, cycle_count);
        
        // Execute main ESC control algorithm
        esc_update(&esc_system);
        
        // Print status every 10 cycles (1 second at 10Hz) or when ESC is active
        if (verbose_output || (cycle_count % 10 == 0) || 
            (esc_system.status == ESC_STATUS_ACTIVE)) {
            print_system_status(&esc_system);
        }
        
        cycle_count++;
        
        // Calculate sleep time to maintain precise timing
        clock_gettime(CLOCK_MONOTONIC, &loop_end);
        long elapsed_ms = (loop_end.tv_sec - loop_start.tv_sec) * 1000 +
                         (loop_end.tv_nsec - loop_start.tv_nsec) / 1000000;
        
        int sleep_time = ESC_SAMPLE_TIME_MS - (int)elapsed_ms;
        if (sleep_time > 0) {
            precise_sleep(sleep_time);
        } else if (sleep_time < -5) {
            printf("Warning: Control loop overrun by %d ms\n", -sleep_time);
        }
    }
    
    printf("\nESC system shutdown complete\n");
    printf("Total cycles executed: %u\n", esc_system.cycle_count);
    
    return EXIT_SUCCESS;
}