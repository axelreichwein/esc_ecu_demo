#include "sensor_plausibility.h"
#include <stdio.h>
#include <math.h>

// Create minimal struct definitions for demo
typedef struct {
    float wheel_speed[4];
    float steering_angle;
    float yaw_rate;
    float lateral_acceleration;
    float longitudinal_acceleration;
    float vehicle_speed;
    bool brake_pedal_pressed;
    bool accelerator_pedal_pressed;
    uint32_t timestamp_ms;
} esc_sensor_data_t;

typedef struct {
    float wheelbase;
    float track_width;
    float mass;
    float inertia_z;
    float cg_height;
    float cornering_stiffness_front;
    float cornering_stiffness_rear;
    float wheel_radius;
} vehicle_parameters_t;

void demo_plausibility_checks(void) {
    printf("Electronic Stability Control - Sensor Plausibility Check Demo\n");
    printf("=============================================================\n\n");
    
    // Initialize plausibility system
    plausibility_system_t plausibility_system;
    vehicle_parameters_t vehicle_params = {
        .wheelbase = 2.7f,
        .track_width = 1.5f,
        .mass = 1500.0f,
        .wheel_radius = 0.32f
    };
    
    if (!plausibility_init(&plausibility_system, &vehicle_params)) {
        printf("Failed to initialize plausibility system\n");
        return;
    }
    
    // Configure thresholds
    plausibility_thresholds_t thresholds = plausibility_get_default_thresholds();
    plausibility_configure_thresholds(&plausibility_system, &thresholds);
    
    printf("Plausibility system initialized with default thresholds:\n");
    printf("- Yaw-lateral acceleration max deviation: %.1f m/s²\n", thresholds.yaw_lat_accel_max_deviation);
    printf("- Wheel speed max deviation: %.1f rad/s\n", thresholds.wheel_speed_max_deviation);
    printf("- Vehicle speed max deviation: %.1f m/s\n", thresholds.vehicle_speed_max_deviation);
    printf("- Steering-yaw max deviation: %.3f rad/s\n", thresholds.steering_yaw_max_deviation);
    printf("- Max lateral acceleration: %.1f m/s²\n", thresholds.max_lateral_acceleration);
    printf("- Signal timeout: %u ms\n\n", thresholds.signal_timeout_ms);
    
    // Test Case 1: Normal driving conditions
    printf("=== Test Case 1: Normal Driving Conditions ===\n");
    esc_sensor_data_t normal_data = {
        .wheel_speed = {62.5f, 62.5f, 62.5f, 62.5f}, // ~20 m/s at 0.32m radius
        .steering_angle = 0.1f,      // 5.7 degrees
        .yaw_rate = 0.074f,          // Expected from bicycle model
        .lateral_acceleration = 1.48f, // Expected from yaw rate and speed
        .longitudinal_acceleration = 0.5f,
        .vehicle_speed = 20.0f,
        .timestamp_ms = 1000
    };
    
    plausibility_update(&plausibility_system, &normal_data, &vehicle_params);
    plausibility_status_t status = plausibility_get_overall_status(&plausibility_system);
    printf("Overall Status: %s\n", plausibility_status_to_string(status));
    
    // Show individual check results
    for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
        plausibility_result_t result = plausibility_get_check_result(&plausibility_system, (plausibility_check_type_t)i);
        if (result.status != PLAUSIBILITY_OK) {
            printf("- %s: %s - %s\n", 
                   plausibility_check_type_to_string(result.check_type),
                   plausibility_status_to_string(result.status),
                   result.description);
        }
    }
    if (status == PLAUSIBILITY_OK) {
        printf("All plausibility checks passed ✓\n");
    }
    printf("\n");
    
    // Test Case 2: Yaw-Lateral acceleration inconsistency (oversteer)
    printf("=== Test Case 2: Oversteer Detection ===\n");
    esc_sensor_data_t oversteer_data = {
        .wheel_speed = {62.5f, 62.5f, 62.5f, 62.5f},
        .steering_angle = 0.1f,
        .yaw_rate = 0.35f,           // Much higher than expected
        .lateral_acceleration = 7.0f, // Much higher than expected
        .longitudinal_acceleration = 0.0f,
        .vehicle_speed = 20.0f,
        .timestamp_ms = 2000
    };
    
    plausibility_update(&plausibility_system, &oversteer_data, &vehicle_params);
    status = plausibility_get_overall_status(&plausibility_system);
    printf("Overall Status: %s\n", plausibility_status_to_string(status));
    
    for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
        plausibility_result_t result = plausibility_get_check_result(&plausibility_system, (plausibility_check_type_t)i);
        if (result.status >= PLAUSIBILITY_WARNING) {
            printf("- %s: %s - %s\n", 
                   plausibility_check_type_to_string(result.check_type),
                   plausibility_status_to_string(result.status),
                   result.description);
        }
    }
    printf("\n");
    
    // Test Case 3: Wheel speed inconsistency
    printf("=== Test Case 3: Wheel Speed Inconsistency ===\n");
    esc_sensor_data_t wheel_fault_data = {
        .wheel_speed = {62.5f, 62.5f, 45.0f, 62.5f}, // Rear left wheel much slower
        .steering_angle = 0.0f,
        .yaw_rate = 0.0f,
        .lateral_acceleration = 0.0f,
        .longitudinal_acceleration = 0.0f,
        .vehicle_speed = 20.0f,
        .timestamp_ms = 3000
    };
    
    plausibility_update(&plausibility_system, &wheel_fault_data, &vehicle_params);
    status = plausibility_get_overall_status(&plausibility_system);
    printf("Overall Status: %s\n", plausibility_status_to_string(status));
    
    for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
        plausibility_result_t result = plausibility_get_check_result(&plausibility_system, (plausibility_check_type_t)i);
        if (result.status >= PLAUSIBILITY_WARNING) {
            printf("- %s: %s - %s\n", 
                   plausibility_check_type_to_string(result.check_type),
                   plausibility_status_to_string(result.status),
                   result.description);
        }
    }
    printf("\n");
    
    // Test Case 4: Critical acceleration limit exceeded
    printf("=== Test Case 4: Critical Acceleration Limit ===\n");
    esc_sensor_data_t critical_data = {
        .wheel_speed = {62.5f, 62.5f, 62.5f, 62.5f},
        .steering_angle = 0.0f,
        .yaw_rate = 0.0f,
        .lateral_acceleration = 18.0f, // Exceeds 15 m/s² limit
        .longitudinal_acceleration = 0.0f,
        .vehicle_speed = 20.0f,
        .timestamp_ms = 4000
    };
    
    plausibility_update(&plausibility_system, &critical_data, &vehicle_params);
    status = plausibility_get_overall_status(&plausibility_system);
    printf("Overall Status: %s\n", plausibility_status_to_string(status));
    
    for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
        plausibility_result_t result = plausibility_get_check_result(&plausibility_system, (plausibility_check_type_t)i);
        if (result.status >= PLAUSIBILITY_WARNING) {
            printf("- %s: %s - %s\n", 
                   plausibility_check_type_to_string(result.check_type),
                   plausibility_status_to_string(result.status),
                   result.description);
        }
    }
    printf("\n");
    
    // Show system statistics
    printf("=== System Statistics ===\n");
    printf("Total checks performed: %u\n", plausibility_system.checks_performed);
    printf("Checks passed: %u\n", plausibility_system.checks_passed);
    printf("Checks failed: %u\n", plausibility_system.checks_failed);
    printf("Critical faults: %u\n", plausibility_system.critical_fault_count);
    
    plausibility_shutdown(&plausibility_system);
    printf("\nPlausibility checking demo completed.\n");
}

int main(void) {
    demo_plausibility_checks();
    return 0;
}