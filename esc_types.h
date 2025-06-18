#ifndef ESC_TYPES_H
#define ESC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_WHEEL_COUNT 4

typedef enum {
    WHEEL_FL = 0,  // Front Left
    WHEEL_FR = 1,  // Front Right
    WHEEL_RL = 2,  // Rear Left
    WHEEL_RR = 3   // Rear Right
} wheel_position_t;

typedef struct {
    float wheel_speed[MAX_WHEEL_COUNT];     // rad/s
    float steering_angle;                   // radians (positive = left turn)
    float yaw_rate;                        // rad/s (positive = counterclockwise)
    float lateral_acceleration;            // m/s^2 (positive = left)
    float longitudinal_acceleration;       // m/s^2 (positive = forward)
    float vehicle_speed;                   // m/s
    bool brake_pedal_pressed;
    bool accelerator_pedal_pressed;
    uint32_t timestamp_ms;
} esc_sensor_data_t;

typedef struct {
    float wheelbase;                       // m
    float track_width;                     // m
    float mass;                           // kg
    float inertia_z;                      // kg*m^2
    float cg_height;                      // m (center of gravity height)
    float cornering_stiffness_front;      // N/rad
    float cornering_stiffness_rear;       // N/rad
    float wheel_radius;                   // m
} vehicle_parameters_t;

#endif // ESC_TYPES_H