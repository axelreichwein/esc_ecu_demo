#ifndef ESC_CONTROL_ALGORITHM_H
#define ESC_CONTROL_ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_interface.h"
#include "esc_types.h"

// ESC Control States
typedef enum {
    ESC_STATE_INACTIVE = 0,          // ESC system disabled
    ESC_STATE_STANDBY = 1,           // Monitoring, no intervention
    ESC_STATE_UNDERSTEER = 2,        // Active understeer correction
    ESC_STATE_OVERSTEER = 3,         // Active oversteer correction
    ESC_STATE_ROLLOVER_PREVENTION = 4, // Rollover prevention active
    ESC_STATE_BRAKE_ASSIST = 5,      // Enhanced braking assistance
    ESC_STATE_FAULT = 6              // System fault, limited operation
} esc_control_state_t;

// Vehicle dynamics conditions
typedef enum {
    VEHICLE_STABLE = 0,
    VEHICLE_UNDERSTEER_MILD = 1,
    VEHICLE_UNDERSTEER_SEVERE = 2,
    VEHICLE_OVERSTEER_MILD = 3,
    VEHICLE_OVERSTEER_SEVERE = 4,
    VEHICLE_ROLLOVER_RISK = 5,
    VEHICLE_EXCESSIVE_SLIP = 6
} vehicle_condition_t;

// Control intervention types
typedef enum {
    INTERVENTION_NONE = 0,
    INTERVENTION_BRAKE_ONLY = 1,
    INTERVENTION_ENGINE_ONLY = 2,
    INTERVENTION_BRAKE_AND_ENGINE = 3,
    INTERVENTION_EMERGENCY = 4
} intervention_type_t;

// Individual wheel brake force calculation
typedef struct {
    float brake_force[4];           // Brake force for each wheel (N)
    float brake_pressure[4];        // Brake pressure for each wheel (bar)
    float yaw_moment_target;        // Target yaw moment (N⋅m)
    float yaw_moment_actual;        // Actual yaw moment from braking (N⋅m)
    bool wheel_locked[4];           // Wheel lock prevention flags
    float brake_efficiency[4];      // Brake efficiency factor per wheel
} brake_control_t;

// Engine torque control
typedef struct {
    float torque_reduction_percent; // Percentage reduction (0-100%)
    float torque_reduction_nm;      // Absolute torque reduction (N⋅m)
    float target_torque;           // Target engine torque (N⋅m)
    float ramp_rate;               // Torque change rate (N⋅m/s)
    bool torque_limiting_active;   // Torque limiting flag
    uint32_t intervention_duration_ms; // Duration of intervention
} engine_control_t;

// Vehicle dynamics state
typedef struct {
    float sideslip_angle;          // Vehicle sideslip angle (rad)
    float sideslip_rate;           // Rate of sideslip change (rad/s)
    float yaw_acceleration;        // Yaw acceleration (rad/s²)
    float lateral_velocity;        // Lateral velocity (m/s)
    float longitudinal_velocity;   // Longitudinal velocity (m/s)
    float load_transfer_front;     // Front axle load transfer (N)
    float load_transfer_rear;      // Rear axle load transfer (N)
    float tire_forces[4];          // Individual tire forces (N)
    float cornering_stiffness[4];  // Dynamic cornering stiffness per tire
} vehicle_dynamics_state_t;

// Control algorithm parameters
typedef struct {
    // Detection thresholds
    float understeer_threshold_mild;    // Mild understeer threshold (rad/s)
    float understeer_threshold_severe;  // Severe understeer threshold (rad/s)
    float oversteer_threshold_mild;     // Mild oversteer threshold (rad/s)
    float oversteer_threshold_severe;   // Severe oversteer threshold (rad/s)
    float rollover_threshold;           // Rollover threshold (m/s²)
    
    // Control gains
    float yaw_rate_gain;               // Yaw rate error gain
    float sideslip_gain;               // Sideslip angle gain
    float brake_force_gain;            // Brake force proportional gain
    float engine_torque_gain;          // Engine torque reduction gain
    
    // Control limits
    float max_brake_force;             // Maximum brake force per wheel (N)
    float max_brake_pressure;          // Maximum brake pressure (bar)
    float max_torque_reduction;        // Maximum engine torque reduction (%)
    float max_yaw_moment;              // Maximum corrective yaw moment (N⋅m)
    
    // Timing parameters
    float intervention_delay_ms;       // Delay before intervention (ms)
    float fade_out_time_ms;           // Time to fade out intervention (ms)
    float min_speed_activation;        // Minimum speed for ESC activation (m/s)
    
    // Advanced parameters
    float load_transfer_factor;        // Load transfer compensation factor
    float tire_grip_factor;           // Tire grip utilization factor
    float road_friction_estimate;      // Current road friction estimate (0-1)
} control_parameters_t;

// ESC control system state
typedef struct {
    esc_control_state_t current_state;
    esc_control_state_t previous_state;
    vehicle_condition_t vehicle_condition;
    intervention_type_t intervention_type;
    
    brake_control_t brake_control;
    engine_control_t engine_control;
    vehicle_dynamics_state_t dynamics;
    control_parameters_t parameters;
    
    // State machine timing
    uint32_t state_entry_time_ms;
    uint32_t state_duration_ms;
    uint32_t intervention_start_time_ms;
    
    // Control loop variables
    float yaw_rate_error;
    float yaw_rate_error_integral;
    float yaw_rate_error_derivative;
    float last_yaw_rate_error;
    
    float sideslip_error;
    float sideslip_error_integral;
    float sideslip_error_derivative;
    float last_sideslip_error;
    
    // Performance metrics
    uint32_t understeer_events;
    uint32_t oversteer_events;
    uint32_t rollover_events;
    float max_correction_magnitude;
    float average_intervention_duration_ms;
    
    // System health
    bool control_active;
    bool emergency_intervention;
    float system_confidence;           // System confidence level (0-1)
    uint32_t last_update_time_ms;
} esc_control_system_t;

// Function prototypes

// System initialization and configuration
bool esc_control_init(esc_control_system_t* control_system);
void esc_control_shutdown(esc_control_system_t* control_system);
bool esc_control_configure_parameters(esc_control_system_t* control_system, const control_parameters_t* params);

// Main control loop
void esc_control_update(esc_control_system_t* control_system, 
                       const esc_sensor_data_t* sensor_data,
                       const vehicle_parameters_t* vehicle_params,
                       uint32_t current_time_ms);

// State machine functions
esc_control_state_t esc_control_state_machine(esc_control_system_t* control_system,
                                              const esc_sensor_data_t* sensor_data,
                                              const vehicle_parameters_t* vehicle_params);

// Vehicle condition assessment
vehicle_condition_t esc_assess_vehicle_condition(const esc_control_system_t* control_system,
                                                const esc_sensor_data_t* sensor_data,
                                                const vehicle_parameters_t* vehicle_params);

// Control algorithms
void esc_control_understeer(esc_control_system_t* control_system,
                           const esc_sensor_data_t* sensor_data,
                           const vehicle_parameters_t* vehicle_params);

void esc_control_oversteer(esc_control_system_t* control_system,
                          const esc_sensor_data_t* sensor_data,
                          const vehicle_parameters_t* vehicle_params);

void esc_control_rollover_prevention(esc_control_system_t* control_system,
                                    const esc_sensor_data_t* sensor_data,
                                    const vehicle_parameters_t* vehicle_params);

// Brake force calculation
void esc_calculate_brake_forces(esc_control_system_t* control_system,
                               float target_yaw_moment,
                               const esc_sensor_data_t* sensor_data,
                               const vehicle_parameters_t* vehicle_params);

// Engine torque control
void esc_calculate_engine_torque_reduction(esc_control_system_t* control_system,
                                          float severity_factor,
                                          const esc_sensor_data_t* sensor_data);

// Vehicle dynamics calculations
void esc_update_vehicle_dynamics(esc_control_system_t* control_system,
                                const esc_sensor_data_t* sensor_data,
                                const vehicle_parameters_t* vehicle_params);

// Utility functions
float esc_calculate_reference_yaw_rate(float steering_angle, float vehicle_speed, float wheelbase);
float esc_calculate_yaw_moment_from_brakes(const brake_control_t* brake_control, 
                                          const vehicle_parameters_t* vehicle_params);
float esc_calculate_sideslip_angle(float lateral_velocity, float longitudinal_velocity);
float esc_estimate_road_friction(const esc_sensor_data_t* sensor_data);
float esc_calculate_load_transfer(float lateral_acceleration, float longitudinal_acceleration,
                                 const vehicle_parameters_t* vehicle_params);

// PID controller functions
float esc_pid_controller(float error, float* error_integral, float* error_derivative, 
                        float last_error, float kp, float ki, float kd, float dt);

// Default parameter configurations
control_parameters_t esc_get_default_control_parameters(void);
control_parameters_t esc_get_sport_control_parameters(void);
control_parameters_t esc_get_comfort_control_parameters(void);

// Diagnostic and monitoring functions
void esc_control_reset_statistics(esc_control_system_t* control_system);
void esc_control_print_status(const esc_control_system_t* control_system);
const char* esc_control_state_to_string(esc_control_state_t state);
const char* esc_vehicle_condition_to_string(vehicle_condition_t condition);

// Safety functions
bool esc_control_safety_check(const esc_control_system_t* control_system,
                             const esc_sensor_data_t* sensor_data);
void esc_control_emergency_intervention(esc_control_system_t* control_system);
void esc_control_fade_out_intervention(esc_control_system_t* control_system, float fade_rate);

#endif // ESC_CONTROL_ALGORITHM_H