#include "esc_control_algorithm.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ESC sensor data and vehicle parameters are now defined in esc_ecu.h

// Constants
#define PI 3.14159265359f
#define GRAVITY 9.81f
#define MAX_WHEEL_COUNT 4
#define WHEEL_FL 0
#define WHEEL_FR 1
#define WHEEL_RL 2
#define WHEEL_RR 3

// Control algorithm constants
#define STATE_TRANSITION_HYSTERESIS 0.1f
#define MIN_INTERVENTION_TIME_MS 100
#define MAX_INTERVENTION_TIME_MS 5000
#define CONTROL_LOOP_DT 0.01f  // 100Hz control loop

bool esc_control_init(esc_control_system_t* control_system) {
    if (!control_system) return false;
    
    memset(control_system, 0, sizeof(esc_control_system_t));
    
    // Initialize state machine
    control_system->current_state = ESC_STATE_STANDBY;
    control_system->previous_state = ESC_STATE_INACTIVE;
    control_system->vehicle_condition = VEHICLE_STABLE;
    control_system->intervention_type = INTERVENTION_NONE;
    
    // Initialize control parameters with defaults
    control_system->parameters = esc_get_default_control_parameters();
    
    // Initialize system health
    control_system->control_active = true;
    control_system->emergency_intervention = false;
    control_system->system_confidence = 1.0f;
    
    printf("ESC control algorithm initialized\n");
    return true;
}

void esc_control_shutdown(esc_control_system_t* control_system) {
    if (!control_system) return;
    
    memset(control_system, 0, sizeof(esc_control_system_t));
    printf("ESC control algorithm shutdown\n");
}

bool esc_control_configure_parameters(esc_control_system_t* control_system, const control_parameters_t* params) {
    if (!control_system || !params) return false;
    
    memcpy(&control_system->parameters, params, sizeof(control_parameters_t));
    printf("ESC control parameters updated\n");
    return true;
}

void esc_control_update(esc_control_system_t* control_system,
                       const esc_sensor_data_t* sensor_data,
                       const vehicle_parameters_t* vehicle_params,
                       uint32_t current_time_ms) {
    if (!control_system || !sensor_data || !vehicle_params) return;
    
    control_system->last_update_time_ms = current_time_ms;
    
    // Update vehicle dynamics state
    esc_update_vehicle_dynamics(control_system, sensor_data, vehicle_params);
    
    // Assess current vehicle condition
    control_system->vehicle_condition = esc_assess_vehicle_condition(control_system, sensor_data, vehicle_params);
    
    // Run state machine
    esc_control_state_t new_state = esc_control_state_machine(control_system, sensor_data, vehicle_params);
    
    // Handle state transitions
    if (new_state != control_system->current_state) {
        control_system->previous_state = control_system->current_state;
        control_system->current_state = new_state;
        control_system->state_entry_time_ms = current_time_ms;
        
        printf("ESC state transition: %s -> %s\n",
               esc_control_state_to_string(control_system->previous_state),
               esc_control_state_to_string(control_system->current_state));
    }
    
    // Update state duration
    control_system->state_duration_ms = current_time_ms - control_system->state_entry_time_ms;
    
    // Execute control algorithms based on current state
    switch (control_system->current_state) {
        case ESC_STATE_UNDERSTEER:
            esc_control_understeer(control_system, sensor_data, vehicle_params);
            break;
            
        case ESC_STATE_OVERSTEER:
            esc_control_oversteer(control_system, sensor_data, vehicle_params);
            break;
            
        case ESC_STATE_ROLLOVER_PREVENTION:
            esc_control_rollover_prevention(control_system, sensor_data, vehicle_params);
            break;
            
        case ESC_STATE_STANDBY:
            // Reset control outputs but maintain monitoring
            memset(&control_system->brake_control, 0, sizeof(brake_control_t));
            memset(&control_system->engine_control, 0, sizeof(engine_control_t));
            break;
            
        case ESC_STATE_FAULT:
            // Limited operation mode
            esc_control_emergency_intervention(control_system);
            break;
            
        default:
            break;
    }
}

esc_control_state_t esc_control_state_machine(esc_control_system_t* control_system,
                                              const esc_sensor_data_t* sensor_data,
                                              const vehicle_parameters_t* vehicle_params) {
    if (!control_system || !sensor_data || !vehicle_params) {
        return ESC_STATE_FAULT;
    }
    
    // Check if ESC should be active (minimum speed requirement)
    if (sensor_data->vehicle_speed < control_system->parameters.min_speed_activation) {
        return ESC_STATE_STANDBY;
    }
    
    // Safety check
    if (!esc_control_safety_check(control_system, sensor_data)) {
        return ESC_STATE_FAULT;
    }
    
    esc_control_state_t current_state = control_system->current_state;
    vehicle_condition_t condition = control_system->vehicle_condition;
    
    switch (current_state) {
        case ESC_STATE_STANDBY:
            // Transition to active states based on vehicle condition
            switch (condition) {
                case VEHICLE_UNDERSTEER_MILD:
                case VEHICLE_UNDERSTEER_SEVERE:
                    return ESC_STATE_UNDERSTEER;
                    
                case VEHICLE_OVERSTEER_MILD:
                case VEHICLE_OVERSTEER_SEVERE:
                    return ESC_STATE_OVERSTEER;
                    
                case VEHICLE_ROLLOVER_RISK:
                    return ESC_STATE_ROLLOVER_PREVENTION;
                    
                default:
                    return ESC_STATE_STANDBY;
            }
            break;
            
        case ESC_STATE_UNDERSTEER:
            // Stay in understeer state or transition
            if (condition == VEHICLE_ROLLOVER_RISK) {
                return ESC_STATE_ROLLOVER_PREVENTION;
            } else if (condition == VEHICLE_OVERSTEER_MILD || condition == VEHICLE_OVERSTEER_SEVERE) {
                return ESC_STATE_OVERSTEER;
            } else if (condition == VEHICLE_STABLE) {
                // Add hysteresis to prevent rapid state changes
                if (control_system->state_duration_ms > MIN_INTERVENTION_TIME_MS) {
                    return ESC_STATE_STANDBY;
                }
            }
            return ESC_STATE_UNDERSTEER;
            
        case ESC_STATE_OVERSTEER:
            // Stay in oversteer state or transition
            if (condition == VEHICLE_ROLLOVER_RISK) {
                return ESC_STATE_ROLLOVER_PREVENTION;
            } else if (condition == VEHICLE_UNDERSTEER_MILD || condition == VEHICLE_UNDERSTEER_SEVERE) {
                return ESC_STATE_UNDERSTEER;
            } else if (condition == VEHICLE_STABLE) {
                if (control_system->state_duration_ms > MIN_INTERVENTION_TIME_MS) {
                    return ESC_STATE_STANDBY;
                }
            }
            return ESC_STATE_OVERSTEER;
            
        case ESC_STATE_ROLLOVER_PREVENTION:
            // Highest priority state - only exit when safe
            if (condition != VEHICLE_ROLLOVER_RISK && 
                control_system->state_duration_ms > MIN_INTERVENTION_TIME_MS) {
                if (condition == VEHICLE_OVERSTEER_MILD || condition == VEHICLE_OVERSTEER_SEVERE) {
                    return ESC_STATE_OVERSTEER;
                } else if (condition == VEHICLE_UNDERSTEER_MILD || condition == VEHICLE_UNDERSTEER_SEVERE) {
                    return ESC_STATE_UNDERSTEER;
                } else {
                    return ESC_STATE_STANDBY;
                }
            }
            return ESC_STATE_ROLLOVER_PREVENTION;
            
        case ESC_STATE_FAULT:
            // Can only exit fault state through system reset
            return ESC_STATE_FAULT;
            
        default:
            return ESC_STATE_STANDBY;
    }
}

vehicle_condition_t esc_assess_vehicle_condition(const esc_control_system_t* control_system,
                                                const esc_sensor_data_t* sensor_data,
                                                const vehicle_parameters_t* vehicle_params) {
    if (!control_system || !sensor_data || !vehicle_params) {
        return VEHICLE_STABLE;
    }
    
    // Calculate reference yaw rate
    float reference_yaw_rate = esc_calculate_reference_yaw_rate(
        sensor_data->steering_angle, sensor_data->vehicle_speed, vehicle_params->wheelbase);
    
    // Calculate yaw rate error
    float yaw_rate_error = sensor_data->yaw_rate - reference_yaw_rate;
    
    // Check for rollover risk first (highest priority)
    if (fabsf(sensor_data->lateral_acceleration) > control_system->parameters.rollover_threshold) {
        return VEHICLE_ROLLOVER_RISK;
    }
    
    // Assess understeer condition
    if (yaw_rate_error < -control_system->parameters.understeer_threshold_severe) {
        return VEHICLE_UNDERSTEER_SEVERE;
    } else if (yaw_rate_error < -control_system->parameters.understeer_threshold_mild) {
        return VEHICLE_UNDERSTEER_MILD;
    }
    
    // Assess oversteer condition
    if (yaw_rate_error > control_system->parameters.oversteer_threshold_severe) {
        return VEHICLE_OVERSTEER_SEVERE;
    } else if (yaw_rate_error > control_system->parameters.oversteer_threshold_mild) {
        return VEHICLE_OVERSTEER_MILD;
    }
    
    return VEHICLE_STABLE;
}

void esc_control_understeer(esc_control_system_t* control_system,
                           const esc_sensor_data_t* sensor_data,
                           const vehicle_parameters_t* vehicle_params) {
    if (!control_system || !sensor_data || !vehicle_params) return;
    
    // Update event counter
    if (control_system->state_duration_ms == 0) {
        control_system->understeer_events++;
    }
    
    // Calculate yaw rate error and desired correction
    float reference_yaw_rate = esc_calculate_reference_yaw_rate(
        sensor_data->steering_angle, sensor_data->vehicle_speed, vehicle_params->wheelbase);
    float yaw_rate_error = sensor_data->yaw_rate - reference_yaw_rate;
    
    // Store for PID controller
    control_system->yaw_rate_error = yaw_rate_error;
    
    // Determine severity factor based on error magnitude
    float severity_factor = fabsf(yaw_rate_error) / control_system->parameters.understeer_threshold_severe;
    severity_factor = fminf(severity_factor, 1.0f);
    
    // Calculate required yaw moment to correct understeer
    // For understeer, apply brake to inside rear wheel to induce yaw moment
    float target_yaw_moment = -yaw_rate_error * control_system->parameters.yaw_rate_gain * 
                              vehicle_params->inertia_z;
    
    // Limit target yaw moment
    target_yaw_moment = fmaxf(target_yaw_moment, -control_system->parameters.max_yaw_moment);
    target_yaw_moment = fminf(target_yaw_moment, control_system->parameters.max_yaw_moment);
    
    // Calculate brake forces
    esc_calculate_brake_forces(control_system, target_yaw_moment, sensor_data, vehicle_params);
    
    // Calculate engine torque reduction (moderate for understeer)
    esc_calculate_engine_torque_reduction(control_system, severity_factor * 0.3f, sensor_data);
    
    // Set intervention type
    if (control_system->engine_control.torque_reduction_percent > 0) {
        control_system->intervention_type = INTERVENTION_BRAKE_AND_ENGINE;
    } else {
        control_system->intervention_type = INTERVENTION_BRAKE_ONLY;
    }
    
    printf("Understeer correction: yaw_error=%.3f, target_moment=%.1f N⋅m, severity=%.2f\n",
           yaw_rate_error, target_yaw_moment, severity_factor);
}

void esc_control_oversteer(esc_control_system_t* control_system,
                          const esc_sensor_data_t* sensor_data,
                          const vehicle_parameters_t* vehicle_params) {
    if (!control_system || !sensor_data || !vehicle_params) return;
    
    // Update event counter
    if (control_system->state_duration_ms == 0) {
        control_system->oversteer_events++;
    }
    
    // Calculate yaw rate error and desired correction
    float reference_yaw_rate = esc_calculate_reference_yaw_rate(
        sensor_data->steering_angle, sensor_data->vehicle_speed, vehicle_params->wheelbase);
    float yaw_rate_error = sensor_data->yaw_rate - reference_yaw_rate;
    
    control_system->yaw_rate_error = yaw_rate_error;
    
    // Determine severity factor
    float severity_factor = fabsf(yaw_rate_error) / control_system->parameters.oversteer_threshold_severe;
    severity_factor = fminf(severity_factor, 1.0f);
    
    // Calculate required yaw moment to correct oversteer
    // For oversteer, apply brake to outside front wheel to reduce yaw moment
    float target_yaw_moment = -yaw_rate_error * control_system->parameters.yaw_rate_gain * 
                              vehicle_params->inertia_z;
    
    // Limit target yaw moment
    target_yaw_moment = fmaxf(target_yaw_moment, -control_system->parameters.max_yaw_moment);
    target_yaw_moment = fminf(target_yaw_moment, control_system->parameters.max_yaw_moment);
    
    // Calculate brake forces (focus on outside front wheel for oversteer)
    esc_calculate_brake_forces(control_system, target_yaw_moment, sensor_data, vehicle_params);
    
    // Calculate engine torque reduction (more aggressive for oversteer)
    esc_calculate_engine_torque_reduction(control_system, severity_factor * 0.6f, sensor_data);
    
    // Set intervention type
    control_system->intervention_type = INTERVENTION_BRAKE_AND_ENGINE;
    
    printf("Oversteer correction: yaw_error=%.3f, target_moment=%.1f N⋅m, severity=%.2f\n",
           yaw_rate_error, target_yaw_moment, severity_factor);
}

void esc_control_rollover_prevention(esc_control_system_t* control_system,
                                    const esc_sensor_data_t* sensor_data,
                                    const vehicle_parameters_t* vehicle_params) {
    if (!control_system || !sensor_data || !vehicle_params) return;
    
    // Update event counter
    if (control_system->state_duration_ms == 0) {
        control_system->rollover_events++;
    }
    
    // Calculate rollover severity
    float lateral_accel_ratio = fabsf(sensor_data->lateral_acceleration) / 
                               control_system->parameters.rollover_threshold;
    float severity_factor = fminf(lateral_accel_ratio, 1.5f);
    
    // Emergency braking on all wheels with priority on outside wheels
    float base_brake_force = severity_factor * control_system->parameters.max_brake_force * 0.8f;
    
    // Determine which side is outside based on lateral acceleration direction
    bool left_turn = sensor_data->lateral_acceleration > 0;
    
    if (left_turn) {
        // Left turn - right side is outside
        control_system->brake_control.brake_force[WHEEL_FR] = base_brake_force;
        control_system->brake_control.brake_force[WHEEL_RR] = base_brake_force;
        control_system->brake_control.brake_force[WHEEL_FL] = base_brake_force * 0.6f;
        control_system->brake_control.brake_force[WHEEL_RL] = base_brake_force * 0.6f;
    } else {
        // Right turn - left side is outside
        control_system->brake_control.brake_force[WHEEL_FL] = base_brake_force;
        control_system->brake_control.brake_force[WHEEL_RL] = base_brake_force;
        control_system->brake_control.brake_force[WHEEL_FR] = base_brake_force * 0.6f;
        control_system->brake_control.brake_force[WHEEL_RR] = base_brake_force * 0.6f;
    }
    
    // Convert brake forces to pressures
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        // Simplified conversion: assuming brake efficiency factor
        control_system->brake_control.brake_pressure[i] = 
            control_system->brake_control.brake_force[i] / 1000.0f; // N to bar conversion
        
        // Limit brake pressure
        if (control_system->brake_control.brake_pressure[i] > control_system->parameters.max_brake_pressure) {
            control_system->brake_control.brake_pressure[i] = control_system->parameters.max_brake_pressure;
        }
    }
    
    // Aggressive engine torque reduction for rollover prevention
    esc_calculate_engine_torque_reduction(control_system, severity_factor * 0.8f, sensor_data);
    
    // Set intervention type to emergency
    control_system->intervention_type = INTERVENTION_EMERGENCY;
    control_system->emergency_intervention = true;
    
    printf("Rollover prevention: lat_accel=%.2f m/s², severity=%.2f, emergency_braking=ON\n",
           sensor_data->lateral_acceleration, severity_factor);
}

void esc_calculate_brake_forces(esc_control_system_t* control_system,
                               float target_yaw_moment,
                               const esc_sensor_data_t* sensor_data,
                               const vehicle_parameters_t* vehicle_params) {
    if (!control_system || !sensor_data || !vehicle_params) return;
    
    // Initialize brake forces to zero
    memset(control_system->brake_control.brake_force, 0, sizeof(control_system->brake_control.brake_force));
    
    control_system->brake_control.yaw_moment_target = target_yaw_moment;
    
    // Determine which wheel(s) to brake based on yaw moment direction and steering
    float steering_angle = sensor_data->steering_angle;
    bool left_turn = steering_angle > 0;
    
    if (target_yaw_moment > 0) {
        // Need positive (counterclockwise) yaw moment
        if (left_turn) {
            // Understeer in left turn - brake inside rear (left rear)
            control_system->brake_control.brake_force[WHEEL_RL] = 
                fabsf(target_yaw_moment) / (vehicle_params->track_width * 0.5f);
        } else {
            // Oversteer correction or right turn understeer - brake outside front (left front)
            control_system->brake_control.brake_force[WHEEL_FL] = 
                fabsf(target_yaw_moment) / (vehicle_params->wheelbase * 0.7f + vehicle_params->track_width * 0.5f);
        }
    } else if (target_yaw_moment < 0) {
        // Need negative (clockwise) yaw moment
        if (left_turn) {
            // Oversteer correction in left turn - brake outside front (right front)
            control_system->brake_control.brake_force[WHEEL_FR] = 
                fabsf(target_yaw_moment) / (vehicle_params->wheelbase * 0.7f + vehicle_params->track_width * 0.5f);
        } else {
            // Understeer in right turn - brake inside rear (right rear)
            control_system->brake_control.brake_force[WHEEL_RR] = 
                fabsf(target_yaw_moment) / (vehicle_params->track_width * 0.5f);
        }
    }
    
    // Apply brake force limits and calculate pressures
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        // Limit brake force
        if (control_system->brake_control.brake_force[i] > control_system->parameters.max_brake_force) {
            control_system->brake_control.brake_force[i] = control_system->parameters.max_brake_force;
        }
        
        // Convert force to pressure (simplified model)
        // Real implementation would use brake system characteristics
        control_system->brake_control.brake_pressure[i] = 
            control_system->brake_control.brake_force[i] / 1500.0f; // Simplified conversion
        
        // Limit brake pressure
        if (control_system->brake_control.brake_pressure[i] > control_system->parameters.max_brake_pressure) {
            control_system->brake_control.brake_pressure[i] = control_system->parameters.max_brake_pressure;
            control_system->brake_control.brake_force[i] = 
                control_system->parameters.max_brake_pressure * 1500.0f;
        }
    }
    
    // Calculate actual yaw moment from brake forces
    control_system->brake_control.yaw_moment_actual = esc_calculate_yaw_moment_from_brakes(
        &control_system->brake_control, vehicle_params);
}

void esc_calculate_engine_torque_reduction(esc_control_system_t* control_system,
                                          float severity_factor,
                                          const esc_sensor_data_t* sensor_data) {
    if (!control_system || !sensor_data) return;
    
    // Calculate torque reduction based on severity and vehicle condition
    float max_reduction = control_system->parameters.max_torque_reduction;
    float target_reduction = severity_factor * max_reduction;
    
    // Increase reduction if driver is still accelerating during intervention
    if (sensor_data->accelerator_pedal_pressed && !sensor_data->brake_pedal_pressed) {
        target_reduction = fminf(target_reduction * 1.5f, max_reduction);
    }
    
    // Apply rate limiting for smooth torque reduction
    float current_reduction = control_system->engine_control.torque_reduction_percent;
    float max_change_rate = 100.0f; // %/second
    float max_change = max_change_rate * CONTROL_LOOP_DT;
    
    if (target_reduction > current_reduction) {
        // Increasing torque reduction
        control_system->engine_control.torque_reduction_percent = 
            fminf(current_reduction + max_change, target_reduction);
    } else {
        // Decreasing torque reduction (fade out)
        float fade_rate = 50.0f; // %/second for fade out
        float fade_change = fade_rate * CONTROL_LOOP_DT;
        control_system->engine_control.torque_reduction_percent = 
            fmaxf(current_reduction - fade_change, target_reduction);
    }
    
    // Set torque limiting flag
    control_system->engine_control.torque_limiting_active = 
        (control_system->engine_control.torque_reduction_percent > 1.0f);
    
    // Calculate ramp rate for ECU communication
    control_system->engine_control.ramp_rate = 
        fabsf(control_system->engine_control.torque_reduction_percent - current_reduction) / CONTROL_LOOP_DT;
}

void esc_update_vehicle_dynamics(esc_control_system_t* control_system,
                                const esc_sensor_data_t* sensor_data,
                                const vehicle_parameters_t* vehicle_params) {
    if (!control_system || !sensor_data || !vehicle_params) return;
    
    vehicle_dynamics_state_t* dynamics = &control_system->dynamics;
    
    // Calculate sideslip angle using lateral velocity estimation
    if (sensor_data->vehicle_speed > 1.0f) {
        // Estimate lateral velocity from lateral acceleration (simplified)
        dynamics->lateral_velocity = sensor_data->lateral_acceleration * CONTROL_LOOP_DT;
        dynamics->longitudinal_velocity = sensor_data->vehicle_speed;
        
        dynamics->sideslip_angle = esc_calculate_sideslip_angle(
            dynamics->lateral_velocity, dynamics->longitudinal_velocity);
    } else {
        dynamics->sideslip_angle = 0.0f;
        dynamics->lateral_velocity = 0.0f;
        dynamics->longitudinal_velocity = sensor_data->vehicle_speed;
    }
    
    // Calculate yaw acceleration (derivative of yaw rate)
    static float last_yaw_rate = 0.0f;
    dynamics->yaw_acceleration = (sensor_data->yaw_rate - last_yaw_rate) / CONTROL_LOOP_DT;
    last_yaw_rate = sensor_data->yaw_rate;
    
    // Calculate load transfer
    dynamics->load_transfer_front = esc_calculate_load_transfer(
        sensor_data->lateral_acceleration, sensor_data->longitudinal_acceleration, vehicle_params);
    
    // Update road friction estimate
    control_system->parameters.road_friction_estimate = esc_estimate_road_friction(sensor_data);
}

// Utility function implementations
float esc_calculate_reference_yaw_rate(float steering_angle, float vehicle_speed, float wheelbase) {
    if (vehicle_speed < 0.1f) return 0.0f;
    
    // Bicycle model: yaw_rate = (vehicle_speed / wheelbase) * tan(steering_angle)
    return (vehicle_speed / wheelbase) * tanf(steering_angle);
}

float esc_calculate_yaw_moment_from_brakes(const brake_control_t* brake_control,
                                          const vehicle_parameters_t* vehicle_params) {
    if (!brake_control || !vehicle_params) return 0.0f;
    
    // Calculate yaw moment from individual wheel brake forces
    float track_width = vehicle_params->track_width;
    float wheelbase = vehicle_params->wheelbase;
    
    // Moment arms for each wheel
    float moment_fl = (wheelbase * 0.7f) + (track_width * 0.5f); // Front left
    float moment_fr = (wheelbase * 0.7f) - (track_width * 0.5f); // Front right
    float moment_rl = (track_width * 0.5f);                      // Rear left
    float moment_rr = -(track_width * 0.5f);                     // Rear right
    
    float total_moment = 
        brake_control->brake_force[WHEEL_FL] * moment_fl +
        brake_control->brake_force[WHEEL_FR] * moment_fr +
        brake_control->brake_force[WHEEL_RL] * moment_rl +
        brake_control->brake_force[WHEEL_RR] * moment_rr;
    
    return total_moment;
}

float esc_calculate_sideslip_angle(float lateral_velocity, float longitudinal_velocity) {
    if (fabsf(longitudinal_velocity) < 0.1f) return 0.0f;
    
    return atanf(lateral_velocity / longitudinal_velocity);
}

float esc_estimate_road_friction(const esc_sensor_data_t* sensor_data) {
    if (!sensor_data) return 0.7f; // Default friction coefficient
    
    // Simplified friction estimation based on achievable lateral acceleration
    float max_observed_accel = fabsf(sensor_data->lateral_acceleration);
    float estimated_friction = max_observed_accel / GRAVITY;
    
    // Limit to reasonable range
    estimated_friction = fmaxf(estimated_friction, 0.2f); // Ice/snow minimum
    estimated_friction = fminf(estimated_friction, 1.2f); // Dry asphalt maximum
    
    return estimated_friction;
}

float esc_calculate_load_transfer(float lateral_acceleration, float longitudinal_acceleration,
                                 const vehicle_parameters_t* vehicle_params) {
    if (!vehicle_params) return 0.0f;
    
    // Calculate load transfer due to lateral acceleration
    float lateral_load_transfer = (vehicle_params->mass * lateral_acceleration * 
                                  vehicle_params->cg_height) / vehicle_params->track_width;
    
    return lateral_load_transfer;
}

float esc_pid_controller(float error, float* error_integral, float* error_derivative,
                        float last_error, float kp, float ki, float kd, float dt) {
    if (!error_integral || !error_derivative) return 0.0f;
    
    // Update integral term
    *error_integral += error * dt;
    
    // Integral windup protection
    float max_integral = 10.0f;
    if (*error_integral > max_integral) *error_integral = max_integral;
    if (*error_integral < -max_integral) *error_integral = -max_integral;
    
    // Calculate derivative term
    *error_derivative = (error - last_error) / dt;
    
    // Calculate PID output
    float output = kp * error + ki * (*error_integral) + kd * (*error_derivative);
    
    return output;
}

// Default parameter configurations
control_parameters_t esc_get_default_control_parameters(void) {
    control_parameters_t params = {
        // Detection thresholds
        .understeer_threshold_mild = 0.1f,      // rad/s
        .understeer_threshold_severe = 0.3f,    // rad/s
        .oversteer_threshold_mild = 0.1f,       // rad/s
        .oversteer_threshold_severe = 0.3f,     // rad/s
        .rollover_threshold = 8.0f,             // m/s²
        
        // Control gains
        .yaw_rate_gain = 2000.0f,               // N⋅m/(rad/s)
        .sideslip_gain = 1000.0f,               // N⋅m/rad
        .brake_force_gain = 1.0f,
        .engine_torque_gain = 0.5f,
        
        // Control limits
        .max_brake_force = 8000.0f,             // N
        .max_brake_pressure = 180.0f,           // bar
        .max_torque_reduction = 60.0f,          // %
        .max_yaw_moment = 4000.0f,              // N⋅m
        
        // Timing parameters
        .intervention_delay_ms = 50.0f,         // ms
        .fade_out_time_ms = 1000.0f,           // ms
        .min_speed_activation = 5.0f,           // m/s
        
        // Advanced parameters
        .load_transfer_factor = 0.8f,
        .tire_grip_factor = 0.9f,
        .road_friction_estimate = 0.8f
    };
    return params;
}

control_parameters_t esc_get_sport_control_parameters(void) {
    control_parameters_t params = esc_get_default_control_parameters();
    
    // More aggressive thresholds for sport mode
    params.understeer_threshold_mild = 0.15f;
    params.understeer_threshold_severe = 0.4f;
    params.oversteer_threshold_mild = 0.15f;
    params.oversteer_threshold_severe = 0.4f;
    params.max_torque_reduction = 40.0f;        // Less torque reduction
    params.tire_grip_factor = 0.95f;            // Higher grip utilization
    
    return params;
}

control_parameters_t esc_get_comfort_control_parameters(void) {
    control_parameters_t params = esc_get_default_control_parameters();
    
    // More conservative thresholds for comfort mode
    params.understeer_threshold_mild = 0.05f;
    params.understeer_threshold_severe = 0.2f;
    params.oversteer_threshold_mild = 0.05f;
    params.oversteer_threshold_severe = 0.2f;
    params.max_torque_reduction = 80.0f;        // More aggressive torque reduction
    params.tire_grip_factor = 0.8f;             // Conservative grip utilization
    
    return params;
}

// Safety and diagnostic functions
bool esc_control_safety_check(const esc_control_system_t* control_system,
                             const esc_sensor_data_t* sensor_data) {
    if (!control_system || !sensor_data) return false;
    
    // Check for reasonable sensor values
    if (sensor_data->vehicle_speed < 0 || sensor_data->vehicle_speed > 100.0f) return false;
    if (fabsf(sensor_data->yaw_rate) > 10.0f) return false;
    if (fabsf(sensor_data->lateral_acceleration) > 20.0f) return false;
    if (fabsf(sensor_data->steering_angle) > 1.57f) return false; // ±90 degrees
    
    return true;
}

void esc_control_emergency_intervention(esc_control_system_t* control_system) {
    if (!control_system) return;
    
    // Emergency intervention - apply moderate braking to all wheels
    float emergency_brake_force = control_system->parameters.max_brake_force * 0.3f;
    
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        control_system->brake_control.brake_force[i] = emergency_brake_force;
        control_system->brake_control.brake_pressure[i] = emergency_brake_force / 1500.0f;
    }
    
    // Moderate engine torque reduction
    control_system->engine_control.torque_reduction_percent = 30.0f;
    control_system->intervention_type = INTERVENTION_EMERGENCY;
    control_system->emergency_intervention = true;
}

void esc_control_fade_out_intervention(esc_control_system_t* control_system, float fade_rate) {
    if (!control_system) return;
    
    // Gradually reduce brake forces
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        control_system->brake_control.brake_force[i] *= (1.0f - fade_rate);
        control_system->brake_control.brake_pressure[i] *= (1.0f - fade_rate);
        
        if (control_system->brake_control.brake_force[i] < 100.0f) {
            control_system->brake_control.brake_force[i] = 0.0f;
            control_system->brake_control.brake_pressure[i] = 0.0f;
        }
    }
    
    // Gradually reduce engine torque reduction
    control_system->engine_control.torque_reduction_percent *= (1.0f - fade_rate);
    if (control_system->engine_control.torque_reduction_percent < 1.0f) {
        control_system->engine_control.torque_reduction_percent = 0.0f;
        control_system->engine_control.torque_limiting_active = false;
    }
}

void esc_control_reset_statistics(esc_control_system_t* control_system) {
    if (!control_system) return;
    
    control_system->understeer_events = 0;
    control_system->oversteer_events = 0;
    control_system->rollover_events = 0;
    control_system->max_correction_magnitude = 0.0f;
    control_system->average_intervention_duration_ms = 0.0f;
}

// String conversion functions
const char* esc_control_state_to_string(esc_control_state_t state) {
    switch (state) {
        case ESC_STATE_INACTIVE: return "INACTIVE";
        case ESC_STATE_STANDBY: return "STANDBY";
        case ESC_STATE_UNDERSTEER: return "UNDERSTEER";
        case ESC_STATE_OVERSTEER: return "OVERSTEER";
        case ESC_STATE_ROLLOVER_PREVENTION: return "ROLLOVER_PREVENTION";
        case ESC_STATE_BRAKE_ASSIST: return "BRAKE_ASSIST";
        case ESC_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char* esc_vehicle_condition_to_string(vehicle_condition_t condition) {
    switch (condition) {
        case VEHICLE_STABLE: return "STABLE";
        case VEHICLE_UNDERSTEER_MILD: return "UNDERSTEER_MILD";
        case VEHICLE_UNDERSTEER_SEVERE: return "UNDERSTEER_SEVERE";
        case VEHICLE_OVERSTEER_MILD: return "OVERSTEER_MILD";
        case VEHICLE_OVERSTEER_SEVERE: return "OVERSTEER_SEVERE";
        case VEHICLE_ROLLOVER_RISK: return "ROLLOVER_RISK";
        case VEHICLE_EXCESSIVE_SLIP: return "EXCESSIVE_SLIP";
        default: return "UNKNOWN";
    }
}

void esc_control_print_status(const esc_control_system_t* control_system) {
    if (!control_system) return;
    
    printf("\n=== ESC Control Algorithm Status ===\n");
    printf("State: %s\n", esc_control_state_to_string(control_system->current_state));
    printf("Vehicle Condition: %s\n", esc_vehicle_condition_to_string(control_system->vehicle_condition));
    printf("Intervention Type: %d\n", control_system->intervention_type);
    printf("System Confidence: %.2f\n", control_system->system_confidence);
    
    if (control_system->intervention_type != INTERVENTION_NONE) {
        printf("Brake Forces [FL,FR,RL,RR]: [%.0f, %.0f, %.0f, %.0f] N\n",
               control_system->brake_control.brake_force[0],
               control_system->brake_control.brake_force[1],
               control_system->brake_control.brake_force[2],
               control_system->brake_control.brake_force[3]);
        printf("Engine Torque Reduction: %.1f%%\n", 
               control_system->engine_control.torque_reduction_percent);
    }
    
    printf("Events: Understeer=%u, Oversteer=%u, Rollover=%u\n",
           control_system->understeer_events,
           control_system->oversteer_events,
           control_system->rollover_events);
    printf("=====================================\n");
}