#include "sensor_plausibility.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Constants for plausibility calculations
#define PI 3.14159265359f
#define GRAVITY 9.81f
#define MIN_CORRELATION_SAMPLES 10
#define DEBOUNCE_FAULT_COUNT 3
#define DEBOUNCE_RECOVERY_COUNT 5

bool plausibility_init(plausibility_system_t* system, const vehicle_parameters_t* vehicle_params) {
    if (!system || !vehicle_params) {
        return false;
    }
    
    memset(system, 0, sizeof(plausibility_system_t));
    
    // Initialize with default thresholds
    system->thresholds = plausibility_get_default_thresholds();
    
    // Enable all checks by default
    for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
        system->checks_enabled[i] = true;
        system->fault_debounce_count[i] = DEBOUNCE_FAULT_COUNT;
        system->recovery_debounce_count[i] = DEBOUNCE_RECOVERY_COUNT;
        plausibility_reset_check_result(&system->results[i], (plausibility_check_type_t)i);
    }
    
    // Initialize sensor history buffers
    for (int i = 0; i < SENSOR_COUNT; i++) {
        memset(&system->sensor_history[i], 0, sizeof(sensor_history_t));
    }
    
    system->overall_status = PLAUSIBILITY_OK;
    
    printf("Sensor plausibility checking system initialized\n");
    return true;
}

void plausibility_shutdown(plausibility_system_t* system) {
    if (!system) return;
    
    memset(system, 0, sizeof(plausibility_system_t));
    printf("Sensor plausibility checking system shutdown\n");
}

bool plausibility_configure_thresholds(plausibility_system_t* system, const plausibility_thresholds_t* thresholds) {
    if (!system || !thresholds) {
        return false;
    }
    
    memcpy(&system->thresholds, thresholds, sizeof(plausibility_thresholds_t));
    printf("Plausibility thresholds updated\n");
    return true;
}

void plausibility_enable_check(plausibility_system_t* system, plausibility_check_type_t check_type, bool enable) {
    if (!system || check_type >= PLAUSIBILITY_CHECK_COUNT) {
        return;
    }
    
    system->checks_enabled[check_type] = enable;
    printf("Plausibility check %s %s\n", 
           plausibility_check_type_to_string(check_type),
           enable ? "enabled" : "disabled");
}

void plausibility_update(plausibility_system_t* system, const esc_sensor_data_t* sensor_data, const vehicle_parameters_t* vehicle_params) {
    if (!system || !sensor_data || !vehicle_params) {
        return;
    }
    
    uint32_t current_time = sensor_data->timestamp_ms;
    system->checks_performed++;
    
    // Update sensor history
    plausibility_update_sensor_history(&system->sensor_history[SENSOR_WHEEL_SPEED_FL], 
                                     sensor_data->wheel_speed[0], current_time);
    plausibility_update_sensor_history(&system->sensor_history[SENSOR_WHEEL_SPEED_FR], 
                                     sensor_data->wheel_speed[1], current_time);
    plausibility_update_sensor_history(&system->sensor_history[SENSOR_WHEEL_SPEED_RL], 
                                     sensor_data->wheel_speed[2], current_time);
    plausibility_update_sensor_history(&system->sensor_history[SENSOR_WHEEL_SPEED_RR], 
                                     sensor_data->wheel_speed[3], current_time);
    plausibility_update_sensor_history(&system->sensor_history[SENSOR_STEERING_ANGLE], 
                                     sensor_data->steering_angle, current_time);
    plausibility_update_sensor_history(&system->sensor_history[SENSOR_YAW_RATE], 
                                     sensor_data->yaw_rate, current_time);
    plausibility_update_sensor_history(&system->sensor_history[SENSOR_LATERAL_ACCEL], 
                                     sensor_data->lateral_acceleration, current_time);
    plausibility_update_sensor_history(&system->sensor_history[SENSOR_LONGITUDINAL_ACCEL], 
                                     sensor_data->longitudinal_acceleration, current_time);
    
    // Perform individual plausibility checks
    bool any_fault = false;
    
    // Yaw rate vs lateral acceleration consistency
    if (system->checks_enabled[PLAUSIBILITY_YAW_LATERAL_CONSISTENCY]) {
        system->results[PLAUSIBILITY_YAW_LATERAL_CONSISTENCY] = 
            check_yaw_lateral_consistency(sensor_data, &system->thresholds, vehicle_params);
        if (system->results[PLAUSIBILITY_YAW_LATERAL_CONSISTENCY].status >= PLAUSIBILITY_MINOR_FAULT) {
            any_fault = true;
        }
    }
    
    // Wheel speed plausibility
    if (system->checks_enabled[PLAUSIBILITY_WHEEL_SPEED_COMPARISON]) {
        system->results[PLAUSIBILITY_WHEEL_SPEED_COMPARISON] = 
            check_wheel_speed_plausibility(sensor_data, &system->thresholds, system->sensor_history);
        if (system->results[PLAUSIBILITY_WHEEL_SPEED_COMPARISON].status >= PLAUSIBILITY_MINOR_FAULT) {
            any_fault = true;
        }
    }
    
    // Vehicle speed validation
    if (system->checks_enabled[PLAUSIBILITY_VEHICLE_SPEED_VALIDATION]) {
        system->results[PLAUSIBILITY_VEHICLE_SPEED_VALIDATION] = 
            check_vehicle_speed_validation(sensor_data, &system->thresholds, system->sensor_history);
        if (system->results[PLAUSIBILITY_VEHICLE_SPEED_VALIDATION].status >= PLAUSIBILITY_MINOR_FAULT) {
            any_fault = true;
        }
    }
    
    // Steering angle vs yaw rate consistency
    if (system->checks_enabled[PLAUSIBILITY_STEERING_YAW_CONSISTENCY]) {
        system->results[PLAUSIBILITY_STEERING_YAW_CONSISTENCY] = 
            check_steering_yaw_consistency(sensor_data, &system->thresholds, vehicle_params);
        if (system->results[PLAUSIBILITY_STEERING_YAW_CONSISTENCY].status >= PLAUSIBILITY_MINOR_FAULT) {
            any_fault = true;
        }
    }
    
    // Acceleration limits
    if (system->checks_enabled[PLAUSIBILITY_ACCELERATION_LIMITS]) {
        system->results[PLAUSIBILITY_ACCELERATION_LIMITS] = 
            check_acceleration_limits(sensor_data, &system->thresholds);
        if (system->results[PLAUSIBILITY_ACCELERATION_LIMITS].status >= PLAUSIBILITY_MINOR_FAULT) {
            any_fault = true;
        }
    }
    
    // Signal timeout checks
    if (system->checks_enabled[PLAUSIBILITY_SIGNAL_TIMEOUT]) {
        plausibility_result_t timeout_result = {0};
        timeout_result.check_type = PLAUSIBILITY_SIGNAL_TIMEOUT;
        timeout_result.status = PLAUSIBILITY_OK;
        
        for (int i = 0; i < SENSOR_COUNT; i++) {
            plausibility_result_t sensor_timeout = check_signal_timeout(
                &system->sensor_history[i], &system->thresholds, (sensor_type_t)i);
            if (sensor_timeout.status > timeout_result.status) {
                timeout_result = sensor_timeout;
            }
        }
        system->results[PLAUSIBILITY_SIGNAL_TIMEOUT] = timeout_result;
        if (timeout_result.status >= PLAUSIBILITY_MINOR_FAULT) {
            any_fault = true;
        }
    }
    
    // Signal gradient checks
    if (system->checks_enabled[PLAUSIBILITY_SIGNAL_GRADIENT]) {
        plausibility_result_t gradient_result = {0};
        gradient_result.check_type = PLAUSIBILITY_SIGNAL_GRADIENT;
        gradient_result.status = PLAUSIBILITY_OK;
        
        for (int i = 0; i < SENSOR_COUNT; i++) {
            plausibility_result_t sensor_gradient = check_signal_gradient(
                &system->sensor_history[i], &system->thresholds, (sensor_type_t)i);
            if (sensor_gradient.status > gradient_result.status) {
                gradient_result = sensor_gradient;
            }
        }
        system->results[PLAUSIBILITY_SIGNAL_GRADIENT] = gradient_result;
        if (gradient_result.status >= PLAUSIBILITY_MINOR_FAULT) {
            any_fault = true;
        }
    }
    
    // Update overall system status
    plausibility_status_t max_status = PLAUSIBILITY_OK;
    system->critical_fault_count = 0;
    
    for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
        if (system->checks_enabled[i]) {
            if (system->results[i].status > max_status) {
                max_status = system->results[i].status;
            }
            if (system->results[i].status == PLAUSIBILITY_CRITICAL_FAULT) {
                system->critical_fault_count++;
            }
        }
    }
    
    system->overall_status = max_status;
    system->system_degraded = (max_status >= PLAUSIBILITY_MINOR_FAULT);
    
    if (any_fault) {
        system->checks_failed++;
        system->total_fault_count++;
    } else {
        system->checks_passed++;
    }
}

plausibility_result_t check_yaw_lateral_consistency(const esc_sensor_data_t* sensor_data, 
                                                   const plausibility_thresholds_t* thresholds,
                                                   const vehicle_parameters_t* vehicle_params) {
    plausibility_result_t result = {0};
    result.check_type = PLAUSIBILITY_YAW_LATERAL_CONSISTENCY;
    
    // Skip check if vehicle speed is too low
    if (sensor_data->vehicle_speed < thresholds->min_vehicle_speed_for_yaw_check) {
        result.status = PLAUSIBILITY_OK;
        strncpy(result.description, "Speed too low for yaw-lateral check", sizeof(result.description) - 1);
        return result;
    }
    
    // Calculate expected lateral acceleration from yaw rate and vehicle speed
    float expected_lat_accel = plausibility_calculate_expected_lateral_accel(
        sensor_data->yaw_rate, sensor_data->vehicle_speed);
    
    // Calculate deviation between measured and expected lateral acceleration
    float deviation = fabsf(sensor_data->lateral_acceleration - expected_lat_accel);
    
    result.deviation_value = deviation;
    result.threshold_value = thresholds->yaw_lat_accel_max_deviation;
    
    // Determine status based on deviation
    if (deviation > thresholds->yaw_lat_accel_max_deviation) {
        result.status = PLAUSIBILITY_MAJOR_FAULT;
        result.fault_count++;
        snprintf(result.description, sizeof(result.description), 
                "Yaw-lateral deviation %.2f > %.2f m/s²", deviation, thresholds->yaw_lat_accel_max_deviation);
    } else if (deviation > thresholds->yaw_lat_accel_warning_threshold) {
        result.status = PLAUSIBILITY_WARNING;
        snprintf(result.description, sizeof(result.description), 
                "Yaw-lateral deviation %.2f > warning %.2f m/s²", deviation, thresholds->yaw_lat_accel_warning_threshold);
    } else {
        result.status = PLAUSIBILITY_OK;
        snprintf(result.description, sizeof(result.description), 
                "Yaw-lateral consistency OK (dev: %.2f m/s²)", deviation);
    }
    
    return result;
}

plausibility_result_t check_wheel_speed_plausibility(const esc_sensor_data_t* sensor_data,
                                                    const plausibility_thresholds_t* thresholds,
                                                    const sensor_history_t* history) {
    plausibility_result_t result = {0};
    result.check_type = PLAUSIBILITY_WHEEL_SPEED_COMPARISON;
    
    // Calculate average wheel speed
    float avg_wheel_speed = 0.0f;
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        avg_wheel_speed += sensor_data->wheel_speed[i];
    }
    avg_wheel_speed /= MAX_WHEEL_COUNT;
    
    // Find maximum deviation from average
    float max_deviation = 0.0f;
    int worst_wheel = -1;
    
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        float deviation = fabsf(sensor_data->wheel_speed[i] - avg_wheel_speed);
        if (deviation > max_deviation) {
            max_deviation = deviation;
            worst_wheel = i;
        }
    }
    
    result.deviation_value = max_deviation;
    result.threshold_value = thresholds->wheel_speed_max_deviation;
    
    // Check wheel acceleration limits if we have history
    bool excessive_acceleration = false;
    for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
        if (history[i].count >= 2) {
            float dt = (sensor_data->timestamp_ms - history[i].timestamps[(history[i].index - 1 + 32) % 32]) / 1000.0f;
            if (dt > 0) {
                float acceleration = fabsf(sensor_data->wheel_speed[i] - history[i].values[(history[i].index - 1 + 32) % 32]) / dt;
                if (acceleration > thresholds->max_wheel_acceleration) {
                    excessive_acceleration = true;
                    break;
                }
            }
        }
    }
    
    // Determine status
    if (max_deviation > thresholds->wheel_speed_max_deviation || excessive_acceleration) {
        result.status = PLAUSIBILITY_MAJOR_FAULT;
        result.fault_count++;
        if (excessive_acceleration) {
            snprintf(result.description, sizeof(result.description), 
                    "Excessive wheel acceleration detected");
        } else {
            snprintf(result.description, sizeof(result.description), 
                    "Wheel %d deviation %.2f > %.2f rad/s", worst_wheel, max_deviation, thresholds->wheel_speed_max_deviation);
        }
    } else if (max_deviation > thresholds->wheel_speed_warning_threshold) {
        result.status = PLAUSIBILITY_WARNING;
        snprintf(result.description, sizeof(result.description), 
                "Wheel %d deviation %.2f > warning %.2f rad/s", worst_wheel, max_deviation, thresholds->wheel_speed_warning_threshold);
    } else {
        result.status = PLAUSIBILITY_OK;
        snprintf(result.description, sizeof(result.description), 
                "Wheel speed consistency OK (max dev: %.2f rad/s)", max_deviation);
    }
    
    return result;
}

plausibility_result_t check_vehicle_speed_validation(const esc_sensor_data_t* sensor_data,
                                                    const plausibility_thresholds_t* thresholds,
                                                    const sensor_history_t* history) {
    plausibility_result_t result = {0};
    result.check_type = PLAUSIBILITY_VEHICLE_SPEED_VALIDATION;
    
    // Calculate expected vehicle speed from wheel speeds (average of rear wheels)
    float calculated_vehicle_speed = (sensor_data->wheel_speed[2] + sensor_data->wheel_speed[3]) / 2.0f;
    // Convert from rad/s to m/s (assuming wheel radius from vehicle params)
    calculated_vehicle_speed *= 0.32f; // Default wheel radius
    
    // Compare with reported vehicle speed
    float speed_deviation = fabsf(sensor_data->vehicle_speed - calculated_vehicle_speed);
    
    result.deviation_value = speed_deviation;
    result.threshold_value = thresholds->vehicle_speed_max_deviation;
    
    // Check for excessive speed change if we have history
    bool excessive_speed_change = false;
    const sensor_history_t* speed_history = &history[SENSOR_WHEEL_SPEED_FL]; // Use as proxy for vehicle speed history
    if (speed_history->count >= 2) {
        float dt = (sensor_data->timestamp_ms - speed_history->last_valid_timestamp) / 1000.0f;
        if (dt > 0) {
            float speed_change = fabsf(sensor_data->vehicle_speed - speed_history->last_valid_value) / dt;
            if (speed_change > thresholds->vehicle_speed_change_limit) {
                excessive_speed_change = true;
            }
        }
    }
    
    // Determine status
    if (speed_deviation > thresholds->vehicle_speed_max_deviation || excessive_speed_change) {
        result.status = PLAUSIBILITY_MAJOR_FAULT;
        result.fault_count++;
        if (excessive_speed_change) {
            snprintf(result.description, sizeof(result.description), 
                    "Excessive vehicle speed change detected");
        } else {
            snprintf(result.description, sizeof(result.description), 
                    "Vehicle speed deviation %.2f > %.2f m/s", speed_deviation, thresholds->vehicle_speed_max_deviation);
        }
    } else if (speed_deviation > thresholds->vehicle_speed_max_deviation * 0.7f) {
        result.status = PLAUSIBILITY_WARNING;
        snprintf(result.description, sizeof(result.description), 
                "Vehicle speed deviation %.2f approaching limit", speed_deviation);
    } else {
        result.status = PLAUSIBILITY_OK;
        snprintf(result.description, sizeof(result.description), 
                "Vehicle speed validation OK (dev: %.2f m/s)", speed_deviation);
    }
    
    return result;
}

plausibility_result_t check_steering_yaw_consistency(const esc_sensor_data_t* sensor_data,
                                                    const plausibility_thresholds_t* thresholds,
                                                    const vehicle_parameters_t* vehicle_params) {
    plausibility_result_t result = {0};
    result.check_type = PLAUSIBILITY_STEERING_YAW_CONSISTENCY;
    
    // Skip check if vehicle speed is too low
    if (sensor_data->vehicle_speed < thresholds->min_vehicle_speed_for_yaw_check) {
        result.status = PLAUSIBILITY_OK;
        strncpy(result.description, "Speed too low for steering-yaw check", sizeof(result.description) - 1);
        return result;
    }
    
    // Calculate expected yaw rate from steering angle
    float expected_yaw_rate = plausibility_calculate_expected_yaw_rate(
        sensor_data->steering_angle, sensor_data->vehicle_speed, vehicle_params->wheelbase);
    
    // Calculate deviation between measured and expected yaw rate
    float deviation = fabsf(sensor_data->yaw_rate - expected_yaw_rate);
    
    result.deviation_value = deviation;
    result.threshold_value = thresholds->steering_yaw_max_deviation;
    
    // Determine status based on deviation
    if (deviation > thresholds->steering_yaw_max_deviation) {
        result.status = PLAUSIBILITY_MAJOR_FAULT;
        result.fault_count++;
        snprintf(result.description, sizeof(result.description), 
                "Steering-yaw deviation %.3f > %.3f rad/s", deviation, thresholds->steering_yaw_max_deviation);
    } else if (deviation > thresholds->steering_yaw_warning_threshold) {
        result.status = PLAUSIBILITY_WARNING;
        snprintf(result.description, sizeof(result.description), 
                "Steering-yaw deviation %.3f > warning %.3f rad/s", deviation, thresholds->steering_yaw_warning_threshold);
    } else {
        result.status = PLAUSIBILITY_OK;
        snprintf(result.description, sizeof(result.description), 
                "Steering-yaw consistency OK (dev: %.3f rad/s)", deviation);
    }
    
    return result;
}

plausibility_result_t check_acceleration_limits(const esc_sensor_data_t* sensor_data,
                                               const plausibility_thresholds_t* thresholds) {
    plausibility_result_t result = {0};
    result.check_type = PLAUSIBILITY_ACCELERATION_LIMITS;
    
    // Check lateral acceleration limit
    if (fabsf(sensor_data->lateral_acceleration) > thresholds->max_lateral_acceleration) {
        result.status = PLAUSIBILITY_CRITICAL_FAULT;
        result.fault_count++;
        result.deviation_value = fabsf(sensor_data->lateral_acceleration);
        result.threshold_value = thresholds->max_lateral_acceleration;
        snprintf(result.description, sizeof(result.description), 
                "Lateral accel %.2f > limit %.2f m/s²", 
                fabsf(sensor_data->lateral_acceleration), thresholds->max_lateral_acceleration);
        return result;
    }
    
    // Check longitudinal acceleration limit
    if (fabsf(sensor_data->longitudinal_acceleration) > thresholds->max_longitudinal_acceleration) {
        result.status = PLAUSIBILITY_CRITICAL_FAULT;
        result.fault_count++;
        result.deviation_value = fabsf(sensor_data->longitudinal_acceleration);
        result.threshold_value = thresholds->max_longitudinal_acceleration;
        snprintf(result.description, sizeof(result.description), 
                "Longitudinal accel %.2f > limit %.2f m/s²", 
                fabsf(sensor_data->longitudinal_acceleration), thresholds->max_longitudinal_acceleration);
        return result;
    }
    
    // Check yaw rate limit
    if (fabsf(sensor_data->yaw_rate) > thresholds->max_yaw_rate) {
        result.status = PLAUSIBILITY_CRITICAL_FAULT;
        result.fault_count++;
        result.deviation_value = fabsf(sensor_data->yaw_rate);
        result.threshold_value = thresholds->max_yaw_rate;
        snprintf(result.description, sizeof(result.description), 
                "Yaw rate %.3f > limit %.3f rad/s", 
                fabsf(sensor_data->yaw_rate), thresholds->max_yaw_rate);
        return result;
    }
    
    result.status = PLAUSIBILITY_OK;
    strncpy(result.description, "All acceleration limits OK", sizeof(result.description) - 1);
    return result;
}

plausibility_result_t check_signal_timeout(const sensor_history_t* history,
                                          const plausibility_thresholds_t* thresholds,
                                          sensor_type_t sensor_type) {
    plausibility_result_t result = {0};
    result.check_type = PLAUSIBILITY_SIGNAL_TIMEOUT;
    
    uint32_t current_time = 0; // Would get from system timestamp
    if (history->count > 0) {
        current_time = history->timestamps[history->index == 0 ? 31 : history->index - 1];
    }
    
    bool timeout = plausibility_is_signal_timeout(history, current_time, thresholds->signal_timeout_ms);
    
    if (timeout) {
        result.status = PLAUSIBILITY_CRITICAL_FAULT;
        result.fault_count++;
        snprintf(result.description, sizeof(result.description), 
                "Sensor %d signal timeout", (int)sensor_type);
    } else {
        result.status = PLAUSIBILITY_OK;
        snprintf(result.description, sizeof(result.description), 
                "Sensor %d signal OK", (int)sensor_type);
    }
    
    return result;
}

plausibility_result_t check_signal_gradient(const sensor_history_t* history,
                                           const plausibility_thresholds_t* thresholds,
                                           sensor_type_t sensor_type) {
    plausibility_result_t result = {0};
    result.check_type = PLAUSIBILITY_SIGNAL_GRADIENT;
    
    if (history->count < 2) {
        result.status = PLAUSIBILITY_OK;
        strncpy(result.description, "Insufficient data for gradient check", sizeof(result.description) - 1);
        return result;
    }
    
    float gradient = plausibility_calculate_gradient(history, thresholds->gradient_check_window_ms);
    
    result.deviation_value = fabsf(gradient);
    result.threshold_value = thresholds->max_signal_gradient;
    
    if (fabsf(gradient) > thresholds->max_signal_gradient) {
        result.status = PLAUSIBILITY_MAJOR_FAULT;
        result.fault_count++;
        snprintf(result.description, sizeof(result.description), 
                "Sensor %d excessive gradient %.3f", (int)sensor_type, gradient);
    } else {
        result.status = PLAUSIBILITY_OK;
        snprintf(result.description, sizeof(result.description), 
                "Sensor %d gradient OK (%.3f)", (int)sensor_type, gradient);
    }
    
    return result;
}

plausibility_result_t check_sensor_correlation(const sensor_history_t* sensor1_history,
                                              const sensor_history_t* sensor2_history,
                                              const plausibility_thresholds_t* thresholds) {
    plausibility_result_t result = {0};
    result.check_type = PLAUSIBILITY_SENSOR_CORRELATION;
    
    if (sensor1_history->count < MIN_CORRELATION_SAMPLES || 
        sensor2_history->count < MIN_CORRELATION_SAMPLES) {
        result.status = PLAUSIBILITY_OK;
        strncpy(result.description, "Insufficient data for correlation", sizeof(result.description) - 1);
        return result;
    }
    
    float correlation = plausibility_calculate_correlation(sensor1_history, sensor2_history);
    
    result.deviation_value = correlation;
    result.threshold_value = thresholds->correlation_threshold;
    
    if (correlation < thresholds->correlation_threshold) {
        result.status = PLAUSIBILITY_MINOR_FAULT;
        result.fault_count++;
        snprintf(result.description, sizeof(result.description), 
                "Low sensor correlation %.3f < %.3f", correlation, thresholds->correlation_threshold);
    } else {
        result.status = PLAUSIBILITY_OK;
        snprintf(result.description, sizeof(result.description), 
                "Sensor correlation OK (%.3f)", correlation);
    }
    
    return result;
}

// Utility function implementations
void plausibility_update_sensor_history(sensor_history_t* history, float value, uint32_t timestamp) {
    if (!history) return;
    
    history->values[history->index] = value;
    history->timestamps[history->index] = timestamp;
    history->last_valid_value = value;
    history->last_valid_timestamp = timestamp;
    
    history->index = (history->index + 1) % 32;
    if (history->count < 32) {
        history->count++;
    }
}

float plausibility_calculate_expected_lateral_accel(float yaw_rate, float vehicle_speed) {
    // Lateral acceleration = yaw_rate * vehicle_speed
    return yaw_rate * vehicle_speed;
}

float plausibility_calculate_expected_yaw_rate(float steering_angle, float vehicle_speed, float wheelbase) {
    if (vehicle_speed < 0.1f) {
        return 0.0f; // Avoid division by very small numbers
    }
    
    // Bicycle model: yaw_rate = (vehicle_speed / wheelbase) * tan(steering_angle)
    return (vehicle_speed / wheelbase) * tanf(steering_angle);
}

float plausibility_calculate_correlation(const sensor_history_t* history1, const sensor_history_t* history2) {
    if (!history1 || !history2 || history1->count < MIN_CORRELATION_SAMPLES || history2->count < MIN_CORRELATION_SAMPLES) {
        return 0.0f;
    }
    
    int n = (history1->count < history2->count) ? history1->count : history2->count;
    if (n < MIN_CORRELATION_SAMPLES) {
        return 0.0f;
    }
    
    // Calculate means
    float sum1 = 0.0f, sum2 = 0.0f;
    for (int i = 0; i < n; i++) {
        sum1 += history1->values[i];
        sum2 += history2->values[i];
    }
    float mean1 = sum1 / n;
    float mean2 = sum2 / n;
    
    // Calculate correlation coefficient
    float numerator = 0.0f;
    float sum_sq1 = 0.0f, sum_sq2 = 0.0f;
    
    for (int i = 0; i < n; i++) {
        float diff1 = history1->values[i] - mean1;
        float diff2 = history2->values[i] - mean2;
        numerator += diff1 * diff2;
        sum_sq1 += diff1 * diff1;
        sum_sq2 += diff2 * diff2;
    }
    
    float denominator = sqrtf(sum_sq1 * sum_sq2);
    if (denominator < 1e-6f) {
        return 0.0f;
    }
    
    return numerator / denominator;
}

float plausibility_calculate_gradient(const sensor_history_t* history, uint32_t time_window_ms) {
    if (!history || history->count < 2) {
        return 0.0f;
    }
    
    // Simple gradient calculation using first and last values in window
    int last_idx = (history->index == 0) ? 31 : history->index - 1;
    int first_idx = last_idx;
    
    // Find first sample within time window
    uint32_t target_time = history->timestamps[last_idx] - time_window_ms;
    for (int i = 1; i < history->count; i++) {
        int idx = (last_idx - i + 32) % 32;
        if (history->timestamps[idx] < target_time) {
            break;
        }
        first_idx = idx;
    }
    
    if (first_idx == last_idx) {
        return 0.0f;
    }
    
    float dt = (history->timestamps[last_idx] - history->timestamps[first_idx]) / 1000.0f;
    if (dt <= 0) {
        return 0.0f;
    }
    
    return (history->values[last_idx] - history->values[first_idx]) / dt;
}

bool plausibility_is_signal_timeout(const sensor_history_t* history, uint32_t current_time, uint32_t timeout_ms) {
    if (!history || history->count == 0) {
        return true;
    }
    
    return (current_time - history->last_valid_timestamp) > timeout_ms;
}

// Default threshold configurations
plausibility_thresholds_t plausibility_get_default_thresholds(void) {
    plausibility_thresholds_t thresholds = {
        // Yaw rate vs lateral acceleration consistency
        .yaw_lat_accel_max_deviation = 3.0f,        // 3 m/s² maximum deviation
        .yaw_lat_accel_warning_threshold = 2.0f,     // 2 m/s² warning threshold
        .min_vehicle_speed_for_yaw_check = 5.0f,    // 5 m/s minimum speed
        
        // Wheel speed plausibility
        .wheel_speed_max_deviation = 15.0f,         // 15 rad/s maximum wheel speed difference
        .wheel_speed_warning_threshold = 10.0f,      // 10 rad/s warning threshold
        .max_wheel_acceleration = 50.0f,            // 50 rad/s² max wheel acceleration
        
        // Vehicle speed validation
        .vehicle_speed_max_deviation = 3.0f,        // 3 m/s maximum deviation
        .vehicle_speed_change_limit = 8.0f,         // 8 m/s per second max change
        
        // Steering angle vs yaw rate consistency
        .steering_yaw_max_deviation = 0.5f,         // 0.5 rad/s maximum deviation
        .steering_yaw_warning_threshold = 0.3f,      // 0.3 rad/s warning threshold
        
        // Acceleration limits
        .max_lateral_acceleration = 15.0f,          // 15 m/s² (~1.5g) maximum lateral
        .max_longitudinal_acceleration = 12.0f,     // 12 m/s² (~1.2g) maximum longitudinal
        .max_yaw_rate = 3.0f,                      // 3 rad/s maximum yaw rate
        
        // Signal quality and timeout
        .signal_timeout_ms = 200,                   // 200ms timeout
        .gradient_check_window_ms = 100,            // 100ms window for gradient
        .max_signal_gradient = 50.0f,              // 50 units/s maximum gradient
        
        // Correlation checks
        .correlation_threshold = 0.7f,              // 0.7 minimum correlation
        .correlation_window_samples = 20           // 20 samples for correlation
    };
    return thresholds;
}

plausibility_thresholds_t plausibility_get_conservative_thresholds(void) {
    plausibility_thresholds_t thresholds = plausibility_get_default_thresholds();
    
    // Tighten thresholds for conservative checking
    thresholds.yaw_lat_accel_max_deviation = 2.0f;
    thresholds.yaw_lat_accel_warning_threshold = 1.5f;
    thresholds.wheel_speed_max_deviation = 10.0f;
    thresholds.vehicle_speed_max_deviation = 2.0f;
    thresholds.steering_yaw_max_deviation = 0.3f;
    thresholds.max_lateral_acceleration = 12.0f;
    thresholds.max_longitudinal_acceleration = 10.0f;
    thresholds.correlation_threshold = 0.8f;
    
    return thresholds;
}

plausibility_thresholds_t plausibility_get_aggressive_thresholds(void) {
    plausibility_thresholds_t thresholds = plausibility_get_default_thresholds();
    
    // Relax thresholds for aggressive/performance checking
    thresholds.yaw_lat_accel_max_deviation = 4.0f;
    thresholds.yaw_lat_accel_warning_threshold = 3.0f;
    thresholds.wheel_speed_max_deviation = 20.0f;
    thresholds.vehicle_speed_max_deviation = 4.0f;
    thresholds.steering_yaw_max_deviation = 0.7f;
    thresholds.max_lateral_acceleration = 18.0f;
    thresholds.max_longitudinal_acceleration = 15.0f;
    thresholds.correlation_threshold = 0.6f;
    
    return thresholds;
}

// Helper functions for fault management and diagnostics
void plausibility_reset_fault_counters(plausibility_system_t* system) {
    if (!system) return;
    
    for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
        system->results[i].fault_count = 0;
        system->results[i].fault_persistent = false;
    }
    system->total_fault_count = 0;
    system->critical_fault_count = 0;
}

void plausibility_reset_check_result(plausibility_result_t* result, plausibility_check_type_t check_type) {
    if (!result) return;
    
    memset(result, 0, sizeof(plausibility_result_t));
    result->check_type = check_type;
    result->status = PLAUSIBILITY_OK;
}

bool plausibility_is_fault_persistent(const plausibility_result_t* result, uint32_t debounce_count) {
    return result && (result->fault_count >= debounce_count);
}

plausibility_status_t plausibility_get_overall_status(const plausibility_system_t* system) {
    return system ? system->overall_status : PLAUSIBILITY_CRITICAL_FAULT;
}

plausibility_result_t plausibility_get_check_result(const plausibility_system_t* system, plausibility_check_type_t check_type) {
    if (!system || check_type >= PLAUSIBILITY_CHECK_COUNT) {
        plausibility_result_t invalid_result = {0};
        invalid_result.status = PLAUSIBILITY_CRITICAL_FAULT;
        return invalid_result;
    }
    
    return system->results[check_type];
}

// Diagnostic and string conversion functions
const char* plausibility_status_to_string(plausibility_status_t status) {
    switch (status) {
        case PLAUSIBILITY_OK: return "OK";
        case PLAUSIBILITY_WARNING: return "WARNING";
        case PLAUSIBILITY_MINOR_FAULT: return "MINOR_FAULT";
        case PLAUSIBILITY_MAJOR_FAULT: return "MAJOR_FAULT";
        case PLAUSIBILITY_CRITICAL_FAULT: return "CRITICAL_FAULT";
        default: return "UNKNOWN";
    }
}

const char* plausibility_check_type_to_string(plausibility_check_type_t check_type) {
    switch (check_type) {
        case PLAUSIBILITY_YAW_LATERAL_CONSISTENCY: return "YAW_LATERAL_CONSISTENCY";
        case PLAUSIBILITY_WHEEL_SPEED_COMPARISON: return "WHEEL_SPEED_COMPARISON";
        case PLAUSIBILITY_VEHICLE_SPEED_VALIDATION: return "VEHICLE_SPEED_VALIDATION";
        case PLAUSIBILITY_STEERING_YAW_CONSISTENCY: return "STEERING_YAW_CONSISTENCY";
        case PLAUSIBILITY_ACCELERATION_LIMITS: return "ACCELERATION_LIMITS";
        case PLAUSIBILITY_SIGNAL_TIMEOUT: return "SIGNAL_TIMEOUT";
        case PLAUSIBILITY_SIGNAL_GRADIENT: return "SIGNAL_GRADIENT";
        case PLAUSIBILITY_SENSOR_CORRELATION: return "SENSOR_CORRELATION";
        default: return "UNKNOWN_CHECK";
    }
}

void plausibility_print_status(const plausibility_system_t* system) {
    if (!system) return;
    
    printf("\n=== Sensor Plausibility Status ===\n");
    printf("Overall Status: %s\n", plausibility_status_to_string(system->overall_status));
    printf("System Degraded: %s\n", system->system_degraded ? "YES" : "NO");
    printf("Total Checks: %u, Passed: %u, Failed: %u\n", 
           system->checks_performed, system->checks_passed, system->checks_failed);
    printf("Critical Faults: %u\n", system->critical_fault_count);
    
    for (int i = 0; i < PLAUSIBILITY_CHECK_COUNT; i++) {
        if (system->checks_enabled[i] && system->results[i].status != PLAUSIBILITY_OK) {
            plausibility_print_check_result(&system->results[i]);
        }
    }
    printf("================================\n");
}

void plausibility_print_check_result(const plausibility_result_t* result) {
    if (!result) return;
    
    printf("Check: %s | Status: %s | %s\n",
           plausibility_check_type_to_string(result->check_type),
           plausibility_status_to_string(result->status),
           result->description);
}