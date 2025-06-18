#ifndef SENSOR_PLAUSIBILITY_H
#define SENSOR_PLAUSIBILITY_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_interface.h"
#include "esc_types.h"

// Plausibility check result status
typedef enum {
    PLAUSIBILITY_OK = 0,
    PLAUSIBILITY_WARNING = 1,
    PLAUSIBILITY_MINOR_FAULT = 2,
    PLAUSIBILITY_MAJOR_FAULT = 3,
    PLAUSIBILITY_CRITICAL_FAULT = 4
} plausibility_status_t;

// Plausibility check types
typedef enum {
    PLAUSIBILITY_YAW_LATERAL_CONSISTENCY = 0,
    PLAUSIBILITY_WHEEL_SPEED_COMPARISON = 1,
    PLAUSIBILITY_VEHICLE_SPEED_VALIDATION = 2,
    PLAUSIBILITY_STEERING_YAW_CONSISTENCY = 3,
    PLAUSIBILITY_ACCELERATION_LIMITS = 4,
    PLAUSIBILITY_SIGNAL_TIMEOUT = 5,
    PLAUSIBILITY_SIGNAL_GRADIENT = 6,
    PLAUSIBILITY_SENSOR_CORRELATION = 7,
    PLAUSIBILITY_CHECK_COUNT
} plausibility_check_type_t;

// Plausibility thresholds configuration
typedef struct {
    // Yaw rate vs lateral acceleration consistency
    float yaw_lat_accel_max_deviation;     // Maximum allowed deviation (m/s²)
    float yaw_lat_accel_warning_threshold; // Warning threshold (m/s²)
    float min_vehicle_speed_for_yaw_check; // Minimum speed for yaw consistency check
    
    // Wheel speed plausibility
    float wheel_speed_max_deviation;       // Maximum wheel speed difference (rad/s)
    float wheel_speed_warning_threshold;   // Warning threshold for wheel speed diff
    float max_wheel_acceleration;          // Maximum allowed wheel acceleration (rad/s²)
    
    // Vehicle speed validation
    float vehicle_speed_max_deviation;     // Maximum deviation from calculated speed
    float vehicle_speed_change_limit;      // Maximum speed change per cycle (m/s)
    
    // Steering angle vs yaw rate consistency
    float steering_yaw_max_deviation;      // Maximum deviation from expected yaw rate
    float steering_yaw_warning_threshold;  // Warning threshold for steering-yaw consistency
    
    // Acceleration limits
    float max_lateral_acceleration;        // Maximum physically possible lateral accel
    float max_longitudinal_acceleration;   // Maximum physically possible long accel
    float max_yaw_rate;                   // Maximum physically possible yaw rate
    
    // Signal quality and timeout
    uint32_t signal_timeout_ms;           // Maximum time without valid signal
    uint32_t gradient_check_window_ms;    // Window for gradient checking
    float max_signal_gradient;            // Maximum allowed signal gradient per second
    
    // Correlation checks
    float correlation_threshold;          // Minimum correlation coefficient
    uint32_t correlation_window_samples;  // Number of samples for correlation
} plausibility_thresholds_t;

// Individual plausibility check result
typedef struct {
    plausibility_check_type_t check_type;
    plausibility_status_t status;
    float deviation_value;               // Actual deviation measured
    float threshold_value;               // Threshold that was exceeded
    uint32_t fault_count;               // Number of consecutive faults
    uint32_t last_fault_time_ms;        // Timestamp of last fault
    bool fault_persistent;              // True if fault persists beyond debounce
    char description[64];               // Human-readable description
} plausibility_result_t;

// Historical data for plausibility checks
typedef struct {
    float values[32];                   // Circular buffer for historical values
    uint32_t timestamps[32];            // Timestamps for each value
    uint8_t index;                      // Current index in circular buffer
    uint8_t count;                      // Number of valid entries
    float last_valid_value;             // Last known good value
    uint32_t last_valid_timestamp;      // Timestamp of last valid value
} sensor_history_t;

// Plausibility check system state
typedef struct {
    plausibility_thresholds_t thresholds;
    plausibility_result_t results[PLAUSIBILITY_CHECK_COUNT];
    sensor_history_t sensor_history[SENSOR_COUNT];
    
    // System-wide plausibility status
    plausibility_status_t overall_status;
    uint32_t total_fault_count;
    uint32_t critical_fault_count;
    bool system_degraded;
    
    // Statistics
    uint32_t checks_performed;
    uint32_t checks_passed;
    uint32_t checks_failed;
    
    // Configuration
    bool checks_enabled[PLAUSIBILITY_CHECK_COUNT];
    uint32_t fault_debounce_count[PLAUSIBILITY_CHECK_COUNT];
    uint32_t recovery_debounce_count[PLAUSIBILITY_CHECK_COUNT];
} plausibility_system_t;

// Function prototypes

// System initialization and configuration
bool plausibility_init(plausibility_system_t* system, const vehicle_parameters_t* vehicle_params);
void plausibility_shutdown(plausibility_system_t* system);
bool plausibility_configure_thresholds(plausibility_system_t* system, const plausibility_thresholds_t* thresholds);
void plausibility_enable_check(plausibility_system_t* system, plausibility_check_type_t check_type, bool enable);

// Main plausibility checking functions
void plausibility_update(plausibility_system_t* system, const esc_sensor_data_t* sensor_data, const vehicle_parameters_t* vehicle_params);
plausibility_status_t plausibility_get_overall_status(const plausibility_system_t* system);
plausibility_result_t plausibility_get_check_result(const plausibility_system_t* system, plausibility_check_type_t check_type);

// Individual plausibility check algorithms
plausibility_result_t check_yaw_lateral_consistency(const esc_sensor_data_t* sensor_data, 
                                                   const plausibility_thresholds_t* thresholds,
                                                   const vehicle_parameters_t* vehicle_params);

plausibility_result_t check_wheel_speed_plausibility(const esc_sensor_data_t* sensor_data,
                                                    const plausibility_thresholds_t* thresholds,
                                                    const sensor_history_t* history);

plausibility_result_t check_vehicle_speed_validation(const esc_sensor_data_t* sensor_data,
                                                    const plausibility_thresholds_t* thresholds,
                                                    const sensor_history_t* history);

plausibility_result_t check_steering_yaw_consistency(const esc_sensor_data_t* sensor_data,
                                                    const plausibility_thresholds_t* thresholds,
                                                    const vehicle_parameters_t* vehicle_params);

plausibility_result_t check_acceleration_limits(const esc_sensor_data_t* sensor_data,
                                               const plausibility_thresholds_t* thresholds);

plausibility_result_t check_signal_timeout(const sensor_history_t* history,
                                          const plausibility_thresholds_t* thresholds,
                                          sensor_type_t sensor_type);

plausibility_result_t check_signal_gradient(const sensor_history_t* history,
                                           const plausibility_thresholds_t* thresholds,
                                           sensor_type_t sensor_type);

plausibility_result_t check_sensor_correlation(const sensor_history_t* sensor1_history,
                                              const sensor_history_t* sensor2_history,
                                              const plausibility_thresholds_t* thresholds);

// Utility functions
void plausibility_update_sensor_history(sensor_history_t* history, float value, uint32_t timestamp);
float plausibility_calculate_expected_lateral_accel(float yaw_rate, float vehicle_speed);
float plausibility_calculate_expected_yaw_rate(float steering_angle, float vehicle_speed, float wheelbase);
float plausibility_calculate_correlation(const sensor_history_t* history1, const sensor_history_t* history2);
float plausibility_calculate_gradient(const sensor_history_t* history, uint32_t time_window_ms);
bool plausibility_is_signal_timeout(const sensor_history_t* history, uint32_t current_time, uint32_t timeout_ms);

// Fault management
void plausibility_reset_fault_counters(plausibility_system_t* system);
void plausibility_reset_check_result(plausibility_result_t* result, plausibility_check_type_t check_type);
bool plausibility_is_fault_persistent(const plausibility_result_t* result, uint32_t debounce_count);

// Diagnostic and reporting functions
void plausibility_print_status(const plausibility_system_t* system);
void plausibility_print_check_result(const plausibility_result_t* result);
const char* plausibility_status_to_string(plausibility_status_t status);
const char* plausibility_check_type_to_string(plausibility_check_type_t check_type);

// Default threshold configurations
plausibility_thresholds_t plausibility_get_default_thresholds(void);
plausibility_thresholds_t plausibility_get_conservative_thresholds(void);
plausibility_thresholds_t plausibility_get_aggressive_thresholds(void);

#endif // SENSOR_PLAUSIBILITY_H