#ifndef ESC_ECU_H
#define ESC_ECU_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_interface.h"
#include "esc_types.h"

// Forward declarations for brake actuation, engine torque, watchdog, DTC, POST, and logging systems
struct brake_actuation_system_t;
struct engine_torque_interface_t;
struct watchdog_supervision_t;
struct dtc_manager_t;
struct post_system_t;
struct esc_logging_system_t;
#define ESC_SAMPLE_RATE_HZ 100
#define ESC_SAMPLE_TIME_MS (1000 / ESC_SAMPLE_RATE_HZ)

typedef enum {
    ESC_STATUS_INACTIVE = 0,
    ESC_STATUS_STANDBY,
    ESC_STATUS_ACTIVE,
    ESC_STATUS_FAULT
} esc_status_t;

// esc_sensor_data_t and vehicle_parameters_t are now defined in esc_types.h

typedef struct {
    float reference_yaw_rate;              // rad/s
    float yaw_rate_error;                  // rad/s
    float slip_angle;                      // radians
    float vehicle_beta;                    // vehicle sideslip angle
    bool understeer_detected;
    bool oversteer_detected;
    bool rollover_risk;
} vehicle_dynamics_t;

typedef struct {
    float brake_pressure[MAX_WHEEL_COUNT]; // bar (0-250 typical)
    float engine_torque_reduction;         // % (0-100)
    bool brake_light_active;
    bool esc_warning_light;
} actuator_commands_t;

typedef struct {
    bool sensor_fault[8];                  // Various sensor fault flags
    bool actuator_fault[4];                // Brake actuator faults
    bool system_fault;
    bool calibration_required;
    uint16_t fault_code;
} diagnostic_data_t;

// Forward declaration for plausibility system
struct plausibility_system_t;

typedef struct {
    esc_status_t status;
    esc_sensor_data_t sensors;
    vehicle_dynamics_t dynamics;
    actuator_commands_t actuators;
    diagnostic_data_t diagnostics;
    struct plausibility_system_t* plausibility;   // Pointer to sensor plausibility checking system
    struct brake_actuation_system_t* brake_system; // Pointer to brake actuation system
    struct engine_torque_interface_t* engine_system; // Pointer to engine torque interface
    struct watchdog_supervision_t* watchdog_system; // Pointer to watchdog supervision system
    struct dtc_manager_t* dtc_manager;             // Pointer to DTC management system
    struct post_system_t* post_system;            // Pointer to POST system
    struct esc_logging_system_t* logging_system;  // Pointer to logging system
    bool initialization_complete;                  // POST initialization completion flag
    bool system_ready;                            // Overall system ready flag
    uint32_t cycle_count;
    float dt;                              // Delta time in seconds
} esc_system_t;

// vehicle_parameters_t is now defined in esc_types.h

// Function prototypes
void esc_init(esc_system_t* esc, const vehicle_parameters_t* params);
void esc_update(esc_system_t* esc);
void esc_sensor_acquisition(esc_system_t* esc);
void esc_vehicle_dynamics_calculation(esc_system_t* esc, const vehicle_parameters_t* params);
void esc_stability_control(esc_system_t* esc, const vehicle_parameters_t* params);
void esc_actuator_control(esc_system_t* esc);
void esc_diagnostics(esc_system_t* esc);
bool esc_self_test(esc_system_t* esc);

// Sensor interface integration functions
bool esc_configure_sensors(esc_system_t* esc);
void esc_sensor_callback(sensor_type_t sensor_id, const sensor_data_t* data);

// Brake actuation integration functions
bool esc_configure_brake_system(esc_system_t* esc);
bool esc_execute_brake_commands(esc_system_t* esc);

// Engine torque integration functions
bool esc_configure_engine_system(esc_system_t* esc);
bool esc_execute_engine_commands(esc_system_t* esc);

// Watchdog supervision integration functions
bool esc_configure_watchdog_system(esc_system_t* esc);
void esc_update_watchdog_monitoring(esc_system_t* esc);
bool esc_handle_watchdog_safe_state(esc_system_t* esc);

// DTC management integration functions
bool esc_configure_dtc_system(esc_system_t* esc);
void esc_update_dtc_monitoring(esc_system_t* esc);
bool esc_process_uds_request(esc_system_t* esc, const uint8_t* request, uint16_t request_length, 
                            uint8_t* response, uint16_t* response_length);
void esc_capture_freeze_frame_data(esc_system_t* esc, uint32_t dtc_code);

// POST system integration functions
bool esc_configure_post_system(esc_system_t* esc);
bool esc_execute_post_sequence(esc_system_t* esc);
bool esc_check_system_readiness(esc_system_t* esc);
void esc_handle_post_failure(esc_system_t* esc, const char* failure_reason);
bool esc_validate_initialization_complete(esc_system_t* esc);

// Logging system integration functions
bool esc_configure_logging_system(esc_system_t* esc);
void esc_log_intervention(esc_system_t* esc, const char* intervention_type);
void esc_log_system_anomaly(esc_system_t* esc, const char* anomaly_description);
void esc_update_logging(esc_system_t* esc);

// Utility functions
float esc_calculate_vehicle_speed(const float wheel_speeds[MAX_WHEEL_COUNT]);
float esc_calculate_reference_yaw_rate(float vehicle_speed, float steering_angle, float wheelbase);
float esc_low_pass_filter(float current_value, float new_value, float alpha);
float esc_limit_value(float value, float min_val, float max_val);

#endif // ESC_ECU_H