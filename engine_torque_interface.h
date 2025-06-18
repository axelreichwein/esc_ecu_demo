#ifndef ENGINE_TORQUE_INTERFACE_H
#define ENGINE_TORQUE_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "esc_types.h"

// Engine torque reduction request status
typedef enum {
    ENGINE_REQUEST_PENDING = 0,
    ENGINE_REQUEST_ACCEPTED = 1,
    ENGINE_REQUEST_DENIED = 2,
    ENGINE_REQUEST_TIMEOUT = 3,
    ENGINE_REQUEST_ECU_FAULT = 4
} engine_request_status_t;

// Engine ECU response codes
typedef enum {
    ENGINE_RESPONSE_OK = 0x00,
    ENGINE_RESPONSE_DENIED_SAFETY = 0x01,
    ENGINE_RESPONSE_DENIED_LIMIT = 0x02,
    ENGINE_RESPONSE_DENIED_MODE = 0x03,
    ENGINE_RESPONSE_TEMPORARY_UNAVAILABLE = 0x04,
    ENGINE_RESPONSE_PERMANENT_FAULT = 0x05,
    ENGINE_RESPONSE_INVALID_REQUEST = 0x06
} engine_response_code_t;

// Engine operating modes that affect torque reduction capability
typedef enum {
    ENGINE_MODE_NORMAL = 0,
    ENGINE_MODE_LIMP = 1,
    ENGINE_MODE_IDLE = 2,
    ENGINE_MODE_STARTUP = 3,
    ENGINE_MODE_SHUTDOWN = 4,
    ENGINE_MODE_FAULT = 5
} engine_operating_mode_t;

// Engine torque reduction request
typedef struct {
    float torque_reduction_percent;      // Percentage reduction (0-100%)
    float torque_reduction_rate;         // Rate of reduction (% per second)
    uint16_t duration_ms;                // Duration of reduction (0 = until cancelled)
    uint8_t priority;                    // Request priority (0-7, 7 = highest)
    uint8_t requester_id;                // ID of requesting system (ESC = 0x02)
    bool gradual_reduction;              // True for gradual, false for immediate
    uint32_t request_timestamp_ms;       // Request generation timestamp
    uint16_t sequence_number;            // Request sequence number
} engine_torque_request_t;

// Engine torque response
typedef struct {
    engine_request_status_t status;      // Response status
    engine_response_code_t response_code; // Detailed response code
    float actual_reduction_percent;      // Actual torque reduction applied
    float max_available_reduction;       // Maximum reduction currently available
    uint16_t estimated_duration_ms;      // Estimated duration ECU can maintain reduction
    uint32_t response_timestamp_ms;      // Response timestamp
    uint16_t sequence_number;            // Matching request sequence number
    bool reduction_active;               // True if reduction is currently active
} engine_torque_response_t;

// Engine ECU status information
typedef struct {
    engine_operating_mode_t operating_mode;
    float current_torque_nm;             // Current engine torque (Nm)
    float max_torque_nm;                 // Maximum available torque (Nm)
    float current_rpm;                   // Current engine RPM
    float throttle_position_percent;     // Throttle position (%)
    bool torque_reduction_capable;       // ECU supports torque reduction
    bool communication_active;           // CAN communication active
    uint32_t last_heartbeat_ms;          // Last heartbeat timestamp
    uint8_t ecu_fault_code;              // ECU fault code (0 = no fault)
} engine_ecu_status_t;

// Fallback strategy configuration
typedef struct {
    bool enable_fallback;                // Enable fallback when engine doesn't respond
    float max_brake_compensation;        // Max additional brake pressure (bar)
    float brake_ramp_rate;               // Brake pressure ramp rate (bar/s)
    uint16_t engine_timeout_ms;          // Timeout for engine response
    uint8_t max_retry_attempts;          // Maximum retry attempts
    uint16_t retry_interval_ms;          // Interval between retries
    bool force_limp_mode;                // Force vehicle to limp mode on engine fault
} engine_fallback_config_t;

// Engine torque interface system state
typedef struct {
    engine_torque_request_t current_request;
    engine_torque_response_t last_response;
    engine_ecu_status_t ecu_status;
    engine_fallback_config_t fallback_config;
    
    // Communication state
    bool communication_enabled;
    bool system_initialized;
    uint32_t can_id_request;             // CAN ID for sending requests
    uint32_t can_id_response;            // CAN ID for receiving responses
    uint32_t can_id_heartbeat;           // CAN ID for ECU heartbeat
    
    // Request tracking
    uint16_t sequence_counter;
    uint32_t last_request_time_ms;
    uint32_t last_response_time_ms;
    uint8_t retry_count;
    bool request_pending;
    
    // Fallback state
    bool fallback_active;
    float fallback_brake_pressure;
    uint32_t fallback_start_time_ms;
    
    // Statistics
    uint32_t requests_sent;
    uint32_t responses_received;
    uint32_t timeouts;
    uint32_t denials;
    uint32_t fallback_activations;
    
    // Threading
    pthread_mutex_t request_mutex;
    pthread_t monitoring_thread;
    bool monitoring_active;
} engine_torque_interface_t;

// Hardware interface for CAN communication
typedef struct {
    bool (*can_init)(uint32_t baudrate);
    bool (*can_send_message)(uint32_t can_id, const uint8_t* data, uint8_t length);
    bool (*can_receive_message)(uint32_t* can_id, uint8_t* data, uint8_t* length);
    bool (*can_set_filter)(uint32_t can_id, uint32_t mask);
    uint32_t (*get_timestamp_ms)(void);
} engine_can_interface_t;

// Function prototypes

// System initialization and configuration
bool engine_torque_init(engine_torque_interface_t* system, const engine_can_interface_t* can_interface);
void engine_torque_shutdown(engine_torque_interface_t* system);
bool engine_torque_configure_can_ids(engine_torque_interface_t* system, 
                                     uint32_t request_id, uint32_t response_id, uint32_t heartbeat_id);
bool engine_torque_configure_fallback(engine_torque_interface_t* system, 
                                      const engine_fallback_config_t* config);

// Torque reduction requests
bool engine_request_torque_reduction(engine_torque_interface_t* system, 
                                    float reduction_percent, uint16_t duration_ms, uint8_t priority);
bool engine_request_gradual_torque_reduction(engine_torque_interface_t* system,
                                            float reduction_percent, float rate_percent_per_sec, 
                                            uint16_t duration_ms, uint8_t priority);
bool engine_cancel_torque_reduction(engine_torque_interface_t* system);
bool engine_modify_torque_reduction(engine_torque_interface_t* system, float new_reduction_percent);

// Status and monitoring
engine_request_status_t engine_get_request_status(const engine_torque_interface_t* system);
engine_torque_response_t engine_get_last_response(const engine_torque_interface_t* system);
engine_ecu_status_t engine_get_ecu_status(const engine_torque_interface_t* system);
float engine_get_actual_reduction(const engine_torque_interface_t* system);
bool engine_is_reduction_active(const engine_torque_interface_t* system);

// Communication and protocol
void engine_update_communication(engine_torque_interface_t* system);
bool engine_process_can_messages(engine_torque_interface_t* system);
bool engine_send_torque_request(engine_torque_interface_t* system, const engine_torque_request_t* request);
bool engine_parse_torque_response(const uint8_t* can_data, uint8_t length, engine_torque_response_t* response);
bool engine_parse_ecu_heartbeat(const uint8_t* can_data, uint8_t length, engine_ecu_status_t* status);

// Fallback handling
bool engine_activate_fallback(engine_torque_interface_t* system, float target_brake_pressure);
void engine_update_fallback(engine_torque_interface_t* system);
bool engine_deactivate_fallback(engine_torque_interface_t* system);
float engine_calculate_brake_compensation(float torque_reduction_percent);

// CAN message encoding/decoding
void engine_encode_torque_request(const engine_torque_request_t* request, uint8_t* can_data);
void engine_encode_heartbeat_request(uint8_t* can_data);
bool engine_decode_torque_response(const uint8_t* can_data, engine_torque_response_t* response);
bool engine_decode_ecu_heartbeat(const uint8_t* can_data, engine_ecu_status_t* status);

// Utility functions
bool engine_is_request_timeout(const engine_torque_interface_t* system);
bool engine_should_retry_request(const engine_torque_interface_t* system);
uint8_t engine_calculate_checksum(const uint8_t* data, uint8_t length);
bool engine_validate_request_parameters(float reduction_percent, uint16_t duration_ms, uint8_t priority);

// Diagnostic and monitoring functions
void engine_print_system_status(const engine_torque_interface_t* system);
void engine_print_statistics(const engine_torque_interface_t* system);
void engine_log_request(const engine_torque_request_t* request);
void engine_log_response(const engine_torque_response_t* response);
const char* engine_request_status_to_string(engine_request_status_t status);
const char* engine_response_code_to_string(engine_response_code_t code);
const char* engine_operating_mode_to_string(engine_operating_mode_t mode);

// Default configurations
engine_fallback_config_t engine_get_default_fallback_config(void);
engine_torque_request_t engine_get_default_request(void);

#endif // ENGINE_TORQUE_INTERFACE_H