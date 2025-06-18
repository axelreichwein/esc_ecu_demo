#include "engine_torque_interface.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

// Constants for engine torque interface
#define ENGINE_REQUEST_TIMEOUT_MS 200      // Default timeout for engine response
#define ENGINE_HEARTBEAT_TIMEOUT_MS 1000   // Timeout for ECU heartbeat
#define ENGINE_MAX_RETRY_ATTEMPTS 3        // Maximum retry attempts
#define ENGINE_RETRY_INTERVAL_MS 100       // Interval between retries
#define ENGINE_MAX_REDUCTION_PERCENT 80.0f // Maximum allowed torque reduction
#define ENGINE_MIN_REDUCTION_PERCENT 0.0f  // Minimum torque reduction
#define ENGINE_CAN_DATA_LENGTH 8           // Standard CAN message length
#define REQUESTER_ID_ESC 0x02              // ESC system ID

// Global CAN interface
static const engine_can_interface_t* g_can_interface = NULL;

// Forward declarations for internal functions
static void* engine_monitoring_thread(void* arg);
static bool engine_validate_ecu_response(const engine_torque_interface_t* system, 
                                        const engine_torque_response_t* response);
static void engine_handle_request_timeout(engine_torque_interface_t* system);
static void engine_handle_ecu_fault(engine_torque_interface_t* system);

// System initialization and configuration
bool engine_torque_init(engine_torque_interface_t* system, const engine_can_interface_t* can_interface) {
    if (!system || !can_interface) {
        return false;
    }
    
    memset(system, 0, sizeof(engine_torque_interface_t));
    g_can_interface = can_interface;
    
    // Initialize system state
    system->communication_enabled = false;
    system->system_initialized = false;
    system->sequence_counter = 1;
    system->request_pending = false;
    system->fallback_active = false;
    system->monitoring_active = false;
    
    // Default CAN IDs (automotive standard)
    system->can_id_request = 0x700;    // ESC -> Engine torque request
    system->can_id_response = 0x701;   // Engine -> ESC torque response
    system->can_id_heartbeat = 0x702;  // Engine ECU heartbeat
    
    // Initialize default fallback configuration
    system->fallback_config = engine_get_default_fallback_config();
    
    // Initialize ECU status
    system->ecu_status.operating_mode = ENGINE_MODE_NORMAL;
    system->ecu_status.torque_reduction_capable = true;
    system->ecu_status.communication_active = false;
    
    // Initialize mutex for thread safety
    if (pthread_mutex_init(&system->request_mutex, NULL) != 0) {
        printf("Failed to initialize engine torque request mutex\\n");
        return false;
    }
    
    // Initialize CAN interface
    if (g_can_interface->can_init) {
        if (!g_can_interface->can_init(500000)) { // 500 kbps CAN
            printf("Failed to initialize CAN interface for engine torque\\n");
            pthread_mutex_destroy(&system->request_mutex);
            return false;
        }
    }
    
    // Set up CAN filters for engine messages
    if (g_can_interface->can_set_filter) {
        g_can_interface->can_set_filter(system->can_id_response, 0x7FF);
        g_can_interface->can_set_filter(system->can_id_heartbeat, 0x7FF);
    }
    
    // Start monitoring thread
    system->monitoring_active = true;
    if (pthread_create(&system->monitoring_thread, NULL, engine_monitoring_thread, system) != 0) {
        printf("Failed to create engine monitoring thread\\n");
        system->monitoring_active = false;
        pthread_mutex_destroy(&system->request_mutex);
        return false;
    }
    
    system->communication_enabled = true;
    system->system_initialized = true;
    
    printf("Engine torque interface initialized successfully\\n");
    return true;
}

void engine_torque_shutdown(engine_torque_interface_t* system) {
    if (!system) return;
    
    // Stop monitoring thread
    if (system->monitoring_active) {
        system->monitoring_active = false;
        pthread_join(system->monitoring_thread, NULL);
    }
    
    // Cancel any pending torque reduction
    engine_cancel_torque_reduction(system);
    
    // Deactivate fallback if active
    if (system->fallback_active) {
        engine_deactivate_fallback(system);
    }
    
    // Destroy mutex
    pthread_mutex_destroy(&system->request_mutex);
    
    memset(system, 0, sizeof(engine_torque_interface_t));
    printf("Engine torque interface shutdown complete\\n");
}

bool engine_torque_configure_can_ids(engine_torque_interface_t* system, 
                                     uint32_t request_id, uint32_t response_id, uint32_t heartbeat_id) {
    if (!system) return false;
    
    pthread_mutex_lock(&system->request_mutex);
    
    system->can_id_request = request_id;
    system->can_id_response = response_id;
    system->can_id_heartbeat = heartbeat_id;
    
    // Update CAN filters
    if (g_can_interface && g_can_interface->can_set_filter) {
        g_can_interface->can_set_filter(response_id, 0x7FF);
        g_can_interface->can_set_filter(heartbeat_id, 0x7FF);
    }
    
    pthread_mutex_unlock(&system->request_mutex);
    
    printf("Engine CAN IDs configured: Request=0x%03X, Response=0x%03X, Heartbeat=0x%03X\\n",
           request_id, response_id, heartbeat_id);
    return true;
}

bool engine_torque_configure_fallback(engine_torque_interface_t* system, 
                                      const engine_fallback_config_t* config) {
    if (!system || !config) return false;
    
    pthread_mutex_lock(&system->request_mutex);
    memcpy(&system->fallback_config, config, sizeof(engine_fallback_config_t));
    pthread_mutex_unlock(&system->request_mutex);
    
    printf("Engine fallback configuration updated\\n");
    return true;
}

// Torque reduction requests
bool engine_request_torque_reduction(engine_torque_interface_t* system, 
                                    float reduction_percent, uint16_t duration_ms, uint8_t priority) {
    if (!system || !system->system_initialized) {
        return false;
    }
    
    if (!engine_validate_request_parameters(reduction_percent, duration_ms, priority)) {
        printf("Invalid torque reduction parameters\\n");
        return false;
    }
    
    pthread_mutex_lock(&system->request_mutex);
    
    // Check if ECU is capable of torque reduction
    if (!system->ecu_status.torque_reduction_capable) {
        printf("Engine ECU does not support torque reduction\\n");
        pthread_mutex_unlock(&system->request_mutex);
        
        // Activate fallback if enabled
        if (system->fallback_config.enable_fallback) {
            float brake_compensation = engine_calculate_brake_compensation(reduction_percent);
            engine_activate_fallback(system, brake_compensation);
        }
        return false;
    }
    
    // Prepare torque reduction request
    engine_torque_request_t request = {
        .torque_reduction_percent = reduction_percent,
        .torque_reduction_rate = 100.0f, // Immediate reduction
        .duration_ms = duration_ms,
        .priority = priority,
        .requester_id = REQUESTER_ID_ESC,
        .gradual_reduction = false,
        .request_timestamp_ms = g_can_interface->get_timestamp_ms(),
        .sequence_number = system->sequence_counter++
    };
    
    // Store current request
    memcpy(&system->current_request, &request, sizeof(engine_torque_request_t));
    system->request_pending = true;
    system->retry_count = 0;
    system->last_request_time_ms = request.request_timestamp_ms;
    
    pthread_mutex_unlock(&system->request_mutex);
    
    // Send the request
    bool success = engine_send_torque_request(system, &request);
    if (success) {
        system->requests_sent++;
        printf("Torque reduction request sent: %.1f%% for %d ms\\n", 
               reduction_percent, duration_ms);
    } else {
        printf("Failed to send torque reduction request\\n");
        pthread_mutex_lock(&system->request_mutex);
        system->request_pending = false;
        pthread_mutex_unlock(&system->request_mutex);
    }
    
    return success;
}

bool engine_request_gradual_torque_reduction(engine_torque_interface_t* system,
                                            float reduction_percent, float rate_percent_per_sec, 
                                            uint16_t duration_ms, uint8_t priority) {
    if (!system || !system->system_initialized) {
        return false;
    }
    
    if (!engine_validate_request_parameters(reduction_percent, duration_ms, priority)) {
        return false;
    }
    
    pthread_mutex_lock(&system->request_mutex);
    
    engine_torque_request_t request = {
        .torque_reduction_percent = reduction_percent,
        .torque_reduction_rate = rate_percent_per_sec,
        .duration_ms = duration_ms,
        .priority = priority,
        .requester_id = REQUESTER_ID_ESC,
        .gradual_reduction = true,
        .request_timestamp_ms = g_can_interface->get_timestamp_ms(),
        .sequence_number = system->sequence_counter++
    };
    
    memcpy(&system->current_request, &request, sizeof(engine_torque_request_t));
    system->request_pending = true;
    system->retry_count = 0;
    system->last_request_time_ms = request.request_timestamp_ms;
    
    pthread_mutex_unlock(&system->request_mutex);
    
    bool success = engine_send_torque_request(system, &request);
    if (success) {
        system->requests_sent++;
        printf("Gradual torque reduction request sent: %.1f%% at %.1f%%/s for %d ms\\n", 
               reduction_percent, rate_percent_per_sec, duration_ms);
    }
    
    return success;
}

bool engine_cancel_torque_reduction(engine_torque_interface_t* system) {
    if (!system || !system->system_initialized) {
        return false;
    }
    
    // Send cancellation request (0% reduction with immediate effect)
    bool success = engine_request_torque_reduction(system, 0.0f, 0, 7); // Highest priority
    
    if (success) {
        printf("Torque reduction cancellation sent\\n");
    }
    
    return success;
}

bool engine_modify_torque_reduction(engine_torque_interface_t* system, float new_reduction_percent) {
    if (!system || !system->system_initialized) {
        return false;
    }
    
    // Use the same duration and priority as current request
    uint16_t duration = system->current_request.duration_ms;
    uint8_t priority = system->current_request.priority;
    
    return engine_request_torque_reduction(system, new_reduction_percent, duration, priority);
}

// Status and monitoring
engine_request_status_t engine_get_request_status(const engine_torque_interface_t* system) {
    if (!system) {
        return ENGINE_REQUEST_ECU_FAULT;
    }
    
    if (engine_is_request_timeout(system)) {
        return ENGINE_REQUEST_TIMEOUT;
    }
    
    return system->last_response.status;
}

engine_torque_response_t engine_get_last_response(const engine_torque_interface_t* system) {
    engine_torque_response_t response = {0};
    if (!system) {
        return response;
    }
    return system->last_response;
}

engine_ecu_status_t engine_get_ecu_status(const engine_torque_interface_t* system) {
    engine_ecu_status_t status = {0};
    if (!system) {
        return status;
    }
    return system->ecu_status;
}

float engine_get_actual_reduction(const engine_torque_interface_t* system) {
    if (!system) return 0.0f;
    return system->last_response.actual_reduction_percent;
}

bool engine_is_reduction_active(const engine_torque_interface_t* system) {
    if (!system) return false;
    return system->last_response.reduction_active;
}

// Communication and protocol
void engine_update_communication(engine_torque_interface_t* system) {
    if (!system || !system->system_initialized) {
        return;
    }
    
    // Process incoming CAN messages
    engine_process_can_messages(system);
    
    // Check for request timeout
    if (system->request_pending && engine_is_request_timeout(system)) {
        engine_handle_request_timeout(system);
    }
    
    // Check for ECU heartbeat timeout
    uint32_t current_time = g_can_interface->get_timestamp_ms();
    if (current_time - system->ecu_status.last_heartbeat_ms > ENGINE_HEARTBEAT_TIMEOUT_MS) {
        system->ecu_status.communication_active = false;
        engine_handle_ecu_fault(system);
    }
    
    // Update fallback if active
    if (system->fallback_active) {
        engine_update_fallback(system);
    }
}

bool engine_process_can_messages(engine_torque_interface_t* system) {
    if (!system || !g_can_interface || !g_can_interface->can_receive_message) {
        return false;
    }
    
    uint32_t can_id;
    uint8_t data[ENGINE_CAN_DATA_LENGTH];
    uint8_t length;
    bool message_processed = false;
    
    // Process all available messages
    while (g_can_interface->can_receive_message(&can_id, data, &length)) {
        if (can_id == system->can_id_response && length == ENGINE_CAN_DATA_LENGTH) {
            // Process torque response message
            engine_torque_response_t response;
            if (engine_parse_torque_response(data, length, &response)) {
                pthread_mutex_lock(&system->request_mutex);
                
                // Validate response matches pending request
                if (engine_validate_ecu_response(system, &response)) {
                    memcpy(&system->last_response, &response, sizeof(engine_torque_response_t));
                    system->last_response_time_ms = g_can_interface->get_timestamp_ms();
                    system->request_pending = false;
                    system->retry_count = 0;
                    system->responses_received++;
                    
                    // Handle response based on status
                    switch (response.status) {
                        case ENGINE_REQUEST_ACCEPTED:
                            printf("Engine torque reduction accepted: %.1f%% applied\\n", 
                                   response.actual_reduction_percent);
                            // Deactivate fallback if it was active
                            if (system->fallback_active) {
                                engine_deactivate_fallback(system);
                            }
                            break;
                            
                        case ENGINE_REQUEST_DENIED:
                            printf("Engine torque reduction denied (code: %s)\\n", 
                                   engine_response_code_to_string(response.response_code));
                            system->denials++;
                            
                            // Activate fallback for denied requests
                            if (system->fallback_config.enable_fallback) {
                                float brake_compensation = engine_calculate_brake_compensation(
                                    system->current_request.torque_reduction_percent);
                                engine_activate_fallback(system, brake_compensation);
                            }
                            break;
                            
                        default:
                            break;
                    }
                    
                    message_processed = true;
                }
                
                pthread_mutex_unlock(&system->request_mutex);
            }
        }
        else if (can_id == system->can_id_heartbeat && length == ENGINE_CAN_DATA_LENGTH) {
            // Process ECU heartbeat message
            engine_ecu_status_t status;
            if (engine_parse_ecu_heartbeat(data, length, &status)) {
                pthread_mutex_lock(&system->request_mutex);
                memcpy(&system->ecu_status, &status, sizeof(engine_ecu_status_t));
                system->ecu_status.last_heartbeat_ms = g_can_interface->get_timestamp_ms();
                system->ecu_status.communication_active = true;
                pthread_mutex_unlock(&system->request_mutex);
                message_processed = true;
            }
        }
    }
    
    return message_processed;
}

bool engine_send_torque_request(engine_torque_interface_t* system, const engine_torque_request_t* request) {
    if (!system || !request || !g_can_interface || !g_can_interface->can_send_message) {
        return false;
    }
    
    uint8_t can_data[ENGINE_CAN_DATA_LENGTH];
    engine_encode_torque_request(request, can_data);
    
    bool success = g_can_interface->can_send_message(system->can_id_request, can_data, ENGINE_CAN_DATA_LENGTH);
    
    if (success) {
        engine_log_request(request);
    }
    
    return success;
}

bool engine_parse_torque_response(const uint8_t* can_data, uint8_t length, engine_torque_response_t* response) {
    if (!can_data || !response || length != ENGINE_CAN_DATA_LENGTH) {
        return false;
    }
    
    // Decode torque response message format:
    // Byte 0: Status and response code
    // Bytes 1-2: Actual reduction percentage (scaled)
    // Bytes 3-4: Maximum available reduction (scaled)
    // Bytes 5-6: Estimated duration (ms)
    // Byte 7: Checksum
    
    uint8_t checksum = engine_calculate_checksum(can_data, 7);
    if (checksum != can_data[7]) {
        return false; // Invalid checksum
    }
    
    response->status = (engine_request_status_t)(can_data[0] & 0x0F);
    response->response_code = (engine_response_code_t)((can_data[0] >> 4) & 0x0F);
    
    uint16_t reduction_scaled = (uint16_t)(can_data[1] | (can_data[2] << 8));
    response->actual_reduction_percent = (float)reduction_scaled / 100.0f;
    
    uint16_t max_reduction_scaled = (uint16_t)(can_data[3] | (can_data[4] << 8));
    response->max_available_reduction = (float)max_reduction_scaled / 100.0f;
    
    response->estimated_duration_ms = (uint16_t)(can_data[5] | (can_data[6] << 8));
    response->reduction_active = (response->actual_reduction_percent > 0.1f);
    response->response_timestamp_ms = g_can_interface->get_timestamp_ms();
    
    return true;
}

bool engine_parse_ecu_heartbeat(const uint8_t* can_data, uint8_t length, engine_ecu_status_t* status) {
    if (!can_data || !status || length != ENGINE_CAN_DATA_LENGTH) {
        return false;
    }
    
    // Decode ECU heartbeat message format:
    // Byte 0: Operating mode and capabilities
    // Bytes 1-2: Current RPM (scaled)
    // Byte 3: Throttle position (%)
    // Byte 4: Fault code
    // Bytes 5-6: Current torque (scaled, Nm)
    // Byte 7: Checksum
    
    uint8_t checksum = engine_calculate_checksum(can_data, 7);
    if (checksum != can_data[7]) {
        return false;
    }
    
    status->operating_mode = (engine_operating_mode_t)(can_data[0] & 0x07);
    status->torque_reduction_capable = (can_data[0] & 0x08) != 0;
    
    status->current_rpm = (float)((can_data[1] | (can_data[2] << 8)) * 10); // Scale factor 10
    status->throttle_position_percent = (float)can_data[3];
    status->ecu_fault_code = can_data[4];
    
    uint16_t torque_scaled = (uint16_t)(can_data[5] | (can_data[6] << 8));
    status->current_torque_nm = (float)torque_scaled / 10.0f; // Scale factor 0.1
    
    return true;
}

// Fallback handling
bool engine_activate_fallback(engine_torque_interface_t* system, float target_brake_pressure) {
    if (!system || !system->fallback_config.enable_fallback) {
        return false;
    }
    
    pthread_mutex_lock(&system->request_mutex);
    
    system->fallback_active = true;
    system->fallback_brake_pressure = fminf(target_brake_pressure, 
                                           system->fallback_config.max_brake_compensation);
    system->fallback_start_time_ms = g_can_interface->get_timestamp_ms();
    system->fallback_activations++;
    
    pthread_mutex_unlock(&system->request_mutex);
    
    printf("Engine torque fallback activated: %.1f bar brake compensation\\n", 
           system->fallback_brake_pressure);
    
    return true;
}

void engine_update_fallback(engine_torque_interface_t* system) {
    if (!system || !system->fallback_active) {
        return;
    }
    
    // Gradual application of brake compensation using ramp rate
    uint32_t current_time = g_can_interface->get_timestamp_ms();
    float elapsed_time_s = (float)(current_time - system->fallback_start_time_ms) / 1000.0f;
    float max_pressure = system->fallback_config.brake_ramp_rate * elapsed_time_s;
    
    if (max_pressure >= system->fallback_brake_pressure) {
        // Target pressure reached
        printf("Fallback brake pressure target reached: %.1f bar\\n", 
               system->fallback_brake_pressure);
    }
    
    // In a real implementation, this would command the brake system
    // to apply the calculated compensation pressure
}

bool engine_deactivate_fallback(engine_torque_interface_t* system) {
    if (!system || !system->fallback_active) {
        return false;
    }
    
    pthread_mutex_lock(&system->request_mutex);
    system->fallback_active = false;
    system->fallback_brake_pressure = 0.0f;
    pthread_mutex_unlock(&system->request_mutex);
    
    printf("Engine torque fallback deactivated\\n");
    return true;
}

float engine_calculate_brake_compensation(float torque_reduction_percent) {
    // Simple conversion from torque reduction to brake pressure
    // In reality, this would involve complex vehicle dynamics calculations
    float compensation_factor = 1.5f; // bar per % torque reduction
    return torque_reduction_percent * compensation_factor;
}

// CAN message encoding/decoding
void engine_encode_torque_request(const engine_torque_request_t* request, uint8_t* can_data) {
    if (!request || !can_data) return;
    
    // Encode torque request message format:
    // Byte 0: Requester ID and priority
    // Bytes 1-2: Torque reduction percentage (scaled to 0-10000 for 0-100.00%)
    // Bytes 3-4: Duration (ms)
    // Byte 5: Rate and flags
    // Byte 6: Sequence number
    // Byte 7: Checksum
    
    can_data[0] = (request->requester_id & 0x0F) | ((request->priority & 0x07) << 4);
    
    uint16_t reduction_scaled = (uint16_t)(request->torque_reduction_percent * 100.0f);
    can_data[1] = (uint8_t)(reduction_scaled & 0xFF);
    can_data[2] = (uint8_t)((reduction_scaled >> 8) & 0xFF);
    
    can_data[3] = (uint8_t)(request->duration_ms & 0xFF);
    can_data[4] = (uint8_t)((request->duration_ms >> 8) & 0xFF);
    
    uint8_t rate_scaled = (uint8_t)fminf(request->torque_reduction_rate, 255.0f);
    can_data[5] = rate_scaled | (request->gradual_reduction ? 0x80 : 0x00);
    
    can_data[6] = (uint8_t)(request->sequence_number & 0xFF);
    
    can_data[7] = engine_calculate_checksum(can_data, 7);
}

void engine_encode_heartbeat_request(uint8_t* can_data) {
    if (!can_data) return;
    
    // Simple heartbeat request from ESC to engine ECU
    memset(can_data, 0, ENGINE_CAN_DATA_LENGTH);
    can_data[0] = REQUESTER_ID_ESC;
    can_data[1] = 0x01; // Heartbeat request
    can_data[7] = engine_calculate_checksum(can_data, 7);
}

bool engine_decode_torque_response(const uint8_t* can_data, engine_torque_response_t* response) {
    return engine_parse_torque_response(can_data, ENGINE_CAN_DATA_LENGTH, response);
}

bool engine_decode_ecu_heartbeat(const uint8_t* can_data, engine_ecu_status_t* status) {
    return engine_parse_ecu_heartbeat(can_data, ENGINE_CAN_DATA_LENGTH, status);
}

// Utility functions
bool engine_is_request_timeout(const engine_torque_interface_t* system) {
    if (!system || !system->request_pending) {
        return false;
    }
    
    uint32_t current_time = g_can_interface->get_timestamp_ms();
    uint32_t elapsed_time = current_time - system->last_request_time_ms;
    
    return elapsed_time > system->fallback_config.engine_timeout_ms;
}

bool engine_should_retry_request(const engine_torque_interface_t* system) {
    if (!system || !system->request_pending) {
        return false;
    }
    
    return system->retry_count < system->fallback_config.max_retry_attempts;
}

uint8_t engine_calculate_checksum(const uint8_t* data, uint8_t length) {
    if (!data) return 0;
    
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool engine_validate_request_parameters(float reduction_percent, uint16_t duration_ms, uint8_t priority) {
    if (reduction_percent < ENGINE_MIN_REDUCTION_PERCENT || 
        reduction_percent > ENGINE_MAX_REDUCTION_PERCENT) {
        return false;
    }
    
    if (priority > 7) {
        return false;
    }
    
    // Duration can be 0 (indefinite) or any positive value
    (void)duration_ms; // Suppress unused parameter warning
    
    return true;
}

// Internal helper functions
static void* engine_monitoring_thread(void* arg) {
    engine_torque_interface_t* system = (engine_torque_interface_t*)arg;
    
    while (system->monitoring_active) {
        engine_update_communication(system);
        usleep(50000); // 50ms monitoring interval
    }
    
    return NULL;
}

static bool engine_validate_ecu_response(const engine_torque_interface_t* system, 
                                        const engine_torque_response_t* response) {
    if (!system || !response) return false;
    
    // Validate response sequence number matches current request
    return response->sequence_number == system->current_request.sequence_number;
}

static void engine_handle_request_timeout(engine_torque_interface_t* system) {
    if (!system) return;
    
    printf("Engine torque request timeout\\n");
    system->timeouts++;
    
    // Retry if possible
    if (engine_should_retry_request(system)) {
        system->retry_count++;
        printf("Retrying engine torque request (attempt %d/%d)\\n", 
               system->retry_count, system->fallback_config.max_retry_attempts);
        
        // Resend the request
        engine_send_torque_request(system, &system->current_request);
        system->last_request_time_ms = g_can_interface->get_timestamp_ms();
    } else {
        // Maximum retries exceeded, activate fallback
        printf("Maximum retry attempts exceeded, activating fallback\\n");
        
        pthread_mutex_lock(&system->request_mutex);
        system->request_pending = false;
        pthread_mutex_unlock(&system->request_mutex);
        
        if (system->fallback_config.enable_fallback) {
            float brake_compensation = engine_calculate_brake_compensation(
                system->current_request.torque_reduction_percent);
            engine_activate_fallback(system, brake_compensation);
        }
    }
}

static void engine_handle_ecu_fault(engine_torque_interface_t* system) {
    if (!system) return;
    
    printf("Engine ECU communication fault detected\\n");
    
    // Set ECU to fault mode
    system->ecu_status.operating_mode = ENGINE_MODE_FAULT;
    system->ecu_status.torque_reduction_capable = false;
    
    // Activate fallback if there's a pending request
    if (system->request_pending && system->fallback_config.enable_fallback) {
        float brake_compensation = engine_calculate_brake_compensation(
            system->current_request.torque_reduction_percent);
        engine_activate_fallback(system, brake_compensation);
        
        pthread_mutex_lock(&system->request_mutex);
        system->request_pending = false;
        pthread_mutex_unlock(&system->request_mutex);
    }
    
    // Force limp mode if configured
    if (system->fallback_config.force_limp_mode) {
        printf("Forcing vehicle to limp mode due to engine ECU fault\\n");
        // In a real implementation, this would command the vehicle to limp mode
    }
}

// Diagnostic and monitoring functions
void engine_print_system_status(const engine_torque_interface_t* system) {
    if (!system) return;
    
    printf("\\n=== Engine Torque Interface Status ===\\n");
    printf("System Initialized: %s\\n", system->system_initialized ? "YES" : "NO");
    printf("Communication Active: %s\\n", system->ecu_status.communication_active ? "YES" : "NO");
    printf("ECU Operating Mode: %s\\n", engine_operating_mode_to_string(system->ecu_status.operating_mode));
    printf("Torque Reduction Capable: %s\\n", system->ecu_status.torque_reduction_capable ? "YES" : "NO");
    printf("Request Pending: %s\\n", system->request_pending ? "YES" : "NO");
    printf("Fallback Active: %s\\n", system->fallback_active ? "YES" : "NO");
    
    if (system->last_response.reduction_active) {
        printf("Current Torque Reduction: %.1f%%\\n", system->last_response.actual_reduction_percent);
    }
    
    if (system->fallback_active) {
        printf("Fallback Brake Pressure: %.1f bar\\n", system->fallback_brake_pressure);
    }
    
    printf("=====================================\\n");
}

void engine_print_statistics(const engine_torque_interface_t* system) {
    if (!system) return;
    
    printf("\\n=== Engine Torque Interface Statistics ===\\n");
    printf("Requests Sent: %u\\n", system->requests_sent);
    printf("Responses Received: %u\\n", system->responses_received);
    printf("Timeouts: %u\\n", system->timeouts);
    printf("Denials: %u\\n", system->denials);
    printf("Fallback Activations: %u\\n", system->fallback_activations);
    printf("=========================================\\n");
}

void engine_log_request(const engine_torque_request_t* request) {
    if (!request) return;
    
    printf("Engine Torque Request - Seq:%d, Reduction:%.1f%%, Duration:%dms, Priority:%d\\n",
           request->sequence_number, request->torque_reduction_percent, 
           request->duration_ms, request->priority);
}

void engine_log_response(const engine_torque_response_t* response) {
    if (!response) return;
    
    printf("Engine Torque Response - Seq:%d, Status:%s, Actual:%.1f%%\\n",
           response->sequence_number, engine_request_status_to_string(response->status),
           response->actual_reduction_percent);
}

const char* engine_request_status_to_string(engine_request_status_t status) {
    switch (status) {
        case ENGINE_REQUEST_PENDING: return "PENDING";
        case ENGINE_REQUEST_ACCEPTED: return "ACCEPTED";
        case ENGINE_REQUEST_DENIED: return "DENIED";
        case ENGINE_REQUEST_TIMEOUT: return "TIMEOUT";
        case ENGINE_REQUEST_ECU_FAULT: return "ECU_FAULT";
        default: return "UNKNOWN";
    }
}

const char* engine_response_code_to_string(engine_response_code_t code) {
    switch (code) {
        case ENGINE_RESPONSE_OK: return "OK";
        case ENGINE_RESPONSE_DENIED_SAFETY: return "DENIED_SAFETY";
        case ENGINE_RESPONSE_DENIED_LIMIT: return "DENIED_LIMIT";
        case ENGINE_RESPONSE_DENIED_MODE: return "DENIED_MODE";
        case ENGINE_RESPONSE_TEMPORARY_UNAVAILABLE: return "TEMP_UNAVAILABLE";
        case ENGINE_RESPONSE_PERMANENT_FAULT: return "PERMANENT_FAULT";
        case ENGINE_RESPONSE_INVALID_REQUEST: return "INVALID_REQUEST";
        default: return "UNKNOWN";
    }
}

const char* engine_operating_mode_to_string(engine_operating_mode_t mode) {
    switch (mode) {
        case ENGINE_MODE_NORMAL: return "NORMAL";
        case ENGINE_MODE_LIMP: return "LIMP";
        case ENGINE_MODE_IDLE: return "IDLE";
        case ENGINE_MODE_STARTUP: return "STARTUP";
        case ENGINE_MODE_SHUTDOWN: return "SHUTDOWN";
        case ENGINE_MODE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

// Default configurations
engine_fallback_config_t engine_get_default_fallback_config(void) {
    engine_fallback_config_t config = {
        .enable_fallback = true,
        .max_brake_compensation = 50.0f,     // 50 bar max brake compensation
        .brake_ramp_rate = 25.0f,            // 25 bar/s ramp rate
        .engine_timeout_ms = ENGINE_REQUEST_TIMEOUT_MS,
        .max_retry_attempts = ENGINE_MAX_RETRY_ATTEMPTS,
        .retry_interval_ms = ENGINE_RETRY_INTERVAL_MS,
        .force_limp_mode = false
    };
    
    return config;
}

engine_torque_request_t engine_get_default_request(void) {
    engine_torque_request_t request = {
        .torque_reduction_percent = 0.0f,
        .torque_reduction_rate = 100.0f,
        .duration_ms = 0,
        .priority = 5,
        .requester_id = REQUESTER_ID_ESC,
        .gradual_reduction = false,
        .request_timestamp_ms = 0,
        .sequence_number = 0
    };
    
    return request;
}