#include "dtc_management.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Internal constants
#define DTC_STORAGE_MAGIC_NUMBER 0x44544315
#define DTC_STORAGE_VERSION 0x0001
#define DTC_AGING_CYCLE_THRESHOLD 40        // Drive cycles for DTC aging
#define DTC_CONFIRMATION_THRESHOLD 2        // Occurrences needed for confirmation
#define DTC_PENDING_TO_CONFIRMED_CYCLES 2   // Cycles to move from pending to confirmed

// Internal state tracking
static uint32_t g_drive_cycle_count = 0;
static uint32_t g_ignition_cycle_count = 0;
static bool g_emissions_readiness_complete = false;

// DTC Manager Core Functions
bool dtc_manager_init(dtc_manager_t* manager) {
    if (!manager) {
        return false;
    }
    
    // Initialize manager structure
    memset(manager, 0, sizeof(dtc_manager_t));
    
    // Set initial values
    manager->system_start_time = dtc_get_current_timestamp();
    manager->last_clear_timestamp = 0;
    manager->mil_status = false;
    manager->mil_request_count = 0;
    manager->diagnostic_session_active = false;
    manager->current_session_type = 0x01; // Default session
    
    // Initialize all DTC records
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        manager->stored_dtcs[i].dtc_code = 0;
        manager->stored_dtcs[i].status_byte = 0;
        manager->stored_dtcs[i].is_active = false;
        manager->stored_dtcs[i].is_confirmed = false;
        manager->stored_dtcs[i].is_pending = false;
        manager->stored_dtcs[i].occurrence_count = 0;
        manager->stored_dtcs[i].has_freeze_frame = false;
        memset(manager->stored_dtcs[i].description, 0, sizeof(manager->stored_dtcs[i].description));
    }
    
    // Try to load stored DTCs from persistent storage
    if (!dtc_load_from_persistent_storage(manager)) {
        printf("DTC Manager: No persistent storage found, starting fresh\n");
    }
    
    printf("DTC Manager initialized successfully\n");
    return true;
}

void dtc_manager_shutdown(dtc_manager_t* manager) {
    if (!manager) return;
    
    // Save current state to persistent storage
    dtc_save_to_persistent_storage(manager);
    
    printf("DTC Manager: Shutdown complete. Statistics:\n");
    printf("  Total DTCs: %u\n", manager->total_dtc_count);
    printf("  Active DTCs: %u\n", manager->active_dtc_count);
    printf("  Confirmed DTCs: %u\n", manager->confirmed_dtc_count);
    printf("  MIL Requests: %u\n", manager->mil_request_count);
    
    // Clear manager structure
    memset(manager, 0, sizeof(dtc_manager_t));
}

bool dtc_set_code(dtc_manager_t* manager, dtc_code_t code, const char* description) {
    if (!manager || code == 0) {
        return false;
    }
    
    // Check if DTC already exists
    dtc_record_t* existing_record = dtc_get_record(manager, code);
    
    if (existing_record) {
        // Update existing DTC
        existing_record->occurrence_count++;
        existing_record->last_occurrence_timestamp = dtc_get_current_timestamp();
        existing_record->last_occurrence_odometer = dtc_get_current_odometer();
        existing_record->is_active = true;
        
        // Update status byte
        existing_record->status_byte |= DTC_STATUS_TEST_FAILED;
        existing_record->status_byte |= DTC_STATUS_TEST_FAILED_SINCE_CLEAR;
        
        // Check for confirmation
        if (existing_record->occurrence_count >= DTC_CONFIRMATION_THRESHOLD) {
            dtc_confirm_code(manager, code);
        } else {
            dtc_set_pending(manager, code);
        }
        
        printf("DTC %s updated: occurrence #%u\n", dtc_code_to_string(code), existing_record->occurrence_count);
        return true;
    }
    
    // Find empty slot for new DTC
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code == 0) {
            dtc_record_t* record = &manager->stored_dtcs[i];
            
            // Initialize new DTC record
            record->dtc_code = code;
            record->status_byte = DTC_STATUS_TEST_FAILED | DTC_STATUS_TEST_FAILED_SINCE_CLEAR;
            record->severity = DTC_SEVERITY_CHECK_AT_NEXT_HALT; // Default severity
            record->category = DTC_CATEGORY_CONTROL_SYSTEM_FAULT; // Default category
            record->occurrence_count = 1;
            record->first_occurrence_timestamp = dtc_get_current_timestamp();
            record->last_occurrence_timestamp = record->first_occurrence_timestamp;
            record->first_occurrence_odometer = dtc_get_current_odometer();
            record->last_occurrence_odometer = record->first_occurrence_odometer;
            record->is_active = true;
            record->is_pending = true;
            record->is_confirmed = false;
            record->has_freeze_frame = false;
            
            if (description) {
                strncpy(record->description, description, sizeof(record->description) - 1);
                record->description[sizeof(record->description) - 1] = '\0';
            }
            
            // Update counters
            manager->total_dtc_count++;
            manager->active_dtc_count++;
            manager->pending_dtc_count++;
            
            // Set pending status
            dtc_set_pending(manager, code);
            
            printf("DTC %s set: %s\n", dtc_code_to_string(code), 
                   description ? description : "No description");
            
            return true;
        }
    }
    
    printf("DTC Manager: Storage full, cannot store DTC %s\n", dtc_code_to_string(code));
    return false;
}

bool dtc_clear_code(dtc_manager_t* manager, dtc_code_t code) {
    if (!manager || code == 0) {
        return false;
    }
    
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code == code) {
            dtc_record_t* record = &manager->stored_dtcs[i];
            
            // Update counters
            if (record->is_active) {
                manager->active_dtc_count--;
            }
            if (record->is_confirmed) {
                manager->confirmed_dtc_count--;
            }
            if (record->is_pending) {
                manager->pending_dtc_count--;
            }
            manager->total_dtc_count--;
            
            // Clear freeze frame
            dtc_clear_freeze_frame(manager, code);
            
            // Clear the record
            memset(record, 0, sizeof(dtc_record_t));
            
            printf("DTC %s cleared\n", dtc_code_to_string(code));
            return true;
        }
    }
    
    return false; // DTC not found
}

bool dtc_clear_all_codes(dtc_manager_t* manager) {
    if (!manager) {
        return false;
    }
    
    uint16_t cleared_count = 0;
    
    // Clear all stored DTCs
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code != 0) {
            cleared_count++;
            memset(&manager->stored_dtcs[i], 0, sizeof(dtc_record_t));
        }
    }
    
    // Clear all freeze frames
    memset(manager->freeze_frames, 0, sizeof(manager->freeze_frames));
    
    // Reset counters
    manager->active_dtc_count = 0;
    manager->confirmed_dtc_count = 0;
    manager->pending_dtc_count = 0;
    manager->total_dtc_count = 0;
    manager->mil_request_count = 0;
    manager->mil_status = false;
    manager->last_clear_timestamp = dtc_get_current_timestamp();
    
    printf("DTC Manager: Cleared %u DTCs\n", cleared_count);
    return true;
}

bool dtc_update_status(dtc_manager_t* manager, dtc_code_t code, uint8_t status_mask) {
    if (!manager || code == 0) {
        return false;
    }
    
    dtc_record_t* record = dtc_get_record(manager, code);
    if (!record) {
        return false;
    }
    
    record->status_byte = status_mask;
    
    // Update flags based on status byte
    record->is_active = (status_mask & DTC_STATUS_TEST_FAILED) != 0;
    record->is_confirmed = (status_mask & DTC_STATUS_CONFIRMED_DTC) != 0;
    record->is_pending = (status_mask & DTC_STATUS_PENDING_DTC) != 0;
    
    return true;
}

bool dtc_confirm_code(dtc_manager_t* manager, dtc_code_t code) {
    if (!manager || code == 0) {
        return false;
    }
    
    dtc_record_t* record = dtc_get_record(manager, code);
    if (!record) {
        return false;
    }
    
    if (!record->is_confirmed) {
        record->is_confirmed = true;
        record->status_byte |= DTC_STATUS_CONFIRMED_DTC;
        manager->confirmed_dtc_count++;
        
        // Move from pending to confirmed
        if (record->is_pending) {
            record->is_pending = false;
            record->status_byte &= ~DTC_STATUS_PENDING_DTC;
            manager->pending_dtc_count--;
        }
        
        // Request MIL illumination for emissions-related DTCs
        if ((code & 0xF00000) == 0x000000 || (code & 0xF00000) == 0x100000) { // P0xxx or P1xxx codes
            manager->mil_request_count++;
            dtc_update_mil_status(manager);
        }
        
        printf("DTC %s confirmed\n", dtc_code_to_string(code));
    }
    
    return true;
}

bool dtc_set_pending(dtc_manager_t* manager, dtc_code_t code) {
    if (!manager || code == 0) {
        return false;
    }
    
    dtc_record_t* record = dtc_get_record(manager, code);
    if (!record) {
        return false;
    }
    
    if (!record->is_pending && !record->is_confirmed) {
        record->is_pending = true;
        record->status_byte |= DTC_STATUS_PENDING_DTC;
        manager->pending_dtc_count++;
        
        printf("DTC %s set to pending\n", dtc_code_to_string(code));
    }
    
    return true;
}

// Freeze Frame Management
bool dtc_capture_freeze_frame(dtc_manager_t* manager, dtc_code_t code, const uint8_t* data, uint16_t length) {
    if (!manager || code == 0 || !data || length == 0 || length > DTC_MAX_FREEZE_FRAME_SIZE) {
        return false;
    }
    
    dtc_record_t* record = dtc_get_record(manager, code);
    if (!record) {
        return false;
    }
    
    // Copy freeze frame data
    memcpy(record->freeze_frame_data, data, length);
    record->freeze_frame_length = length;
    record->has_freeze_frame = true;
    
    // Also store in freeze frame array for UDS access
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->freeze_frames[i].dtc_code == 0 || manager->freeze_frames[i].dtc_code == code) {
            freeze_frame_t* frame = &manager->freeze_frames[i];
            frame->dtc_code = code;
            frame->frame_number = 0; // Single freeze frame per DTC for now
            frame->timestamp = dtc_get_current_timestamp();
            frame->parameter_count = 0; // Will be parsed from data if needed
            break;
        }
    }
    
    printf("Freeze frame captured for DTC %s (%u bytes)\n", dtc_code_to_string(code), length);
    return true;
}

bool dtc_get_freeze_frame(const dtc_manager_t* manager, dtc_code_t code, freeze_frame_t* frame) {
    if (!manager || code == 0 || !frame) {
        return false;
    }
    
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->freeze_frames[i].dtc_code == code) {
            memcpy(frame, &manager->freeze_frames[i], sizeof(freeze_frame_t));
            return true;
        }
    }
    
    return false;
}

bool dtc_clear_freeze_frame(dtc_manager_t* manager, dtc_code_t code) {
    if (!manager || code == 0) {
        return false;
    }
    
    // Clear from DTC record
    dtc_record_t* record = dtc_get_record(manager, code);
    if (record) {
        memset(record->freeze_frame_data, 0, sizeof(record->freeze_frame_data));
        record->freeze_frame_length = 0;
        record->has_freeze_frame = false;
    }
    
    // Clear from freeze frame array
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->freeze_frames[i].dtc_code == code) {
            memset(&manager->freeze_frames[i], 0, sizeof(freeze_frame_t));
            break;
        }
    }
    
    return true;
}

// DTC Query and Status Functions
bool dtc_is_code_active(const dtc_manager_t* manager, dtc_code_t code) {
    if (!manager || code == 0) {
        return false;
    }
    
    const dtc_record_t* record = dtc_get_record(manager, code);
    return record ? record->is_active : false;
}

bool dtc_is_code_confirmed(const dtc_manager_t* manager, dtc_code_t code) {
    if (!manager || code == 0) {
        return false;
    }
    
    const dtc_record_t* record = dtc_get_record(manager, code);
    return record ? record->is_confirmed : false;
}

bool dtc_is_code_pending(const dtc_manager_t* manager, dtc_code_t code) {
    if (!manager || code == 0) {
        return false;
    }
    
    const dtc_record_t* record = dtc_get_record(manager, code);
    return record ? record->is_pending : false;
}

dtc_record_t* dtc_get_record(const dtc_manager_t* manager, dtc_code_t code) {
    if (!manager || code == 0) {
        return NULL;
    }
    
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code == code) {
            return (dtc_record_t*)&manager->stored_dtcs[i];
        }
    }
    
    return NULL;
}

uint16_t dtc_get_count_by_status(const dtc_manager_t* manager, uint8_t status_mask) {
    if (!manager) {
        return 0;
    }
    
    uint16_t count = 0;
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code != 0) {
            if ((manager->stored_dtcs[i].status_byte & status_mask) == status_mask) {
                count++;
            }
        }
    }
    
    return count;
}

uint16_t dtc_get_codes_by_status(const dtc_manager_t* manager, uint8_t status_mask, 
                                dtc_code_t* codes, uint16_t max_codes) {
    if (!manager || !codes || max_codes == 0) {
        return 0;
    }
    
    uint16_t count = 0;
    for (int i = 0; i < DTC_MAX_STORED_CODES && count < max_codes; i++) {
        if (manager->stored_dtcs[i].dtc_code != 0) {
            if ((manager->stored_dtcs[i].status_byte & status_mask) == status_mask) {
                codes[count++] = manager->stored_dtcs[i].dtc_code;
            }
        }
    }
    
    return count;
}

// UDS Service Implementation
bool uds_service_read_dtc_information(dtc_manager_t* manager, const uds_request_t* request, uds_response_t* response) {
    if (!manager || !request || !response) {
        return false;
    }
    
    // Initialize response
    memset(response, 0, sizeof(uds_response_t));
    response->service_id = request->service_id + 0x40; // Positive response
    response->is_positive_response = true;
    
    printf("UDS: ReadDTCInformation sub-function 0x%02X\n", request->sub_function);
    
    switch (request->sub_function) {
        case UDS_READ_NUMBER_OF_DTC_BY_STATUS_MASK:
            return uds_read_number_of_dtc_by_status_mask(manager, request->status_mask, response);
            
        case UDS_READ_DTC_BY_STATUS_MASK:
            return uds_read_dtc_by_status_mask(manager, request->status_mask, response);
            
        case UDS_READ_DTC_SNAPSHOT_IDENTIFICATION:
            return uds_read_dtc_snapshot_identification(manager, response);
            
        case UDS_READ_DTC_SNAPSHOT_BY_DTC_NUMBER:
            return uds_read_dtc_snapshot_by_dtc_number(manager, request->dtc_mask, request->record_number, response);
            
        case UDS_READ_DTC_EXTENDED_DATA_BY_DTC_NUMBER:
            return uds_read_dtc_extended_data_by_dtc_number(manager, request->dtc_mask, request->record_number, response);
            
        case UDS_READ_SUPPORTED_DTC:
            // Return supported DTCs
            response->response_data[0] = request->sub_function;
            response->response_length = 1;
            return true;
            
        default:
            // Sub-function not supported
            response->service_id = 0x7F;
            response->response_code = UDS_NRC_SUB_FUNCTION_NOT_SUPPORTED;
            response->response_data[0] = request->service_id;
            response->response_data[1] = response->response_code;
            response->response_length = 2;
            response->is_positive_response = false;
            return false;
    }
}

bool uds_service_clear_dtc_information(dtc_manager_t* manager, const uds_request_t* request, uds_response_t* response) {
    if (!manager || !request || !response) {
        return false;
    }
    
    // Initialize response
    memset(response, 0, sizeof(uds_response_t));
    response->service_id = request->service_id + 0x40; // Positive response
    response->is_positive_response = true;
    
    printf("UDS: ClearDTCInformation\n");
    
    // Check if request length is correct (should be 4 bytes total: SID + 3 bytes DTC mask)
    if (request->request_length < 3) {
        response->service_id = 0x7F;
        response->response_code = UDS_NRC_INCORRECT_MESSAGE_LENGTH;
        response->response_data[0] = request->service_id;
        response->response_data[1] = response->response_code;
        response->response_length = 2;
        response->is_positive_response = false;
        return false;
    }
    
    // Extract DTC mask from request (3 bytes)
    uint32_t dtc_mask = (request->request_data[0] << 16) | 
                       (request->request_data[1] << 8) | 
                       request->request_data[2];
    
    if (dtc_mask == 0xFFFFFF) {
        // Clear all DTCs
        dtc_clear_all_codes(manager);
        printf("UDS: All DTCs cleared\n");
    } else {
        // Clear specific DTC group or specific DTC
        if (dtc_clear_code(manager, dtc_mask)) {
            printf("UDS: DTC %s cleared\n", dtc_code_to_string(dtc_mask));
        } else {
            // DTC not found
            response->service_id = 0x7F;
            response->response_code = UDS_NRC_REQUEST_OUT_OF_RANGE;
            response->response_data[0] = request->service_id;
            response->response_data[1] = response->response_code;
            response->response_length = 2;
            response->is_positive_response = false;
            return false;
        }
    }
    
    // Positive response (just the service ID)
    response->response_length = 0;
    return true;
}

bool uds_process_diagnostic_request(dtc_manager_t* manager, const uds_request_t* request, uds_response_t* response) {
    if (!manager || !request || !response) {
        return false;
    }
    
    printf("UDS: Processing service 0x%02X\n", request->service_id);
    
    switch (request->service_id) {
        case UDS_SERVICE_READ_DTC_INFORMATION:
            return uds_service_read_dtc_information(manager, request, response);
            
        case UDS_SERVICE_CLEAR_DTC_INFORMATION:
            return uds_service_clear_dtc_information(manager, request, response);
            
        default:
            // Service not supported
            memset(response, 0, sizeof(uds_response_t));
            response->service_id = 0x7F;
            response->response_code = UDS_NRC_SERVICE_NOT_SUPPORTED;
            response->response_data[0] = request->service_id;
            response->response_data[1] = response->response_code;
            response->response_length = 2;
            response->is_positive_response = false;
            return false;
    }
}

// UDS Sub-service Functions
bool uds_read_number_of_dtc_by_status_mask(const dtc_manager_t* manager, uint8_t status_mask, uds_response_t* response) {
    if (!manager || !response) {
        return false;
    }
    
    uint16_t count = dtc_get_count_by_status(manager, status_mask);
    
    response->response_data[0] = UDS_READ_NUMBER_OF_DTC_BY_STATUS_MASK;
    response->response_data[1] = 0x00; // Status availability mask (high byte)
    response->response_data[2] = 0xFF; // Status availability mask (low byte)
    response->response_data[3] = 0x01; // DTC format identifier (ISO 15031-6)
    response->response_data[4] = (count >> 8) & 0xFF; // DTC count high byte
    response->response_data[5] = count & 0xFF; // DTC count low byte
    response->response_length = 6;
    
    printf("UDS: Found %u DTCs with status mask 0x%02X\n", count, status_mask);
    return true;
}

bool uds_read_dtc_by_status_mask(const dtc_manager_t* manager, uint8_t status_mask, uds_response_t* response) {
    if (!manager || !response) {
        return false;
    }
    
    response->response_data[0] = UDS_READ_DTC_BY_STATUS_MASK;
    response->response_data[1] = 0x00; // Status availability mask (high byte)
    response->response_data[2] = 0xFF; // Status availability mask (low byte)
    response->response_length = 3;
    
    // Add DTCs that match the status mask
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code != 0) {
            if ((manager->stored_dtcs[i].status_byte & status_mask) == status_mask) {
                if (response->response_length + 4 <= sizeof(response->response_data)) {
                    // Add DTC code (3 bytes) and status byte (1 byte)
                    uint32_t dtc_code = manager->stored_dtcs[i].dtc_code;
                    response->response_data[response->response_length++] = (dtc_code >> 16) & 0xFF;
                    response->response_data[response->response_length++] = (dtc_code >> 8) & 0xFF;
                    response->response_data[response->response_length++] = dtc_code & 0xFF;
                    response->response_data[response->response_length++] = manager->stored_dtcs[i].status_byte;
                }
            }
        }
    }
    
    printf("UDS: Returned DTCs with status mask 0x%02X (response length: %u)\n", 
           status_mask, response->response_length);
    return true;
}

bool uds_read_dtc_snapshot_identification(const dtc_manager_t* manager, uds_response_t* response) {
    if (!manager || !response) {
        return false;
    }
    
    response->response_data[0] = UDS_READ_DTC_SNAPSHOT_IDENTIFICATION;
    response->response_length = 1;
    
    // Add snapshot records for DTCs with freeze frames
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->freeze_frames[i].dtc_code != 0) {
            if (response->response_length + 5 <= sizeof(response->response_data)) {
                uint32_t dtc_code = manager->freeze_frames[i].dtc_code;
                response->response_data[response->response_length++] = (dtc_code >> 16) & 0xFF;
                response->response_data[response->response_length++] = (dtc_code >> 8) & 0xFF;
                response->response_data[response->response_length++] = dtc_code & 0xFF;
                response->response_data[response->response_length++] = manager->freeze_frames[i].frame_number;
                response->response_data[response->response_length++] = 0x01; // Number of snapshot records
            }
        }
    }
    
    printf("UDS: Snapshot identification returned (%u bytes)\n", response->response_length);
    return true;
}

bool uds_read_dtc_snapshot_by_dtc_number(const dtc_manager_t* manager, uint32_t dtc_code, 
                                        uint8_t record_number, uds_response_t* response) {
    if (!manager || !response) {
        return false;
    }
    
    // Find the freeze frame for the specified DTC
    freeze_frame_t frame;
    if (!dtc_get_freeze_frame(manager, dtc_code, &frame)) {
        response->service_id = 0x7F;
        response->response_code = UDS_NRC_REQUEST_OUT_OF_RANGE;
        response->response_data[0] = UDS_SERVICE_READ_DTC_INFORMATION;
        response->response_data[1] = response->response_code;
        response->response_length = 2;
        response->is_positive_response = false;
        return false;
    }
    
    response->response_data[0] = UDS_READ_DTC_SNAPSHOT_BY_DTC_NUMBER;
    response->response_data[1] = (dtc_code >> 16) & 0xFF;
    response->response_data[2] = (dtc_code >> 8) & 0xFF;
    response->response_data[3] = dtc_code & 0xFF;
    response->response_data[4] = record_number;
    response->response_data[5] = 0x01; // Number of identifiers
    response->response_length = 6;
    
    // Add freeze frame data (simplified)
    const dtc_record_t* record = dtc_get_record(manager, dtc_code);
    if (record && record->has_freeze_frame) {
        uint16_t copy_length = (record->freeze_frame_length > 64) ? 64 : record->freeze_frame_length;
        memcpy(&response->response_data[response->response_length], record->freeze_frame_data, copy_length);
        response->response_length += copy_length;
    }
    
    printf("UDS: Snapshot data for DTC %s returned (%u bytes)\n", 
           dtc_code_to_string(dtc_code), response->response_length);
    return true;
}

bool uds_read_dtc_extended_data_by_dtc_number(const dtc_manager_t* manager, uint32_t dtc_code, 
                                             uint8_t record_number, uds_response_t* response) {
    if (!manager || !response) {
        return false;
    }
    
    const dtc_record_t* record = dtc_get_record(manager, dtc_code);
    if (!record) {
        response->service_id = 0x7F;
        response->response_code = UDS_NRC_REQUEST_OUT_OF_RANGE;
        response->response_data[0] = UDS_SERVICE_READ_DTC_INFORMATION;
        response->response_data[1] = response->response_code;
        response->response_length = 2;
        response->is_positive_response = false;
        return false;
    }
    
    response->response_data[0] = UDS_READ_DTC_EXTENDED_DATA_BY_DTC_NUMBER;
    response->response_data[1] = (dtc_code >> 16) & 0xFF;
    response->response_data[2] = (dtc_code >> 8) & 0xFF;
    response->response_data[3] = dtc_code & 0xFF;
    response->response_length = 4;
    
    // Add extended data based on record number
    if (record_number == 0x01 || record_number == 0xFF) {
        // Extended data record 1: Occurrence counter
        response->response_data[response->response_length++] = 0x01; // Record number
        response->response_data[response->response_length++] = (record->occurrence_count >> 24) & 0xFF;
        response->response_data[response->response_length++] = (record->occurrence_count >> 16) & 0xFF;
        response->response_data[response->response_length++] = (record->occurrence_count >> 8) & 0xFF;
        response->response_data[response->response_length++] = record->occurrence_count & 0xFF;
    }
    
    if (record_number == 0x02 || record_number == 0xFF) {
        // Extended data record 2: Timestamps
        response->response_data[response->response_length++] = 0x02; // Record number
        response->response_data[response->response_length++] = (record->first_occurrence_timestamp >> 24) & 0xFF;
        response->response_data[response->response_length++] = (record->first_occurrence_timestamp >> 16) & 0xFF;
        response->response_data[response->response_length++] = (record->first_occurrence_timestamp >> 8) & 0xFF;
        response->response_data[response->response_length++] = record->first_occurrence_timestamp & 0xFF;
        response->response_data[response->response_length++] = (record->last_occurrence_timestamp >> 24) & 0xFF;
        response->response_data[response->response_length++] = (record->last_occurrence_timestamp >> 16) & 0xFF;
        response->response_data[response->response_length++] = (record->last_occurrence_timestamp >> 8) & 0xFF;
        response->response_data[response->response_length++] = record->last_occurrence_timestamp & 0xFF;
    }
    
    printf("UDS: Extended data for DTC %s returned (%u bytes)\n", 
           dtc_code_to_string(dtc_code), response->response_length);
    return true;
}

// MIL Management
bool dtc_update_mil_status(dtc_manager_t* manager) {
    if (!manager) {
        return false;
    }
    
    // MIL should be illuminated if there are confirmed emissions-related DTCs
    bool mil_required = false;
    
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code != 0 && manager->stored_dtcs[i].is_confirmed) {
            // Check if it's an emissions-related DTC (P0xxx or P1xxx)
            uint32_t dtc_code = manager->stored_dtcs[i].dtc_code;
            if ((dtc_code & 0xF00000) == 0x000000 || (dtc_code & 0xF00000) == 0x100000) {
                mil_required = true;
                break;
            }
        }
    }
    
    if (mil_required != manager->mil_status) {
        manager->mil_status = mil_required;
        printf("MIL status changed: %s\n", mil_required ? "ON" : "OFF");
    }
    
    return true;
}

bool dtc_get_mil_status(const dtc_manager_t* manager) {
    return manager ? manager->mil_status : false;
}

uint32_t dtc_get_mil_request_count(const dtc_manager_t* manager) {
    return manager ? manager->mil_request_count : 0;
}

// Diagnostic Session Management
bool dtc_start_diagnostic_session(dtc_manager_t* manager, uint8_t session_type) {
    if (!manager) {
        return false;
    }
    
    manager->diagnostic_session_active = true;
    manager->current_session_type = session_type;
    
    printf("Diagnostic session started: type 0x%02X\n", session_type);
    return true;
}

bool dtc_stop_diagnostic_session(dtc_manager_t* manager) {
    if (!manager) {
        return false;
    }
    
    manager->diagnostic_session_active = false;
    manager->current_session_type = 0x01; // Default session
    
    printf("Diagnostic session stopped\n");
    return true;
}

bool dtc_is_diagnostic_session_active(const dtc_manager_t* manager) {
    return manager ? manager->diagnostic_session_active : false;
}

// Utility and Conversion Functions
const char* dtc_code_to_string(dtc_code_t code) {
    static char dtc_string[16];
    
    // Convert to SAE J2012 format (Pxxxx, Bxxxx, Cxxxx, Uxxxx)
    char prefix;
    uint32_t dtc_number = code & 0x3FFF; // Lower 14 bits
    uint8_t system = (code >> 14) & 0x03; // Bits 14-15
    
    switch ((code >> 16) & 0xC0) {
        case 0x00: prefix = 'P'; break; // Powertrain
        case 0x40: prefix = 'B'; break; // Body
        case 0x80: prefix = 'C'; break; // Chassis
        case 0xC0: prefix = 'U'; break; // Network
        default: prefix = 'P'; break;
    }
    
    snprintf(dtc_string, sizeof(dtc_string), "%c%01X%03X", prefix, system, dtc_number);
    return dtc_string;
}

const char* dtc_status_to_string(uint8_t status) {
    static char status_string[256];
    status_string[0] = '\0';
    
    if (status & DTC_STATUS_TEST_FAILED) strcat(status_string, "FAILED ");
    if (status & DTC_STATUS_TEST_FAILED_SINCE_CLEAR) strcat(status_string, "FAILED_SINCE_CLEAR ");
    if (status & DTC_STATUS_PENDING_DTC) strcat(status_string, "PENDING ");
    if (status & DTC_STATUS_CONFIRMED_DTC) strcat(status_string, "CONFIRMED ");
    if (status & DTC_STATUS_WARNING_INDICATOR_REQUESTED) strcat(status_string, "MIL_REQUEST ");
    
    if (strlen(status_string) == 0) {
        strcpy(status_string, "PASSED");
    }
    
    return status_string;
}

const char* dtc_severity_to_string(dtc_severity_t severity) {
    switch (severity) {
        case DTC_SEVERITY_NO_SEVERITY: return "NO_SEVERITY";
        case DTC_SEVERITY_MAINTENANCE_ONLY: return "MAINTENANCE_ONLY";
        case DTC_SEVERITY_CHECK_AT_NEXT_HALT: return "CHECK_AT_NEXT_HALT";
        case DTC_SEVERITY_CHECK_IMMEDIATELY: return "CHECK_IMMEDIATELY";
        case DTC_SEVERITY_CRITICAL_SAFETY: return "CRITICAL_SAFETY";
        default: return "UNKNOWN_SEVERITY";
    }
}

const char* dtc_category_to_string(dtc_category_t category) {
    switch (category) {
        case DTC_CATEGORY_SENSOR_FAULT: return "SENSOR_FAULT";
        case DTC_CATEGORY_ACTUATOR_FAULT: return "ACTUATOR_FAULT";
        case DTC_CATEGORY_COMMUNICATION_FAULT: return "COMMUNICATION_FAULT";
        case DTC_CATEGORY_CONTROL_SYSTEM_FAULT: return "CONTROL_SYSTEM_FAULT";
        case DTC_CATEGORY_CALIBRATION_FAULT: return "CALIBRATION_FAULT";
        case DTC_CATEGORY_WATCHDOG_FAULT: return "WATCHDOG_FAULT";
        case DTC_CATEGORY_BRAKE_SYSTEM_FAULT: return "BRAKE_SYSTEM_FAULT";
        case DTC_CATEGORY_ENGINE_SYSTEM_FAULT: return "ENGINE_SYSTEM_FAULT";
        default: return "UNKNOWN_CATEGORY";
    }
}

uint32_t dtc_get_current_timestamp(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

uint32_t dtc_get_current_odometer(void) {
    // Simulate odometer reading (in km)
    // In real implementation, this would read from vehicle odometer
    static uint32_t simulated_odometer = 50000; // Start at 50,000 km
    simulated_odometer += (dtc_get_current_timestamp() / 60000); // Increment based on time
    return simulated_odometer;
}

// Data Persistence Functions
bool dtc_save_to_persistent_storage(const dtc_manager_t* manager) {
    if (!manager) {
        return false;
    }
    
    // Simulate saving to EEPROM/Flash
    // In real implementation, this would write to non-volatile memory
    printf("DTC Manager: Saving %u DTCs to persistent storage\n", manager->total_dtc_count);
    
    // For demonstration, save to a file
    FILE* file = fopen("dtc_storage.dat", "wb");
    if (!file) {
        printf("DTC Manager: Failed to open storage file for writing\n");
        return false;
    }
    
    // Write magic number and version
    uint32_t magic = DTC_STORAGE_MAGIC_NUMBER;
    uint16_t version = DTC_STORAGE_VERSION;
    fwrite(&magic, sizeof(magic), 1, file);
    fwrite(&version, sizeof(version), 1, file);
    
    // Write manager data
    fwrite(manager, sizeof(dtc_manager_t), 1, file);
    
    fclose(file);
    printf("DTC Manager: Data saved successfully\n");
    return true;
}

bool dtc_load_from_persistent_storage(dtc_manager_t* manager) {
    if (!manager) {
        return false;
    }
    
    // Simulate loading from EEPROM/Flash
    FILE* file = fopen("dtc_storage.dat", "rb");
    if (!file) {
        return false; // No saved data
    }
    
    // Read and verify magic number and version
    uint32_t magic;
    uint16_t version;
    if (fread(&magic, sizeof(magic), 1, file) != 1 || magic != DTC_STORAGE_MAGIC_NUMBER) {
        fclose(file);
        printf("DTC Manager: Invalid storage file format\n");
        return false;
    }
    
    if (fread(&version, sizeof(version), 1, file) != 1 || version != DTC_STORAGE_VERSION) {
        fclose(file);
        printf("DTC Manager: Unsupported storage file version\n");
        return false;
    }
    
    // Read manager data
    dtc_manager_t loaded_manager;
    if (fread(&loaded_manager, sizeof(dtc_manager_t), 1, file) == 1) {
        // Copy loaded data to manager (preserve current system start time)
        uint32_t current_start_time = manager->system_start_time;
        memcpy(manager, &loaded_manager, sizeof(dtc_manager_t));
        manager->system_start_time = current_start_time;
        
        printf("DTC Manager: Loaded %u DTCs from persistent storage\n", manager->total_dtc_count);
        fclose(file);
        return true;
    }
    
    fclose(file);
    return false;
}

bool dtc_erase_persistent_storage(void) {
    if (remove("dtc_storage.dat") == 0) {
        printf("DTC Manager: Persistent storage erased\n");
        return true;
    }
    return false;
}

// Integration Functions for ESC System
bool dtc_process_esc_sensor_fault(dtc_manager_t* manager, uint8_t sensor_id, bool fault_active) {
    if (!manager) {
        return false;
    }
    
    dtc_code_t dtc_code = 0;
    const char* description = "";
    
    switch (sensor_id) {
        case 0: // Front left wheel speed
            dtc_code = DTC_ESC_WHEEL_SPEED_SENSOR_FL;
            description = "Front left wheel speed sensor fault";
            break;
        case 1: // Front right wheel speed
            dtc_code = DTC_ESC_WHEEL_SPEED_SENSOR_FR;
            description = "Front right wheel speed sensor fault";
            break;
        case 2: // Rear left wheel speed
            dtc_code = DTC_ESC_WHEEL_SPEED_SENSOR_RL;
            description = "Rear left wheel speed sensor fault";
            break;
        case 3: // Rear right wheel speed
            dtc_code = DTC_ESC_WHEEL_SPEED_SENSOR_RR;
            description = "Rear right wheel speed sensor fault";
            break;
        case 4: // Steering angle
            dtc_code = DTC_ESC_STEERING_ANGLE_SENSOR;
            description = "Steering angle sensor fault";
            break;
        case 5: // Yaw rate
            dtc_code = DTC_ESC_YAW_RATE_SENSOR;
            description = "Yaw rate sensor fault";
            break;
        case 6: // Lateral acceleration
            dtc_code = DTC_ESC_LATERAL_ACCEL_SENSOR;
            description = "Lateral acceleration sensor fault";
            break;
        case 7: // Longitudinal acceleration
            dtc_code = DTC_ESC_LONGITUDINAL_ACCEL_SENSOR;
            description = "Longitudinal acceleration sensor fault";
            break;
        default:
            return false;
    }
    
    if (fault_active) {
        return dtc_set_code(manager, dtc_code, description);
    } else {
        // Fault no longer active - update status but don't clear (aging will handle it)
        dtc_record_t* record = dtc_get_record(manager, dtc_code);
        if (record) {
            record->is_active = false;
            record->status_byte &= ~DTC_STATUS_TEST_FAILED;
        }
        return true;
    }
}

bool dtc_process_esc_actuator_fault(dtc_manager_t* manager, uint8_t actuator_id, bool fault_active) {
    if (!manager) {
        return false;
    }
    
    dtc_code_t dtc_code = 0;
    const char* description = "";
    
    switch (actuator_id) {
        case 0: // Front left brake
            dtc_code = DTC_ESC_BRAKE_ACTUATOR_FL;
            description = "Front left brake actuator fault";
            break;
        case 1: // Front right brake
            dtc_code = DTC_ESC_BRAKE_ACTUATOR_FR;
            description = "Front right brake actuator fault";
            break;
        case 2: // Rear left brake
            dtc_code = DTC_ESC_BRAKE_ACTUATOR_RL;
            description = "Rear left brake actuator fault";
            break;
        case 3: // Rear right brake
            dtc_code = DTC_ESC_BRAKE_ACTUATOR_RR;
            description = "Rear right brake actuator fault";
            break;
        default:
            return false;
    }
    
    if (fault_active) {
        return dtc_set_code(manager, dtc_code, description);
    } else {
        dtc_record_t* record = dtc_get_record(manager, dtc_code);
        if (record) {
            record->is_active = false;
            record->status_byte &= ~DTC_STATUS_TEST_FAILED;
        }
        return true;
    }
}

bool dtc_process_esc_communication_fault(dtc_manager_t* manager, uint8_t comm_type, bool fault_active) {
    if (!manager) {
        return false;
    }
    
    dtc_code_t dtc_code = 0;
    const char* description = "";
    
    switch (comm_type) {
        case 0: // CAN bus
            dtc_code = DTC_ESC_CAN_BUS_FAULT;
            description = "CAN bus communication fault";
            break;
        case 1: // Engine ECU
            dtc_code = DTC_ESC_ENGINE_ECU_COMM_FAULT;
            description = "Engine ECU communication fault";
            break;
        case 2: // Diagnostic
            dtc_code = DTC_ESC_DIAGNOSTIC_COMM_FAULT;
            description = "Diagnostic communication fault";
            break;
        default:
            return false;
    }
    
    if (fault_active) {
        return dtc_set_code(manager, dtc_code, description);
    } else {
        dtc_record_t* record = dtc_get_record(manager, dtc_code);
        if (record) {
            record->is_active = false;
            record->status_byte &= ~DTC_STATUS_TEST_FAILED;
        }
        return true;
    }
}

bool dtc_process_esc_control_fault(dtc_manager_t* manager, uint8_t fault_type, bool fault_active) {
    if (!manager) {
        return false;
    }
    
    dtc_code_t dtc_code = 0;
    const char* description = "";
    
    switch (fault_type) {
        case 0: // Control module
            dtc_code = DTC_ESC_CONTROL_MODULE_FAULT;
            description = "ESC control module internal fault";
            break;
        case 1: // Plausibility
            dtc_code = DTC_ESC_PLAUSIBILITY_FAULT;
            description = "Sensor plausibility fault";
            break;
        case 2: // Calibration
            dtc_code = DTC_ESC_CALIBRATION_FAULT;
            description = "Calibration data fault";
            break;
        case 3: // Memory
            dtc_code = DTC_ESC_MEMORY_FAULT;
            description = "Memory fault";
            break;
        case 4: // Watchdog
            dtc_code = DTC_ESC_WATCHDOG_FAULT;
            description = "Watchdog supervision fault";
            break;
        default:
            return false;
    }
    
    if (fault_active) {
        return dtc_set_code(manager, dtc_code, description);
    } else {
        dtc_record_t* record = dtc_get_record(manager, dtc_code);
        if (record) {
            record->is_active = false;
            record->status_byte &= ~DTC_STATUS_TEST_FAILED;
        }
        return true;
    }
}

// Statistics and Reporting
void dtc_print_system_status(const dtc_manager_t* manager) {
    if (!manager) return;
    
    printf("\n=== DTC System Status ===\n");
    printf("Total DTCs: %u\n", manager->total_dtc_count);
    printf("Active DTCs: %u\n", manager->active_dtc_count);
    printf("Confirmed DTCs: %u\n", manager->confirmed_dtc_count);
    printf("Pending DTCs: %u\n", manager->pending_dtc_count);
    printf("MIL Status: %s\n", manager->mil_status ? "ON" : "OFF");
    printf("MIL Request Count: %u\n", manager->mil_request_count);
    printf("Diagnostic Session: %s\n", manager->diagnostic_session_active ? "ACTIVE" : "INACTIVE");
    printf("System Uptime: %u ms\n", dtc_get_current_timestamp() - manager->system_start_time);
    printf("==========================\n");
}

void dtc_print_active_codes(const dtc_manager_t* manager) {
    if (!manager) return;
    
    printf("\n=== Active DTCs ===\n");
    bool found_active = false;
    
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code != 0 && manager->stored_dtcs[i].is_active) {
            const dtc_record_t* record = &manager->stored_dtcs[i];
            printf("DTC: %s\n", dtc_code_to_string(record->dtc_code));
            printf("  Status: %s\n", dtc_status_to_string(record->status_byte));
            printf("  Severity: %s\n", dtc_severity_to_string(record->severity));
            printf("  Occurrences: %u\n", record->occurrence_count);
            printf("  Description: %s\n", record->description);
            printf("  Freeze Frame: %s\n", record->has_freeze_frame ? "Available" : "None");
            printf("\n");
            found_active = true;
        }
    }
    
    if (!found_active) {
        printf("No active DTCs\n");
    }
    printf("==================\n");
}

void dtc_print_all_codes(const dtc_manager_t* manager) {
    if (!manager) return;
    
    printf("\n=== All Stored DTCs ===\n");
    bool found_any = false;
    
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code != 0) {
            const dtc_record_t* record = &manager->stored_dtcs[i];
            printf("DTC: %s [%s]\n", dtc_code_to_string(record->dtc_code),
                   record->is_active ? "ACTIVE" : 
                   record->is_confirmed ? "CONFIRMED" : 
                   record->is_pending ? "PENDING" : "STORED");
            printf("  Status: %s\n", dtc_status_to_string(record->status_byte));
            printf("  Category: %s\n", dtc_category_to_string(record->category));
            printf("  Severity: %s\n", dtc_severity_to_string(record->severity));
            printf("  Occurrences: %u\n", record->occurrence_count);
            printf("  First: %u ms, Last: %u ms\n", 
                   record->first_occurrence_timestamp, record->last_occurrence_timestamp);
            printf("  Description: %s\n", record->description);
            printf("\n");
            found_any = true;
        }
    }
    
    if (!found_any) {
        printf("No stored DTCs\n");
    }
    printf("======================\n");
}

void dtc_print_statistics(const dtc_manager_t* manager) {
    if (!manager) return;
    
    printf("\n=== DTC Statistics ===\n");
    printf("System Start Time: %u ms\n", manager->system_start_time);
    printf("Last Clear Time: %u ms\n", manager->last_clear_timestamp);
    printf("Drive Cycles: %u\n", g_drive_cycle_count);
    printf("Ignition Cycles: %u\n", g_ignition_cycle_count);
    printf("Emissions Readiness: %s\n", g_emissions_readiness_complete ? "COMPLETE" : "INCOMPLETE");
    
    // Count DTCs by category
    uint16_t category_counts[8] = {0};
    for (int i = 0; i < DTC_MAX_STORED_CODES; i++) {
        if (manager->stored_dtcs[i].dtc_code != 0) {
            if (manager->stored_dtcs[i].category < 8) {
                category_counts[manager->stored_dtcs[i].category]++;
            }
        }
    }
    
    printf("DTCs by Category:\n");
    for (int i = 1; i <= 8; i++) {
        if (category_counts[i-1] > 0) {
            printf("  %s: %u\n", dtc_category_to_string((dtc_category_t)i), category_counts[i-1]);
        }
    }
    printf("=====================\n");
}