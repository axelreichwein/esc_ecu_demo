#include "dtc_management.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

// Demo function prototypes
void demo_basic_dtc_operations(void);
void demo_uds_read_dtc_services(void);
void demo_uds_clear_dtc_services(void);
void demo_freeze_frame_capture(void);
void demo_esc_integration(void);
void demo_mil_functionality(void);

// UDS request simulation functions
bool simulate_uds_request(dtc_manager_t* manager, const uint8_t* request, uint16_t request_length);
void print_uds_response(const uint8_t* response, uint16_t response_length);

int main(void) {
    printf("DTC Management System Demonstration\n");
    printf("===================================\n");
    
    demo_basic_dtc_operations();
    demo_freeze_frame_capture();
    demo_uds_read_dtc_services();
    demo_uds_clear_dtc_services();
    demo_mil_functionality();
    demo_esc_integration();
    
    printf("\nDTC management demonstration completed.\n");
    return 0;
}

void demo_basic_dtc_operations(void) {
    printf("\n=== Basic DTC Operations Demo ===\n");
    
    dtc_manager_t manager;
    if (!dtc_manager_init(&manager)) {
        printf("Failed to initialize DTC manager\n");
        return;
    }
    
    printf("\nSetting various DTCs:\n");
    
    // Set some sensor fault DTCs
    dtc_set_code(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FL, "Front left wheel speed sensor intermittent signal");
    dtc_set_code(&manager, DTC_ESC_YAW_RATE_SENSOR, "Yaw rate sensor out of range");
    dtc_set_code(&manager, DTC_ESC_STEERING_ANGLE_SENSOR, "Steering angle sensor implausible signal");
    
    // Set an actuator fault
    dtc_set_code(&manager, DTC_ESC_BRAKE_ACTUATOR_RR, "Rear right brake actuator stuck");
    
    // Set a communication fault
    dtc_set_code(&manager, DTC_ESC_ENGINE_ECU_COMM_FAULT, "Engine ECU communication timeout");
    
    // Set a control system fault
    dtc_set_code(&manager, DTC_ESC_PLAUSIBILITY_FAULT, "Sensor plausibility check failed");
    
    printf("\nInitial system status:\n");
    dtc_print_system_status(&manager);
    
    printf("\nAll active DTCs:\n");
    dtc_print_active_codes(&manager);
    
    // Simulate repeated faults to trigger confirmation
    printf("\nSimulating repeated faults to trigger confirmation:\n");
    dtc_set_code(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FL, "Front left wheel speed sensor intermittent signal");
    dtc_set_code(&manager, DTC_ESC_YAW_RATE_SENSOR, "Yaw rate sensor out of range");
    
    printf("\nStatus after repeated faults:\n");
    dtc_print_system_status(&manager);
    
    printf("\nAll stored DTCs:\n");
    dtc_print_all_codes(&manager);
    
    printf("\nDTC Statistics:\n");
    dtc_print_statistics(&manager);
    
    dtc_manager_shutdown(&manager);
}

void demo_freeze_frame_capture(void) {
    printf("\n=== Freeze Frame Capture Demo ===\n");
    
    dtc_manager_t manager;
    if (!dtc_manager_init(&manager)) {
        printf("Failed to initialize DTC manager\n");
        return;
    }
    
    // Set a DTC and capture freeze frame data
    dtc_set_code(&manager, DTC_ESC_CONTROL_MODULE_FAULT, "ESC control module internal fault");
    
    // Simulate freeze frame data (ISO 15031-5 format)
    uint8_t freeze_frame_data[] = {
        0x01, 0x02, 0x00, 0x50,  // Vehicle speed: 80 km/h
        0x04, 0x01, 0x3C,        // Engine load: 60%
        0x05, 0x01, 0x55,        // Coolant temp: 85°C
        0x0C, 0x02, 0x08, 0x00,  // Engine RPM: 2048 RPM
        0x11, 0x01, 0x32,        // Throttle position: 50%
        0x80, 0x02, 0x00, 0x64,  // Yaw rate: 1.0 deg/s
        0x81, 0x02, 0x01, 0xF4,  // Lat accel: 5.0 m/s²
        0x82, 0x02, 0x00, 0xC8,  // Steering: 20.0 degrees
    };
    
    printf("Capturing freeze frame data...\n");
    dtc_capture_freeze_frame(&manager, DTC_ESC_CONTROL_MODULE_FAULT, freeze_frame_data, sizeof(freeze_frame_data));
    
    // Retrieve and display freeze frame
    freeze_frame_t frame;
    if (dtc_get_freeze_frame(&manager, DTC_ESC_CONTROL_MODULE_FAULT, &frame)) {
        printf("Freeze frame retrieved successfully:\n");
        printf("  DTC: %s\n", dtc_code_to_string(frame.dtc_code));
        printf("  Frame number: %u\n", frame.frame_number);
        printf("  Timestamp: %u ms\n", frame.timestamp);
        printf("  Parameter count: %u\n", frame.parameter_count);
    }
    
    dtc_manager_shutdown(&manager);
}

void demo_uds_read_dtc_services(void) {
    printf("\n=== UDS ReadDTCInformation Services Demo ===\n");
    
    dtc_manager_t manager;
    if (!dtc_manager_init(&manager)) {
        printf("Failed to initialize DTC manager\n");
        return;
    }
    
    // Set up some test DTCs
    dtc_set_code(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FL, "FL wheel speed sensor fault");
    dtc_set_code(&manager, DTC_ESC_BRAKE_ACTUATOR_FR, "FR brake actuator fault");
    dtc_confirm_code(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FL); // Confirm one DTC
    
    // Capture freeze frame for confirmed DTC
    uint8_t freeze_data[] = {0x01, 0x02, 0x00, 0x32, 0x04, 0x01, 0x28};
    dtc_capture_freeze_frame(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FL, freeze_data, sizeof(freeze_data));
    
    printf("\nTesting UDS ReadDTCInformation services:\n");
    
    // Test 1: Read number of DTCs by status mask
    printf("\n1. Read Number of DTCs by Status Mask:\n");
    uint8_t request1[] = {0x19, 0x01, 0x08}; // Service 0x19, sub 0x01, confirmed DTCs
    simulate_uds_request(&manager, request1, sizeof(request1));
    
    // Test 2: Read DTCs by status mask
    printf("\n2. Read DTCs by Status Mask:\n");
    uint8_t request2[] = {0x19, 0x02, 0x08}; // Service 0x19, sub 0x02, confirmed DTCs
    simulate_uds_request(&manager, request2, sizeof(request2));
    
    // Test 3: Read DTC snapshot identification
    printf("\n3. Read DTC Snapshot Identification:\n");
    uint8_t request3[] = {0x19, 0x03}; // Service 0x19, sub 0x03
    simulate_uds_request(&manager, request3, sizeof(request3));
    
    // Test 4: Read DTC snapshot by DTC number
    printf("\n4. Read DTC Snapshot by DTC Number:\n");
    uint32_t dtc_code = DTC_ESC_WHEEL_SPEED_SENSOR_FL;
    uint8_t request4[] = {0x19, 0x04, 
                         (dtc_code >> 16) & 0xFF, (dtc_code >> 8) & 0xFF, dtc_code & 0xFF, 
                         0x01}; // Service 0x19, sub 0x04, DTC, record 1
    simulate_uds_request(&manager, request4, sizeof(request4));
    
    // Test 5: Read DTC extended data
    printf("\n5. Read DTC Extended Data:\n");
    uint8_t request5[] = {0x19, 0x06, 
                         (dtc_code >> 16) & 0xFF, (dtc_code >> 8) & 0xFF, dtc_code & 0xFF, 
                         0xFF}; // Service 0x19, sub 0x06, DTC, all records
    simulate_uds_request(&manager, request5, sizeof(request5));
    
    dtc_manager_shutdown(&manager);
}

void demo_uds_clear_dtc_services(void) {
    printf("\n=== UDS ClearDTCInformation Services Demo ===\n");
    
    dtc_manager_t manager;
    if (!dtc_manager_init(&manager)) {
        printf("Failed to initialize DTC manager\n");
        return;
    }
    
    // Set up test DTCs
    dtc_set_code(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FL, "FL wheel speed sensor fault");
    dtc_set_code(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FR, "FR wheel speed sensor fault");
    dtc_set_code(&manager, DTC_ESC_BRAKE_ACTUATOR_RL, "RL brake actuator fault");
    
    printf("Initial DTCs:\n");
    dtc_print_system_status(&manager);
    
    // Test 1: Clear specific DTC
    printf("\n1. Clear Specific DTC:\n");
    uint32_t dtc_code = DTC_ESC_WHEEL_SPEED_SENSOR_FL;
    uint8_t request1[] = {0x14, 
                         (dtc_code >> 16) & 0xFF, (dtc_code >> 8) & 0xFF, dtc_code & 0xFF};
    simulate_uds_request(&manager, request1, sizeof(request1));
    
    printf("After clearing specific DTC:\n");
    dtc_print_system_status(&manager);
    
    // Test 2: Clear all DTCs
    printf("\n2. Clear All DTCs:\n");
    uint8_t request2[] = {0x14, 0xFF, 0xFF, 0xFF}; // Clear all DTCs
    simulate_uds_request(&manager, request2, sizeof(request2));
    
    printf("After clearing all DTCs:\n");
    dtc_print_system_status(&manager);
    
    dtc_manager_shutdown(&manager);
}

void demo_mil_functionality(void) {
    printf("\n=== MIL (Malfunction Indicator Lamp) Demo ===\n");
    
    dtc_manager_t manager;
    if (!dtc_manager_init(&manager)) {
        printf("Failed to initialize DTC manager\n");
        return;
    }
    
    printf("Initial MIL status: %s\n", dtc_get_mil_status(&manager) ? "ON" : "OFF");
    
    // Set a non-emissions related DTC (should not trigger MIL)
    printf("\nSetting non-emissions DTC (C-code):\n");
    dtc_set_code(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FL, "Wheel speed sensor fault");
    dtc_confirm_code(&manager, DTC_ESC_WHEEL_SPEED_SENSOR_FL);
    dtc_update_mil_status(&manager);
    printf("MIL status after C-code: %s\n", dtc_get_mil_status(&manager) ? "ON" : "OFF");
    
    // Set an emissions-related DTC (should trigger MIL)
    printf("\nSetting emissions-related DTC (P0-code):\n");
    dtc_code_t emissions_dtc = 0x000100; // P0100 - Mass Air Flow sensor fault
    dtc_set_code(&manager, emissions_dtc, "Mass Air Flow sensor circuit malfunction");
    dtc_confirm_code(&manager, emissions_dtc);
    dtc_update_mil_status(&manager);
    printf("MIL status after P0-code: %s\n", dtc_get_mil_status(&manager) ? "ON" : "OFF");
    printf("MIL request count: %u\n", dtc_get_mil_request_count(&manager));
    
    // Clear the emissions DTC
    printf("\nClearing emissions DTC:\n");
    dtc_clear_code(&manager, emissions_dtc);
    dtc_update_mil_status(&manager);
    printf("MIL status after clearing: %s\n", dtc_get_mil_status(&manager) ? "ON" : "OFF");
    
    dtc_manager_shutdown(&manager);
}

void demo_esc_integration(void) {
    printf("\n=== ESC Integration Demo ===\n");
    
    dtc_manager_t manager;
    if (!dtc_manager_init(&manager)) {
        printf("Failed to initialize DTC manager\n");
        return;
    }
    
    printf("Testing ESC-specific DTC integration functions:\n");
    
    // Simulate sensor faults
    printf("\n1. Sensor Fault Processing:\n");
    dtc_process_esc_sensor_fault(&manager, 0, true);  // FL wheel speed sensor
    dtc_process_esc_sensor_fault(&manager, 5, true);  // Yaw rate sensor
    
    // Simulate actuator faults
    printf("\n2. Actuator Fault Processing:\n");
    dtc_process_esc_actuator_fault(&manager, 1, true); // FR brake actuator
    
    // Simulate communication faults
    printf("\n3. Communication Fault Processing:\n");
    dtc_process_esc_communication_fault(&manager, 1, true); // Engine ECU comm
    
    // Simulate control system faults
    printf("\n4. Control System Fault Processing:\n");
    dtc_process_esc_control_fault(&manager, 1, true); // Plausibility fault
    dtc_process_esc_control_fault(&manager, 4, true); // Watchdog fault
    
    printf("\nESC system status with DTCs:\n");
    dtc_print_system_status(&manager);
    dtc_print_active_codes(&manager);
    
    // Test fault recovery
    printf("\n5. Fault Recovery Simulation:\n");
    dtc_process_esc_sensor_fault(&manager, 0, false); // FL sensor recovered
    dtc_process_esc_actuator_fault(&manager, 1, false); // FR actuator recovered
    
    printf("Status after fault recovery:\n");
    dtc_print_active_codes(&manager);
    
    dtc_manager_shutdown(&manager);
}

// Helper function to simulate UDS requests
bool simulate_uds_request(dtc_manager_t* manager, const uint8_t* request, uint16_t request_length) {
    uds_request_t uds_request = {0};
    uds_response_t uds_response = {0};
    
    if (request_length < 1) {
        return false;
    }
    
    // Parse request
    uds_request.service_id = request[0];
    
    if (uds_request.service_id == UDS_SERVICE_READ_DTC_INFORMATION && request_length >= 2) {
        uds_request.sub_function = request[1];
        if (request_length >= 3) {
            uds_request.status_mask = request[2];
        }
        if (request_length >= 6) {
            uds_request.dtc_mask = (request[2] << 16) | (request[3] << 8) | request[4];
            uds_request.record_number = request[5];
        }
    } else if (uds_request.service_id == UDS_SERVICE_CLEAR_DTC_INFORMATION && request_length >= 4) {
        uds_request.request_length = request_length - 1;
        memcpy(uds_request.request_data, &request[1], uds_request.request_length);
    }
    
    // Process request
    bool success = uds_process_diagnostic_request(manager, &uds_request, &uds_response);
    
    // Print request and response
    printf("Request: ");
    for (int i = 0; i < request_length; i++) {
        printf("%02X ", request[i]);
    }
    printf("\n");
    
    if (success && uds_response.is_positive_response) {
        printf("Response (Positive): %02X ", uds_response.service_id);
        for (int i = 0; i < uds_response.response_length; i++) {
            printf("%02X ", uds_response.response_data[i]);
        }
        printf("\n");
    } else {
        printf("Response (Negative): 7F %02X %02X\n", 
               uds_request.service_id, uds_response.response_code);
    }
    
    return success;
}

void print_uds_response(const uint8_t* response, uint16_t response_length) {
    printf("UDS Response (%u bytes): ", response_length);
    for (int i = 0; i < response_length; i++) {
        printf("%02X ", response[i]);
    }
    printf("\n");
}