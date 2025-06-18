#ifndef DTC_MANAGEMENT_H
#define DTC_MANAGEMENT_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

// ISO 15031 / OBD-II compliance definitions
#define DTC_CODE_LENGTH 5              // P0XXX format (5 characters)
#define DTC_MAX_STORED_CODES 255       // Maximum DTCs that can be stored
#define DTC_MAX_FREEZE_FRAME_SIZE 256  // Maximum freeze frame data size
#define DTC_MAX_SNAPSHOT_RECORDS 8     // Maximum snapshot records per DTC

// UDS (ISO 14229) service identifiers
#define UDS_SERVICE_READ_DTC_INFORMATION 0x19
#define UDS_SERVICE_CLEAR_DTC_INFORMATION 0x14
#define UDS_SERVICE_IO_CONTROL_BY_IDENTIFIER 0x2F
#define UDS_SERVICE_READ_DATA_BY_IDENTIFIER 0x22

// UDS ReadDTCInformation sub-functions
#define UDS_READ_DTC_BY_STATUS_MASK 0x02
#define UDS_READ_DTC_SNAPSHOT_IDENTIFICATION 0x03
#define UDS_READ_DTC_SNAPSHOT_BY_DTC_NUMBER 0x04
#define UDS_READ_DTC_SNAPSHOT_BY_RECORD_NUMBER 0x05
#define UDS_READ_DTC_EXTENDED_DATA_BY_DTC_NUMBER 0x06
#define UDS_READ_NUMBER_OF_DTC_BY_STATUS_MASK 0x01
#define UDS_READ_SUPPORTED_DTC 0x0A

// OBD-II DTC status byte (ISO 15031-6)
typedef enum {
    DTC_STATUS_TEST_FAILED                    = 0x01,  // Test failed this drive cycle
    DTC_STATUS_TEST_FAILED_SINCE_CLEAR        = 0x02,  // Test failed since DTC cleared
    DTC_STATUS_PENDING_DTC                    = 0x04,  // Pending DTC
    DTC_STATUS_CONFIRMED_DTC                  = 0x08,  // Confirmed DTC
    DTC_STATUS_TEST_NOT_COMPLETED_SINCE_CLEAR = 0x10,  // Test not completed since clear
    DTC_STATUS_TEST_FAILED_SINCE_CLEAR_2      = 0x20,  // Test failed since clear (duplicate)
    DTC_STATUS_TEST_NOT_COMPLETED_THIS_CYCLE  = 0x40,  // Test not completed this cycle
    DTC_STATUS_WARNING_INDICATOR_REQUESTED    = 0x80   // Warning indicator requested
} dtc_status_bit_t;

// DTC severity levels (ISO 14229)
typedef enum {
    DTC_SEVERITY_NO_SEVERITY                  = 0x00,
    DTC_SEVERITY_MAINTENANCE_ONLY             = 0x20,
    DTC_SEVERITY_CHECK_AT_NEXT_HALT           = 0x40,
    DTC_SEVERITY_CHECK_IMMEDIATELY            = 0x60,
    DTC_SEVERITY_CRITICAL_SAFETY              = 0x80
} dtc_severity_t;

// DTC categories (ESC specific)
typedef enum {
    DTC_CATEGORY_SENSOR_FAULT         = 0x01,
    DTC_CATEGORY_ACTUATOR_FAULT       = 0x02,
    DTC_CATEGORY_COMMUNICATION_FAULT  = 0x03,
    DTC_CATEGORY_CONTROL_SYSTEM_FAULT = 0x04,
    DTC_CATEGORY_CALIBRATION_FAULT    = 0x05,
    DTC_CATEGORY_WATCHDOG_FAULT       = 0x06,
    DTC_CATEGORY_BRAKE_SYSTEM_FAULT   = 0x07,
    DTC_CATEGORY_ENGINE_SYSTEM_FAULT  = 0x08
} dtc_category_t;

// ESC-specific DTC codes (following SAE J2012 format)
typedef enum {
    // Sensor faults (C0xxx series for chassis systems)
    DTC_ESC_WHEEL_SPEED_SENSOR_FL     = 0xC0035,  // Front left wheel speed sensor
    DTC_ESC_WHEEL_SPEED_SENSOR_FR     = 0xC0040,  // Front right wheel speed sensor
    DTC_ESC_WHEEL_SPEED_SENSOR_RL     = 0xC0045,  // Rear left wheel speed sensor
    DTC_ESC_WHEEL_SPEED_SENSOR_RR     = 0xC0050,  // Rear right wheel speed sensor
    DTC_ESC_YAW_RATE_SENSOR           = 0xC0055,  // Yaw rate sensor fault
    DTC_ESC_LATERAL_ACCEL_SENSOR      = 0xC0060,  // Lateral acceleration sensor
    DTC_ESC_LONGITUDINAL_ACCEL_SENSOR = 0xC0065,  // Longitudinal acceleration sensor
    DTC_ESC_STEERING_ANGLE_SENSOR     = 0xC0070,  // Steering angle sensor
    
    // Actuator faults
    DTC_ESC_BRAKE_ACTUATOR_FL         = 0xC0110,  // Front left brake actuator
    DTC_ESC_BRAKE_ACTUATOR_FR         = 0xC0115,  // Front right brake actuator
    DTC_ESC_BRAKE_ACTUATOR_RL         = 0xC0120,  // Rear left brake actuator
    DTC_ESC_BRAKE_ACTUATOR_RR         = 0xC0125,  // Rear right brake actuator
    DTC_ESC_BRAKE_PRESSURE_SENSOR     = 0xC0130,  // Brake pressure sensor
    DTC_ESC_BRAKE_TEMPERATURE_SENSOR  = 0xC0135,  // Brake temperature sensor
    
    // Communication faults
    DTC_ESC_CAN_BUS_FAULT             = 0xC0100,  // CAN bus communication fault
    DTC_ESC_ENGINE_ECU_COMM_FAULT     = 0xC0101,  // Engine ECU communication fault
    DTC_ESC_DIAGNOSTIC_COMM_FAULT     = 0xC0102,  // Diagnostic communication fault
    
    // Control system faults
    DTC_ESC_CONTROL_MODULE_FAULT      = 0xC0200,  // ESC control module internal fault
    DTC_ESC_PLAUSIBILITY_FAULT        = 0xC0205,  // Sensor plausibility fault
    DTC_ESC_CALIBRATION_FAULT         = 0xC0210,  // Calibration data fault
    DTC_ESC_MEMORY_FAULT              = 0xC0215,  // Memory fault
    DTC_ESC_WATCHDOG_FAULT            = 0xC0220,  // Watchdog supervision fault
    
    // System performance faults
    DTC_ESC_SYSTEM_PERFORMANCE        = 0xC0300,  // System performance degraded
    DTC_ESC_OVERHEAT_PROTECTION       = 0xC0305,  // Overheat protection active
    DTC_ESC_VOLTAGE_FAULT             = 0xC0310,  // Supply voltage fault
    
    // Functional safety faults (ISO 26262)
    DTC_ESC_SAFETY_MECHANISM_FAULT    = 0xC0400,  // Safety mechanism fault
    DTC_ESC_REDUNDANCY_FAULT          = 0xC0405   // Redundancy system fault
} dtc_code_t;

// DTC record structure
typedef struct {
    uint32_t dtc_code;                          // DTC code (24-bit as per ISO 15031)
    uint8_t status_byte;                        // Status byte (ISO 15031-6)
    dtc_severity_t severity;                    // Severity level
    dtc_category_t category;                    // DTC category
    uint32_t occurrence_count;                  // Number of times this DTC occurred
    uint32_t first_occurrence_timestamp;       // First occurrence time (ms since system start)
    uint32_t last_occurrence_timestamp;        // Last occurrence time (ms since system start)
    uint32_t first_occurrence_odometer;        // Odometer reading at first occurrence (km)
    uint32_t last_occurrence_odometer;         // Odometer reading at last occurrence (km)
    uint8_t freeze_frame_data[DTC_MAX_FREEZE_FRAME_SIZE]; // Freeze frame data
    uint16_t freeze_frame_length;              // Actual freeze frame data length
    bool has_freeze_frame;                     // Whether freeze frame data is available
    bool is_active;                            // Whether DTC is currently active
    bool is_confirmed;                         // Whether DTC is confirmed
    bool is_pending;                           // Whether DTC is pending
    char description[64];                      // Human-readable description
} dtc_record_t;

// Freeze frame data structure (ISO 15031-5)
typedef struct {
    uint16_t parameter_id;                     // Parameter identification
    uint8_t data_length;                       // Length of parameter data
    uint8_t data[16];                          // Parameter data
} freeze_frame_parameter_t;

typedef struct {
    uint32_t dtc_code;                         // Associated DTC code
    uint8_t frame_number;                      // Freeze frame number
    uint32_t timestamp;                        // Timestamp when freeze frame was captured
    uint16_t parameter_count;                  // Number of parameters in freeze frame
    freeze_frame_parameter_t parameters[32];   // Freeze frame parameters
} freeze_frame_t;

// DTC storage and management
typedef struct {
    dtc_record_t stored_dtcs[DTC_MAX_STORED_CODES];  // Stored DTCs
    uint16_t active_dtc_count;                       // Number of active DTCs
    uint16_t confirmed_dtc_count;                    // Number of confirmed DTCs
    uint16_t pending_dtc_count;                      // Number of pending DTCs
    uint16_t total_dtc_count;                        // Total number of stored DTCs
    freeze_frame_t freeze_frames[DTC_MAX_STORED_CODES]; // Freeze frame storage
    uint32_t mil_request_count;                      // MIL (Malfunction Indicator Lamp) requests
    bool mil_status;                                 // Current MIL status
    uint32_t total_emissions_dtc_count;              // Total emissions-related DTCs
    uint32_t system_start_time;                      // System start time for timestamps
    uint32_t last_clear_timestamp;                   // Last time DTCs were cleared
    bool diagnostic_session_active;                  // UDS diagnostic session status
    uint8_t current_session_type;                    // Current diagnostic session type
} dtc_manager_t;

// UDS request/response structures
typedef struct {
    uint8_t service_id;                        // UDS service identifier
    uint8_t sub_function;                      // Sub-function identifier
    uint8_t status_mask;                       // Status mask for filtering
    uint32_t dtc_mask;                         // DTC mask for specific DTC selection
    uint8_t record_number;                     // Record number for snapshot data
    uint16_t data_identifier;                  // Data identifier for read operations
    uint8_t request_data[256];                 // Additional request data
    uint16_t request_length;                   // Length of request data
} uds_request_t;

typedef struct {
    uint8_t service_id;                        // Response service identifier (request + 0x40)
    uint8_t response_code;                     // Positive/negative response code
    uint8_t response_data[1024];               // Response data
    uint16_t response_length;                  // Length of response data
    bool is_positive_response;                 // Whether response is positive
} uds_response_t;

// UDS negative response codes (ISO 14229-1)
typedef enum {
    UDS_NRC_GENERAL_REJECT                    = 0x10,
    UDS_NRC_SERVICE_NOT_SUPPORTED             = 0x11,
    UDS_NRC_SUB_FUNCTION_NOT_SUPPORTED        = 0x12,
    UDS_NRC_INCORRECT_MESSAGE_LENGTH          = 0x13,
    UDS_NRC_RESPONSE_TOO_LONG                 = 0x14,
    UDS_NRC_CONDITIONS_NOT_CORRECT            = 0x22,
    UDS_NRC_REQUEST_SEQUENCE_ERROR            = 0x24,
    UDS_NRC_REQUEST_OUT_OF_RANGE              = 0x31,
    UDS_NRC_SECURITY_ACCESS_DENIED            = 0x33,
    UDS_NRC_INVALID_KEY                       = 0x35,
    UDS_NRC_EXCEED_NUMBER_OF_ATTEMPTS         = 0x36,
    UDS_NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED   = 0x37
} uds_negative_response_code_t;

// Function prototypes

// DTC Management Core Functions
bool dtc_manager_init(dtc_manager_t* manager);
void dtc_manager_shutdown(dtc_manager_t* manager);
bool dtc_set_code(dtc_manager_t* manager, dtc_code_t code, const char* description);
bool dtc_clear_code(dtc_manager_t* manager, dtc_code_t code);
bool dtc_clear_all_codes(dtc_manager_t* manager);
bool dtc_update_status(dtc_manager_t* manager, dtc_code_t code, uint8_t status_mask);
bool dtc_confirm_code(dtc_manager_t* manager, dtc_code_t code);
bool dtc_set_pending(dtc_manager_t* manager, dtc_code_t code);

// Freeze Frame Management
bool dtc_capture_freeze_frame(dtc_manager_t* manager, dtc_code_t code, const uint8_t* data, uint16_t length);
bool dtc_get_freeze_frame(const dtc_manager_t* manager, dtc_code_t code, freeze_frame_t* frame);
bool dtc_clear_freeze_frame(dtc_manager_t* manager, dtc_code_t code);

// DTC Query and Status Functions
bool dtc_is_code_active(const dtc_manager_t* manager, dtc_code_t code);
bool dtc_is_code_confirmed(const dtc_manager_t* manager, dtc_code_t code);
bool dtc_is_code_pending(const dtc_manager_t* manager, dtc_code_t code);
dtc_record_t* dtc_get_record(const dtc_manager_t* manager, dtc_code_t code);
uint16_t dtc_get_count_by_status(const dtc_manager_t* manager, uint8_t status_mask);
uint16_t dtc_get_codes_by_status(const dtc_manager_t* manager, uint8_t status_mask, dtc_code_t* codes, uint16_t max_codes);

// UDS Service Implementation
bool uds_service_read_dtc_information(dtc_manager_t* manager, const uds_request_t* request, uds_response_t* response);
bool uds_service_clear_dtc_information(dtc_manager_t* manager, const uds_request_t* request, uds_response_t* response);
bool uds_process_diagnostic_request(dtc_manager_t* manager, const uds_request_t* request, uds_response_t* response);

// UDS Sub-service Functions
bool uds_read_number_of_dtc_by_status_mask(const dtc_manager_t* manager, uint8_t status_mask, uds_response_t* response);
bool uds_read_dtc_by_status_mask(const dtc_manager_t* manager, uint8_t status_mask, uds_response_t* response);
bool uds_read_dtc_snapshot_identification(const dtc_manager_t* manager, uds_response_t* response);
bool uds_read_dtc_snapshot_by_dtc_number(const dtc_manager_t* manager, uint32_t dtc_code, uint8_t record_number, uds_response_t* response);
bool uds_read_dtc_extended_data_by_dtc_number(const dtc_manager_t* manager, uint32_t dtc_code, uint8_t record_number, uds_response_t* response);

// MIL (Malfunction Indicator Lamp) Management
bool dtc_update_mil_status(dtc_manager_t* manager);
bool dtc_get_mil_status(const dtc_manager_t* manager);
uint32_t dtc_get_mil_request_count(const dtc_manager_t* manager);

// Diagnostic Session Management
bool dtc_start_diagnostic_session(dtc_manager_t* manager, uint8_t session_type);
bool dtc_stop_diagnostic_session(dtc_manager_t* manager);
bool dtc_is_diagnostic_session_active(const dtc_manager_t* manager);

// Utility and Conversion Functions
const char* dtc_code_to_string(dtc_code_t code);
const char* dtc_status_to_string(uint8_t status);
const char* dtc_severity_to_string(dtc_severity_t severity);
const char* dtc_category_to_string(dtc_category_t category);
uint32_t dtc_get_current_timestamp(void);
uint32_t dtc_get_current_odometer(void);

// Data Persistence (EEPROM/Flash simulation)
bool dtc_save_to_persistent_storage(const dtc_manager_t* manager);
bool dtc_load_from_persistent_storage(dtc_manager_t* manager);
bool dtc_erase_persistent_storage(void);

// Diagnostic Test Functions
bool dtc_run_self_test(dtc_manager_t* manager);
bool dtc_validate_stored_dtcs(const dtc_manager_t* manager);

// Integration Functions for ESC System
bool dtc_process_esc_sensor_fault(dtc_manager_t* manager, uint8_t sensor_id, bool fault_active);
bool dtc_process_esc_actuator_fault(dtc_manager_t* manager, uint8_t actuator_id, bool fault_active);
bool dtc_process_esc_communication_fault(dtc_manager_t* manager, uint8_t comm_type, bool fault_active);
bool dtc_process_esc_control_fault(dtc_manager_t* manager, uint8_t fault_type, bool fault_active);

// Statistics and Reporting
void dtc_print_system_status(const dtc_manager_t* manager);
void dtc_print_active_codes(const dtc_manager_t* manager);
void dtc_print_all_codes(const dtc_manager_t* manager);
void dtc_print_statistics(const dtc_manager_t* manager);

#endif // DTC_MANAGEMENT_H