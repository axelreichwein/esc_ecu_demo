#ifndef POST_SYSTEM_H
#define POST_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

// POST (Power-On Self-Test) system definitions
#define POST_MAX_TEST_MODULES 32           // Maximum number of test modules
#define POST_MAX_ERROR_LOG_SIZE 64         // Maximum error log entries
#define POST_CHECKSUM_BLOCK_SIZE 1024      // Memory checksum block size
#define POST_SENSOR_VALIDATION_TIMEOUT 5000 // Sensor validation timeout (ms)
#define POST_INITIALIZATION_TIMEOUT 10000  // Overall initialization timeout (ms)
#define POST_RETRY_LIMIT 3                 // Maximum retries for failed tests

// POST test result codes
typedef enum {
    POST_RESULT_PASS                = 0x00,  // Test passed
    POST_RESULT_FAIL                = 0x01,  // Test failed
    POST_RESULT_WARNING             = 0x02,  // Test passed with warning
    POST_RESULT_SKIP                = 0x03,  // Test skipped
    POST_RESULT_TIMEOUT             = 0x04,  // Test timed out
    POST_RESULT_NOT_READY           = 0x05,  // Component not ready
    POST_RESULT_CRITICAL_FAIL       = 0x06   // Critical failure - system halt required
} post_result_t;

// POST test phases
typedef enum {
    POST_PHASE_HARDWARE_INIT        = 0x01,  // Hardware initialization
    POST_PHASE_MEMORY_TEST          = 0x02,  // Memory integrity tests
    POST_PHASE_SENSOR_VALIDATION    = 0x03,  // Sensor readiness validation
    POST_PHASE_COMMUNICATION_TEST   = 0x04,  // Communication system tests
    POST_PHASE_SAFETY_VALIDATION    = 0x05,  // Safety system validation
    POST_PHASE_CALIBRATION_CHECK    = 0x06,  // Calibration data validation
    POST_PHASE_SYSTEM_INTEGRATION   = 0x07,  // System integration tests
    POST_PHASE_FINAL_VERIFICATION   = 0x08   // Final system verification
} post_phase_t;

// POST test categories
typedef enum {
    POST_CATEGORY_CRITICAL          = 0x01,  // Critical tests (must pass)
    POST_CATEGORY_IMPORTANT         = 0x02,  // Important tests (failure = warning)
    POST_CATEGORY_INFORMATIONAL     = 0x03,  // Informational tests (optional)
    POST_CATEGORY_DIAGNOSTIC        = 0x04   // Diagnostic tests (for troubleshooting)
} post_category_t;

// Memory test types
typedef enum {
    MEMORY_TEST_RAM                 = 0x01,  // RAM integrity test
    MEMORY_TEST_FLASH               = 0x02,  // Flash/ROM checksum test
    MEMORY_TEST_EEPROM              = 0x03,  // EEPROM/NVRAM test
    MEMORY_TEST_STACK               = 0x04,  // Stack overflow protection test
    MEMORY_TEST_HEAP                = 0x05,  // Heap integrity test
    MEMORY_TEST_CALIBRATION_DATA    = 0x06   // Calibration data integrity
} memory_test_type_t;

// Sensor validation types
typedef enum {
    SENSOR_VALIDATION_PRESENCE      = 0x01,  // Sensor presence detection
    SENSOR_VALIDATION_RANGE         = 0x02,  // Value range validation
    SENSOR_VALIDATION_PLAUSIBILITY  = 0x03,  // Plausibility checks
    SENSOR_VALIDATION_CALIBRATION   = 0x04,  // Calibration validation
    SENSOR_VALIDATION_COMMUNICATION = 0x05,  // Communication validation
    SENSOR_VALIDATION_SELF_TEST     = 0x06   // Sensor self-test
} sensor_validation_type_t;

// ESC sensor identifiers
typedef enum {
    ESC_SENSOR_WHEEL_SPEED_FL       = 0x01,  // Front left wheel speed
    ESC_SENSOR_WHEEL_SPEED_FR       = 0x02,  // Front right wheel speed
    ESC_SENSOR_WHEEL_SPEED_RL       = 0x03,  // Rear left wheel speed
    ESC_SENSOR_WHEEL_SPEED_RR       = 0x04,  // Rear right wheel speed
    ESC_SENSOR_YAW_RATE             = 0x05,  // Yaw rate sensor
    ESC_SENSOR_LATERAL_ACCEL        = 0x06,  // Lateral acceleration sensor
    ESC_SENSOR_LONGITUDINAL_ACCEL   = 0x07,  // Longitudinal acceleration sensor
    ESC_SENSOR_STEERING_ANGLE       = 0x08,  // Steering angle sensor
    ESC_SENSOR_BRAKE_PRESSURE       = 0x09,  // Brake pressure sensor
    ESC_SENSOR_BRAKE_TEMPERATURE    = 0x0A,  // Brake temperature sensor
    ESC_SENSOR_SUPPLY_VOLTAGE       = 0x0B   // Supply voltage sensor
} esc_sensor_id_t;

// POST test module structure
typedef struct {
    uint8_t module_id;                      // Unique module identifier
    const char* module_name;                // Human-readable module name
    post_phase_t phase;                     // Test phase
    post_category_t category;               // Test category
    uint32_t timeout_ms;                    // Test timeout in milliseconds
    post_result_t (*test_function)(void*);  // Test function pointer
    void* test_context;                     // Test-specific context data
    bool enabled;                           // Whether test is enabled
    uint32_t execution_count;               // Number of times test was executed
    uint32_t pass_count;                    // Number of times test passed
    uint32_t fail_count;                    // Number of times test failed
    uint32_t last_execution_time;           // Last execution timestamp
    uint32_t last_execution_duration_ms;    // Last execution duration
} post_test_module_t;

// POST error log entry
typedef struct {
    uint32_t timestamp;                     // Error timestamp
    uint8_t module_id;                      // Failed module ID
    post_result_t result;                   // Test result
    uint32_t error_code;                    // Specific error code
    char description[64];                   // Error description
} post_error_log_entry_t;

// Memory region descriptor
typedef struct {
    void* start_address;                    // Start address of memory region
    uint32_t size;                          // Size of memory region in bytes
    memory_test_type_t test_type;          // Type of memory test
    uint32_t expected_checksum;             // Expected checksum (for ROM/Flash)
    bool critical;                          // Whether this memory is critical
    const char* region_name;                // Human-readable region name
} memory_region_t;

// Sensor calibration data
typedef struct {
    esc_sensor_id_t sensor_id;              // Sensor identifier
    float min_valid_value;                  // Minimum valid sensor value
    float max_valid_value;                  // Maximum valid sensor value
    float nominal_value;                    // Nominal/expected value
    float tolerance;                        // Acceptable tolerance
    uint32_t calibration_timestamp;         // Calibration timestamp
    uint32_t calibration_checksum;          // Calibration data checksum
    bool requires_warmup;                   // Whether sensor needs warmup time
    uint32_t warmup_time_ms;                // Required warmup time
} post_sensor_calibration_t;

// POST system status
typedef struct {
    bool initialization_complete;           // Overall initialization status
    bool esc_system_enabled;                // ESC system enable status
    bool critical_failure_detected;        // Critical failure flag
    uint32_t total_test_count;              // Total number of tests
    uint32_t passed_test_count;             // Number of passed tests
    uint32_t failed_test_count;             // Number of failed tests
    uint32_t warning_test_count;            // Number of tests with warnings
    uint32_t initialization_start_time;     // Initialization start time
    uint32_t initialization_duration_ms;    // Total initialization duration
    post_phase_t current_phase;             // Current test phase
    uint8_t current_module_id;              // Currently executing module
} post_system_status_t;

// Main POST system structure
typedef struct {
    post_test_module_t test_modules[POST_MAX_TEST_MODULES];  // Test modules
    uint8_t module_count;                                    // Number of test modules
    memory_region_t memory_regions[16];                      // Memory regions to test
    uint8_t memory_region_count;                             // Number of memory regions
    post_sensor_calibration_t sensor_calibrations[16];            // Sensor calibration data
    uint8_t sensor_count;                                    // Number of sensors
    post_error_log_entry_t error_log[POST_MAX_ERROR_LOG_SIZE]; // Error log
    uint8_t error_log_count;                                 // Number of error log entries
    post_system_status_t status;                             // System status
    bool verbose_logging;                                    // Enable verbose logging
    bool halt_on_critical_failure;                          // Halt system on critical failure
    uint32_t retry_delay_ms;                                 // Delay between retries
} post_system_t;

// Hardware abstraction for POST tests
typedef struct {
    bool (*hw_memory_test)(void* address, uint32_t size);
    uint32_t (*hw_calculate_checksum)(void* address, uint32_t size);
    bool (*hw_sensor_power_on)(esc_sensor_id_t sensor_id);
    bool (*hw_sensor_power_off)(esc_sensor_id_t sensor_id);
    float (*hw_sensor_read_value)(esc_sensor_id_t sensor_id);
    bool (*hw_sensor_self_test)(esc_sensor_id_t sensor_id);
    bool (*hw_communication_test)(uint8_t interface_id);
    uint32_t (*hw_get_timestamp)(void);
    void (*hw_delay_ms)(uint32_t milliseconds);
    void (*hw_system_reset)(void);
} post_hardware_interface_t;

// Function prototypes

// POST System Initialization and Control
bool post_system_init(post_system_t* post, const post_hardware_interface_t* hw_interface);
void post_system_shutdown(post_system_t* post);
bool post_execute_all_tests(post_system_t* post);
bool post_execute_phase(post_system_t* post, post_phase_t phase);
bool post_execute_test_module(post_system_t* post, uint8_t module_id);

// Test Module Management
bool post_register_test_module(post_system_t* post, const post_test_module_t* module);
bool post_enable_test_module(post_system_t* post, uint8_t module_id, bool enable);
bool post_set_test_timeout(post_system_t* post, uint8_t module_id, uint32_t timeout_ms);

// Memory Test Functions
post_result_t post_test_ram_integrity(void* context);
post_result_t post_test_flash_checksum(void* context);
post_result_t post_test_eeprom_integrity(void* context);
post_result_t post_test_stack_protection(void* context);
post_result_t post_test_heap_integrity(void* context);
post_result_t post_test_calibration_data(void* context);

// Sensor Validation Functions
post_result_t post_validate_wheel_speed_sensors(void* context);
post_result_t post_validate_inertial_sensors(void* context);
post_result_t post_validate_steering_angle_sensor(void* context);
post_result_t post_validate_brake_sensors(void* context);
post_result_t post_validate_supply_voltage(void* context);
post_result_t post_validate_sensor_plausibility(void* context);

// Communication Test Functions
post_result_t post_test_can_communication(void* context);
post_result_t post_test_internal_communication(void* context);
post_result_t post_test_diagnostic_communication(void* context);

// Safety System Validation Functions
post_result_t post_validate_watchdog_system(void* context);
post_result_t post_validate_safety_mechanisms(void* context);
post_result_t post_validate_fault_handling(void* context);
post_result_t post_validate_emergency_shutdown(void* context);

// System Integration Tests
post_result_t post_test_brake_system_integration(void* context);
post_result_t post_test_engine_interface_integration(void* context);
post_result_t post_test_dtc_system_integration(void* context);
post_result_t post_test_overall_system_integration(void* context);

// Memory Region Management
bool post_register_memory_region(post_system_t* post, const memory_region_t* region);
bool post_validate_memory_regions(post_system_t* post);
uint32_t post_calculate_memory_checksum(const void* address, uint32_t size);

// Sensor Calibration Management
bool post_register_sensor_calibration(post_system_t* post, const post_sensor_calibration_t* calibration);
bool post_validate_sensor_calibrations(post_system_t* post);
bool post_sensor_warmup_complete(const post_sensor_calibration_t* calibration, uint32_t warmup_start_time);

// Error Logging and Reporting
bool post_log_error(post_system_t* post, uint8_t module_id, post_result_t result, uint32_t error_code, const char* description);
void post_clear_error_log(post_system_t* post);
uint8_t post_get_error_count(const post_system_t* post);
bool post_get_error_log_entry(const post_system_t* post, uint8_t index, post_error_log_entry_t* entry);

// Status and Query Functions
bool post_is_initialization_complete(const post_system_t* post);
bool post_is_esc_system_enabled(const post_system_t* post);
bool post_has_critical_failure(const post_system_t* post);
post_phase_t post_get_current_phase(const post_system_t* post);
uint32_t post_get_initialization_duration(const post_system_t* post);

// ESC System Enable/Disable Control
bool post_enable_esc_system(post_system_t* post);
bool post_disable_esc_system(post_system_t* post, const char* reason);
bool post_check_esc_enable_conditions(const post_system_t* post);

// Utility Functions
const char* post_result_to_string(post_result_t result);
const char* post_phase_to_string(post_phase_t phase);
const char* post_category_to_string(post_category_t category);
const char* memory_test_type_to_string(memory_test_type_t type);
const char* sensor_validation_type_to_string(sensor_validation_type_t type);
const char* esc_sensor_id_to_string(esc_sensor_id_t sensor_id);

// Debug and Reporting Functions
void post_print_system_status(const post_system_t* post);
void post_print_test_results(const post_system_t* post);
void post_print_error_log(const post_system_t* post);
void post_print_memory_map(const post_system_t* post);
void post_print_sensor_status(const post_system_t* post);

// Integration Functions
bool post_integrate_with_esc_system(post_system_t* post, void* esc_system);
bool post_setup_default_test_modules(post_system_t* post);
bool post_setup_default_memory_regions(post_system_t* post);
bool post_setup_default_sensor_calibrations(post_system_t* post);

#endif // POST_SYSTEM_H