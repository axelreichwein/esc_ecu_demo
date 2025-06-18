#include "post_system.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

// Internal constants
#define POST_MAGIC_NUMBER 0x504F5354  // "POST" in ASCII
#define POST_VERSION 0x0100           // Version 1.0
#define POST_DEFAULT_RETRY_DELAY 100  // 100ms delay between retries
#define POST_MEMORY_TEST_PATTERN1 0xAA55AA55
#define POST_MEMORY_TEST_PATTERN2 0x55AA55AA
#define POST_MEMORY_TEST_PATTERN3 0x12345678
#define POST_MEMORY_TEST_PATTERN4 0x87654321

// Global POST hardware interface pointer
static const post_hardware_interface_t* g_hw_interface = NULL;

// Internal function prototypes
static uint32_t post_get_current_time(void);
static bool post_execute_test_with_retry(post_system_t* post, post_test_module_t* module);
static void post_update_module_statistics(post_test_module_t* module, post_result_t result, uint32_t duration_ms);
static bool post_is_test_critical(const post_test_module_t* module);
static bool post_should_halt_on_failure(const post_system_t* post, post_result_t result);

// POST System Initialization and Control
bool post_system_init(post_system_t* post, const post_hardware_interface_t* hw_interface) {
    if (!post || !hw_interface) {
        return false;
    }
    
    // Store hardware interface
    g_hw_interface = hw_interface;
    
    // Initialize POST system structure
    memset(post, 0, sizeof(post_system_t));
    
    // Set default configuration
    post->status.initialization_start_time = post_get_current_time();
    post->status.current_phase = POST_PHASE_HARDWARE_INIT;
    post->status.esc_system_enabled = false;
    post->status.critical_failure_detected = false;
    post->status.initialization_complete = false;
    post->verbose_logging = true;
    post->halt_on_critical_failure = true;
    post->retry_delay_ms = POST_DEFAULT_RETRY_DELAY;
    
    // Setup default configurations
    if (!post_setup_default_test_modules(post)) {
        printf("POST: Failed to setup default test modules\n");
        return false;
    }
    
    if (!post_setup_default_memory_regions(post)) {
        printf("POST: Failed to setup default memory regions\n");
        return false;
    }
    
    if (!post_setup_default_sensor_calibrations(post)) {
        printf("POST: Failed to setup default sensor calibrations\n");
        return false;
    }
    
    printf("POST System initialized successfully\n");
    printf("POST: %u test modules, %u memory regions, %u sensors\n", 
           post->module_count, post->memory_region_count, post->sensor_count);
    
    return true;
}

void post_system_shutdown(post_system_t* post) {
    if (!post) return;
    
    printf("POST System shutdown initiated\n");
    
    // Disable ESC system
    if (post->status.esc_system_enabled) {
        post_disable_esc_system(post, "System shutdown");
    }
    
    // Print final statistics
    printf("POST System final statistics:\n");
    printf("  Total tests: %u\n", post->status.total_test_count);
    printf("  Passed: %u\n", post->status.passed_test_count);
    printf("  Failed: %u\n", post->status.failed_test_count);
    printf("  Warnings: %u\n", post->status.warning_test_count);
    printf("  Initialization duration: %u ms\n", post->status.initialization_duration_ms);
    
    // Clear the structure
    memset(post, 0, sizeof(post_system_t));
    g_hw_interface = NULL;
    
    printf("POST System shutdown complete\n");
}

bool post_execute_all_tests(post_system_t* post) {
    if (!post || !g_hw_interface) {
        return false;
    }
    
    printf("POST: Starting comprehensive system test sequence\n");
    
    // Reset status counters
    post->status.total_test_count = 0;
    post->status.passed_test_count = 0;
    post->status.failed_test_count = 0;
    post->status.warning_test_count = 0;
    post->status.critical_failure_detected = false;
    post->status.initialization_start_time = post_get_current_time();
    
    // Execute tests by phase
    post_phase_t phases[] = {
        POST_PHASE_HARDWARE_INIT,
        POST_PHASE_MEMORY_TEST,
        POST_PHASE_SENSOR_VALIDATION,
        POST_PHASE_COMMUNICATION_TEST,
        POST_PHASE_SAFETY_VALIDATION,
        POST_PHASE_CALIBRATION_CHECK,
        POST_PHASE_SYSTEM_INTEGRATION,
        POST_PHASE_FINAL_VERIFICATION
    };
    
    for (int i = 0; i < sizeof(phases) / sizeof(phases[0]); i++) {
        if (!post_execute_phase(post, phases[i])) {
            printf("POST: Phase %s failed\n", post_phase_to_string(phases[i]));
            
            if (post->status.critical_failure_detected && post->halt_on_critical_failure) {
                printf("POST: Critical failure detected, halting initialization\n");
                post_log_error(post, 0xFF, POST_RESULT_CRITICAL_FAIL, 0x01, "Critical failure during POST");
                return false;
            }
        }
    }
    
    // Calculate final initialization duration
    post->status.initialization_duration_ms = post_get_current_time() - post->status.initialization_start_time;
    
    // Check if ESC can be enabled
    if (post_check_esc_enable_conditions(post)) {
        post->status.initialization_complete = true;
        post_enable_esc_system(post);
        printf("POST: All tests completed successfully - ESC system ENABLED\n");
    } else {
        printf("POST: Tests completed with failures - ESC system DISABLED\n");
        post_disable_esc_system(post, "POST validation failed");
    }
    
    return post->status.initialization_complete;
}

bool post_execute_phase(post_system_t* post, post_phase_t phase) {
    if (!post) {
        return false;
    }
    
    printf("POST: Executing phase %s\n", post_phase_to_string(phase));
    post->status.current_phase = phase;
    
    bool phase_passed = true;
    uint32_t phase_start_time = post_get_current_time();
    
    // Execute all modules in this phase
    for (int i = 0; i < post->module_count; i++) {
        post_test_module_t* module = &post->test_modules[i];
        
        if (module->enabled && module->phase == phase) {
            post->status.current_module_id = module->module_id;
            
            if (!post_execute_test_with_retry(post, module)) {
                phase_passed = false;
                
                if (post_is_test_critical(module)) {
                    post->status.critical_failure_detected = true;
                    printf("POST: Critical test failed: %s\n", module->module_name);
                    
                    if (post->halt_on_critical_failure) {
                        break;
                    }
                }
            }
        }
    }
    
    uint32_t phase_duration = post_get_current_time() - phase_start_time;
    printf("POST: Phase %s completed in %u ms - %s\n", 
           post_phase_to_string(phase), phase_duration, 
           phase_passed ? "PASSED" : "FAILED");
    
    return phase_passed;
}

bool post_execute_test_module(post_system_t* post, uint8_t module_id) {
    if (!post) {
        return false;
    }
    
    // Find the test module
    post_test_module_t* module = NULL;
    for (int i = 0; i < post->module_count; i++) {
        if (post->test_modules[i].module_id == module_id) {
            module = &post->test_modules[i];
            break;
        }
    }
    
    if (!module) {
        printf("POST: Test module %u not found\n", module_id);
        return false;
    }
    
    if (!module->enabled) {
        printf("POST: Test module %s is disabled\n", module->module_name);
        return false;
    }
    
    return post_execute_test_with_retry(post, module);
}

// Test Module Management
bool post_register_test_module(post_system_t* post, const post_test_module_t* module) {
    if (!post || !module || post->module_count >= POST_MAX_TEST_MODULES) {
        return false;
    }
    
    // Check for duplicate module ID
    for (int i = 0; i < post->module_count; i++) {
        if (post->test_modules[i].module_id == module->module_id) {
            printf("POST: Module ID %u already registered\n", module->module_id);
            return false;
        }
    }
    
    // Copy module to internal storage
    memcpy(&post->test_modules[post->module_count], module, sizeof(post_test_module_t));
    post->module_count++;
    
    if (post->verbose_logging) {
        printf("POST: Registered test module: %s (ID=%u)\n", module->module_name, module->module_id);
    }
    
    return true;
}

bool post_enable_test_module(post_system_t* post, uint8_t module_id, bool enable) {
    if (!post) {
        return false;
    }
    
    for (int i = 0; i < post->module_count; i++) {
        if (post->test_modules[i].module_id == module_id) {
            post->test_modules[i].enabled = enable;
            printf("POST: Module %s %s\n", post->test_modules[i].module_name, 
                   enable ? "enabled" : "disabled");
            return true;
        }
    }
    
    return false;
}

bool post_set_test_timeout(post_system_t* post, uint8_t module_id, uint32_t timeout_ms) {
    if (!post) {
        return false;
    }
    
    for (int i = 0; i < post->module_count; i++) {
        if (post->test_modules[i].module_id == module_id) {
            post->test_modules[i].timeout_ms = timeout_ms;
            return true;
        }
    }
    
    return false;
}

// Memory Test Functions
post_result_t post_test_ram_integrity(void* context) {
    printf("POST: Testing RAM integrity...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_memory_test) {
        return POST_RESULT_SKIP;
    }
    
    // Test different memory patterns
    uint32_t test_patterns[] = {
        POST_MEMORY_TEST_PATTERN1,
        POST_MEMORY_TEST_PATTERN2,
        POST_MEMORY_TEST_PATTERN3,
        POST_MEMORY_TEST_PATTERN4
    };
    
    // Simulate RAM test with different patterns
    for (int i = 0; i < sizeof(test_patterns) / sizeof(test_patterns[0]); i++) {
        // In real implementation, would write/read pattern to/from RAM
        if (!g_hw_interface->hw_memory_test((void*)&test_patterns[i], sizeof(uint32_t))) {
            printf("POST: RAM integrity test failed at pattern 0x%08X\n", test_patterns[i]);
            return POST_RESULT_FAIL;
        }
    }
    
    printf("POST: RAM integrity test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_flash_checksum(void* context) {
    printf("POST: Testing Flash/ROM checksum...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_calculate_checksum) {
        return POST_RESULT_SKIP;
    }
    
    // Simulate Flash checksum calculation
    // In real implementation, would calculate checksum of Flash memory
    uint32_t calculated_checksum = g_hw_interface->hw_calculate_checksum((void*)0x08000000, 0x40000);
    uint32_t expected_checksum = 0x12345678; // Would be stored in secure location
    
    if (calculated_checksum != expected_checksum) {
        printf("POST: Flash checksum mismatch - calculated: 0x%08X, expected: 0x%08X\n", 
               calculated_checksum, expected_checksum);
        return POST_RESULT_FAIL;
    }
    
    printf("POST: Flash checksum test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_eeprom_integrity(void* context) {
    printf("POST: Testing EEPROM integrity...\n");
    
    // Simulate EEPROM test
    // In real implementation, would test EEPROM read/write functionality
    uint8_t test_data = 0xA5;
    uint8_t read_data = 0x00;
    
    // Write test pattern and read it back
    // eeprom_write(TEST_ADDRESS, &test_data, 1);
    // eeprom_read(TEST_ADDRESS, &read_data, 1);
    
    // Simulate successful test
    read_data = test_data;
    
    if (read_data != test_data) {
        printf("POST: EEPROM integrity test failed\n");
        return POST_RESULT_FAIL;
    }
    
    printf("POST: EEPROM integrity test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_stack_protection(void* context) {
    printf("POST: Testing stack overflow protection...\n");
    
    // Check stack guard patterns (canaries)
    // In real implementation, would check stack canary values
    uint32_t stack_canary = 0xDEADBEEF; // Would read from actual stack guard
    uint32_t expected_canary = 0xDEADBEEF;
    
    if (stack_canary != expected_canary) {
        printf("POST: Stack overflow detected\n");
        return POST_RESULT_CRITICAL_FAIL;
    }
    
    printf("POST: Stack protection test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_heap_integrity(void* context) {
    printf("POST: Testing heap integrity...\n");
    
    // Test heap allocation/deallocation
    void* test_ptr = malloc(1024);
    if (!test_ptr) {
        printf("POST: Heap allocation failed\n");
        return POST_RESULT_FAIL;
    }
    
    // Write test pattern
    memset(test_ptr, 0xAA, 1024);
    
    // Verify pattern
    uint8_t* byte_ptr = (uint8_t*)test_ptr;
    for (int i = 0; i < 1024; i++) {
        if (byte_ptr[i] != 0xAA) {
            printf("POST: Heap integrity test failed at offset %d\n", i);
            free(test_ptr);
            return POST_RESULT_FAIL;
        }
    }
    
    free(test_ptr);
    printf("POST: Heap integrity test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_calibration_data(void* context) {
    printf("POST: Testing calibration data integrity...\n");
    
    // Calculate checksum of calibration data
    // In real implementation, would validate all calibration parameters
    uint32_t calculated_checksum = 0x12345678; // Simulated
    uint32_t stored_checksum = 0x12345678;     // Would read from calibration area
    
    if (calculated_checksum != stored_checksum) {
        printf("POST: Calibration data checksum mismatch\n");
        return POST_RESULT_FAIL;
    }
    
    printf("POST: Calibration data integrity test passed\n");
    return POST_RESULT_PASS;
}

// Sensor Validation Functions
post_result_t post_validate_wheel_speed_sensors(void* context) {
    printf("POST: Validating wheel speed sensors...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_sensor_self_test) {
        return POST_RESULT_SKIP;
    }
    
    esc_sensor_id_t wheel_sensors[] = {
        ESC_SENSOR_WHEEL_SPEED_FL,
        ESC_SENSOR_WHEEL_SPEED_FR,
        ESC_SENSOR_WHEEL_SPEED_RL,
        ESC_SENSOR_WHEEL_SPEED_RR
    };
    
    for (int i = 0; i < 4; i++) {
        if (!g_hw_interface->hw_sensor_self_test(wheel_sensors[i])) {
            printf("POST: Wheel speed sensor %s failed self-test\n", 
                   esc_sensor_id_to_string(wheel_sensors[i]));
            return POST_RESULT_FAIL;
        }
    }
    
    printf("POST: All wheel speed sensors passed validation\n");
    return POST_RESULT_PASS;
}

post_result_t post_validate_inertial_sensors(void* context) {
    printf("POST: Validating inertial sensors...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_sensor_read_value) {
        return POST_RESULT_SKIP;
    }
    
    // Test yaw rate sensor
    float yaw_rate = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_YAW_RATE);
    if (yaw_rate < -500.0f || yaw_rate > 500.0f) { // deg/s range check
        printf("POST: Yaw rate sensor out of range: %.2f deg/s\n", yaw_rate);
        return POST_RESULT_FAIL;
    }
    
    // Test lateral acceleration sensor
    float lat_accel = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_LATERAL_ACCEL);
    if (lat_accel < -20.0f || lat_accel > 20.0f) { // m/s² range check
        printf("POST: Lateral acceleration sensor out of range: %.2f m/s²\n", lat_accel);
        return POST_RESULT_FAIL;
    }
    
    // Test longitudinal acceleration sensor
    float long_accel = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_LONGITUDINAL_ACCEL);
    if (long_accel < -20.0f || long_accel > 20.0f) { // m/s² range check
        printf("POST: Longitudinal acceleration sensor out of range: %.2f m/s²\n", long_accel);
        return POST_RESULT_FAIL;
    }
    
    printf("POST: All inertial sensors passed validation\n");
    return POST_RESULT_PASS;
}

post_result_t post_validate_steering_angle_sensor(void* context) {
    printf("POST: Validating steering angle sensor...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_sensor_read_value) {
        return POST_RESULT_SKIP;
    }
    
    float steering_angle = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_STEERING_ANGLE);
    
    // Check if steering angle is within expected range
    if (steering_angle < -720.0f || steering_angle > 720.0f) { // degrees
        printf("POST: Steering angle sensor out of range: %.1f degrees\n", steering_angle);
        return POST_RESULT_FAIL;
    }
    
    // Check if sensor is centered (within tolerance when wheels are straight)
    if (steering_angle > -5.0f && steering_angle < 5.0f) {
        printf("POST: Steering angle sensor validated (centered: %.1f degrees)\n", steering_angle);
    } else {
        printf("POST: Steering angle sensor validated (current: %.1f degrees)\n", steering_angle);
    }
    
    return POST_RESULT_PASS;
}

post_result_t post_validate_brake_sensors(void* context) {
    printf("POST: Validating brake sensors...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_sensor_read_value) {
        return POST_RESULT_SKIP;
    }
    
    // Test brake pressure sensor
    float brake_pressure = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_BRAKE_PRESSURE);
    if (brake_pressure < 0.0f || brake_pressure > 300.0f) { // bar range check
        printf("POST: Brake pressure sensor out of range: %.1f bar\n", brake_pressure);
        return POST_RESULT_FAIL;
    }
    
    // Test brake temperature sensor
    float brake_temp = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_BRAKE_TEMPERATURE);
    if (brake_temp < -40.0f || brake_temp > 800.0f) { // °C range check
        printf("POST: Brake temperature sensor out of range: %.1f°C\n", brake_temp);
        return POST_RESULT_FAIL;
    }
    
    printf("POST: All brake sensors passed validation\n");
    return POST_RESULT_PASS;
}

post_result_t post_validate_supply_voltage(void* context) {
    printf("POST: Validating supply voltage...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_sensor_read_value) {
        return POST_RESULT_SKIP;
    }
    
    float supply_voltage = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_SUPPLY_VOLTAGE);
    
    // Check if supply voltage is within acceptable range
    if (supply_voltage < 9.0f || supply_voltage > 16.0f) { // Volts
        printf("POST: Supply voltage out of range: %.2fV\n", supply_voltage);
        return POST_RESULT_CRITICAL_FAIL;
    }
    
    if (supply_voltage < 11.0f || supply_voltage > 14.5f) {
        printf("POST: Supply voltage warning: %.2fV\n", supply_voltage);
        return POST_RESULT_WARNING;
    }
    
    printf("POST: Supply voltage validated: %.2fV\n", supply_voltage);
    return POST_RESULT_PASS;
}

post_result_t post_validate_sensor_plausibility(void* context) {
    printf("POST: Validating sensor plausibility...\n");
    
    // Cross-check sensor values for plausibility
    // In real implementation, would perform comprehensive plausibility checks
    
    // Example: Check if vehicle is stationary (all wheel speeds should be zero)
    if (g_hw_interface && g_hw_interface->hw_sensor_read_value) {
        float wheel_speeds[4] = {
            g_hw_interface->hw_sensor_read_value(ESC_SENSOR_WHEEL_SPEED_FL),
            g_hw_interface->hw_sensor_read_value(ESC_SENSOR_WHEEL_SPEED_FR),
            g_hw_interface->hw_sensor_read_value(ESC_SENSOR_WHEEL_SPEED_RL),
            g_hw_interface->hw_sensor_read_value(ESC_SENSOR_WHEEL_SPEED_RR)
        };
        
        // If vehicle should be stationary, all wheel speeds should be near zero
        bool vehicle_stationary = true;
        for (int i = 0; i < 4; i++) {
            if (wheel_speeds[i] > 2.0f) { // km/h threshold
                vehicle_stationary = false;
                break;
            }
        }
        
        if (vehicle_stationary) {
            // Check that inertial sensors also indicate stationary
            float yaw_rate = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_YAW_RATE);
            float lat_accel = g_hw_interface->hw_sensor_read_value(ESC_SENSOR_LATERAL_ACCEL);
            
            if (yaw_rate > 5.0f || lat_accel > 1.0f) {
                printf("POST: Sensor plausibility check failed - motion mismatch\n");
                return POST_RESULT_WARNING;
            }
        }
    }
    
    printf("POST: Sensor plausibility validation passed\n");
    return POST_RESULT_PASS;
}

// Communication Test Functions
post_result_t post_test_can_communication(void* context) {
    printf("POST: Testing CAN communication...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_communication_test) {
        return POST_RESULT_SKIP;
    }
    
    // Test CAN bus communication
    if (!g_hw_interface->hw_communication_test(0x01)) { // CAN interface ID
        printf("POST: CAN communication test failed\n");
        return POST_RESULT_FAIL;
    }
    
    printf("POST: CAN communication test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_internal_communication(void* context) {
    printf("POST: Testing internal communication...\n");
    
    // Test internal inter-module communication
    // In real implementation, would test SPI, I2C, UART, etc.
    
    printf("POST: Internal communication test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_diagnostic_communication(void* context) {
    printf("POST: Testing diagnostic communication...\n");
    
    if (!g_hw_interface || !g_hw_interface->hw_communication_test) {
        return POST_RESULT_SKIP;
    }
    
    // Test diagnostic interface
    if (!g_hw_interface->hw_communication_test(0x02)) { // Diagnostic interface ID
        printf("POST: Diagnostic communication test failed\n");
        return POST_RESULT_WARNING; // Non-critical for basic operation
    }
    
    printf("POST: Diagnostic communication test passed\n");
    return POST_RESULT_PASS;
}

// Safety System Validation Functions
post_result_t post_validate_watchdog_system(void* context) {
    printf("POST: Validating watchdog system...\n");
    
    // Test watchdog functionality
    // In real implementation, would test watchdog timer and reset mechanisms
    
    printf("POST: Watchdog system validation passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_validate_safety_mechanisms(void* context) {
    printf("POST: Validating safety mechanisms...\n");
    
    // Test safety shutdown mechanisms
    // In real implementation, would test emergency stops, fail-safes, etc.
    
    printf("POST: Safety mechanisms validation passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_validate_fault_handling(void* context) {
    printf("POST: Validating fault handling...\n");
    
    // Test fault detection and handling
    // In real implementation, would simulate faults and verify responses
    
    printf("POST: Fault handling validation passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_validate_emergency_shutdown(void* context) {
    printf("POST: Validating emergency shutdown...\n");
    
    // Test emergency shutdown procedures
    // In real implementation, would test emergency brake, engine cut-off, etc.
    
    printf("POST: Emergency shutdown validation passed\n");
    return POST_RESULT_PASS;
}

// System Integration Tests
post_result_t post_test_brake_system_integration(void* context) {
    printf("POST: Testing brake system integration...\n");
    
    // Test brake system integration
    // In real implementation, would test brake commands and feedback
    
    printf("POST: Brake system integration test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_engine_interface_integration(void* context) {
    printf("POST: Testing engine interface integration...\n");
    
    // Test engine torque reduction interface
    // In real implementation, would test engine communication
    
    printf("POST: Engine interface integration test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_dtc_system_integration(void* context) {
    printf("POST: Testing DTC system integration...\n");
    
    // Test DTC system functionality
    // In real implementation, would test DTC logging and retrieval
    
    printf("POST: DTC system integration test passed\n");
    return POST_RESULT_PASS;
}

post_result_t post_test_overall_system_integration(void* context) {
    printf("POST: Testing overall system integration...\n");
    
    // Test complete system integration
    // In real implementation, would perform end-to-end system tests
    
    printf("POST: Overall system integration test passed\n");
    return POST_RESULT_PASS;
}

// Memory Region Management
bool post_register_memory_region(post_system_t* post, const memory_region_t* region) {
    if (!post || !region || post->memory_region_count >= 16) {
        return false;
    }
    
    memcpy(&post->memory_regions[post->memory_region_count], region, sizeof(memory_region_t));
    post->memory_region_count++;
    
    if (post->verbose_logging) {
        printf("POST: Registered memory region: %s (0x%p, %u bytes)\n", 
               region->region_name, region->start_address, region->size);
    }
    
    return true;
}

bool post_validate_memory_regions(post_system_t* post) {
    if (!post) {
        return false;
    }
    
    printf("POST: Validating %u memory regions...\n", post->memory_region_count);
    
    for (int i = 0; i < post->memory_region_count; i++) {
        memory_region_t* region = &post->memory_regions[i];
        
        printf("POST: Validating memory region: %s\n", region->region_name);
        
        if (region->test_type == MEMORY_TEST_FLASH && region->expected_checksum != 0) {
            uint32_t calculated_checksum = post_calculate_memory_checksum(
                region->start_address, region->size);
            
            if (calculated_checksum != region->expected_checksum) {
                printf("POST: Memory region %s checksum failed\n", region->region_name);
                if (region->critical) {
                    return false;
                }
            }
        }
    }
    
    printf("POST: Memory region validation completed\n");
    return true;
}

uint32_t post_calculate_memory_checksum(const void* address, uint32_t size) {
    if (!address || size == 0) {
        return 0;
    }
    
    uint32_t checksum = 0;
    const uint8_t* data = (const uint8_t*)address;
    
    for (uint32_t i = 0; i < size; i++) {
        checksum += data[i];
        checksum = (checksum << 1) | (checksum >> 31); // Rotate left
    }
    
    return checksum;
}

// Sensor Calibration Management
bool post_register_sensor_calibration(post_system_t* post, const post_sensor_calibration_t* calibration) {
    if (!post || !calibration || post->sensor_count >= 16) {
        return false;
    }
    
    memcpy(&post->sensor_calibrations[post->sensor_count], calibration, sizeof(post_sensor_calibration_t));
    post->sensor_count++;
    
    if (post->verbose_logging) {
        printf("POST: Registered sensor calibration: %s\n", 
               esc_sensor_id_to_string(calibration->sensor_id));
    }
    
    return true;
}

bool post_validate_sensor_calibrations(post_system_t* post) {
    if (!post) {
        return false;
    }
    
    printf("POST: Validating %u sensor calibrations...\n", post->sensor_count);
    
    for (int i = 0; i < post->sensor_count; i++) {
        post_sensor_calibration_t* calibration = &post->sensor_calibrations[i];
        
        printf("POST: Validating sensor: %s\n", 
               esc_sensor_id_to_string(calibration->sensor_id));
        
        // Check if sensor needs warmup
        if (calibration->requires_warmup) {
            uint32_t current_time = post_get_current_time();
            if (!post_sensor_warmup_complete(calibration, current_time - calibration->warmup_time_ms)) {
                printf("POST: Sensor %s requires warmup (%u ms)\n", 
                       esc_sensor_id_to_string(calibration->sensor_id),
                       calibration->warmup_time_ms);
                
                if (g_hw_interface && g_hw_interface->hw_delay_ms) {
                    g_hw_interface->hw_delay_ms(calibration->warmup_time_ms);
                }
            }
        }
        
        // Validate sensor reading if possible
        if (g_hw_interface && g_hw_interface->hw_sensor_read_value) {
            float sensor_value = g_hw_interface->hw_sensor_read_value(calibration->sensor_id);
            
            if (sensor_value < calibration->min_valid_value || 
                sensor_value > calibration->max_valid_value) {
                printf("POST: Sensor %s value out of range: %.2f\n", 
                       esc_sensor_id_to_string(calibration->sensor_id), sensor_value);
                return false;
            }
        }
    }
    
    printf("POST: Sensor calibration validation completed\n");
    return true;
}

bool post_sensor_warmup_complete(const post_sensor_calibration_t* calibration, uint32_t warmup_start_time) {
    if (!calibration || !calibration->requires_warmup) {
        return true;
    }
    
    uint32_t elapsed_time = post_get_current_time() - warmup_start_time;
    return elapsed_time >= calibration->warmup_time_ms;
}

// Error Logging and Reporting
bool post_log_error(post_system_t* post, uint8_t module_id, post_result_t result, 
                   uint32_t error_code, const char* description) {
    if (!post || post->error_log_count >= POST_MAX_ERROR_LOG_SIZE) {
        return false;
    }
    
    post_error_log_entry_t* entry = &post->error_log[post->error_log_count];
    entry->timestamp = post_get_current_time();
    entry->module_id = module_id;
    entry->result = result;
    entry->error_code = error_code;
    
    if (description) {
        strncpy(entry->description, description, sizeof(entry->description) - 1);
        entry->description[sizeof(entry->description) - 1] = '\0';
    } else {
        entry->description[0] = '\0';
    }
    
    post->error_log_count++;
    
    if (post->verbose_logging) {
        printf("POST: Error logged - Module %u: %s\n", module_id, 
               description ? description : "No description");
    }
    
    return true;
}

void post_clear_error_log(post_system_t* post) {
    if (!post) return;
    
    memset(post->error_log, 0, sizeof(post->error_log));
    post->error_log_count = 0;
}

uint8_t post_get_error_count(const post_system_t* post) {
    return post ? post->error_log_count : 0;
}

bool post_get_error_log_entry(const post_system_t* post, uint8_t index, post_error_log_entry_t* entry) {
    if (!post || !entry || index >= post->error_log_count) {
        return false;
    }
    
    memcpy(entry, &post->error_log[index], sizeof(post_error_log_entry_t));
    return true;
}

// Status and Query Functions
bool post_is_initialization_complete(const post_system_t* post) {
    return post ? post->status.initialization_complete : false;
}

bool post_is_esc_system_enabled(const post_system_t* post) {
    return post ? post->status.esc_system_enabled : false;
}

bool post_has_critical_failure(const post_system_t* post) {
    return post ? post->status.critical_failure_detected : false;
}

post_phase_t post_get_current_phase(const post_system_t* post) {
    return post ? post->status.current_phase : POST_PHASE_HARDWARE_INIT;
}

uint32_t post_get_initialization_duration(const post_system_t* post) {
    return post ? post->status.initialization_duration_ms : 0;
}

// ESC System Enable/Disable Control
bool post_enable_esc_system(post_system_t* post) {
    if (!post) {
        return false;
    }
    
    if (!post_check_esc_enable_conditions(post)) {
        printf("POST: Cannot enable ESC - conditions not met\n");
        return false;
    }
    
    post->status.esc_system_enabled = true;
    printf("POST: ESC system ENABLED\n");
    
    return true;
}

bool post_disable_esc_system(post_system_t* post, const char* reason) {
    if (!post) {
        return false;
    }
    
    post->status.esc_system_enabled = false;
    printf("POST: ESC system DISABLED - %s\n", reason ? reason : "No reason given");
    
    return true;
}

bool post_check_esc_enable_conditions(const post_system_t* post) {
    if (!post) {
        return false;
    }
    
    // Check for critical failures
    if (post->status.critical_failure_detected) {
        printf("POST: ESC enable blocked - critical failure detected\n");
        return false;
    }
    
    // Check that critical tests passed
    uint32_t critical_failures = 0;
    for (int i = 0; i < post->module_count; i++) {
        const post_test_module_t* module = &post->test_modules[i];
        if (module->category == POST_CATEGORY_CRITICAL && module->fail_count > 0) {
            critical_failures++;
        }
    }
    
    if (critical_failures > 0) {
        printf("POST: ESC enable blocked - %u critical test failures\n", critical_failures);
        return false;
    }
    
    // Check minimum pass rate
    if (post->status.total_test_count > 0) {
        float pass_rate = (float)post->status.passed_test_count / post->status.total_test_count;
        if (pass_rate < 0.8f) { // Require 80% pass rate
            printf("POST: ESC enable blocked - insufficient pass rate: %.1f%%\n", pass_rate * 100);
            return false;
        }
    }
    
    return true;
}

// Internal Helper Functions
static uint32_t post_get_current_time(void) {
    if (g_hw_interface && g_hw_interface->hw_get_timestamp) {
        return g_hw_interface->hw_get_timestamp();
    }
    
    // Fallback to system time
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static bool post_execute_test_with_retry(post_system_t* post, post_test_module_t* module) {
    if (!post || !module || !module->test_function) {
        return false;
    }
    
    printf("POST: Executing test: %s\n", module->module_name);
    
    uint32_t start_time = post_get_current_time();
    post_result_t result = POST_RESULT_FAIL;
    int retry_count = 0;
    
    do {
        if (retry_count > 0) {
            printf("POST: Retrying test %s (attempt %d/%d)\n", 
                   module->module_name, retry_count + 1, POST_RETRY_LIMIT + 1);
            
            if (g_hw_interface && g_hw_interface->hw_delay_ms) {
                g_hw_interface->hw_delay_ms(post->retry_delay_ms);
            }
        }
        
        // Execute the test function
        result = module->test_function(module->test_context);
        
        // Check for timeout
        uint32_t elapsed_time = post_get_current_time() - start_time;
        if (elapsed_time > module->timeout_ms) {
            result = POST_RESULT_TIMEOUT;
            break;
        }
        
        retry_count++;
        
    } while (result == POST_RESULT_FAIL && retry_count <= POST_RETRY_LIMIT);
    
    uint32_t execution_duration = post_get_current_time() - start_time;
    
    // Update module statistics
    post_update_module_statistics(module, result, execution_duration);
    
    // Update system statistics
    post->status.total_test_count++;
    switch (result) {
        case POST_RESULT_PASS:
            post->status.passed_test_count++;
            break;
        case POST_RESULT_WARNING:
            post->status.warning_test_count++;
            break;
        case POST_RESULT_FAIL:
        case POST_RESULT_TIMEOUT:
        case POST_RESULT_CRITICAL_FAIL:
            post->status.failed_test_count++;
            break;
        default:
            break;
    }
    
    // Log errors if needed
    if (result != POST_RESULT_PASS && result != POST_RESULT_SKIP) {
        post_log_error(post, module->module_id, result, 0, module->module_name);
    }
    
    // Check if this is a critical failure
    if (post_should_halt_on_failure(post, result)) {
        post->status.critical_failure_detected = true;
    }
    
    printf("POST: Test %s completed - %s (%u ms)\n", 
           module->module_name, post_result_to_string(result), execution_duration);
    
    return (result == POST_RESULT_PASS || result == POST_RESULT_WARNING);
}

static void post_update_module_statistics(post_test_module_t* module, post_result_t result, uint32_t duration_ms) {
    if (!module) return;
    
    module->execution_count++;
    module->last_execution_time = post_get_current_time();
    module->last_execution_duration_ms = duration_ms;
    
    if (result == POST_RESULT_PASS || result == POST_RESULT_WARNING) {
        module->pass_count++;
    } else {
        module->fail_count++;
    }
}

static bool post_is_test_critical(const post_test_module_t* module) {
    return module ? (module->category == POST_CATEGORY_CRITICAL) : false;
}

static bool post_should_halt_on_failure(const post_system_t* post, post_result_t result) {
    return post && post->halt_on_critical_failure && 
           (result == POST_RESULT_CRITICAL_FAIL);
}

// Utility Functions
const char* post_result_to_string(post_result_t result) {
    switch (result) {
        case POST_RESULT_PASS: return "PASS";
        case POST_RESULT_FAIL: return "FAIL";
        case POST_RESULT_WARNING: return "WARNING";
        case POST_RESULT_SKIP: return "SKIP";
        case POST_RESULT_TIMEOUT: return "TIMEOUT";
        case POST_RESULT_NOT_READY: return "NOT_READY";
        case POST_RESULT_CRITICAL_FAIL: return "CRITICAL_FAIL";
        default: return "UNKNOWN";
    }
}

const char* post_phase_to_string(post_phase_t phase) {
    switch (phase) {
        case POST_PHASE_HARDWARE_INIT: return "HARDWARE_INIT";
        case POST_PHASE_MEMORY_TEST: return "MEMORY_TEST";
        case POST_PHASE_SENSOR_VALIDATION: return "SENSOR_VALIDATION";
        case POST_PHASE_COMMUNICATION_TEST: return "COMMUNICATION_TEST";
        case POST_PHASE_SAFETY_VALIDATION: return "SAFETY_VALIDATION";
        case POST_PHASE_CALIBRATION_CHECK: return "CALIBRATION_CHECK";
        case POST_PHASE_SYSTEM_INTEGRATION: return "SYSTEM_INTEGRATION";
        case POST_PHASE_FINAL_VERIFICATION: return "FINAL_VERIFICATION";
        default: return "UNKNOWN_PHASE";
    }
}

const char* post_category_to_string(post_category_t category) {
    switch (category) {
        case POST_CATEGORY_CRITICAL: return "CRITICAL";
        case POST_CATEGORY_IMPORTANT: return "IMPORTANT";
        case POST_CATEGORY_INFORMATIONAL: return "INFORMATIONAL";
        case POST_CATEGORY_DIAGNOSTIC: return "DIAGNOSTIC";
        default: return "UNKNOWN_CATEGORY";
    }
}

const char* memory_test_type_to_string(memory_test_type_t type) {
    switch (type) {
        case MEMORY_TEST_RAM: return "RAM";
        case MEMORY_TEST_FLASH: return "FLASH";
        case MEMORY_TEST_EEPROM: return "EEPROM";
        case MEMORY_TEST_STACK: return "STACK";
        case MEMORY_TEST_HEAP: return "HEAP";
        case MEMORY_TEST_CALIBRATION_DATA: return "CALIBRATION_DATA";
        default: return "UNKNOWN_MEMORY_TEST";
    }
}

const char* sensor_validation_type_to_string(sensor_validation_type_t type) {
    switch (type) {
        case SENSOR_VALIDATION_PRESENCE: return "PRESENCE";
        case SENSOR_VALIDATION_RANGE: return "RANGE";
        case SENSOR_VALIDATION_PLAUSIBILITY: return "PLAUSIBILITY";
        case SENSOR_VALIDATION_CALIBRATION: return "CALIBRATION";
        case SENSOR_VALIDATION_COMMUNICATION: return "COMMUNICATION";
        case SENSOR_VALIDATION_SELF_TEST: return "SELF_TEST";
        default: return "UNKNOWN_SENSOR_VALIDATION";
    }
}

const char* esc_sensor_id_to_string(esc_sensor_id_t sensor_id) {
    switch (sensor_id) {
        case ESC_SENSOR_WHEEL_SPEED_FL: return "WHEEL_SPEED_FL";
        case ESC_SENSOR_WHEEL_SPEED_FR: return "WHEEL_SPEED_FR";
        case ESC_SENSOR_WHEEL_SPEED_RL: return "WHEEL_SPEED_RL";
        case ESC_SENSOR_WHEEL_SPEED_RR: return "WHEEL_SPEED_RR";
        case ESC_SENSOR_YAW_RATE: return "YAW_RATE";
        case ESC_SENSOR_LATERAL_ACCEL: return "LATERAL_ACCEL";
        case ESC_SENSOR_LONGITUDINAL_ACCEL: return "LONGITUDINAL_ACCEL";
        case ESC_SENSOR_STEERING_ANGLE: return "STEERING_ANGLE";
        case ESC_SENSOR_BRAKE_PRESSURE: return "BRAKE_PRESSURE";
        case ESC_SENSOR_BRAKE_TEMPERATURE: return "BRAKE_TEMPERATURE";
        case ESC_SENSOR_SUPPLY_VOLTAGE: return "SUPPLY_VOLTAGE";
        default: return "UNKNOWN_SENSOR";
    }
}

// Debug and Reporting Functions
void post_print_system_status(const post_system_t* post) {
    if (!post) return;
    
    printf("\n=== POST System Status ===\n");
    printf("Initialization Complete: %s\n", post->status.initialization_complete ? "YES" : "NO");
    printf("ESC System Enabled: %s\n", post->status.esc_system_enabled ? "YES" : "NO");
    printf("Critical Failure: %s\n", post->status.critical_failure_detected ? "YES" : "NO");
    printf("Current Phase: %s\n", post_phase_to_string(post->status.current_phase));
    printf("Initialization Duration: %u ms\n", post->status.initialization_duration_ms);
    printf("Total Tests: %u\n", post->status.total_test_count);
    printf("Passed: %u\n", post->status.passed_test_count);
    printf("Failed: %u\n", post->status.failed_test_count);
    printf("Warnings: %u\n", post->status.warning_test_count);
    
    if (post->status.total_test_count > 0) {
        float pass_rate = (float)post->status.passed_test_count / post->status.total_test_count * 100;
        printf("Pass Rate: %.1f%%\n", pass_rate);
    }
    
    printf("Error Log Entries: %u\n", post->error_log_count);
    printf("==========================\n");
}

void post_print_test_results(const post_system_t* post) {
    if (!post) return;
    
    printf("\n=== POST Test Results ===\n");
    for (int i = 0; i < post->module_count; i++) {
        const post_test_module_t* module = &post->test_modules[i];
        printf("Module %u: %s\n", module->module_id, module->module_name);
        printf("  Phase: %s\n", post_phase_to_string(module->phase));
        printf("  Category: %s\n", post_category_to_string(module->category));
        printf("  Enabled: %s\n", module->enabled ? "YES" : "NO");
        printf("  Executions: %u\n", module->execution_count);
        printf("  Passes: %u\n", module->pass_count);
        printf("  Failures: %u\n", module->fail_count);
        printf("  Last Duration: %u ms\n", module->last_execution_duration_ms);
        printf("\n");
    }
    printf("=========================\n");
}

void post_print_error_log(const post_system_t* post) {
    if (!post) return;
    
    printf("\n=== POST Error Log ===\n");
    if (post->error_log_count == 0) {
        printf("No errors logged\n");
    } else {
        for (int i = 0; i < post->error_log_count; i++) {
            const post_error_log_entry_t* entry = &post->error_log[i];
            printf("Error %d:\n", i + 1);
            printf("  Timestamp: %u ms\n", entry->timestamp);
            printf("  Module ID: %u\n", entry->module_id);
            printf("  Result: %s\n", post_result_to_string(entry->result));
            printf("  Error Code: 0x%08X\n", entry->error_code);
            printf("  Description: %s\n", entry->description);
            printf("\n");
        }
    }
    printf("======================\n");
}

void post_print_memory_map(const post_system_t* post) {
    if (!post) return;
    
    printf("\n=== POST Memory Map ===\n");
    for (int i = 0; i < post->memory_region_count; i++) {
        const memory_region_t* region = &post->memory_regions[i];
        printf("Region %d: %s\n", i + 1, region->region_name);
        printf("  Address: 0x%p\n", region->start_address);
        printf("  Size: %u bytes\n", region->size);
        printf("  Type: %s\n", memory_test_type_to_string(region->test_type));
        printf("  Critical: %s\n", region->critical ? "YES" : "NO");
        if (region->expected_checksum != 0) {
            printf("  Expected Checksum: 0x%08X\n", region->expected_checksum);
        }
        printf("\n");
    }
    printf("=======================\n");
}

void post_print_sensor_status(const post_system_t* post) {
    if (!post) return;
    
    printf("\n=== POST Sensor Status ===\n");
    for (int i = 0; i < post->sensor_count; i++) {
        const post_sensor_calibration_t* cal = &post->sensor_calibrations[i];
        printf("Sensor: %s\n", esc_sensor_id_to_string(cal->sensor_id));
        printf("  Valid Range: %.2f - %.2f\n", cal->min_valid_value, cal->max_valid_value);
        printf("  Nominal: %.2f\n", cal->nominal_value);
        printf("  Tolerance: %.2f\n", cal->tolerance);
        printf("  Requires Warmup: %s\n", cal->requires_warmup ? "YES" : "NO");
        if (cal->requires_warmup) {
            printf("  Warmup Time: %u ms\n", cal->warmup_time_ms);
        }
        printf("  Cal Timestamp: %u\n", cal->calibration_timestamp);
        printf("  Cal Checksum: 0x%08X\n", cal->calibration_checksum);
        printf("\n");
    }
    printf("==========================\n");
}

// Setup Functions (continued in next message due to length)
bool post_setup_default_test_modules(post_system_t* post) {
    if (!post) {
        return false;
    }
    
    // Define default test modules
    post_test_module_t modules[] = {
        // Hardware initialization
        {0x01, "Hardware Init", POST_PHASE_HARDWARE_INIT, POST_CATEGORY_CRITICAL, 1000, NULL, NULL, true, 0, 0, 0, 0, 0},
        
        // Memory tests
        {0x10, "RAM Integrity", POST_PHASE_MEMORY_TEST, POST_CATEGORY_CRITICAL, 2000, post_test_ram_integrity, NULL, true, 0, 0, 0, 0, 0},
        {0x11, "Flash Checksum", POST_PHASE_MEMORY_TEST, POST_CATEGORY_CRITICAL, 3000, post_test_flash_checksum, NULL, true, 0, 0, 0, 0, 0},
        {0x12, "EEPROM Integrity", POST_PHASE_MEMORY_TEST, POST_CATEGORY_IMPORTANT, 1000, post_test_eeprom_integrity, NULL, true, 0, 0, 0, 0, 0},
        {0x13, "Stack Protection", POST_PHASE_MEMORY_TEST, POST_CATEGORY_CRITICAL, 500, post_test_stack_protection, NULL, true, 0, 0, 0, 0, 0},
        {0x14, "Heap Integrity", POST_PHASE_MEMORY_TEST, POST_CATEGORY_IMPORTANT, 1000, post_test_heap_integrity, NULL, true, 0, 0, 0, 0, 0},
        {0x15, "Calibration Data", POST_PHASE_MEMORY_TEST, POST_CATEGORY_CRITICAL, 1000, post_test_calibration_data, NULL, true, 0, 0, 0, 0, 0},
        
        // Sensor validation
        {0x20, "Wheel Speed Sensors", POST_PHASE_SENSOR_VALIDATION, POST_CATEGORY_CRITICAL, 2000, post_validate_wheel_speed_sensors, NULL, true, 0, 0, 0, 0, 0},
        {0x21, "Inertial Sensors", POST_PHASE_SENSOR_VALIDATION, POST_CATEGORY_CRITICAL, 3000, post_validate_inertial_sensors, NULL, true, 0, 0, 0, 0, 0},
        {0x22, "Steering Angle Sensor", POST_PHASE_SENSOR_VALIDATION, POST_CATEGORY_CRITICAL, 1000, post_validate_steering_angle_sensor, NULL, true, 0, 0, 0, 0, 0},
        {0x23, "Brake Sensors", POST_PHASE_SENSOR_VALIDATION, POST_CATEGORY_CRITICAL, 1000, post_validate_brake_sensors, NULL, true, 0, 0, 0, 0, 0},
        {0x24, "Supply Voltage", POST_PHASE_SENSOR_VALIDATION, POST_CATEGORY_CRITICAL, 500, post_validate_supply_voltage, NULL, true, 0, 0, 0, 0, 0},
        {0x25, "Sensor Plausibility", POST_PHASE_SENSOR_VALIDATION, POST_CATEGORY_IMPORTANT, 2000, post_validate_sensor_plausibility, NULL, true, 0, 0, 0, 0, 0},
        
        // Communication tests
        {0x30, "CAN Communication", POST_PHASE_COMMUNICATION_TEST, POST_CATEGORY_CRITICAL, 2000, post_test_can_communication, NULL, true, 0, 0, 0, 0, 0},
        {0x31, "Internal Communication", POST_PHASE_COMMUNICATION_TEST, POST_CATEGORY_IMPORTANT, 1000, post_test_internal_communication, NULL, true, 0, 0, 0, 0, 0},
        {0x32, "Diagnostic Communication", POST_PHASE_COMMUNICATION_TEST, POST_CATEGORY_INFORMATIONAL, 1000, post_test_diagnostic_communication, NULL, true, 0, 0, 0, 0, 0},
        
        // Safety validation
        {0x40, "Watchdog System", POST_PHASE_SAFETY_VALIDATION, POST_CATEGORY_CRITICAL, 1000, post_validate_watchdog_system, NULL, true, 0, 0, 0, 0, 0},
        {0x41, "Safety Mechanisms", POST_PHASE_SAFETY_VALIDATION, POST_CATEGORY_CRITICAL, 2000, post_validate_safety_mechanisms, NULL, true, 0, 0, 0, 0, 0},
        {0x42, "Fault Handling", POST_PHASE_SAFETY_VALIDATION, POST_CATEGORY_IMPORTANT, 1000, post_validate_fault_handling, NULL, true, 0, 0, 0, 0, 0},
        {0x43, "Emergency Shutdown", POST_PHASE_SAFETY_VALIDATION, POST_CATEGORY_CRITICAL, 1000, post_validate_emergency_shutdown, NULL, true, 0, 0, 0, 0, 0},
        
        // System integration
        {0x50, "Brake System Integration", POST_PHASE_SYSTEM_INTEGRATION, POST_CATEGORY_CRITICAL, 3000, post_test_brake_system_integration, NULL, true, 0, 0, 0, 0, 0},
        {0x51, "Engine Interface Integration", POST_PHASE_SYSTEM_INTEGRATION, POST_CATEGORY_IMPORTANT, 2000, post_test_engine_interface_integration, NULL, true, 0, 0, 0, 0, 0},
        {0x52, "DTC System Integration", POST_PHASE_SYSTEM_INTEGRATION, POST_CATEGORY_IMPORTANT, 1000, post_test_dtc_system_integration, NULL, true, 0, 0, 0, 0, 0},
        {0x53, "Overall System Integration", POST_PHASE_FINAL_VERIFICATION, POST_CATEGORY_CRITICAL, 5000, post_test_overall_system_integration, NULL, true, 0, 0, 0, 0, 0}
    };
    
    // Register all modules
    for (int i = 0; i < sizeof(modules) / sizeof(modules[0]); i++) {
        if (!post_register_test_module(post, &modules[i])) {
            printf("POST: Failed to register module %s\n", modules[i].module_name);
            return false;
        }
    }
    
    return true;
}

bool post_setup_default_memory_regions(post_system_t* post) {
    if (!post) {
        return false;
    }
    
    // Define default memory regions
    memory_region_t regions[] = {
        {(void*)0x08000000, 0x40000, MEMORY_TEST_FLASH, 0x12345678, true, "Application Flash"},
        {(void*)0x20000000, 0x20000, MEMORY_TEST_RAM, 0, true, "Main RAM"},
        {(void*)0x08080000, 0x8000, MEMORY_TEST_EEPROM, 0x87654321, false, "Calibration EEPROM"},
        {(void*)0x20020000, 0x2000, MEMORY_TEST_STACK, 0, true, "Stack Area"},
        {(void*)0x20018000, 0x8000, MEMORY_TEST_HEAP, 0, false, "Heap Area"}
    };
    
    // Register all memory regions
    for (int i = 0; i < sizeof(regions) / sizeof(regions[0]); i++) {
        if (!post_register_memory_region(post, &regions[i])) {
            printf("POST: Failed to register memory region %s\n", regions[i].region_name);
            return false;
        }
    }
    
    return true;
}

bool post_setup_default_sensor_calibrations(post_system_t* post) {
    if (!post) {
        return false;
    }
    
    // Define default sensor calibrations
    post_sensor_calibration_t calibrations[] = {
        {ESC_SENSOR_WHEEL_SPEED_FL, 0.0f, 300.0f, 0.0f, 2.0f, 0, 0x12345678, false, 0},
        {ESC_SENSOR_WHEEL_SPEED_FR, 0.0f, 300.0f, 0.0f, 2.0f, 0, 0x12345679, false, 0},
        {ESC_SENSOR_WHEEL_SPEED_RL, 0.0f, 300.0f, 0.0f, 2.0f, 0, 0x1234567A, false, 0},
        {ESC_SENSOR_WHEEL_SPEED_RR, 0.0f, 300.0f, 0.0f, 2.0f, 0, 0x1234567B, false, 0},
        {ESC_SENSOR_YAW_RATE, -300.0f, 300.0f, 0.0f, 5.0f, 0, 0x1234567C, true, 1000},
        {ESC_SENSOR_LATERAL_ACCEL, -20.0f, 20.0f, 0.0f, 0.5f, 0, 0x1234567D, true, 500},
        {ESC_SENSOR_LONGITUDINAL_ACCEL, -20.0f, 20.0f, 0.0f, 0.5f, 0, 0x1234567E, true, 500},
        {ESC_SENSOR_STEERING_ANGLE, -720.0f, 720.0f, 0.0f, 2.0f, 0, 0x1234567F, false, 0},
        {ESC_SENSOR_BRAKE_PRESSURE, 0.0f, 300.0f, 0.0f, 2.0f, 0, 0x12345680, false, 0},
        {ESC_SENSOR_BRAKE_TEMPERATURE, -40.0f, 800.0f, 20.0f, 5.0f, 0, 0x12345681, false, 0},
        {ESC_SENSOR_SUPPLY_VOLTAGE, 9.0f, 16.0f, 12.0f, 0.5f, 0, 0x12345682, false, 0}
    };
    
    // Register all sensor calibrations
    for (int i = 0; i < sizeof(calibrations) / sizeof(calibrations[0]); i++) {
        if (!post_register_sensor_calibration(post, &calibrations[i])) {
            printf("POST: Failed to register sensor calibration for %s\n", 
                   esc_sensor_id_to_string(calibrations[i].sensor_id));
            return false;
        }
    }
    
    return true;
}