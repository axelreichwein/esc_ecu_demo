#include "post_system.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

// Demo hardware simulation state
static uint32_t demo_timestamp_ms = 0;
static float demo_sensor_values[16] = {0};
static bool demo_sensor_health[16] = {true};
static bool demo_memory_test_results[8] = {true};
static bool demo_communication_interfaces[4] = {true};

// Demo hardware interface implementations
static bool demo_hw_memory_test(void* address, uint32_t size);
static uint32_t demo_hw_calculate_checksum(void* address, uint32_t size);
static bool demo_hw_sensor_power_on(esc_sensor_id_t sensor_id);
static bool demo_hw_sensor_power_off(esc_sensor_id_t sensor_id);
static float demo_hw_sensor_read_value(esc_sensor_id_t sensor_id);
static bool demo_hw_sensor_self_test(esc_sensor_id_t sensor_id);
static bool demo_hw_communication_test(uint8_t interface_id);
static uint32_t demo_hw_get_timestamp(void);
static void demo_hw_delay_ms(uint32_t milliseconds);
static void demo_hw_system_reset(void);

// Demo scenario functions
void demo_post_normal_startup(void);
void demo_post_memory_failure(void);
void demo_post_sensor_failure(void);
void demo_post_communication_failure(void);
void demo_post_critical_failure(void);

// Hardware interface structure
static const post_hardware_interface_t demo_hw_interface = {
    .hw_memory_test = demo_hw_memory_test,
    .hw_calculate_checksum = demo_hw_calculate_checksum,
    .hw_sensor_power_on = demo_hw_sensor_power_on,
    .hw_sensor_power_off = demo_hw_sensor_power_off,
    .hw_sensor_read_value = demo_hw_sensor_read_value,
    .hw_sensor_self_test = demo_hw_sensor_self_test,
    .hw_communication_test = demo_hw_communication_test,
    .hw_get_timestamp = demo_hw_get_timestamp,
    .hw_delay_ms = demo_hw_delay_ms,
    .hw_system_reset = demo_hw_system_reset
};

int main(void) {
    printf("POST System Demonstration\n");
    printf("=========================\n");
    
    // Initialize demo environment
    demo_timestamp_ms = 0;
    
    // Set realistic sensor values
    demo_sensor_values[ESC_SENSOR_WHEEL_SPEED_FL] = 0.0f;   // km/h (stationary)
    demo_sensor_values[ESC_SENSOR_WHEEL_SPEED_FR] = 0.0f;
    demo_sensor_values[ESC_SENSOR_WHEEL_SPEED_RL] = 0.0f;
    demo_sensor_values[ESC_SENSOR_WHEEL_SPEED_RR] = 0.0f;
    demo_sensor_values[ESC_SENSOR_YAW_RATE] = 0.5f;         // deg/s (slight sensor noise)
    demo_sensor_values[ESC_SENSOR_LATERAL_ACCEL] = 0.1f;    // m/s² (gravity component)
    demo_sensor_values[ESC_SENSOR_LONGITUDINAL_ACCEL] = 0.0f;
    demo_sensor_values[ESC_SENSOR_STEERING_ANGLE] = 2.0f;   // degrees (slightly off-center)
    demo_sensor_values[ESC_SENSOR_BRAKE_PRESSURE] = 0.0f;   // bar (no braking)
    demo_sensor_values[ESC_SENSOR_BRAKE_TEMPERATURE] = 25.0f; // °C (ambient)
    demo_sensor_values[ESC_SENSOR_SUPPLY_VOLTAGE] = 12.3f;  // V (normal battery)
    
    // Run demonstration scenarios
    demo_post_normal_startup();
    demo_post_memory_failure();
    demo_post_sensor_failure();
    demo_post_communication_failure();
    demo_post_critical_failure();
    
    printf("\nPOST system demonstration completed.\n");
    return 0;
}

void demo_post_normal_startup(void) {
    printf("\n=== Normal Startup Scenario ===\n");
    
    post_system_t post_system;
    
    // Reset demo state for normal operation
    for (int i = 0; i < 16; i++) {
        demo_sensor_health[i] = true;
    }
    for (int i = 0; i < 8; i++) {
        demo_memory_test_results[i] = true;
    }
    for (int i = 0; i < 4; i++) {
        demo_communication_interfaces[i] = true;
    }
    
    // Initialize POST system
    if (!post_system_init(&post_system, &demo_hw_interface)) {
        printf("Failed to initialize POST system\n");
        return;
    }
    
    printf("POST system initialized with %u test modules\n", post_system.module_count);
    
    // Execute complete POST sequence
    printf("\nExecuting complete POST sequence...\n");
    bool initialization_success = post_execute_all_tests(&post_system);
    
    // Print results
    printf("\nPOST Results:\n");
    post_print_system_status(&post_system);
    
    if (initialization_success) {
        printf("✓ Normal startup completed successfully\n");
        printf("✓ ESC system is ENABLED and ready for operation\n");
    } else {
        printf("✗ Startup failed\n");
    }
    
    // Cleanup
    post_system_shutdown(&post_system);
}

void demo_post_memory_failure(void) {
    printf("\n=== Memory Failure Scenario ===\n");
    
    post_system_t post_system;
    
    // Simulate memory failure
    demo_memory_test_results[1] = false; // Flash checksum failure
    
    if (!post_system_init(&post_system, &demo_hw_interface)) {
        printf("Failed to initialize POST system\n");
        return;
    }
    
    printf("Simulating Flash memory checksum failure...\n");
    bool initialization_success = post_execute_all_tests(&post_system);
    
    printf("\nMemory Failure Results:\n");
    post_print_system_status(&post_system);
    post_print_error_log(&post_system);
    
    if (!initialization_success) {
        printf("✓ Memory failure correctly detected\n");
        printf("✓ ESC system correctly DISABLED due to critical failure\n");
    }
    
    // Reset for next test
    demo_memory_test_results[1] = true;
    post_system_shutdown(&post_system);
}

void demo_post_sensor_failure(void) {
    printf("\n=== Sensor Failure Scenario ===\n");
    
    post_system_t post_system;
    
    // Simulate sensor failures
    demo_sensor_health[ESC_SENSOR_YAW_RATE] = false;         // Critical sensor failure
    demo_sensor_values[ESC_SENSOR_SUPPLY_VOLTAGE] = 8.5f;    // Under-voltage condition
    
    if (!post_system_init(&post_system, &demo_hw_interface)) {
        printf("Failed to initialize POST system\n");
        return;
    }
    
    printf("Simulating yaw rate sensor failure and under-voltage...\n");
    bool initialization_success = post_execute_all_tests(&post_system);
    
    printf("\nSensor Failure Results:\n");
    post_print_system_status(&post_system);
    post_print_error_log(&post_system);
    
    if (!initialization_success) {
        printf("✓ Sensor failures correctly detected\n");
        printf("✓ ESC system correctly DISABLED due to sensor failures\n");
    }
    
    // Reset for next test
    demo_sensor_health[ESC_SENSOR_YAW_RATE] = true;
    demo_sensor_values[ESC_SENSOR_SUPPLY_VOLTAGE] = 12.3f;
    post_system_shutdown(&post_system);
}

void demo_post_communication_failure(void) {
    printf("\n=== Communication Failure Scenario ===\n");
    
    post_system_t post_system;
    
    // Simulate CAN communication failure
    demo_communication_interfaces[0] = false; // CAN interface failure
    
    if (!post_system_init(&post_system, &demo_hw_interface)) {
        printf("Failed to initialize POST system\n");
        return;
    }
    
    printf("Simulating CAN bus communication failure...\n");
    bool initialization_success = post_execute_all_tests(&post_system);
    
    printf("\nCommunication Failure Results:\n");
    post_print_system_status(&post_system);
    post_print_error_log(&post_system);
    
    if (!initialization_success) {
        printf("✓ Communication failure correctly detected\n");
        printf("✓ ESC system correctly DISABLED due to CAN failure\n");
    }
    
    // Reset for next test
    demo_communication_interfaces[0] = true;
    post_system_shutdown(&post_system);
}

void demo_post_critical_failure(void) {
    printf("\n=== Critical Failure Scenario ===\n");
    
    post_system_t post_system;
    
    // Simulate multiple critical failures
    demo_memory_test_results[0] = false; // RAM failure
    demo_sensor_health[ESC_SENSOR_WHEEL_SPEED_FL] = false;
    demo_sensor_health[ESC_SENSOR_WHEEL_SPEED_FR] = false;
    demo_sensor_values[ESC_SENSOR_SUPPLY_VOLTAGE] = 7.0f; // Critical under-voltage
    
    if (!post_system_init(&post_system, &demo_hw_interface)) {
        printf("Failed to initialize POST system\n");
        return;
    }
    
    // Enable halt on critical failure
    post_system.halt_on_critical_failure = true;
    
    printf("Simulating multiple critical failures...\n");
    bool initialization_success = post_execute_all_tests(&post_system);
    
    printf("\nCritical Failure Results:\n");
    post_print_system_status(&post_system);
    post_print_error_log(&post_system);
    
    if (post_has_critical_failure(&post_system)) {
        printf("✓ Critical failures correctly detected\n");
        printf("✓ System correctly halted due to critical failures\n");
    }
    
    // Reset for next test
    demo_memory_test_results[0] = true;
    demo_sensor_health[ESC_SENSOR_WHEEL_SPEED_FL] = true;
    demo_sensor_health[ESC_SENSOR_WHEEL_SPEED_FR] = true;
    demo_sensor_values[ESC_SENSOR_SUPPLY_VOLTAGE] = 12.3f;
    post_system_shutdown(&post_system);
}

// Hardware interface implementations
static bool demo_hw_memory_test(void* address, uint32_t size) {
    (void)size; // Suppress unused parameter warning
    
    // Simulate memory test based on demo state
    uintptr_t addr = (uintptr_t)address;
    
    if (addr >= 0x20000000 && addr < 0x20020000) {
        // RAM test
        return demo_memory_test_results[0];
    } else if (addr >= 0x08000000 && addr < 0x08080000) {
        // Flash test
        return demo_memory_test_results[1];
    } else if (addr >= 0x08080000 && addr < 0x08088000) {
        // EEPROM test
        return demo_memory_test_results[2];
    }
    
    return true; // Default to pass
}

static uint32_t demo_hw_calculate_checksum(void* address, uint32_t size) {
    (void)size; // Suppress unused parameter warning
    
    // Simulate checksum calculation
    uintptr_t addr = (uintptr_t)address;
    
    if (addr == 0x08000000 && demo_memory_test_results[1]) {
        return 0x12345678; // Expected checksum for Flash
    } else if (addr == 0x08000000 && !demo_memory_test_results[1]) {
        return 0xDEADBEEF; // Wrong checksum to simulate failure
    }
    
    return 0x87654321; // Other checksums
}

static bool demo_hw_sensor_power_on(esc_sensor_id_t sensor_id) {
    // Simulate sensor power on
    demo_hw_delay_ms(10); // Power-on delay
    return demo_sensor_health[sensor_id];
}

static bool demo_hw_sensor_power_off(esc_sensor_id_t sensor_id) {
    (void)sensor_id; // Suppress unused parameter warning
    // Simulate sensor power off
    return true;
}

static float demo_hw_sensor_read_value(esc_sensor_id_t sensor_id) {
    // Return simulated sensor values
    if (sensor_id < 16) {
        return demo_sensor_values[sensor_id];
    }
    return 0.0f;
}

static bool demo_hw_sensor_self_test(esc_sensor_id_t sensor_id) {
    // Simulate sensor self-test
    demo_hw_delay_ms(50); // Self-test delay
    return demo_sensor_health[sensor_id];
}

static bool demo_hw_communication_test(uint8_t interface_id) {
    // Simulate communication test
    demo_hw_delay_ms(100); // Communication test delay
    
    if (interface_id < 4) {
        return demo_communication_interfaces[interface_id];
    }
    return true;
}

static uint32_t demo_hw_get_timestamp(void) {
    // Return simulated timestamp
    return demo_timestamp_ms;
}

static void demo_hw_delay_ms(uint32_t milliseconds) {
    // Simulate delay by advancing timestamp
    demo_timestamp_ms += milliseconds;
    
    // For demonstration, use actual delay for some operations
    if (milliseconds <= 10) {
        usleep(milliseconds * 1000);
    }
}

static void demo_hw_system_reset(void) {
    // Simulate system reset
    printf("SYSTEM RESET TRIGGERED\n");
    demo_timestamp_ms = 0;
}