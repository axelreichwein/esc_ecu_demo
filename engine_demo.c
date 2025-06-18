#include "engine_torque_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

// Demo CAN interface functions
static bool demo_engine_can_init(uint32_t baudrate) {
    printf("Demo Engine CAN interface initialized at %u bps\n", baudrate);
    return true;
}

static bool demo_engine_can_send_message(uint32_t can_id, const uint8_t* data, uint8_t length) {
    printf("Engine CAN TX: ID=0x%03X, Data=[", can_id);
    for (int i = 0; i < length; i++) {
        printf("%02X", data[i]);
        if (i < length - 1) printf(" ");
    }
    printf("]\n");
    
    // Decode and display the torque request
    if (can_id == 0x700 && length == 8) {
        uint8_t requester_id = data[0] & 0x0F;
        uint8_t priority = (data[0] >> 4) & 0x07;
        uint16_t reduction_scaled = (uint16_t)(data[1] | (data[2] << 8));
        float reduction_percent = (float)reduction_scaled / 100.0f;
        uint16_t duration_ms = (uint16_t)(data[3] | (data[4] << 8));
        float rate = (float)(data[5] & 0x7F);
        bool gradual = (data[5] & 0x80) != 0;
        
        printf("  -> Torque Request: %.1f%% reduction, %dms duration, priority %d, %s\n",
               reduction_percent, duration_ms, priority, gradual ? "gradual" : "immediate");
    }
    
    return true;
}

static bool demo_engine_can_receive_message(uint32_t* can_id, uint8_t* data, uint8_t* length) {
    static int counter = 0;
    static int response_delay = 0;
    static bool deny_next_request = false;
    
    counter++;
    
    // Simulate occasional denied requests
    if (counter > 200 && counter < 220) {
        deny_next_request = true;
    } else {
        deny_next_request = false;
    }
    
    // Send engine response after some delay
    if (response_delay > 0) {
        response_delay--;
        if (response_delay == 0) {
            *can_id = 0x701; // Engine response ID
            *length = 8;
            
            if (deny_next_request) {
                // Simulate denied request
                data[0] = 0x02; // Status: DENIED
                data[1] = 0x00; // No reduction applied
                data[2] = 0x00;
                data[3] = 0x32; // Max available: 50%
                data[4] = 0x00;
                data[5] = 0x00; // Duration: 0
                data[6] = 0x00;
                data[7] = 0x02 ^ 0x00 ^ 0x32; // Checksum
                printf("Engine Response: DENIED (simulated safety intervention)\n");
            } else {
                // Simulate accepted request
                data[0] = 0x01; // Status: ACCEPTED
                data[1] = 0x96; // 15.0% reduction applied (scaled)
                data[2] = 0x00;
                data[3] = 0x50; // Max available: 80%
                data[4] = 0x00;
                data[5] = 0xD0; // Duration: 2000ms
                data[6] = 0x07;
                data[7] = 0x01 ^ 0x96 ^ 0x00 ^ 0x50 ^ 0x00 ^ 0xD0 ^ 0x07; // Checksum
                printf("Engine Response: ACCEPTED (%.1f%% torque reduction applied)\n", 15.0f);
            }
            return true;
        }
    }
    
    // Trigger response delay when we receive a request
    if (counter % 15 == 1) { // Simulate receiving a request
        response_delay = 3; // Respond after 3 calls (simulate processing delay)
    }
    
    // Send periodic heartbeat
    if (counter % 25 == 0) {
        *can_id = 0x702; // Engine heartbeat ID
        *length = 8;
        
        // Simulate normal engine operation
        data[0] = 0x08; // Normal mode + torque capable
        data[1] = 0xC8; // RPM: 2000 (200 * 10)
        data[2] = 0x07;
        data[3] = 0x46; // 70% throttle
        data[4] = 0x00; // No fault
        data[5] = 0x2C; // Torque: 180 Nm (1800 / 10)
        data[6] = 0x07;
        data[7] = 0x08 ^ 0xC8 ^ 0x07 ^ 0x46 ^ 0x00 ^ 0x2C ^ 0x07; // Checksum
        return true;
    }
    
    return false;
}

static bool demo_engine_can_set_filter(uint32_t can_id, uint32_t mask) {
    printf("CAN filter set: ID=0x%03X, Mask=0x%03X\n", can_id, mask);
    return true;
}

static uint32_t demo_engine_get_timestamp_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static const engine_can_interface_t demo_can_interface = {
    .can_init = demo_engine_can_init,
    .can_send_message = demo_engine_can_send_message,
    .can_receive_message = demo_engine_can_receive_message,
    .can_set_filter = demo_engine_can_set_filter,
    .get_timestamp_ms = demo_engine_get_timestamp_ms
};

void demo_basic_engine_torque_reduction(void) {
    printf("\n=== Basic Engine Torque Reduction Demo ===\n");
    
    engine_torque_interface_t engine_system;
    
    // Initialize engine torque interface
    if (!engine_torque_init(&engine_system, &demo_can_interface)) {
        printf("Failed to initialize engine torque interface\n");
        return;
    }
    
    // Configure CAN IDs
    engine_torque_configure_can_ids(&engine_system, 0x700, 0x701, 0x702);
    
    printf("\nTesting basic torque reduction requests:\n");
    
    // Test 1: Basic torque reduction request
    printf("\nTest 1: Requesting 25%% torque reduction for 3 seconds\n");
    engine_request_torque_reduction(&engine_system, 25.0f, 3000, 7);
    
    // Monitor for response
    for (int i = 0; i < 20; i++) {
        engine_update_communication(&engine_system);
        
        engine_request_status_t status = engine_get_request_status(&engine_system);
        if (status == ENGINE_REQUEST_ACCEPTED) {
            printf("Torque reduction accepted! Actual reduction: %.1f%%\n", 
                   engine_get_actual_reduction(&engine_system));
            break;
        } else if (status == ENGINE_REQUEST_DENIED) {
            printf("Torque reduction denied!\n");
            break;
        }
        
        usleep(100000); // 100ms
    }
    
    // Wait a bit then cancel
    sleep(1);
    printf("\nCancelling torque reduction...\n");
    engine_cancel_torque_reduction(&engine_system);
    
    for (int i = 0; i < 10; i++) {
        engine_update_communication(&engine_system);
        if (!engine_is_reduction_active(&engine_system)) {
            printf("Torque reduction successfully cancelled\n");
            break;
        }
        usleep(100000);
    }
    
    engine_torque_shutdown(&engine_system);
}

void demo_gradual_torque_reduction(void) {
    printf("\n=== Gradual Torque Reduction Demo ===\n");
    
    engine_torque_interface_t engine_system;
    
    if (!engine_torque_init(&engine_system, &demo_can_interface)) {
        printf("Failed to initialize engine torque interface\n");
        return;
    }
    
    engine_torque_configure_can_ids(&engine_system, 0x700, 0x701, 0x702);
    
    printf("\nTesting gradual torque reduction:\n");
    
    // Test gradual reduction
    printf("\nRequesting gradual 40%% torque reduction at 10%%/s rate\n");
    engine_request_gradual_torque_reduction(&engine_system, 40.0f, 10.0f, 5000, 6);
    
    // Monitor the gradual application
    for (int i = 0; i < 30; i++) {
        engine_update_communication(&engine_system);
        
        if (engine_is_reduction_active(&engine_system)) {
            printf("Active torque reduction: %.1f%%\n", engine_get_actual_reduction(&engine_system));
        }
        
        usleep(200000); // 200ms
    }
    
    engine_torque_shutdown(&engine_system);
}

void demo_fallback_handling(void) {
    printf("\n=== Fallback Handling Demo ===\n");
    
    engine_torque_interface_t engine_system;
    
    if (!engine_torque_init(&engine_system, &demo_can_interface)) {
        printf("Failed to initialize engine torque interface\n");
        return;
    }
    
    engine_torque_configure_can_ids(&engine_system, 0x700, 0x701, 0x702);
    
    // Configure aggressive fallback settings for demo
    engine_fallback_config_t fallback_config = engine_get_default_fallback_config();
    fallback_config.enable_fallback = true;
    fallback_config.max_brake_compensation = 60.0f;
    fallback_config.brake_ramp_rate = 30.0f;
    fallback_config.engine_timeout_ms = 500;
    fallback_config.max_retry_attempts = 1; // Only 1 retry for demo
    
    engine_torque_configure_fallback(&engine_system, &fallback_config);
    
    printf("\nTesting fallback when engine denies request:\n");
    
    // This request will be denied (see demo CAN receive function)
    printf("\nRequesting 30%% torque reduction (will be denied)...\n");
    engine_request_torque_reduction(&engine_system, 30.0f, 2000, 7);
    
    // Monitor for denial and fallback activation
    for (int i = 0; i < 30; i++) {
        engine_update_communication(&engine_system);
        
        engine_request_status_t status = engine_get_request_status(&engine_system);
        if (status == ENGINE_REQUEST_DENIED) {
            printf("Engine denied torque reduction request!\n");
            
            // Check if fallback was activated
            if (engine_system.fallback_active) {
                printf("Fallback activated: %.1f bar brake compensation\n", 
                       engine_system.fallback_brake_pressure);
                break;
            }
        }
        
        usleep(150000); // 150ms
    }
    
    // Let fallback run for a bit
    sleep(2);
    
    // Deactivate fallback
    printf("\nDeactivating fallback...\n");
    engine_deactivate_fallback(&engine_system);
    
    engine_torque_shutdown(&engine_system);
}

void demo_system_monitoring(void) {
    printf("\n=== System Monitoring Demo ===\n");
    
    engine_torque_interface_t engine_system;
    
    if (!engine_torque_init(&engine_system, &demo_can_interface)) {
        printf("Failed to initialize engine torque interface\n");
        return;
    }
    
    engine_torque_configure_can_ids(&engine_system, 0x700, 0x701, 0x702);
    
    printf("\nMonitoring engine ECU status:\n");
    
    // Monitor for a few seconds
    for (int i = 0; i < 50; i++) {
        engine_update_communication(&engine_system);
        
        if (i % 10 == 0) {
            engine_ecu_status_t ecu_status = engine_get_ecu_status(&engine_system);
            
            printf("\nEngine ECU Status:\n");
            printf("  Operating Mode: %s\n", engine_operating_mode_to_string(ecu_status.operating_mode));
            printf("  Torque Capable: %s\n", ecu_status.torque_reduction_capable ? "YES" : "NO");
            printf("  Current RPM: %.0f\n", ecu_status.current_rpm);
            printf("  Throttle Position: %.1f%%\n", ecu_status.throttle_position_percent);
            printf("  Current Torque: %.1f Nm\n", ecu_status.current_torque_nm);
            printf("  Communication: %s\n", ecu_status.communication_active ? "ACTIVE" : "LOST");
        }
        
        usleep(100000); // 100ms
    }
    
    // Print final statistics
    engine_print_system_status(&engine_system);
    engine_print_statistics(&engine_system);
    
    engine_torque_shutdown(&engine_system);
}

int main(void) {
    printf("Engine Torque Reduction Interface Demonstration\n");
    printf("==============================================\n");
    
    demo_basic_engine_torque_reduction();
    demo_gradual_torque_reduction();
    demo_fallback_handling();
    demo_system_monitoring();
    
    printf("\nEngine torque interface demonstration completed.\n");
    return 0;
}