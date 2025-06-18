#include "can_bus_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

// Demo callback functions
static void demo_message_callback(const can_frame_t* frame);
static void demo_error_callback(can_error_type_t error, const char* description);
static void demo_bus_state_callback(can_bus_state_t old_state, can_bus_state_t new_state);

// Demo hardware simulation functions
static bool demo_hw_init(can_baudrate_t baudrate);
static bool demo_hw_deinit(void);
static bool demo_hw_send_frame(const can_frame_t* frame);
static bool demo_hw_receive_frame(can_frame_t* frame);
static bool demo_hw_set_filter(uint32_t filter_id, uint32_t id, uint32_t mask);
static bool demo_hw_clear_filter(uint32_t filter_id);
static bool demo_hw_get_status(can_bus_status_t* status);
static bool demo_hw_reset_controller(void);
static uint32_t demo_hw_get_timestamp(void);
static bool demo_hw_enter_sleep_mode(void);
static bool demo_hw_exit_sleep_mode(void);

// Demo function prototypes
void demo_basic_can_operations(void);
void demo_message_filtering(void);
void demo_priority_messaging(void);
void demo_error_handling(void);
void demo_timing_supervision(void);
void demo_esc_messaging(void);
void demo_network_management(void);
void demo_loopback_test(void);

// Global demo state
static can_frame_t simulated_rx_messages[10];
static int simulated_rx_count = 0;
static int simulated_rx_index = 0;
static bool hardware_error_simulation = false;
static uint32_t message_counter = 0;

// Hardware interface simulation
static const can_hw_interface_t demo_hw_interface = {
    .hw_init = demo_hw_init,
    .hw_deinit = demo_hw_deinit,
    .hw_send_frame = demo_hw_send_frame,
    .hw_receive_frame = demo_hw_receive_frame,
    .hw_set_filter = demo_hw_set_filter,
    .hw_clear_filter = demo_hw_clear_filter,
    .hw_get_status = demo_hw_get_status,
    .hw_reset_controller = demo_hw_reset_controller,
    .hw_get_timestamp = demo_hw_get_timestamp,
    .hw_enter_sleep_mode = demo_hw_enter_sleep_mode,
    .hw_exit_sleep_mode = demo_hw_exit_sleep_mode
};

int main(void) {
    printf("CAN Bus Interface Demonstration\n");
    printf("===============================\n");
    
    demo_basic_can_operations();
    demo_message_filtering();
    demo_priority_messaging();
    demo_error_handling();
    demo_timing_supervision();
    demo_esc_messaging();
    demo_network_management();
    demo_loopback_test();
    
    printf("\nCAN Bus interface demonstration completed.\n");
    return 0;
}

void demo_basic_can_operations(void) {
    printf("\n=== Basic CAN Operations Demo ===\n");
    
    can_bus_interface_t can_interface;
    
    // Initialize CAN interface
    if (!can_bus_init(&can_interface, &demo_hw_interface)) {
        printf("Failed to initialize CAN interface\n");
        return;
    }
    
    // Register callbacks
    can_register_message_callback(&can_interface, demo_message_callback);
    can_register_error_callback(&can_interface, demo_error_callback);
    can_register_bus_state_callback(&can_interface, demo_bus_state_callback);
    
    printf("\nSending basic CAN messages:\n");
    
    // Send some test messages
    for (int i = 0; i < 5; i++) {
        can_frame_t frame;
        frame.id = 0x100 + i;
        frame.extended_id = false;
        frame.remote_frame = false;
        frame.data_length = 8;
        frame.priority = CAN_PRIORITY_NORMAL;
        
        for (int j = 0; j < 8; j++) {
            frame.data[j] = (i << 4) | j;
        }
        
        if (can_send_message(&can_interface, &frame)) {
            printf("Sent message ID=0x%03X with data pattern\n", frame.id);
        }
        
        usleep(10000); // 10ms delay
    }
    
    // Wait for any received messages
    printf("\nChecking for received messages:\n");
    can_frame_t received_frame;
    for (int i = 0; i < 3; i++) {
        if (can_receive_message_nonblocking(&can_interface, &received_frame)) {
            printf("Received: ");
            can_print_frame(&received_frame);
        }
        usleep(50000); // 50ms
    }
    
    // Print status and statistics
    can_print_status(&can_interface);
    can_print_statistics(&can_interface);
    
    can_bus_shutdown(&can_interface);
}

void demo_message_filtering(void) {
    printf("\n=== Message Filtering Demo ===\n");
    
    can_bus_interface_t can_interface;
    
    if (!can_bus_init(&can_interface, &demo_hw_interface)) {
        printf("Failed to initialize CAN interface\n");
        return;
    }
    
    // Add some message filters
    printf("Adding CAN message filters:\n");
    can_add_filter(&can_interface, 0x200, 0x7F0, false);  // Accept 0x200-0x20F
    can_add_filter(&can_interface, 0x300, 0x700, false);  // Accept 0x300-0x3FF
    can_add_filter(&can_interface, 0x123, 0x7FF, false);  // Accept exactly 0x123
    
    // Prepare simulated received messages (some should be filtered)
    simulated_rx_count = 6;
    simulated_rx_index = 0;
    
    // Message 1: Should be accepted by filter 1 (0x205)
    simulated_rx_messages[0].id = 0x205;
    simulated_rx_messages[0].extended_id = false;
    simulated_rx_messages[0].data_length = 4;
    simulated_rx_messages[0].data[0] = 0xAA;
    simulated_rx_messages[0].data[1] = 0xBB;
    simulated_rx_messages[0].data[2] = 0xCC;
    simulated_rx_messages[0].data[3] = 0xDD;
    
    // Message 2: Should be rejected (0x150)
    simulated_rx_messages[1].id = 0x150;
    simulated_rx_messages[1].extended_id = false;
    simulated_rx_messages[1].data_length = 2;
    simulated_rx_messages[1].data[0] = 0x11;
    simulated_rx_messages[1].data[1] = 0x22;
    
    // Message 3: Should be accepted by filter 2 (0x350)
    simulated_rx_messages[2].id = 0x350;
    simulated_rx_messages[2].extended_id = false;
    simulated_rx_messages[2].data_length = 8;
    for (int i = 0; i < 8; i++) {
        simulated_rx_messages[2].data[i] = i;
    }
    
    // Message 4: Should be accepted by filter 3 (0x123)
    simulated_rx_messages[3].id = 0x123;
    simulated_rx_messages[3].extended_id = false;
    simulated_rx_messages[3].data_length = 1;
    simulated_rx_messages[3].data[0] = 0xFF;
    
    // Message 5: Should be rejected (0x400)
    simulated_rx_messages[4].id = 0x400;
    simulated_rx_messages[4].extended_id = false;
    simulated_rx_messages[4].data_length = 3;
    simulated_rx_messages[4].data[0] = 0x12;
    simulated_rx_messages[4].data[1] = 0x34;
    simulated_rx_messages[4].data[2] = 0x56;
    
    // Message 6: Should be accepted by filter 1 (0x20A)
    simulated_rx_messages[5].id = 0x20A;
    simulated_rx_messages[5].extended_id = false;
    simulated_rx_messages[5].data_length = 6;
    simulated_rx_messages[5].data[0] = 0xDE;
    simulated_rx_messages[5].data[1] = 0xAD;
    simulated_rx_messages[5].data[2] = 0xBE;
    simulated_rx_messages[5].data[3] = 0xEF;
    simulated_rx_messages[5].data[4] = 0xCA;
    simulated_rx_messages[5].data[5] = 0xFE;
    
    printf("\nReceiving messages (some should be filtered):\n");
    usleep(100000); // Wait for messages to be processed
    
    can_frame_t received_frame;
    int received_count = 0;
    for (int i = 0; i < 10; i++) {
        if (can_receive_message_nonblocking(&can_interface, &received_frame)) {
            printf("Accepted: ");
            can_print_frame(&received_frame);
            received_count++;
        }
        usleep(20000); // 20ms
    }
    
    printf("Total accepted messages: %d (expected: 4)\n", received_count);
    
    // Test filter removal
    printf("\nRemoving filter for 0x200-0x20F range:\n");
    can_remove_filter(&can_interface, 0x200);
    
    printf("Clearing all filters:\n");
    can_clear_all_filters(&can_interface);
    
    can_bus_shutdown(&can_interface);
}

void demo_priority_messaging(void) {
    printf("\n=== Priority Messaging Demo ===\n");
    
    can_bus_interface_t can_interface;
    
    if (!can_bus_init(&can_interface, &demo_hw_interface)) {
        printf("Failed to initialize CAN interface\n");
        return;
    }
    
    printf("Sending messages with different priorities:\n");
    
    // Send messages with different priorities (they should be transmitted in priority order)
    can_frame_t frame;
    frame.extended_id = false;
    frame.remote_frame = false;
    frame.data_length = 8;
    
    // Low priority message
    frame.id = 0x400;
    strcpy((char*)frame.data, "LOW_PRI");
    if (can_send_message_priority(&can_interface, &frame, CAN_PRIORITY_LOW)) {
        printf("Queued LOW priority message (ID=0x%03X)\n", frame.id);
    }
    
    // Emergency priority message
    frame.id = 0x401;
    strcpy((char*)frame.data, "EMRGNCY");
    if (can_send_message_priority(&can_interface, &frame, CAN_PRIORITY_EMERGENCY)) {
        printf("Queued EMERGENCY priority message (ID=0x%03X)\n", frame.id);
    }
    
    // Normal priority message
    frame.id = 0x402;
    strcpy((char*)frame.data, "NORMAL ");
    if (can_send_message_priority(&can_interface, &frame, CAN_PRIORITY_NORMAL)) {
        printf("Queued NORMAL priority message (ID=0x%03X)\n", frame.id);
    }
    
    // Critical priority message
    frame.id = 0x403;
    strcpy((char*)frame.data, "CRITICA");
    if (can_send_message_priority(&can_interface, &frame, CAN_PRIORITY_CRITICAL)) {
        printf("Queued CRITICAL priority message (ID=0x%03X)\n", frame.id);
    }
    
    // Background priority message
    frame.id = 0x404;
    strcpy((char*)frame.data, "BACKGND");
    if (can_send_message_priority(&can_interface, &frame, CAN_PRIORITY_BACKGROUND)) {
        printf("Queued BACKGROUND priority message (ID=0x%03X)\n", frame.id);
    }
    
    printf("\nMessages should be transmitted in this order:\n");
    printf("1. EMERGENCY (highest priority)\n");
    printf("2. CRITICAL\n");
    printf("3. NORMAL\n");
    printf("4. LOW\n");
    printf("5. BACKGROUND (lowest priority)\n");
    
    // Wait for transmission
    usleep(200000); // 200ms
    
    // Test cyclic messaging
    printf("\nTesting cyclic message transmission:\n");
    frame.id = 0x500;
    strcpy((char*)frame.data, "CYCLIC ");
    if (can_send_message_cyclic(&can_interface, &frame, 100)) {  // Every 100ms
        printf("Started cyclic transmission of ID=0x%03X every 100ms\n", frame.id);
    }
    
    usleep(350000); // 350ms to see multiple transmissions
    
    // Cancel cyclic message
    can_cancel_cyclic_message(&can_interface, 0x500);
    printf("Cancelled cyclic transmission\n");
    
    can_bus_shutdown(&can_interface);
}

void demo_error_handling(void) {
    printf("\n=== Error Handling Demo ===\n");
    
    can_bus_interface_t can_interface;
    
    if (!can_bus_init(&can_interface, &demo_hw_interface)) {
        printf("Failed to initialize CAN interface\n");
        return;
    }
    
    printf("Testing error handling capabilities:\n");
    
    // Test invalid frame rejection
    printf("\n1. Testing invalid frame rejection:\n");
    can_frame_t invalid_frame;
    invalid_frame.id = 0x800;  // Valid standard ID
    invalid_frame.extended_id = false;
    invalid_frame.remote_frame = false;
    invalid_frame.data_length = 9;  // Invalid - too long
    invalid_frame.priority = CAN_PRIORITY_NORMAL;
    
    if (!can_send_message(&can_interface, &invalid_frame)) {
        printf("Correctly rejected invalid frame (DLC=9)\n");
    }
    
    // Test extended ID validation
    invalid_frame.id = 0x20000000;  // Valid extended ID
    invalid_frame.extended_id = true;
    invalid_frame.data_length = 8;
    
    if (can_send_message(&can_interface, &invalid_frame)) {
        printf("Correctly accepted valid extended ID frame\n");
    }
    
    // Simulate hardware errors
    printf("\n2. Simulating hardware errors:\n");
    hardware_error_simulation = true;
    
    can_frame_t test_frame;
    test_frame.id = 0x600;
    test_frame.extended_id = false;
    test_frame.remote_frame = false;
    test_frame.data_length = 8;
    test_frame.priority = CAN_PRIORITY_NORMAL;
    strcpy((char*)test_frame.data, "ERROR  ");
    
    // This should trigger simulated transmission errors
    for (int i = 0; i < 3; i++) {
        if (!can_send_message(&can_interface, &test_frame)) {
            printf("Transmission error #%d detected\n", i + 1);
        }
        usleep(50000); // 50ms
    }
    
    hardware_error_simulation = false;
    
    // Test bus reset
    printf("\n3. Testing bus reset:\n");
    can_bus_reset(&can_interface);
    printf("Bus reset completed\n");
    
    // Print error statistics
    can_bus_statistics_t stats;
    if (can_get_statistics(&can_interface, &stats)) {
        printf("\nError Statistics:\n");
        printf("TX Errors: %u\n", stats.transmission_errors);
        printf("RX Errors: %u\n", stats.reception_errors);
        printf("CRC Errors: %u\n", stats.crc_errors);
        printf("ACK Errors: %u\n", stats.ack_errors);
    }
    
    can_bus_shutdown(&can_interface);
}

void demo_timing_supervision(void) {
    printf("\n=== Timing Supervision Demo ===\n");
    
    can_bus_interface_t can_interface;
    
    if (!can_bus_init(&can_interface, &demo_hw_interface)) {
        printf("Failed to initialize CAN interface\n");
        return;
    }
    
    printf("Setting up message timing supervision:\n");
    
    // Add timing supervision for specific messages
    can_add_message_timing(&can_interface, 0x300, 100, 200);  // Expected every 100ms, timeout at 200ms
    can_add_message_timing(&can_interface, 0x301, 50, 100);   // Expected every 50ms, timeout at 100ms
    can_add_message_timing(&can_interface, 0x302, 500, 1000); // Expected every 500ms, timeout at 1000ms
    
    // Simulate some messages arriving on time
    printf("\nSimulating message reception:\n");
    simulated_rx_count = 3;
    simulated_rx_index = 0;
    
    simulated_rx_messages[0].id = 0x300;
    simulated_rx_messages[0].data_length = 4;
    
    simulated_rx_messages[1].id = 0x301;
    simulated_rx_messages[1].data_length = 2;
    
    simulated_rx_messages[2].id = 0x302;
    simulated_rx_messages[2].data_length = 8;
    
    // Wait and check for timeouts
    usleep(150000); // 150ms
    printf("Checking timeouts after 150ms:\n");
    bool timeout_status = can_check_message_timeouts(&can_interface);
    printf("Timeout check result: %s\n", timeout_status ? "No timeouts" : "Timeouts detected");
    
    // Wait longer to trigger timeouts
    usleep(200000); // Additional 200ms (total 350ms)
    printf("Checking timeouts after 350ms:\n");
    timeout_status = can_check_message_timeouts(&can_interface);
    printf("Timeout check result: %s\n", timeout_status ? "No timeouts" : "Timeouts detected");
    
    // Remove timing supervision
    can_remove_message_timing(&can_interface, 0x300);
    can_remove_message_timing(&can_interface, 0x301);
    can_remove_message_timing(&can_interface, 0x302);
    
    printf("Removed timing supervision for test messages\n");
    
    can_bus_shutdown(&can_interface);
}

void demo_esc_messaging(void) {
    printf("\n=== ESC-Specific Messaging Demo ===\n");
    
    can_bus_interface_t can_interface;
    
    if (!can_bus_init(&can_interface, &demo_hw_interface)) {
        printf("Failed to initialize CAN interface\n");
        return;
    }
    
    printf("Sending ESC-specific CAN messages:\n");
    
    // ESC Status 1 message
    esc_status_1_message_t esc_status1;
    esc_status1.system_status = 2;  // Active
    esc_status1.intervention_flags = 0x05;  // Understeer + Brake assist
    esc_status1.warning_lights = 0x01;  // ESC warning light on
    esc_status1.traction_control_status = 1;  // Active
    esc_status1.stability_control_status = 2;  // Intervening
    esc_status1.brake_assist_status = 1;  // Active
    esc_status1.fault_code_high = 0xC0;
    esc_status1.fault_code_low = 0x35;
    
    if (can_send_esc_status_1(&can_interface, &esc_status1)) {
        printf("Sent ESC Status 1 message (system active, intervention flags=0x%02X)\n", 
               esc_status1.intervention_flags);
    }
    
    // ESC Status 2 message
    esc_status_2_message_t esc_status2;
    esc_status2.yaw_rate = 150;  // 15.0 deg/s (scaled by 10)
    esc_status2.lateral_acceleration = 250;  // 2.50 m/s² (scaled by 100)
    esc_status2.steering_angle = -200;  // -20.0 degrees (scaled by 10)
    esc_status2.sensor_status = 0xFF;  // All sensors valid
    esc_status2.reserved = 0x00;
    
    if (can_send_esc_status_2(&can_interface, &esc_status2)) {
        printf("Sent ESC Status 2 message (yaw_rate=%.1f deg/s, lat_accel=%.2f m/s²)\n",
               esc_status2.yaw_rate / 10.0f, esc_status2.lateral_acceleration / 100.0f);
    }
    
    // Wheel speeds message
    esc_wheel_speeds_message_t wheel_speeds;
    wheel_speeds.wheel_speed_fl = 800;  // 80.0 km/h (scaled by 10)
    wheel_speeds.wheel_speed_fr = 805;  // 80.5 km/h
    wheel_speeds.wheel_speed_rl = 795;  // 79.5 km/h
    wheel_speeds.wheel_speed_rr = 790;  // 79.0 km/h
    
    if (can_send_esc_wheel_speeds(&can_interface, &wheel_speeds)) {
        printf("Sent wheel speeds message (FL=%.1f, FR=%.1f, RL=%.1f, RR=%.1f km/h)\n",
               wheel_speeds.wheel_speed_fl / 10.0f, wheel_speeds.wheel_speed_fr / 10.0f,
               wheel_speeds.wheel_speed_rl / 10.0f, wheel_speeds.wheel_speed_rr / 10.0f);
    }
    
    // Brake pressures message
    esc_brake_pressure_message_t brake_pressures;
    brake_pressures.brake_pressure_fl = 150;  // 15.0 bar (scaled by 10)
    brake_pressures.brake_pressure_fr = 50;   // 5.0 bar
    brake_pressures.brake_pressure_rl = 0;    // 0.0 bar
    brake_pressures.brake_pressure_rr = 0;    // 0.0 bar
    
    if (can_send_esc_brake_pressures(&can_interface, &brake_pressures)) {
        printf("Sent brake pressures message (FL=%.1f, FR=%.1f, RL=%.1f, RR=%.1f bar)\n",
               brake_pressures.brake_pressure_fl / 10.0f, brake_pressures.brake_pressure_fr / 10.0f,
               brake_pressures.brake_pressure_rl / 10.0f, brake_pressures.brake_pressure_rr / 10.0f);
    }
    
    // Test message decoding
    printf("\nTesting message decoding:\n");
    can_frame_t frame;
    can_encode_esc_status_1(&esc_status1, &frame);
    
    esc_status_1_message_t decoded_status;
    can_decode_esc_status_1(&frame, &decoded_status);
    
    printf("Original intervention flags: 0x%02X, Decoded: 0x%02X %s\n", 
           esc_status1.intervention_flags, decoded_status.intervention_flags,
           (esc_status1.intervention_flags == decoded_status.intervention_flags) ? "(MATCH)" : "(MISMATCH)");
    
    can_bus_shutdown(&can_interface);
}

void demo_network_management(void) {
    printf("\n=== Network Management Demo ===\n");
    
    can_bus_interface_t can_interface;
    
    if (!can_bus_init(&can_interface, &demo_hw_interface)) {
        printf("Failed to initialize CAN interface\n");
        return;
    }
    
    printf("Testing network management functions:\n");
    
    // Send network management message
    printf("\n1. Sending network management message:\n");
    if (can_send_network_management_message(&can_interface, 0x01)) {
        printf("Sent network management message (command=0x01)\n");
    }
    
    // Test sleep mode
    printf("\n2. Testing sleep mode:\n");
    if (can_enter_sleep_mode(&can_interface)) {
        printf("Entered sleep mode\n");
        usleep(100000); // 100ms
        
        if (can_exit_sleep_mode(&can_interface)) {
            printf("Exited sleep mode\n");
        }
    }
    
    // Send heartbeat
    printf("\n3. Sending heartbeat messages:\n");
    for (int i = 0; i < 3; i++) {
        if (can_send_esc_heartbeat(&can_interface, i)) {
            printf("Sent heartbeat #%d\n", i);
        }
        usleep(50000); // 50ms
    }
    
    can_bus_shutdown(&can_interface);
}

void demo_loopback_test(void) {
    printf("\n=== Loopback Test Demo ===\n");
    
    can_bus_interface_t can_interface;
    
    if (!can_bus_init(&can_interface, &demo_hw_interface)) {
        printf("Failed to initialize CAN interface\n");
        return;
    }
    
    printf("Running built-in loopback test:\n");
    
    if (can_run_loopback_test(&can_interface)) {
        printf("Loopback test PASSED\n");
    } else {
        printf("Loopback test FAILED\n");
    }
    
    can_bus_shutdown(&can_interface);
}

// Callback implementations
static void demo_message_callback(const can_frame_t* frame) {
    printf("Message received callback: ID=0x%03X, DLC=%u\n", frame->id, frame->data_length);
}

static void demo_error_callback(can_error_type_t error, const char* description) {
    printf("Error callback: %s - %s\n", can_get_error_string(error), description);
}

static void demo_bus_state_callback(can_bus_state_t old_state, can_bus_state_t new_state) {
    printf("Bus state callback: %s -> %s\n", 
           can_get_bus_state_string(old_state), 
           can_get_bus_state_string(new_state));
}

// Hardware simulation implementations
static bool demo_hw_init(can_baudrate_t baudrate) {
    printf("Demo HW: CAN controller initialized at %u bps\n", baudrate);
    message_counter = 0;
    return true;
}

static bool demo_hw_deinit(void) {
    printf("Demo HW: CAN controller deinitialized\n");
    return true;
}

static bool demo_hw_send_frame(const can_frame_t* frame) {
    if (hardware_error_simulation) {
        printf("Demo HW: Simulated transmission error for ID=0x%03X\n", frame->id);
        return false;
    }
    
    printf("Demo HW: TX Frame ID=0x%03X DLC=%u Data=[", frame->id, frame->data_length);
    for (int i = 0; i < frame->data_length; i++) {
        printf("%02X", frame->data[i]);
        if (i < frame->data_length - 1) printf(" ");
    }
    printf("] Priority=%s\n", can_get_priority_string(frame->priority));
    
    message_counter++;
    return true;
}

static bool demo_hw_receive_frame(can_frame_t* frame) {
    if (simulated_rx_index < simulated_rx_count) {
        memcpy(frame, &simulated_rx_messages[simulated_rx_index], sizeof(can_frame_t));
        frame->timestamp_ms = demo_hw_get_timestamp();
        simulated_rx_index++;
        
        printf("Demo HW: RX Frame ID=0x%03X DLC=%u\n", frame->id, frame->data_length);
        return true;
    }
    
    return false; // No more simulated messages
}

static bool demo_hw_set_filter(uint32_t filter_id, uint32_t id, uint32_t mask) {
    printf("Demo HW: Set filter %u: ID=0x%03X, Mask=0x%03X\n", filter_id, id, mask);
    return true;
}

static bool demo_hw_clear_filter(uint32_t filter_id) {
    printf("Demo HW: Clear filter %u\n", filter_id);
    return true;
}

static bool demo_hw_get_status(can_bus_status_t* status) {
    if (!status) return false;
    
    status->state = CAN_STATE_ERROR_ACTIVE;
    status->tx_error_count = 0;
    status->rx_error_count = 0;
    status->bus_warning = false;
    status->data_overrun = false;
    status->last_error_type = CAN_ERROR_NONE;
    status->last_error_timestamp = 0;
    
    return true;
}

static bool demo_hw_reset_controller(void) {
    printf("Demo HW: CAN controller reset\n");
    message_counter = 0;
    simulated_rx_index = 0;
    return true;
}

static uint32_t demo_hw_get_timestamp(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static bool demo_hw_enter_sleep_mode(void) {
    printf("Demo HW: Entered sleep mode\n");
    return true;
}

static bool demo_hw_exit_sleep_mode(void) {
    printf("Demo HW: Exited sleep mode\n");
    return true;
}