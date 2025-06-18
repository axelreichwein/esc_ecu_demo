#include "can_bus_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

// Internal constants
#define CAN_THREAD_PRIORITY 10
#define CAN_TX_RETRY_LIMIT 3
#define CAN_TIMING_CHECK_INTERVAL_MS 100
#define CAN_HEARTBEAT_INTERVAL_MS 1000
#define CAN_MAX_MESSAGE_AGE_MS 5000

// Internal function prototypes
static void* can_tx_thread_function(void* arg);
static void* can_rx_thread_function(void* arg);
static void* can_monitor_thread_function(void* arg);
static bool can_buffer_init(can_message_buffer_t* buffer);
static void can_buffer_cleanup(can_message_buffer_t* buffer);
static bool can_buffer_push(can_message_buffer_t* buffer, const can_frame_t* frame);
static bool can_buffer_pop(can_message_buffer_t* buffer, can_frame_t* frame);
static void can_handle_error(can_bus_interface_t* interface, can_error_type_t error, const char* description);
static void can_update_bus_state(can_bus_interface_t* interface, can_bus_state_t new_state);
static bool can_priority_queue_insert(can_bus_interface_t* interface, const can_frame_t* frame, can_message_priority_t priority);
static bool can_priority_queue_get_next(can_bus_interface_t* interface, can_frame_t* frame);
static uint32_t can_get_current_time_ms(void);
static bool can_is_filter_match(const can_filter_t* filter, uint32_t message_id, bool extended_id);

// Initialization and Configuration
bool can_bus_init(can_bus_interface_t* interface, const can_hw_interface_t* hw_interface) {
    if (!interface || !hw_interface) {
        return false;
    }
    
    // Initialize the interface structure
    memset(interface, 0, sizeof(can_bus_interface_t));
    interface->hw_interface = hw_interface;
    interface->initialization_time_ms = can_get_current_time_ms();
    
    // Set default configuration
    interface->config.baudrate = CAN_BAUDRATE_500K;
    interface->config.loopback_mode = false;
    interface->config.silent_mode = false;
    interface->config.auto_retransmission = true;
    interface->config.tx_error_threshold = CAN_ERROR_THRESHOLD;
    interface->config.rx_error_threshold = CAN_ERROR_THRESHOLD;
    interface->config.timeout_ms = 1000;
    interface->config.timestamp_enabled = true;
    
    // Initialize status
    interface->status.state = CAN_STATE_STOPPED;
    interface->status.tx_error_count = 0;
    interface->status.rx_error_count = 0;
    
    // Initialize message buffers
    if (!can_buffer_init(&interface->tx_buffer) || !can_buffer_init(&interface->rx_buffer)) {
        printf("CAN: Failed to initialize message buffers\n");
        return false;
    }
    
    // Initialize priority queue mutex
    if (pthread_mutex_init(&interface->priority_queue_mutex, NULL) != 0) {
        printf("CAN: Failed to initialize priority queue mutex\n");
        can_buffer_cleanup(&interface->tx_buffer);
        can_buffer_cleanup(&interface->rx_buffer);
        return false;
    }
    
    // Initialize interface mutex
    if (pthread_mutex_init(&interface->interface_mutex, NULL) != 0) {
        printf("CAN: Failed to initialize interface mutex\n");
        pthread_mutex_destroy(&interface->priority_queue_mutex);
        can_buffer_cleanup(&interface->tx_buffer);
        can_buffer_cleanup(&interface->rx_buffer);
        return false;
    }
    
    // Initialize hardware
    if (!hw_interface->hw_init(interface->config.baudrate)) {
        printf("CAN: Hardware initialization failed\n");
        pthread_mutex_destroy(&interface->interface_mutex);
        pthread_mutex_destroy(&interface->priority_queue_mutex);
        can_buffer_cleanup(&interface->tx_buffer);
        can_buffer_cleanup(&interface->rx_buffer);
        return false;
    }
    
    // Start communication threads
    interface->threads_active = true;
    
    if (pthread_create(&interface->tx_thread, NULL, can_tx_thread_function, interface) != 0) {
        printf("CAN: Failed to create transmission thread\n");
        interface->threads_active = false;
        return false;
    }
    
    if (pthread_create(&interface->rx_thread, NULL, can_rx_thread_function, interface) != 0) {
        printf("CAN: Failed to create reception thread\n");
        interface->threads_active = false;
        pthread_join(interface->tx_thread, NULL);
        return false;
    }
    
    if (pthread_create(&interface->monitor_thread, NULL, can_monitor_thread_function, interface) != 0) {
        printf("CAN: Failed to create monitor thread\n");
        interface->threads_active = false;
        pthread_join(interface->tx_thread, NULL);
        pthread_join(interface->rx_thread, NULL);
        return false;
    }
    
    // Update state to error active (normal operation)
    can_update_bus_state(interface, CAN_STATE_ERROR_ACTIVE);
    interface->system_initialized = true;
    
    printf("CAN Bus interface initialized successfully at %u bps\n", interface->config.baudrate);
    return true;
}

bool can_bus_configure(can_bus_interface_t* interface, const can_bus_config_t* config) {
    if (!interface || !config || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    // Store new configuration
    memcpy(&interface->config, config, sizeof(can_bus_config_t));
    
    // Reconfigure hardware if needed
    bool hw_reconfigure_needed = false;
    if (interface->config.baudrate != config->baudrate) {
        hw_reconfigure_needed = true;
    }
    
    if (hw_reconfigure_needed && interface->hw_interface->hw_init) {
        interface->hw_interface->hw_init(config->baudrate);
        printf("CAN Bus reconfigured with new baudrate: %u bps\n", config->baudrate);
    }
    
    pthread_mutex_unlock(&interface->interface_mutex);
    return true;
}

bool can_bus_shutdown(can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    printf("CAN Bus interface shutdown initiated\n");
    
    // Stop threads
    interface->threads_active = false;
    
    // Wait for threads to finish
    pthread_join(interface->tx_thread, NULL);
    pthread_join(interface->rx_thread, NULL);
    pthread_join(interface->monitor_thread, NULL);
    
    // Shutdown hardware
    if (interface->hw_interface->hw_deinit) {
        interface->hw_interface->hw_deinit();
    }
    
    // Cleanup resources
    can_buffer_cleanup(&interface->tx_buffer);
    can_buffer_cleanup(&interface->rx_buffer);
    pthread_mutex_destroy(&interface->interface_mutex);
    pthread_mutex_destroy(&interface->priority_queue_mutex);
    
    // Print final statistics
    printf("CAN Bus final statistics:\n");
    printf("  Messages TX: %u\n", interface->statistics.messages_transmitted);
    printf("  Messages RX: %u\n", interface->statistics.messages_received);
    printf("  TX Errors: %u\n", interface->statistics.transmission_errors);
    printf("  RX Errors: %u\n", interface->statistics.reception_errors);
    printf("  Bus Off Events: %u\n", interface->statistics.bus_off_events);
    
    interface->system_initialized = false;
    printf("CAN Bus interface shutdown complete\n");
    return true;
}

bool can_bus_reset(can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    // Reset hardware controller
    if (interface->hw_interface->hw_reset_controller) {
        interface->hw_interface->hw_reset_controller();
    }
    
    // Reset error counters
    interface->status.tx_error_count = 0;
    interface->status.rx_error_count = 0;
    interface->status.bus_warning = false;
    interface->status.data_overrun = false;
    
    // Clear buffers
    pthread_mutex_lock(&interface->tx_buffer.mutex);
    interface->tx_buffer.head = 0;
    interface->tx_buffer.tail = 0;
    interface->tx_buffer.count = 0;
    pthread_mutex_unlock(&interface->tx_buffer.mutex);
    
    pthread_mutex_lock(&interface->rx_buffer.mutex);
    interface->rx_buffer.head = 0;
    interface->rx_buffer.tail = 0;
    interface->rx_buffer.count = 0;
    pthread_mutex_unlock(&interface->rx_buffer.mutex);
    
    // Update state to error active
    can_update_bus_state(interface, CAN_STATE_ERROR_ACTIVE);
    
    pthread_mutex_unlock(&interface->interface_mutex);
    
    printf("CAN Bus interface reset completed\n");
    return true;
}

// Message Transmission
bool can_send_message(can_bus_interface_t* interface, const can_frame_t* frame) {
    return can_send_message_priority(interface, frame, CAN_PRIORITY_NORMAL);
}

bool can_send_message_priority(can_bus_interface_t* interface, const can_frame_t* frame, can_message_priority_t priority) {
    if (!interface || !frame || !interface->system_initialized) {
        return false;
    }
    
    // Validate frame
    if (!can_validate_frame(frame)) {
        interface->statistics.transmission_errors++;
        return false;
    }
    
    // Check bus state
    if (interface->status.state == CAN_STATE_BUS_OFF) {
        interface->statistics.transmission_errors++;
        return false;
    }
    
    // Add to priority queue
    return can_priority_queue_insert(interface, frame, priority);
}

bool can_send_message_cyclic(can_bus_interface_t* interface, const can_frame_t* frame, uint32_t period_ms) {
    if (!interface || !frame || !interface->system_initialized || period_ms == 0) {
        return false;
    }
    
    // Find or create timing entry for this message
    pthread_mutex_lock(&interface->interface_mutex);
    
    for (int i = 0; i < interface->monitored_message_count; i++) {
        if (interface->message_timings[i].transmission_period_ms == period_ms) {
            // Update existing entry
            interface->message_timings[i].cyclic_transmission = true;
            interface->message_timings[i].transmission_period_ms = period_ms;
            interface->message_timings[i].next_transmission_time = can_get_current_time_ms() + period_ms;
            pthread_mutex_unlock(&interface->interface_mutex);
            return can_send_message_priority(interface, frame, CAN_PRIORITY_NORMAL);
        }
    }
    
    // Add new cyclic message
    if (interface->monitored_message_count < 32) {
        can_message_timing_t* timing = &interface->message_timings[interface->monitored_message_count++];
        timing->transmission_period_ms = period_ms;
        timing->cyclic_transmission = true;
        timing->next_transmission_time = can_get_current_time_ms() + period_ms;
        timing->last_transmission_time = 0;
    }
    
    pthread_mutex_unlock(&interface->interface_mutex);
    return can_send_message_priority(interface, frame, CAN_PRIORITY_NORMAL);
}

bool can_cancel_cyclic_message(can_bus_interface_t* interface, uint32_t message_id) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    // Find and disable cyclic transmission for this message ID
    for (int i = 0; i < interface->monitored_message_count; i++) {
        // Note: In a real implementation, we'd store message ID in timing structure
        interface->message_timings[i].cyclic_transmission = false;
    }
    
    pthread_mutex_unlock(&interface->interface_mutex);
    
    printf("Cyclic transmission canceled for message ID 0x%03X\n", message_id);
    return true;
}

// Message Reception
bool can_receive_message(can_bus_interface_t* interface, can_frame_t* frame, uint32_t timeout_ms) {
    if (!interface || !frame || !interface->system_initialized) {
        return false;
    }
    
    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += timeout_ms / 1000;
    timeout.tv_nsec += (timeout_ms % 1000) * 1000000;
    if (timeout.tv_nsec >= 1000000000) {
        timeout.tv_sec++;
        timeout.tv_nsec -= 1000000000;
    }
    
    pthread_mutex_lock(&interface->rx_buffer.mutex);
    
    while (interface->rx_buffer.count == 0 && interface->threads_active) {
        int result = pthread_cond_timedwait(&interface->rx_buffer.not_empty, &interface->rx_buffer.mutex, &timeout);
        if (result == ETIMEDOUT) {
            pthread_mutex_unlock(&interface->rx_buffer.mutex);
            return false;
        }
    }
    
    bool success = can_buffer_pop(&interface->rx_buffer, frame);
    pthread_mutex_unlock(&interface->rx_buffer.mutex);
    
    return success;
}

bool can_receive_message_nonblocking(can_bus_interface_t* interface, can_frame_t* frame) {
    if (!interface || !frame || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&interface->rx_buffer.mutex);
    bool success = can_buffer_pop(&interface->rx_buffer, frame);
    pthread_mutex_unlock(&interface->rx_buffer.mutex);
    
    return success;
}

uint16_t can_get_pending_message_count(const can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return 0;
    }
    
    return interface->rx_buffer.count;
}

// Message Filtering
bool can_add_filter(can_bus_interface_t* interface, uint32_t id, uint32_t mask, bool extended_id) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    // Find empty filter slot
    for (int i = 0; i < CAN_FILTER_TABLE_SIZE; i++) {
        if (!interface->filters[i].enabled) {
            interface->filters[i].id = id;
            interface->filters[i].mask = mask;
            interface->filters[i].extended_id = extended_id;
            interface->filters[i].enabled = true;
            interface->filters[i].accept_count = 0;
            interface->filters[i].reject_count = 0;
            interface->active_filter_count++;
            
            // Configure hardware filter
            if (interface->hw_interface->hw_set_filter) {
                interface->hw_interface->hw_set_filter(i, id, mask);
            }
            
            pthread_mutex_unlock(&interface->interface_mutex);
            printf("CAN filter added: ID=0x%03X, Mask=0x%03X, Extended=%s\n", 
                   id, mask, extended_id ? "Yes" : "No");
            return true;
        }
    }
    
    pthread_mutex_unlock(&interface->interface_mutex);
    printf("CAN: No available filter slots\n");
    return false;
}

bool can_remove_filter(can_bus_interface_t* interface, uint32_t id) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    // Find and remove filter
    for (int i = 0; i < CAN_FILTER_TABLE_SIZE; i++) {
        if (interface->filters[i].enabled && interface->filters[i].id == id) {
            interface->filters[i].enabled = false;
            interface->active_filter_count--;
            
            // Clear hardware filter
            if (interface->hw_interface->hw_clear_filter) {
                interface->hw_interface->hw_clear_filter(i);
            }
            
            pthread_mutex_unlock(&interface->interface_mutex);
            printf("CAN filter removed: ID=0x%03X\n", id);
            return true;
        }
    }
    
    pthread_mutex_unlock(&interface->interface_mutex);
    return false;
}

bool can_clear_all_filters(can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    for (int i = 0; i < CAN_FILTER_TABLE_SIZE; i++) {
        if (interface->filters[i].enabled) {
            interface->filters[i].enabled = false;
            
            // Clear hardware filter
            if (interface->hw_interface->hw_clear_filter) {
                interface->hw_interface->hw_clear_filter(i);
            }
        }
    }
    
    interface->active_filter_count = 0;
    pthread_mutex_unlock(&interface->interface_mutex);
    
    printf("All CAN filters cleared\n");
    return true;
}

// Timing Supervision
bool can_add_message_timing(can_bus_interface_t* interface, uint32_t message_id, uint32_t period_ms, uint32_t timeout_ms) {
    if (!interface || !interface->system_initialized || interface->monitored_message_count >= 32) {
        return false;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    can_message_timing_t* timing = &interface->message_timings[interface->monitored_message_count++];
    timing->transmission_period_ms = period_ms;
    timing->timeout_ms = timeout_ms;
    timing->max_age_ms = timeout_ms * 2;
    timing->cyclic_transmission = false;
    timing->next_transmission_time = 0;
    timing->last_transmission_time = 0;
    timing->last_reception_time = can_get_current_time_ms();
    
    pthread_mutex_unlock(&interface->interface_mutex);
    
    printf("CAN message timing added: ID=0x%03X, Period=%ums, Timeout=%ums\n", 
           message_id, period_ms, timeout_ms);
    return true;
}

bool can_remove_message_timing(can_bus_interface_t* interface, uint32_t message_id) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    // In a real implementation, we'd match by message_id
    // For simplicity, just clear the first entry
    if (interface->monitored_message_count > 0) {
        interface->monitored_message_count--;
        memmove(&interface->message_timings[0], &interface->message_timings[1], 
                interface->monitored_message_count * sizeof(can_message_timing_t));
    }
    
    pthread_mutex_unlock(&interface->interface_mutex);
    
    printf("CAN message timing removed: ID=0x%03X\n", message_id);
    return true;
}

bool can_check_message_timeouts(can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    uint32_t current_time = can_get_current_time_ms();
    bool timeout_detected = false;
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    for (int i = 0; i < interface->monitored_message_count; i++) {
        can_message_timing_t* timing = &interface->message_timings[i];
        
        // Check reception timeout
        if (timing->timeout_ms > 0) {
            uint32_t elapsed = current_time - timing->last_reception_time;
            if (elapsed > timing->timeout_ms) {
                printf("CAN message timeout detected: elapsed=%ums, timeout=%ums\n", 
                       elapsed, timing->timeout_ms);
                timeout_detected = true;
            }
        }
        
        // Check if cyclic message needs to be sent
        if (timing->cyclic_transmission && timing->next_transmission_time > 0) {
            if (current_time >= timing->next_transmission_time) {
                timing->next_transmission_time = current_time + timing->transmission_period_ms;
                // Note: In real implementation, would queue the cyclic message here
            }
        }
    }
    
    pthread_mutex_unlock(&interface->interface_mutex);
    
    return !timeout_detected;
}

// Status and Diagnostics
can_bus_state_t can_get_bus_state(const can_bus_interface_t* interface) {
    return interface ? interface->status.state : CAN_STATE_STOPPED;
}

bool can_get_statistics(const can_bus_interface_t* interface, can_bus_statistics_t* stats) {
    if (!interface || !stats || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock((pthread_mutex_t*)&interface->interface_mutex);
    memcpy(stats, &interface->statistics, sizeof(can_bus_statistics_t));
    pthread_mutex_unlock((pthread_mutex_t*)&interface->interface_mutex);
    
    return true;
}

bool can_get_error_status(const can_bus_interface_t* interface, can_bus_status_t* status) {
    if (!interface || !status || !interface->system_initialized) {
        return false;
    }
    
    pthread_mutex_lock((pthread_mutex_t*)&interface->interface_mutex);
    memcpy(status, &interface->status, sizeof(can_bus_status_t));
    status->uptime_ms = can_get_current_time_ms() - interface->initialization_time_ms;
    pthread_mutex_unlock((pthread_mutex_t*)&interface->interface_mutex);
    
    return true;
}

void can_clear_statistics(can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return;
    }
    
    pthread_mutex_lock(&interface->interface_mutex);
    memset(&interface->statistics, 0, sizeof(can_bus_statistics_t));
    pthread_mutex_unlock(&interface->interface_mutex);
    
    printf("CAN statistics cleared\n");
}

// Callback Registration
bool can_register_message_callback(can_bus_interface_t* interface, void (*callback)(const can_frame_t* frame)) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    interface->message_received_callback = callback;
    printf("CAN message received callback registered\n");
    return true;
}

bool can_register_error_callback(can_bus_interface_t* interface, void (*callback)(can_error_type_t error, const char* description)) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    interface->error_callback = callback;
    printf("CAN error callback registered\n");
    return true;
}

bool can_register_bus_state_callback(can_bus_interface_t* interface, void (*callback)(can_bus_state_t old_state, can_bus_state_t new_state)) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    interface->bus_state_callback = callback;
    printf("CAN bus state callback registered\n");
    return true;
}

// ESC-specific Message Functions
bool can_send_esc_status_1(can_bus_interface_t* interface, const esc_status_1_message_t* status) {
    if (!interface || !status) {
        return false;
    }
    
    can_frame_t frame;
    can_encode_esc_status_1(status, &frame);
    return can_send_message_priority(interface, &frame, CAN_PRIORITY_CRITICAL);
}

bool can_send_esc_status_2(can_bus_interface_t* interface, const esc_status_2_message_t* status) {
    if (!interface || !status) {
        return false;
    }
    
    can_frame_t frame;
    can_encode_esc_status_2(status, &frame);
    return can_send_message_priority(interface, &frame, CAN_PRIORITY_HIGH);
}

bool can_send_esc_wheel_speeds(can_bus_interface_t* interface, const esc_wheel_speeds_message_t* speeds) {
    if (!interface || !speeds) {
        return false;
    }
    
    can_frame_t frame;
    frame.id = CAN_MSG_ESC_WHEEL_SPEEDS;
    frame.extended_id = false;
    frame.remote_frame = false;
    frame.data_length = 8;
    frame.timestamp_ms = can_get_current_time_ms();
    frame.priority = CAN_PRIORITY_HIGH;
    
    frame.data[0] = (speeds->wheel_speed_fl >> 8) & 0xFF;
    frame.data[1] = speeds->wheel_speed_fl & 0xFF;
    frame.data[2] = (speeds->wheel_speed_fr >> 8) & 0xFF;
    frame.data[3] = speeds->wheel_speed_fr & 0xFF;
    frame.data[4] = (speeds->wheel_speed_rl >> 8) & 0xFF;
    frame.data[5] = speeds->wheel_speed_rl & 0xFF;
    frame.data[6] = (speeds->wheel_speed_rr >> 8) & 0xFF;
    frame.data[7] = speeds->wheel_speed_rr & 0xFF;
    
    return can_send_message_priority(interface, &frame, CAN_PRIORITY_HIGH);
}

bool can_send_esc_brake_pressures(can_bus_interface_t* interface, const esc_brake_pressure_message_t* pressures) {
    if (!interface || !pressures) {
        return false;
    }
    
    can_frame_t frame;
    frame.id = CAN_MSG_ESC_BRAKE_PRESSURE;
    frame.extended_id = false;
    frame.remote_frame = false;
    frame.data_length = 8;
    frame.timestamp_ms = can_get_current_time_ms();
    frame.priority = CAN_PRIORITY_CRITICAL;
    
    frame.data[0] = (pressures->brake_pressure_fl >> 8) & 0xFF;
    frame.data[1] = pressures->brake_pressure_fl & 0xFF;
    frame.data[2] = (pressures->brake_pressure_fr >> 8) & 0xFF;
    frame.data[3] = pressures->brake_pressure_fr & 0xFF;
    frame.data[4] = (pressures->brake_pressure_rl >> 8) & 0xFF;
    frame.data[5] = pressures->brake_pressure_rl & 0xFF;
    frame.data[6] = (pressures->brake_pressure_rr >> 8) & 0xFF;
    frame.data[7] = pressures->brake_pressure_rr & 0xFF;
    
    return can_send_message_priority(interface, &frame, CAN_PRIORITY_CRITICAL);
}

bool can_send_esc_heartbeat(can_bus_interface_t* interface, uint8_t sequence_counter) {
    if (!interface) {
        return false;
    }
    
    can_frame_t frame;
    frame.id = CAN_MSG_ESC_HEARTBEAT;
    frame.extended_id = false;
    frame.remote_frame = false;
    frame.data_length = 8;
    frame.timestamp_ms = can_get_current_time_ms();
    frame.priority = CAN_PRIORITY_NORMAL;
    
    frame.data[0] = sequence_counter;
    frame.data[1] = (uint8_t)interface->status.state;
    frame.data[2] = interface->status.tx_error_count;
    frame.data[3] = interface->status.rx_error_count;
    frame.data[4] = (interface->statistics.messages_transmitted >> 8) & 0xFF;
    frame.data[5] = interface->statistics.messages_transmitted & 0xFF;
    frame.data[6] = (interface->statistics.messages_received >> 8) & 0xFF;
    frame.data[7] = interface->statistics.messages_received & 0xFF;
    
    return can_send_message_priority(interface, &frame, CAN_PRIORITY_NORMAL);
}

// Message Encoding/Decoding Utilities
void can_encode_esc_status_1(const esc_status_1_message_t* status, can_frame_t* frame) {
    if (!status || !frame) return;
    
    frame->id = CAN_MSG_ESC_STATUS_1;
    frame->extended_id = false;
    frame->remote_frame = false;
    frame->data_length = 8;
    frame->timestamp_ms = can_get_current_time_ms();
    frame->priority = CAN_PRIORITY_CRITICAL;
    
    frame->data[0] = status->system_status;
    frame->data[1] = status->intervention_flags;
    frame->data[2] = status->warning_lights;
    frame->data[3] = status->traction_control_status;
    frame->data[4] = status->stability_control_status;
    frame->data[5] = status->brake_assist_status;
    frame->data[6] = status->fault_code_high;
    frame->data[7] = status->fault_code_low;
}

void can_decode_esc_status_1(const can_frame_t* frame, esc_status_1_message_t* status) {
    if (!frame || !status || frame->data_length < 8) return;
    
    status->system_status = frame->data[0];
    status->intervention_flags = frame->data[1];
    status->warning_lights = frame->data[2];
    status->traction_control_status = frame->data[3];
    status->stability_control_status = frame->data[4];
    status->brake_assist_status = frame->data[5];
    status->fault_code_high = frame->data[6];
    status->fault_code_low = frame->data[7];
}

void can_encode_esc_status_2(const esc_status_2_message_t* status, can_frame_t* frame) {
    if (!status || !frame) return;
    
    frame->id = CAN_MSG_ESC_STATUS_2;
    frame->extended_id = false;
    frame->remote_frame = false;
    frame->data_length = 8;
    frame->timestamp_ms = can_get_current_time_ms();
    frame->priority = CAN_PRIORITY_HIGH;
    
    frame->data[0] = (status->yaw_rate >> 8) & 0xFF;
    frame->data[1] = status->yaw_rate & 0xFF;
    frame->data[2] = (status->lateral_acceleration >> 8) & 0xFF;
    frame->data[3] = status->lateral_acceleration & 0xFF;
    frame->data[4] = (status->steering_angle >> 8) & 0xFF;
    frame->data[5] = status->steering_angle & 0xFF;
    frame->data[6] = status->sensor_status;
    frame->data[7] = status->reserved;
}

void can_decode_esc_status_2(const can_frame_t* frame, esc_status_2_message_t* status) {
    if (!frame || !status || frame->data_length < 8) return;
    
    status->yaw_rate = (int16_t)((frame->data[0] << 8) | frame->data[1]);
    status->lateral_acceleration = (int16_t)((frame->data[2] << 8) | frame->data[3]);
    status->steering_angle = (int16_t)((frame->data[4] << 8) | frame->data[5]);
    status->sensor_status = frame->data[6];
    status->reserved = frame->data[7];
}

// Utility Functions
const char* can_get_error_string(can_error_type_t error) {
    switch (error) {
        case CAN_ERROR_NONE: return "No Error";
        case CAN_ERROR_STUFF: return "Bit Stuffing Error";
        case CAN_ERROR_FORM: return "Form Error";
        case CAN_ERROR_ACK: return "Acknowledgment Error";
        case CAN_ERROR_BIT1: return "Bit1 Error";
        case CAN_ERROR_BIT0: return "Bit0 Error";
        case CAN_ERROR_CRC: return "CRC Error";
        case CAN_ERROR_BUS_OFF: return "Bus Off";
        case CAN_ERROR_PASSIVE: return "Error Passive";
        default: return "Unknown Error";
    }
}

const char* can_get_bus_state_string(can_bus_state_t state) {
    switch (state) {
        case CAN_STATE_ERROR_ACTIVE: return "Error Active";
        case CAN_STATE_ERROR_PASSIVE: return "Error Passive";
        case CAN_STATE_BUS_OFF: return "Bus Off";
        case CAN_STATE_STOPPED: return "Stopped";
        default: return "Unknown State";
    }
}

const char* can_get_priority_string(can_message_priority_t priority) {
    switch (priority) {
        case CAN_PRIORITY_EMERGENCY: return "Emergency";
        case CAN_PRIORITY_CRITICAL: return "Critical";
        case CAN_PRIORITY_HIGH: return "High";
        case CAN_PRIORITY_NORMAL: return "Normal";
        case CAN_PRIORITY_LOW: return "Low";
        case CAN_PRIORITY_BACKGROUND: return "Background";
        default: return "Unknown Priority";
    }
}

bool can_validate_frame(const can_frame_t* frame) {
    if (!frame) return false;
    
    // Check data length
    if (frame->data_length > CAN_FRAME_MAX_DATA_LENGTH) return false;
    
    // Check CAN ID ranges
    if (frame->extended_id) {
        if (frame->id > CAN_EXTENDED_FRAME_ID_MASK) return false;
    } else {
        if (frame->id > CAN_STANDARD_FRAME_ID_MASK) return false;
    }
    
    return true;
}

uint8_t can_calculate_checksum(const uint8_t* data, uint8_t length) {
    if (!data || length == 0) return 0;
    
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Network Management
bool can_enter_sleep_mode(can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    if (interface->hw_interface->hw_enter_sleep_mode) {
        interface->hw_interface->hw_enter_sleep_mode();
        printf("CAN Bus entered sleep mode\n");
        return true;
    }
    
    return false;
}

bool can_exit_sleep_mode(can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    if (interface->hw_interface->hw_exit_sleep_mode) {
        interface->hw_interface->hw_exit_sleep_mode();
        printf("CAN Bus exited sleep mode\n");
        return true;
    }
    
    return false;
}

bool can_send_network_management_message(can_bus_interface_t* interface, uint8_t command) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    can_frame_t frame;
    frame.id = CAN_MSG_NM_ESC;
    frame.extended_id = false;
    frame.remote_frame = false;
    frame.data_length = 8;
    frame.timestamp_ms = can_get_current_time_ms();
    frame.priority = CAN_PRIORITY_HIGH;
    
    frame.data[0] = command;
    frame.data[1] = 0x20; // ESC node ID
    frame.data[2] = 0x01; // Version
    frame.data[3] = 0x00; // Reserved
    frame.data[4] = 0x00; // Reserved
    frame.data[5] = 0x00; // Reserved
    frame.data[6] = 0x00; // Reserved
    frame.data[7] = can_calculate_checksum(frame.data, 7);
    
    return can_send_message_priority(interface, &frame, CAN_PRIORITY_HIGH);
}

// Debug and Testing Functions
void can_print_frame(const can_frame_t* frame) {
    if (!frame) return;
    
    printf("CAN Frame: ID=0x%03X %s %s DLC=%u Data=[",
           frame->id,
           frame->extended_id ? "EXT" : "STD",
           frame->remote_frame ? "RTR" : "DATA",
           frame->data_length);
    
    for (int i = 0; i < frame->data_length; i++) {
        printf("%02X", frame->data[i]);
        if (i < frame->data_length - 1) printf(" ");
    }
    
    printf("] Priority=%s Timestamp=%ums\n",
           can_get_priority_string(frame->priority),
           frame->timestamp_ms);
}

void can_print_statistics(const can_bus_interface_t* interface) {
    if (!interface) return;
    
    printf("\n=== CAN Bus Statistics ===\n");
    printf("Messages Transmitted: %u\n", interface->statistics.messages_transmitted);
    printf("Messages Received: %u\n", interface->statistics.messages_received);
    printf("Transmission Errors: %u\n", interface->statistics.transmission_errors);
    printf("Reception Errors: %u\n", interface->statistics.reception_errors);
    printf("Bus Off Events: %u\n", interface->statistics.bus_off_events);
    printf("Error Passive Events: %u\n", interface->statistics.error_passive_events);
    printf("Arbitration Lost: %u\n", interface->statistics.arbitration_lost_count);
    printf("Buffer Overflows: %u\n", interface->statistics.buffer_overflow_count);
    printf("CRC Errors: %u\n", interface->statistics.crc_errors);
    printf("Form Errors: %u\n", interface->statistics.form_errors);
    printf("ACK Errors: %u\n", interface->statistics.ack_errors);
    printf("Stuff Errors: %u\n", interface->statistics.stuff_errors);
    printf("Bit Errors: %u\n", interface->statistics.bit_errors);
    printf("==========================\n");
}

void can_print_status(const can_bus_interface_t* interface) {
    if (!interface) return;
    
    printf("\n=== CAN Bus Status ===\n");
    printf("Bus State: %s\n", can_get_bus_state_string(interface->status.state));
    printf("TX Error Count: %u\n", interface->status.tx_error_count);
    printf("RX Error Count: %u\n", interface->status.rx_error_count);
    printf("Bus Warning: %s\n", interface->status.bus_warning ? "Yes" : "No");
    printf("Data Overrun: %s\n", interface->status.data_overrun ? "Yes" : "No");
    printf("Uptime: %u ms\n", can_get_current_time_ms() - interface->initialization_time_ms);
    printf("Active Filters: %u\n", interface->active_filter_count);
    printf("Monitored Messages: %u\n", interface->monitored_message_count);
    printf("TX Buffer Count: %u\n", interface->tx_buffer.count);
    printf("RX Buffer Count: %u\n", interface->rx_buffer.count);
    printf("======================\n");
}

bool can_run_loopback_test(can_bus_interface_t* interface) {
    if (!interface || !interface->system_initialized) {
        return false;
    }
    
    printf("Running CAN loopback test...\n");
    
    // Enable loopback mode
    bool original_loopback = interface->config.loopback_mode;
    interface->config.loopback_mode = true;
    
    // Send test message
    can_frame_t test_frame;
    test_frame.id = 0x123;
    test_frame.extended_id = false;
    test_frame.remote_frame = false;
    test_frame.data_length = 8;
    test_frame.timestamp_ms = can_get_current_time_ms();
    test_frame.priority = CAN_PRIORITY_NORMAL;
    
    for (int i = 0; i < 8; i++) {
        test_frame.data[i] = 0xAA + i;
    }
    
    if (!can_send_message(interface, &test_frame)) {
        printf("Loopback test failed: Could not send message\n");
        interface->config.loopback_mode = original_loopback;
        return false;
    }
    
    // Try to receive the message
    can_frame_t received_frame;
    if (!can_receive_message(interface, &received_frame, 1000)) {
        printf("Loopback test failed: No message received\n");
        interface->config.loopback_mode = original_loopback;
        return false;
    }
    
    // Verify message content
    bool content_match = (received_frame.id == test_frame.id &&
                         received_frame.data_length == test_frame.data_length &&
                         memcmp(received_frame.data, test_frame.data, test_frame.data_length) == 0);
    
    interface->config.loopback_mode = original_loopback;
    
    if (content_match) {
        printf("Loopback test passed\n");
        return true;
    } else {
        printf("Loopback test failed: Message content mismatch\n");
        return false;
    }
}

// Internal Helper Functions
static bool can_buffer_init(can_message_buffer_t* buffer) {
    if (!buffer) return false;
    
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
    buffer->overflow_count = 0;
    
    if (pthread_mutex_init(&buffer->mutex, NULL) != 0) {
        return false;
    }
    
    if (pthread_cond_init(&buffer->not_empty, NULL) != 0) {
        pthread_mutex_destroy(&buffer->mutex);
        return false;
    }
    
    if (pthread_cond_init(&buffer->not_full, NULL) != 0) {
        pthread_mutex_destroy(&buffer->mutex);
        pthread_cond_destroy(&buffer->not_empty);
        return false;
    }
    
    return true;
}

static void can_buffer_cleanup(can_message_buffer_t* buffer) {
    if (!buffer) return;
    
    pthread_mutex_destroy(&buffer->mutex);
    pthread_cond_destroy(&buffer->not_empty);
    pthread_cond_destroy(&buffer->not_full);
    
    memset(buffer, 0, sizeof(can_message_buffer_t));
}

static bool can_buffer_push(can_message_buffer_t* buffer, const can_frame_t* frame) {
    if (!buffer || !frame) return false;
    
    if (buffer->count >= CAN_MESSAGE_BUFFER_SIZE) {
        buffer->overflow_count++;
        return false;
    }
    
    memcpy(&buffer->messages[buffer->head], frame, sizeof(can_frame_t));
    buffer->head = (buffer->head + 1) % CAN_MESSAGE_BUFFER_SIZE;
    buffer->count++;
    
    pthread_cond_signal(&buffer->not_empty);
    return true;
}

static bool can_buffer_pop(can_message_buffer_t* buffer, can_frame_t* frame) {
    if (!buffer || !frame || buffer->count == 0) return false;
    
    memcpy(frame, &buffer->messages[buffer->tail], sizeof(can_frame_t));
    buffer->tail = (buffer->tail + 1) % CAN_MESSAGE_BUFFER_SIZE;
    buffer->count--;
    
    pthread_cond_signal(&buffer->not_full);
    return true;
}

static void can_handle_error(can_bus_interface_t* interface, can_error_type_t error, const char* description) {
    if (!interface) return;
    
    pthread_mutex_lock(&interface->interface_mutex);
    
    interface->status.last_error_type = error;
    interface->status.last_error_timestamp = can_get_current_time_ms();
    
    // Update error counters and state
    switch (error) {
        case CAN_ERROR_BUS_OFF:
            interface->statistics.bus_off_events++;
            can_update_bus_state(interface, CAN_STATE_BUS_OFF);
            break;
        case CAN_ERROR_PASSIVE:
            interface->statistics.error_passive_events++;
            can_update_bus_state(interface, CAN_STATE_ERROR_PASSIVE);
            break;
        case CAN_ERROR_CRC:
            interface->statistics.crc_errors++;
            break;
        case CAN_ERROR_FORM:
            interface->statistics.form_errors++;
            break;
        case CAN_ERROR_ACK:
            interface->statistics.ack_errors++;
            break;
        case CAN_ERROR_STUFF:
            interface->statistics.stuff_errors++;
            break;
        case CAN_ERROR_BIT1:
        case CAN_ERROR_BIT0:
            interface->statistics.bit_errors++;
            break;
        default:
            break;
    }
    
    pthread_mutex_unlock(&interface->interface_mutex);
    
    // Call error callback if registered
    if (interface->error_callback) {
        interface->error_callback(error, description);
    }
    
    printf("CAN Error: %s - %s\n", can_get_error_string(error), description ? description : "");
}

static void can_update_bus_state(can_bus_interface_t* interface, can_bus_state_t new_state) {
    if (!interface) return;
    
    can_bus_state_t old_state = interface->status.state;
    
    if (old_state != new_state) {
        interface->status.state = new_state;
        
        // Call bus state callback if registered
        if (interface->bus_state_callback) {
            interface->bus_state_callback(old_state, new_state);
        }
        
        printf("CAN Bus state changed: %s -> %s\n", 
               can_get_bus_state_string(old_state), 
               can_get_bus_state_string(new_state));
    }
}

static bool can_priority_queue_insert(can_bus_interface_t* interface, const can_frame_t* frame, can_message_priority_t priority) {
    if (!interface || !frame) return false;
    
    pthread_mutex_lock(&interface->priority_queue_mutex);
    
    if (interface->priority_queue_size >= CAN_MESSAGE_BUFFER_SIZE) {
        pthread_mutex_unlock(&interface->priority_queue_mutex);
        interface->statistics.buffer_overflow_count++;
        return false;
    }
    
    // Insert message in priority order (insertion sort)
    int insert_pos = interface->priority_queue_size;
    for (int i = 0; i < interface->priority_queue_size; i++) {
        if (priority < interface->priority_queue[i].frame.priority) {
            insert_pos = i;
            break;
        }
    }
    
    // Shift messages to make room
    for (int i = interface->priority_queue_size; i > insert_pos; i--) {
        interface->priority_queue[i] = interface->priority_queue[i-1];
    }
    
    // Insert new message
    can_priority_queue_entry_t* entry = &interface->priority_queue[insert_pos];
    memcpy(&entry->frame, frame, sizeof(can_frame_t));
    entry->frame.priority = priority;
    entry->retry_count = 0;
    entry->pending_transmission = true;
    entry->timing.next_transmission_time = can_get_current_time_ms();
    
    interface->priority_queue_size++;
    
    pthread_mutex_unlock(&interface->priority_queue_mutex);
    
    // Signal TX buffer that data is available
    pthread_cond_signal(&interface->tx_buffer.not_empty);
    
    return true;
}

static bool can_priority_queue_get_next(can_bus_interface_t* interface, can_frame_t* frame) {
    if (!interface || !frame) return false;
    
    pthread_mutex_lock(&interface->priority_queue_mutex);
    
    if (interface->priority_queue_size == 0) {
        pthread_mutex_unlock(&interface->priority_queue_mutex);
        return false;
    }
    
    // Get highest priority message (first in queue)
    memcpy(frame, &interface->priority_queue[0].frame, sizeof(can_frame_t));
    
    // Remove from queue
    for (int i = 0; i < interface->priority_queue_size - 1; i++) {
        interface->priority_queue[i] = interface->priority_queue[i + 1];
    }
    interface->priority_queue_size--;
    
    pthread_mutex_unlock(&interface->priority_queue_mutex);
    return true;
}

static uint32_t can_get_current_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static bool can_is_filter_match(const can_filter_t* filter, uint32_t message_id, bool extended_id) {
    if (!filter || !filter->enabled) return false;
    
    // Check ID type match
    if (filter->extended_id != extended_id) return false;
    
    // Apply mask and compare
    return (message_id & filter->mask) == (filter->id & filter->mask);
}

// Thread Functions
static void* can_tx_thread_function(void* arg) {
    can_bus_interface_t* interface = (can_bus_interface_t*)arg;
    can_frame_t frame;
    
    printf("CAN TX thread started\n");
    
    while (interface->threads_active) {
        // Get next message from priority queue
        if (can_priority_queue_get_next(interface, &frame)) {
            // Attempt to send message
            if (interface->hw_interface->hw_send_frame) {
                if (interface->hw_interface->hw_send_frame(&frame)) {
                    interface->statistics.messages_transmitted++;
                } else {
                    interface->statistics.transmission_errors++;
                    can_handle_error(interface, CAN_ERROR_ACK, "Transmission failed");
                }
            }
        } else {
            // No messages to send, wait a bit
            usleep(1000); // 1ms
        }
    }
    
    printf("CAN TX thread terminated\n");
    return NULL;
}

static void* can_rx_thread_function(void* arg) {
    can_bus_interface_t* interface = (can_bus_interface_t*)arg;
    can_frame_t frame;
    
    printf("CAN RX thread started\n");
    
    while (interface->threads_active) {
        // Try to receive message from hardware
        if (interface->hw_interface->hw_receive_frame) {
            if (interface->hw_interface->hw_receive_frame(&frame)) {
                // Check filters
                bool accepted = false;
                
                if (interface->active_filter_count == 0) {
                    // No filters, accept all messages
                    accepted = true;
                } else {
                    // Check each filter
                    for (int i = 0; i < CAN_FILTER_TABLE_SIZE; i++) {
                        if (can_is_filter_match(&interface->filters[i], frame.id, frame.extended_id)) {
                            interface->filters[i].accept_count++;
                            accepted = true;
                            break;
                        }
                    }
                    
                    if (!accepted) {
                        // Update reject counters
                        for (int i = 0; i < CAN_FILTER_TABLE_SIZE; i++) {
                            if (interface->filters[i].enabled) {
                                interface->filters[i].reject_count++;
                            }
                        }
                    }
                }
                
                if (accepted) {
                    // Add timestamp
                    frame.timestamp_ms = can_get_current_time_ms();
                    
                    // Add to receive buffer
                    pthread_mutex_lock(&interface->rx_buffer.mutex);
                    if (can_buffer_push(&interface->rx_buffer, &frame)) {
                        interface->statistics.messages_received++;
                        
                        // Update message timing
                        for (int i = 0; i < interface->monitored_message_count; i++) {
                            // In real implementation, would match by message ID
                            interface->message_timings[i].last_reception_time = frame.timestamp_ms;
                        }
                        
                        // Call message callback if registered
                        if (interface->message_received_callback) {
                            interface->message_received_callback(&frame);
                        }
                    } else {
                        interface->statistics.reception_errors++;
                    }
                    pthread_mutex_unlock(&interface->rx_buffer.mutex);
                }
            }
        }
        
        usleep(100); // 0.1ms
    }
    
    printf("CAN RX thread terminated\n");
    return NULL;
}

static void* can_monitor_thread_function(void* arg) {
    can_bus_interface_t* interface = (can_bus_interface_t*)arg;
    uint32_t last_check_time = can_get_current_time_ms();
    static uint8_t heartbeat_counter = 0;
    
    printf("CAN monitor thread started\n");
    
    while (interface->threads_active) {
        uint32_t current_time = can_get_current_time_ms();
        
        // Check message timeouts every 100ms
        if (current_time - last_check_time >= CAN_TIMING_CHECK_INTERVAL_MS) {
            can_check_message_timeouts(interface);
            last_check_time = current_time;
        }
        
        // Send heartbeat every second
        static uint32_t last_heartbeat_time = 0;
        if (current_time - last_heartbeat_time >= CAN_HEARTBEAT_INTERVAL_MS) {
            can_send_esc_heartbeat(interface, heartbeat_counter++);
            last_heartbeat_time = current_time;
        }
        
        // Get hardware status
        if (interface->hw_interface->hw_get_status) {
            can_bus_status_t hw_status;
            if (interface->hw_interface->hw_get_status(&hw_status)) {
                pthread_mutex_lock(&interface->interface_mutex);
                interface->status.tx_error_count = hw_status.tx_error_count;
                interface->status.rx_error_count = hw_status.rx_error_count;
                interface->status.bus_warning = hw_status.bus_warning;
                interface->status.data_overrun = hw_status.data_overrun;
                
                // Check for state changes based on error counts
                if (hw_status.tx_error_count > 255 || hw_status.rx_error_count > 255) {
                    can_update_bus_state(interface, CAN_STATE_BUS_OFF);
                } else if (hw_status.tx_error_count > 127 || hw_status.rx_error_count > 127) {
                    can_update_bus_state(interface, CAN_STATE_ERROR_PASSIVE);
                } else {
                    can_update_bus_state(interface, CAN_STATE_ERROR_ACTIVE);
                }
                
                pthread_mutex_unlock(&interface->interface_mutex);
            }
        }
        
        usleep(10000); // 10ms
    }
    
    printf("CAN monitor thread terminated\n");
    return NULL;
}