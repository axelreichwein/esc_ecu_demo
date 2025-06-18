#ifndef CAN_BUS_INTERFACE_H
#define CAN_BUS_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

// CAN Bus Communication Standards and Protocols
#define CAN_FRAME_MAX_DATA_LENGTH 8        // Standard CAN frame data length
#define CAN_EXTENDED_FRAME_ID_MASK 0x1FFFFFFF  // 29-bit extended CAN ID
#define CAN_STANDARD_FRAME_ID_MASK 0x7FF   // 11-bit standard CAN ID
#define CAN_BUS_MAX_NODES 128              // Maximum nodes on CAN bus
#define CAN_MESSAGE_BUFFER_SIZE 256        // Circular buffer size for messages
#define CAN_FILTER_TABLE_SIZE 32           // Number of CAN ID filters
#define CAN_ERROR_THRESHOLD 96             // Error counter threshold for error passive
#define CAN_BUS_OFF_THRESHOLD 255          // Error counter threshold for bus off

// CAN Bus Baudrates (bits per second)
typedef enum {
    CAN_BAUDRATE_125K   = 125000,
    CAN_BAUDRATE_250K   = 250000,
    CAN_BAUDRATE_500K   = 500000,
    CAN_BAUDRATE_1M     = 1000000
} can_baudrate_t;

// CAN Frame Types
typedef enum {
    CAN_FRAME_DATA      = 0,    // Data frame
    CAN_FRAME_REMOTE    = 1,    // Remote transmission request
    CAN_FRAME_ERROR     = 2,    // Error frame
    CAN_FRAME_OVERLOAD  = 3     // Overload frame
} can_frame_type_t;

// CAN Bus States (ISO 11898-1)
typedef enum {
    CAN_STATE_ERROR_ACTIVE  = 0,    // Normal operation
    CAN_STATE_ERROR_PASSIVE = 1,    // Error passive state
    CAN_STATE_BUS_OFF       = 2,    // Bus off state
    CAN_STATE_STOPPED       = 3     // Controller stopped
} can_bus_state_t;

// CAN Error Types
typedef enum {
    CAN_ERROR_NONE          = 0x00,
    CAN_ERROR_STUFF         = 0x01,    // Bit stuffing error
    CAN_ERROR_FORM          = 0x02,    // Form error
    CAN_ERROR_ACK           = 0x04,    // Acknowledgment error
    CAN_ERROR_BIT1          = 0x08,    // Bit1 error (recessive bit read as dominant)
    CAN_ERROR_BIT0          = 0x10,    // Bit0 error (dominant bit read as recessive)
    CAN_ERROR_CRC           = 0x20,    // CRC error
    CAN_ERROR_BUS_OFF       = 0x40,    // Bus off condition
    CAN_ERROR_PASSIVE       = 0x80     // Error passive condition
} can_error_type_t;

// Message Priority Levels (lower number = higher priority)
typedef enum {
    CAN_PRIORITY_EMERGENCY  = 0,    // Emergency messages (airbag, brake failure)
    CAN_PRIORITY_CRITICAL   = 1,    // Critical safety messages (ESC intervention)
    CAN_PRIORITY_HIGH       = 2,    // High priority messages (engine control)
    CAN_PRIORITY_NORMAL     = 3,    // Normal operation messages (sensor data)
    CAN_PRIORITY_LOW        = 4,    // Low priority messages (diagnostic data)
    CAN_PRIORITY_BACKGROUND = 5     // Background messages (configuration)
} can_message_priority_t;

// ESC-specific CAN Message IDs (11-bit standard format)
typedef enum {
    // ESC Transmit Messages (0x200-0x2FF range)
    CAN_MSG_ESC_STATUS_1        = 0x201,   // ESC system status and intervention flags
    CAN_MSG_ESC_STATUS_2        = 0x202,   // ESC sensor data (yaw rate, lateral accel)
    CAN_MSG_ESC_WHEEL_SPEEDS    = 0x203,   // Individual wheel speed data
    CAN_MSG_ESC_BRAKE_PRESSURE  = 0x204,   // Brake pressure per wheel
    CAN_MSG_ESC_DIAGNOSTICS     = 0x205,   // Diagnostic and fault information
    CAN_MSG_ESC_HEARTBEAT       = 0x206,   // ESC system heartbeat/alive signal
    CAN_MSG_ESC_VERSION_INFO    = 0x207,   // Software/hardware version information
    
    // ESC Receive Messages (0x100-0x1FF range)
    CAN_MSG_VEHICLE_DYNAMICS    = 0x101,   // Vehicle dynamics from other systems
    CAN_MSG_STEERING_WHEEL      = 0x102,   // Steering wheel angle and torque
    CAN_MSG_BRAKE_PEDAL         = 0x103,   // Brake pedal position and force
    CAN_MSG_ACCELERATOR_PEDAL   = 0x104,   // Accelerator pedal position
    CAN_MSG_VEHICLE_SPEED       = 0x105,   // Vehicle speed from other sources
    CAN_MSG_ENGINE_STATUS       = 0x106,   // Engine RPM, torque, temperature
    CAN_MSG_TRANSMISSION_STATUS = 0x107,   // Transmission gear, mode
    CAN_MSG_ABS_STATUS          = 0x108,   // ABS system status and intervention
    
    // Diagnostic Messages (0x700-0x7FF range)
    CAN_MSG_DIAGNOSTIC_REQUEST  = 0x7DF,   // UDS diagnostic request (broadcast)
    CAN_MSG_DIAGNOSTIC_RESPONSE = 0x7E8,   // UDS diagnostic response from ESC
    CAN_MSG_DTC_INFORMATION     = 0x7E9,   // DTC status information
    
    // Network Management Messages (0x000-0x07F range)
    CAN_MSG_NM_ESC              = 0x020,   // ESC network management
    CAN_MSG_NM_ALIVE            = 0x021    // Network alive signal
} can_message_id_t;

// CAN Frame Structure
typedef struct {
    uint32_t id;                        // CAN identifier (11-bit or 29-bit)
    bool extended_id;                   // True for 29-bit extended identifier
    bool remote_frame;                  // True for remote transmission request
    uint8_t data_length;                // Data length (0-8 bytes)
    uint8_t data[CAN_FRAME_MAX_DATA_LENGTH]; // Message data
    uint32_t timestamp_ms;              // Message timestamp
    can_message_priority_t priority;    // Message priority level
} can_frame_t;

// CAN Message Filter
typedef struct {
    uint32_t id;                        // CAN ID to match
    uint32_t mask;                      // Mask for ID matching
    bool extended_id;                   // Filter for extended IDs
    bool enabled;                       // Filter enable flag
    uint32_t accept_count;              // Number of accepted messages
    uint32_t reject_count;              // Number of rejected messages
} can_filter_t;

// CAN Message Buffer (Circular Buffer)
typedef struct {
    can_frame_t messages[CAN_MESSAGE_BUFFER_SIZE];
    uint16_t head;                      // Write index
    uint16_t tail;                      // Read index
    uint16_t count;                     // Number of messages in buffer
    pthread_mutex_t mutex;              // Thread safety
    pthread_cond_t not_empty;           // Condition for buffer not empty
    pthread_cond_t not_full;            // Condition for buffer not full
    uint32_t overflow_count;            // Buffer overflow counter
} can_message_buffer_t;

// CAN Bus Statistics
typedef struct {
    uint32_t messages_transmitted;      // Total messages transmitted
    uint32_t messages_received;         // Total messages received
    uint32_t transmission_errors;       // Transmission error count
    uint32_t reception_errors;          // Reception error count
    uint32_t bus_off_events;            // Number of bus off events
    uint32_t error_passive_events;      // Number of error passive events
    uint32_t arbitration_lost_count;    // Arbitration lost events
    uint32_t buffer_overflow_count;     // Buffer overflow events
    uint32_t crc_errors;                // CRC error count
    uint32_t form_errors;               // Form error count
    uint32_t ack_errors;                // Acknowledgment error count
    uint32_t stuff_errors;              // Bit stuffing error count
    uint32_t bit_errors;                // Bit error count
} can_bus_statistics_t;

// CAN Bus Configuration
typedef struct {
    can_baudrate_t baudrate;            // Bus baudrate
    bool loopback_mode;                 // Loopback for testing
    bool silent_mode;                   // Silent mode (listen only)
    bool auto_retransmission;           // Automatic retransmission on error
    uint8_t tx_error_threshold;         // Transmit error threshold
    uint8_t rx_error_threshold;         // Receive error threshold
    uint32_t timeout_ms;                // Message timeout in milliseconds
    bool timestamp_enabled;             // Enable hardware timestamps
} can_bus_config_t;

// CAN Bus Status
typedef struct {
    can_bus_state_t state;              // Current bus state
    uint8_t tx_error_count;             // Transmit error counter
    uint8_t rx_error_count;             // Receive error counter
    uint32_t last_error_timestamp;      // Timestamp of last error
    can_error_type_t last_error_type;   // Type of last error
    bool bus_warning;                   // Bus warning flag
    bool data_overrun;                  // Data overrun flag
    uint32_t uptime_ms;                 // Bus uptime in milliseconds
} can_bus_status_t;

// Message Timing Configuration
typedef struct {
    uint32_t transmission_period_ms;    // Periodic transmission interval
    uint32_t timeout_ms;                // Message reception timeout
    uint32_t max_age_ms;                // Maximum message age before considered stale
    bool cyclic_transmission;           // Enable cyclic transmission
    uint32_t next_transmission_time;    // Next scheduled transmission time
    uint32_t last_transmission_time;    // Last successful transmission time
    uint32_t last_reception_time;       // Last successful reception time
} can_message_timing_t;

// Message Priority Queue Entry
typedef struct {
    can_frame_t frame;                  // CAN frame data
    can_message_timing_t timing;        // Timing information
    uint32_t retry_count;               // Number of transmission retries
    bool pending_transmission;          // Waiting for transmission
} can_priority_queue_entry_t;

// CAN Bus Interface Hardware Abstraction
typedef struct {
    bool (*hw_init)(can_baudrate_t baudrate);
    bool (*hw_deinit)(void);
    bool (*hw_send_frame)(const can_frame_t* frame);
    bool (*hw_receive_frame)(can_frame_t* frame);
    bool (*hw_set_filter)(uint32_t filter_id, uint32_t id, uint32_t mask);
    bool (*hw_clear_filter)(uint32_t filter_id);
    bool (*hw_get_status)(can_bus_status_t* status);
    bool (*hw_reset_controller)(void);
    uint32_t (*hw_get_timestamp)(void);
    bool (*hw_enter_sleep_mode)(void);
    bool (*hw_exit_sleep_mode)(void);
} can_hw_interface_t;

// Main CAN Bus Interface Structure
typedef struct {
    // Configuration and Status
    can_bus_config_t config;
    can_bus_status_t status;
    can_bus_statistics_t statistics;
    
    // Hardware Interface
    const can_hw_interface_t* hw_interface;
    
    // Message Buffers
    can_message_buffer_t tx_buffer;     // Transmission buffer
    can_message_buffer_t rx_buffer;     // Reception buffer
    
    // Message Filters
    can_filter_t filters[CAN_FILTER_TABLE_SIZE];
    uint8_t active_filter_count;
    
    // Priority Queue for Transmission
    can_priority_queue_entry_t priority_queue[CAN_MESSAGE_BUFFER_SIZE];
    uint16_t priority_queue_size;
    pthread_mutex_t priority_queue_mutex;
    
    // Message Timing Supervision
    can_message_timing_t message_timings[32];  // Timing for monitored messages
    uint8_t monitored_message_count;
    
    // Threading and Synchronization
    pthread_t tx_thread;                // Transmission thread
    pthread_t rx_thread;                // Reception thread
    pthread_t monitor_thread;           // Timing supervision thread
    bool threads_active;                // Thread control flag
    pthread_mutex_t interface_mutex;    // Interface protection
    
    // Callback Functions
    void (*message_received_callback)(const can_frame_t* frame);
    void (*error_callback)(can_error_type_t error, const char* description);
    void (*bus_state_callback)(can_bus_state_t old_state, can_bus_state_t new_state);
    
    // System Integration
    bool system_initialized;
    uint32_t initialization_time_ms;
} can_bus_interface_t;

// ESC-specific Message Structures
typedef struct {
    uint8_t system_status;              // ESC system status (0=off, 1=standby, 2=active, 3=fault)
    uint8_t intervention_flags;         // Bit flags for active interventions
    uint8_t warning_lights;             // Warning light status
    uint8_t traction_control_status;    // Traction control status
    uint8_t stability_control_status;   // Stability control status
    uint8_t brake_assist_status;        // Brake assist status
    uint8_t fault_code_high;            // DTC fault code (high byte)
    uint8_t fault_code_low;             // DTC fault code (low byte)
} esc_status_1_message_t;

typedef struct {
    int16_t yaw_rate;                   // Yaw rate (0.1 deg/s, signed)
    int16_t lateral_acceleration;       // Lateral acceleration (0.01 m/s², signed)
    int16_t steering_angle;             // Steering angle (0.1 deg, signed)
    uint8_t sensor_status;              // Sensor validity flags
    uint8_t reserved;                   // Reserved byte
} esc_status_2_message_t;

typedef struct {
    uint16_t wheel_speed_fl;            // Front left wheel speed (0.1 km/h)
    uint16_t wheel_speed_fr;            // Front right wheel speed (0.1 km/h)
    uint16_t wheel_speed_rl;            // Rear left wheel speed (0.1 km/h)
    uint16_t wheel_speed_rr;            // Rear right wheel speed (0.1 km/h)
} esc_wheel_speeds_message_t;

typedef struct {
    uint16_t brake_pressure_fl;         // Front left brake pressure (0.1 bar)
    uint16_t brake_pressure_fr;         // Front right brake pressure (0.1 bar)
    uint16_t brake_pressure_rl;         // Rear left brake pressure (0.1 bar)
    uint16_t brake_pressure_rr;         // Rear right brake pressure (0.1 bar)
} esc_brake_pressure_message_t;

// Function Prototypes

// Initialization and Configuration
bool can_bus_init(can_bus_interface_t* interface, const can_hw_interface_t* hw_interface);
bool can_bus_configure(can_bus_interface_t* interface, const can_bus_config_t* config);
bool can_bus_shutdown(can_bus_interface_t* interface);
bool can_bus_reset(can_bus_interface_t* interface);

// Message Transmission
bool can_send_message(can_bus_interface_t* interface, const can_frame_t* frame);
bool can_send_message_priority(can_bus_interface_t* interface, const can_frame_t* frame, can_message_priority_t priority);
bool can_send_message_cyclic(can_bus_interface_t* interface, const can_frame_t* frame, uint32_t period_ms);
bool can_cancel_cyclic_message(can_bus_interface_t* interface, uint32_t message_id);

// Message Reception
bool can_receive_message(can_bus_interface_t* interface, can_frame_t* frame, uint32_t timeout_ms);
bool can_receive_message_nonblocking(can_bus_interface_t* interface, can_frame_t* frame);
uint16_t can_get_pending_message_count(const can_bus_interface_t* interface);

// Message Filtering
bool can_add_filter(can_bus_interface_t* interface, uint32_t id, uint32_t mask, bool extended_id);
bool can_remove_filter(can_bus_interface_t* interface, uint32_t id);
bool can_clear_all_filters(can_bus_interface_t* interface);

// Timing Supervision
bool can_add_message_timing(can_bus_interface_t* interface, uint32_t message_id, uint32_t period_ms, uint32_t timeout_ms);
bool can_remove_message_timing(can_bus_interface_t* interface, uint32_t message_id);
bool can_check_message_timeouts(can_bus_interface_t* interface);

// Status and Diagnostics
can_bus_state_t can_get_bus_state(const can_bus_interface_t* interface);
bool can_get_statistics(const can_bus_interface_t* interface, can_bus_statistics_t* stats);
bool can_get_error_status(const can_bus_interface_t* interface, can_bus_status_t* status);
void can_clear_statistics(can_bus_interface_t* interface);

// Callback Registration
bool can_register_message_callback(can_bus_interface_t* interface, void (*callback)(const can_frame_t* frame));
bool can_register_error_callback(can_bus_interface_t* interface, void (*callback)(can_error_type_t error, const char* description));
bool can_register_bus_state_callback(can_bus_interface_t* interface, void (*callback)(can_bus_state_t old_state, can_bus_state_t new_state));

// ESC-specific Message Functions
bool can_send_esc_status_1(can_bus_interface_t* interface, const esc_status_1_message_t* status);
bool can_send_esc_status_2(can_bus_interface_t* interface, const esc_status_2_message_t* status);
bool can_send_esc_wheel_speeds(can_bus_interface_t* interface, const esc_wheel_speeds_message_t* speeds);
bool can_send_esc_brake_pressures(can_bus_interface_t* interface, const esc_brake_pressure_message_t* pressures);
bool can_send_esc_heartbeat(can_bus_interface_t* interface, uint8_t sequence_counter);

// Message Encoding/Decoding Utilities
void can_encode_esc_status_1(const esc_status_1_message_t* status, can_frame_t* frame);
void can_decode_esc_status_1(const can_frame_t* frame, esc_status_1_message_t* status);
void can_encode_esc_status_2(const esc_status_2_message_t* status, can_frame_t* frame);
void can_decode_esc_status_2(const can_frame_t* frame, esc_status_2_message_t* status);

// Utility Functions
const char* can_get_error_string(can_error_type_t error);
const char* can_get_bus_state_string(can_bus_state_t state);
const char* can_get_priority_string(can_message_priority_t priority);
bool can_validate_frame(const can_frame_t* frame);
uint8_t can_calculate_checksum(const uint8_t* data, uint8_t length);

// Network Management
bool can_enter_sleep_mode(can_bus_interface_t* interface);
bool can_exit_sleep_mode(can_bus_interface_t* interface);
bool can_send_network_management_message(can_bus_interface_t* interface, uint8_t command);

// Debug and Testing Functions
void can_print_frame(const can_frame_t* frame);
void can_print_statistics(const can_bus_interface_t* interface);
void can_print_status(const can_bus_interface_t* interface);
bool can_run_loopback_test(can_bus_interface_t* interface);

#endif // CAN_BUS_INTERFACE_H