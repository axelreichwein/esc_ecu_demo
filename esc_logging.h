#ifndef ESC_LOGGING_H
#define ESC_LOGGING_H

#include <stdint.h>
#include <stdbool.h>
#include "esc_types.h"

// Logging system configuration
#define LOG_BUFFER_SIZE 2048                    // Circular buffer size
#define LOG_PERSISTENT_BUFFER_SIZE 8192         // Persistent storage buffer
#define LOG_MAX_MESSAGE_LENGTH 256              // Maximum log message length
#define LOG_MAX_EXPORT_ENTRIES 1000             // Maximum entries for export
#define LOG_FILE_SIGNATURE 0x45534C47           // "ESLG" signature
#define LOG_FILE_VERSION 0x0100                 // Version 1.0

// Log levels
typedef enum {
    LOG_LEVEL_TRACE = 0,    // Detailed tracing information
    LOG_LEVEL_DEBUG,        // Debug information
    LOG_LEVEL_INFO,         // General information
    LOG_LEVEL_WARNING,      // Warning conditions
    LOG_LEVEL_ERROR,        // Error conditions
    LOG_LEVEL_CRITICAL,     // Critical failures
    LOG_LEVEL_INTERVENTION, // ESC interventions
    LOG_LEVEL_ANOMALY       // System anomalies
} log_level_t;

// Log categories for filtering
typedef enum {
    LOG_CATEGORY_GENERAL = 0x0001,
    LOG_CATEGORY_SENSOR = 0x0002,
    LOG_CATEGORY_CONTROL = 0x0004,
    LOG_CATEGORY_ACTUATOR = 0x0008,
    LOG_CATEGORY_COMMUNICATION = 0x0010,
    LOG_CATEGORY_DIAGNOSTIC = 0x0020,
    LOG_CATEGORY_SAFETY = 0x0040,
    LOG_CATEGORY_INTERVENTION = 0x0080,
    LOG_CATEGORY_ANOMALY = 0x0100,
    LOG_CATEGORY_PERFORMANCE = 0x0200,
    LOG_CATEGORY_CALIBRATION = 0x0400,
    LOG_CATEGORY_SYSTEM = 0x0800,
    LOG_CATEGORY_ALL = 0xFFFF
} log_category_t;

// ESC intervention types
typedef enum {
    ESC_INTERVENTION_NONE = 0,
    ESC_INTERVENTION_UNDERSTEER_CORRECTION,
    ESC_INTERVENTION_OVERSTEER_CORRECTION,
    ESC_INTERVENTION_ROLLOVER_PREVENTION,
    ESC_INTERVENTION_BRAKE_ASSIST,
    ESC_INTERVENTION_ENGINE_TORQUE_REDUCTION,
    ESC_INTERVENTION_EMERGENCY_BRAKE,
    ESC_INTERVENTION_SAFE_STATE_ACTIVATION,
    ESC_INTERVENTION_SYSTEM_SHUTDOWN
} esc_intervention_type_t;

// System anomaly types
typedef enum {
    ANOMALY_NONE = 0,
    ANOMALY_SENSOR_DRIFT,
    ANOMALY_SENSOR_NOISE,
    ANOMALY_ACTUATOR_DELAY,
    ANOMALY_COMMUNICATION_TIMEOUT,
    ANOMALY_PERFORMANCE_DEGRADATION,
    ANOMALY_UNEXPECTED_BEHAVIOR,
    ANOMALY_CALIBRATION_MISMATCH,
    ANOMALY_TIMING_VIOLATION,
    ANOMALY_RESOURCE_EXHAUSTION,
    ANOMALY_DATA_CORRUPTION
} system_anomaly_type_t;

// Storage types
typedef enum {
    STORAGE_TYPE_RAM = 0,       // Volatile RAM storage
    STORAGE_TYPE_EEPROM,        // Non-volatile EEPROM
    STORAGE_TYPE_FLASH,         // Non-volatile Flash memory
    STORAGE_TYPE_EXTERNAL       // External storage (SD card, etc.)
} storage_type_t;

// Log entry structure
typedef struct {
    uint32_t timestamp_ms;              // System timestamp in milliseconds
    uint32_t sequence_number;           // Sequential entry number
    log_level_t level;                  // Log level
    log_category_t category;            // Log category
    uint8_t module_id;                  // Module that generated the log
    uint32_t event_code;                // Event-specific code
    char message[LOG_MAX_MESSAGE_LENGTH]; // Log message
    uint32_t data_length;               // Length of additional data
    uint8_t data[64];                   // Additional binary data
    uint32_t checksum;                  // Entry integrity checksum
} log_entry_t;

// ESC intervention log entry
typedef struct {
    uint32_t timestamp_ms;              // Intervention timestamp
    esc_intervention_type_t type;       // Type of intervention
    uint32_t duration_ms;               // Duration of intervention
    float vehicle_speed;                // Vehicle speed at intervention
    float yaw_rate;                     // Yaw rate at intervention
    float lateral_acceleration;         // Lateral acceleration
    float steering_angle;               // Steering angle
    float brake_pressures[4];           // Brake pressures applied
    float engine_torque_reduction;      // Engine torque reduction %
    uint32_t trigger_conditions;       // Bitmask of trigger conditions
    uint32_t sequence_id;               // Unique sequence identifier
} esc_intervention_log_t;

// System anomaly log entry
typedef struct {
    uint32_t timestamp_ms;              // Anomaly detection timestamp
    system_anomaly_type_t type;         // Type of anomaly
    uint8_t affected_module;            // Module affected by anomaly
    uint32_t severity_level;            // Severity (0=low, 255=critical)
    float measured_value;               // Measured value causing anomaly
    float expected_value;               // Expected value
    float deviation_percent;            // Percentage deviation
    uint32_t occurrence_count;          // Number of occurrences
    uint32_t duration_ms;               // Duration of anomaly
    char description[128];              // Anomaly description
} system_anomaly_log_t;

// Circular buffer for real-time logging
typedef struct {
    log_entry_t* entries;               // Buffer array
    uint32_t size;                      // Buffer size
    uint32_t head;                      // Write pointer
    uint32_t tail;                      // Read pointer
    uint32_t count;                     // Current entry count
    bool full;                          // Buffer full flag
    uint32_t total_entries;             // Total entries written
    uint32_t dropped_entries;           // Entries dropped due to overflow
} circular_log_buffer_t;

// Persistent storage interface
typedef struct {
    bool (*init)(void);
    bool (*write)(uint32_t address, const void* data, uint32_t size);
    bool (*read)(uint32_t address, void* data, uint32_t size);
    bool (*erase)(uint32_t address, uint32_t size);
    uint32_t (*get_size)(void);
    bool (*sync)(void);
} storage_interface_t;

// Data export format
typedef struct {
    uint32_t signature;                 // File format signature
    uint16_t version;                   // Format version
    uint16_t entry_count;              // Number of entries
    uint32_t timestamp_start;          // Start timestamp
    uint32_t timestamp_end;            // End timestamp
    uint32_t checksum;                 // File checksum
} export_header_t;

// Logging system configuration
typedef struct {
    log_level_t min_level;             // Minimum log level to capture
    log_category_t enabled_categories; // Enabled log categories
    bool enable_persistent_storage;    // Enable persistent storage
    storage_type_t storage_type;       // Storage type to use
    uint32_t max_file_size;           // Maximum log file size
    uint32_t retention_days;          // Log retention period
    bool enable_compression;          // Enable log compression
    bool enable_encryption;           // Enable log encryption
} logging_config_t;

// Main logging system structure
typedef struct {
    logging_config_t config;                    // System configuration
    circular_log_buffer_t ram_buffer;           // RAM circular buffer
    storage_interface_t* storage;               // Storage interface
    uint32_t sequence_counter;                  // Global sequence counter
    uint32_t intervention_counter;              // Intervention counter
    uint32_t anomaly_counter;                   // Anomaly counter
    bool initialized;                           // Initialization flag
    uint32_t last_flush_time;                  // Last storage flush time
    uint32_t flush_interval_ms;                // Flush interval
    esc_intervention_log_t* intervention_buffer; // Intervention log buffer
    uint32_t intervention_buffer_size;          // Intervention buffer size
    uint32_t intervention_buffer_count;         // Current intervention count
    system_anomaly_log_t* anomaly_buffer;      // Anomaly log buffer
    uint32_t anomaly_buffer_size;              // Anomaly buffer size
    uint32_t anomaly_buffer_count;             // Current anomaly count
    uint32_t total_bytes_logged;               // Total bytes logged
    uint32_t storage_errors;                   // Storage error count
} esc_logging_system_t;

// Function prototypes

// System initialization and configuration
bool logging_init(esc_logging_system_t* logging, const logging_config_t* config);
void logging_shutdown(esc_logging_system_t* logging);
bool logging_configure(esc_logging_system_t* logging, const logging_config_t* config);
bool logging_set_storage_interface(esc_logging_system_t* logging, storage_interface_t* storage);

// Core logging functions
bool logging_write(esc_logging_system_t* logging, log_level_t level, log_category_t category,
                   uint8_t module_id, uint32_t event_code, const char* format, ...);
bool logging_write_binary(esc_logging_system_t* logging, log_level_t level, log_category_t category,
                          uint8_t module_id, uint32_t event_code, const char* message,
                          const void* data, uint32_t data_size);

// ESC intervention logging
bool logging_log_intervention(esc_logging_system_t* logging, const esc_intervention_log_t* intervention);
bool logging_start_intervention(esc_logging_system_t* logging, esc_intervention_type_t type,
                               const void* sensor_data, uint32_t* sequence_id);
bool logging_end_intervention(esc_logging_system_t* logging, uint32_t sequence_id,
                             const void* commands);

// System anomaly logging
bool logging_log_anomaly(esc_logging_system_t* logging, const system_anomaly_log_t* anomaly);
bool logging_detect_and_log_anomaly(esc_logging_system_t* logging, system_anomaly_type_t type,
                                    uint8_t module_id, float measured, float expected,
                                    const char* description);

// Buffer management
uint32_t logging_get_entry_count(const esc_logging_system_t* logging);
bool logging_get_entry(const esc_logging_system_t* logging, uint32_t index, log_entry_t* entry);
bool logging_clear_buffer(esc_logging_system_t* logging);
uint32_t logging_get_buffer_usage(const esc_logging_system_t* logging);

// Persistent storage
bool logging_flush_to_storage(esc_logging_system_t* logging);
bool logging_load_from_storage(esc_logging_system_t* logging);
bool logging_archive_logs(esc_logging_system_t* logging, uint32_t older_than_ms);
bool logging_cleanup_old_logs(esc_logging_system_t* logging);

// Data export and analysis
bool logging_export_to_buffer(const esc_logging_system_t* logging, uint8_t* export_buffer,
                              uint32_t buffer_size, uint32_t* bytes_written,
                              uint32_t start_time, uint32_t end_time);
bool logging_export_interventions(const esc_logging_system_t* logging, uint8_t* export_buffer,
                                  uint32_t buffer_size, uint32_t* bytes_written);
bool logging_export_anomalies(const esc_logging_system_t* logging, uint8_t* export_buffer,
                              uint32_t buffer_size, uint32_t* bytes_written);

// Query and filtering
uint32_t logging_count_entries_by_level(const esc_logging_system_t* logging, log_level_t level);
uint32_t logging_count_entries_by_category(const esc_logging_system_t* logging, log_category_t category);
uint32_t logging_count_interventions_by_type(const esc_logging_system_t* logging, esc_intervention_type_t type);
uint32_t logging_count_anomalies_by_type(const esc_logging_system_t* logging, system_anomaly_type_t type);

// Statistics and reporting
void logging_get_statistics(const esc_logging_system_t* logging, uint32_t* total_entries,
                           uint32_t* interventions, uint32_t* anomalies, uint32_t* storage_errors);
void logging_print_summary(const esc_logging_system_t* logging);
void logging_print_recent_entries(const esc_logging_system_t* logging, uint32_t count);
void logging_print_interventions(const esc_logging_system_t* logging);
void logging_print_anomalies(const esc_logging_system_t* logging);

// Utility functions
const char* log_level_to_string(log_level_t level);
const char* log_category_to_string(log_category_t category);
const char* intervention_type_to_string(esc_intervention_type_t type);
const char* anomaly_type_to_string(system_anomaly_type_t type);
uint32_t logging_calculate_checksum(const void* data, uint32_t size);

// Default configurations
logging_config_t logging_get_default_config(void);
logging_config_t logging_get_minimal_config(void);
logging_config_t logging_get_debug_config(void);

// Storage interface implementations
storage_interface_t* logging_get_eeprom_storage_interface(void);
storage_interface_t* logging_get_flash_storage_interface(void);
storage_interface_t* logging_get_external_storage_interface(void);

// Macro helpers for common logging operations
#define LOG_TRACE(logging, category, module, code, ...) \
    logging_write(logging, LOG_LEVEL_TRACE, category, module, code, __VA_ARGS__)

#define LOG_DEBUG(logging, category, module, code, ...) \
    logging_write(logging, LOG_LEVEL_DEBUG, category, module, code, __VA_ARGS__)

#define LOG_INFO(logging, category, module, code, ...) \
    logging_write(logging, LOG_LEVEL_INFO, category, module, code, __VA_ARGS__)

#define LOG_WARNING(logging, category, module, code, ...) \
    logging_write(logging, LOG_LEVEL_WARNING, category, module, code, __VA_ARGS__)

#define LOG_ERROR(logging, category, module, code, ...) \
    logging_write(logging, LOG_LEVEL_ERROR, category, module, code, __VA_ARGS__)

#define LOG_CRITICAL(logging, category, module, code, ...) \
    logging_write(logging, LOG_LEVEL_CRITICAL, category, module, code, __VA_ARGS__)

#define LOG_INTERVENTION(logging, category, module, code, ...) \
    logging_write(logging, LOG_LEVEL_INTERVENTION, category, module, code, __VA_ARGS__)

#define LOG_ANOMALY(logging, category, module, code, ...) \
    logging_write(logging, LOG_LEVEL_ANOMALY, category, module, code, __VA_ARGS__)

// ESC module IDs for logging
#define ESC_MODULE_MAIN 0x01
#define ESC_MODULE_SENSOR 0x02
#define ESC_MODULE_CONTROL 0x03
#define ESC_MODULE_ACTUATOR 0x04
#define ESC_MODULE_BRAKE 0x05
#define ESC_MODULE_ENGINE 0x06
#define ESC_MODULE_WATCHDOG 0x07
#define ESC_MODULE_DTC 0x08
#define ESC_MODULE_POST 0x09
#define ESC_MODULE_LOGGING 0x0A
#define ESC_MODULE_PLAUSIBILITY 0x0B
#define ESC_MODULE_CAN 0x0C

#endif // ESC_LOGGING_H