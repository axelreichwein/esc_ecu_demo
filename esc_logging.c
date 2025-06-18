#include "esc_logging.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <unistd.h>

// Internal constants
#define FLUSH_INTERVAL_MS 5000          // Default flush interval (5 seconds)
#define MAX_INTERVENTION_BUFFER 100     // Maximum intervention entries
#define MAX_ANOMALY_BUFFER 200          // Maximum anomaly entries
#define CHECKSUM_POLYNOMIAL 0xEDB88320  // CRC-32 polynomial

// Global logging system instance
static esc_logging_system_t* g_logging_system = NULL;

// Internal function prototypes
static uint32_t get_system_time_ms(void);
static uint32_t calculate_crc32(const void* data, uint32_t size);
static bool circular_buffer_init(circular_log_buffer_t* buffer, uint32_t size);
static void circular_buffer_cleanup(circular_log_buffer_t* buffer);
static bool circular_buffer_write(circular_log_buffer_t* buffer, const log_entry_t* entry);
static bool circular_buffer_read(const circular_log_buffer_t* buffer, uint32_t index, log_entry_t* entry);
static void create_log_entry(log_entry_t* entry, log_level_t level, log_category_t category,
                            uint8_t module_id, uint32_t event_code, const char* message,
                            const void* data, uint32_t data_size);

// System initialization and configuration
bool logging_init(esc_logging_system_t* logging, const logging_config_t* config) {
    if (!logging || !config) {
        return false;
    }
    
    // Clear the logging system structure
    memset(logging, 0, sizeof(esc_logging_system_t));
    
    // Copy configuration
    memcpy(&logging->config, config, sizeof(logging_config_t));
    
    // Initialize circular buffer for RAM logging
    if (!circular_buffer_init(&logging->ram_buffer, LOG_BUFFER_SIZE)) {
        printf("Logging: Failed to initialize RAM buffer\n");
        return false;
    }
    
    // Allocate intervention buffer
    logging->intervention_buffer_size = MAX_INTERVENTION_BUFFER;
    logging->intervention_buffer = (esc_intervention_log_t*)malloc(
        sizeof(esc_intervention_log_t) * logging->intervention_buffer_size);
    if (!logging->intervention_buffer) {
        circular_buffer_cleanup(&logging->ram_buffer);
        return false;
    }
    
    // Allocate anomaly buffer
    logging->anomaly_buffer_size = MAX_ANOMALY_BUFFER;
    logging->anomaly_buffer = (system_anomaly_log_t*)malloc(
        sizeof(system_anomaly_log_t) * logging->anomaly_buffer_size);
    if (!logging->anomaly_buffer) {
        free(logging->intervention_buffer);
        circular_buffer_cleanup(&logging->ram_buffer);
        return false;
    }
    
    // Initialize counters and settings
    logging->sequence_counter = 1;
    logging->intervention_counter = 0;
    logging->anomaly_counter = 0;
    logging->flush_interval_ms = FLUSH_INTERVAL_MS;
    logging->last_flush_time = get_system_time_ms();
    logging->initialized = true;
    
    // Set global reference
    g_logging_system = logging;
    
    printf("ESC Logging System initialized successfully\n");
    printf("  Buffer size: %u entries\n", LOG_BUFFER_SIZE);
    printf("  Min level: %s\n", log_level_to_string(config->min_level));
    printf("  Categories: 0x%04X\n", config->enabled_categories);
    printf("  Persistent storage: %s\n", config->enable_persistent_storage ? "enabled" : "disabled");
    
    // Log initialization
    LOG_INFO(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0001,
             "ESC Logging System initialized");
    
    return true;
}

void logging_shutdown(esc_logging_system_t* logging) {
    if (!logging || !logging->initialized) {
        return;
    }
    
    printf("ESC Logging System shutdown initiated\n");
    
    // Log shutdown
    LOG_INFO(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0002,
             "ESC Logging System shutdown");
    
    // Flush any remaining data to storage
    if (logging->config.enable_persistent_storage) {
        logging_flush_to_storage(logging);
    }
    
    // Print final statistics
    printf("Final logging statistics:\n");
    printf("  Total entries: %u\n", logging->ram_buffer.total_entries);
    printf("  Dropped entries: %u\n", logging->ram_buffer.dropped_entries);
    printf("  Interventions logged: %u\n", logging->intervention_counter);
    printf("  Anomalies logged: %u\n", logging->anomaly_counter);
    printf("  Total bytes logged: %u\n", logging->total_bytes_logged);
    printf("  Storage errors: %u\n", logging->storage_errors);
    
    // Cleanup buffers
    circular_buffer_cleanup(&logging->ram_buffer);
    
    if (logging->intervention_buffer) {
        free(logging->intervention_buffer);
        logging->intervention_buffer = NULL;
    }
    
    if (logging->anomaly_buffer) {
        free(logging->anomaly_buffer);
        logging->anomaly_buffer = NULL;
    }
    
    // Clear structure
    memset(logging, 0, sizeof(esc_logging_system_t));
    g_logging_system = NULL;
    
    printf("ESC Logging System shutdown complete\n");
}

bool logging_configure(esc_logging_system_t* logging, const logging_config_t* config) {
    if (!logging || !config || !logging->initialized) {
        return false;
    }
    
    memcpy(&logging->config, config, sizeof(logging_config_t));
    
    LOG_INFO(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0003,
             "Logging configuration updated");
    
    return true;
}

bool logging_set_storage_interface(esc_logging_system_t* logging, storage_interface_t* storage) {
    if (!logging || !logging->initialized) {
        return false;
    }
    
    logging->storage = storage;
    
    if (storage && storage->init) {
        if (!storage->init()) {
            LOG_ERROR(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0004,
                     "Failed to initialize storage interface");
            return false;
        }
    }
    
    LOG_INFO(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0005,
             "Storage interface configured");
    
    return true;
}

// Core logging functions
bool logging_write(esc_logging_system_t* logging, log_level_t level, log_category_t category,
                   uint8_t module_id, uint32_t event_code, const char* format, ...) {
    if (!logging || !logging->initialized || !format) {
        return false;
    }
    
    // Check level and category filters
    if (level < logging->config.min_level || 
        !(category & logging->config.enabled_categories)) {
        return true; // Filtered out, but not an error
    }
    
    // Format message
    char message[LOG_MAX_MESSAGE_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    
    // Create log entry
    log_entry_t entry;
    create_log_entry(&entry, level, category, module_id, event_code, message, NULL, 0);
    
    // Write to circular buffer
    if (!circular_buffer_write(&logging->ram_buffer, &entry)) {
        logging->ram_buffer.dropped_entries++;
        return false;
    }
    
    logging->total_bytes_logged += sizeof(log_entry_t);
    
    // Check if flush is needed
    uint32_t current_time = get_system_time_ms();
    if (logging->config.enable_persistent_storage && 
        (current_time - logging->last_flush_time) >= logging->flush_interval_ms) {
        logging_flush_to_storage(logging);
        logging->last_flush_time = current_time;
    }
    
    return true;
}

bool logging_write_binary(esc_logging_system_t* logging, log_level_t level, log_category_t category,
                          uint8_t module_id, uint32_t event_code, const char* message,
                          const void* data, uint32_t data_size) {
    if (!logging || !logging->initialized || !message) {
        return false;
    }
    
    // Check level and category filters
    if (level < logging->config.min_level || 
        !(category & logging->config.enabled_categories)) {
        return true; // Filtered out, but not an error
    }
    
    // Limit data size
    uint32_t limited_size = (data_size > 64) ? 64 : data_size;
    
    // Create log entry
    log_entry_t entry;
    create_log_entry(&entry, level, category, module_id, event_code, message, data, limited_size);
    
    // Write to circular buffer
    if (!circular_buffer_write(&logging->ram_buffer, &entry)) {
        logging->ram_buffer.dropped_entries++;
        return false;
    }
    
    logging->total_bytes_logged += sizeof(log_entry_t);
    
    return true;
}

// ESC intervention logging
bool logging_log_intervention(esc_logging_system_t* logging, const esc_intervention_log_t* intervention) {
    if (!logging || !logging->initialized || !intervention) {
        return false;
    }
    
    // Add to intervention buffer if space available
    if (logging->intervention_buffer_count < logging->intervention_buffer_size) {
        memcpy(&logging->intervention_buffer[logging->intervention_buffer_count], 
               intervention, sizeof(esc_intervention_log_t));
        logging->intervention_buffer_count++;
    }
    
    logging->intervention_counter++;
    
    // Log intervention as regular log entry
    LOG_INTERVENTION(logging, LOG_CATEGORY_INTERVENTION, ESC_MODULE_CONTROL, 0x1000,
                     "ESC Intervention: %s at %.1f km/h, duration %u ms",
                     intervention_type_to_string(intervention->type),
                     intervention->vehicle_speed * 3.6f,
                     intervention->duration_ms);
    
    return true;
}

bool logging_start_intervention(esc_logging_system_t* logging, esc_intervention_type_t type,
                               const void* sensor_data, uint32_t* sequence_id) {
    if (!logging || !logging->initialized || !sensor_data || !sequence_id) {
        return false;
    }
    
    *sequence_id = logging->sequence_counter++;
    
    LOG_INTERVENTION(logging, LOG_CATEGORY_INTERVENTION, ESC_MODULE_CONTROL, 0x1001,
                     "ESC Intervention START: %s (ID=%u)",
                     intervention_type_to_string(type), *sequence_id);
    
    return true;
}

bool logging_end_intervention(esc_logging_system_t* logging, uint32_t sequence_id,
                             const void* commands) {
    if (!logging || !logging->initialized || !commands) {
        return false;
    }
    
    LOG_INTERVENTION(logging, LOG_CATEGORY_INTERVENTION, ESC_MODULE_CONTROL, 0x1002,
                     "ESC Intervention END: (ID=%u)", sequence_id);
    
    return true;
}

// System anomaly logging
bool logging_log_anomaly(esc_logging_system_t* logging, const system_anomaly_log_t* anomaly) {
    if (!logging || !logging->initialized || !anomaly) {
        return false;
    }
    
    // Add to anomaly buffer if space available
    if (logging->anomaly_buffer_count < logging->anomaly_buffer_size) {
        memcpy(&logging->anomaly_buffer[logging->anomaly_buffer_count], 
               anomaly, sizeof(system_anomaly_log_t));
        logging->anomaly_buffer_count++;
    }
    
    logging->anomaly_counter++;
    
    // Log anomaly as regular log entry
    LOG_ANOMALY(logging, LOG_CATEGORY_ANOMALY, anomaly->affected_module, 0x2000,
                "System Anomaly: %s - %s (%.2f%% deviation)",
                anomaly_type_to_string(anomaly->type),
                anomaly->description,
                anomaly->deviation_percent);
    
    return true;
}

bool logging_detect_and_log_anomaly(esc_logging_system_t* logging, system_anomaly_type_t type,
                                    uint8_t module_id, float measured, float expected,
                                    const char* description) {
    if (!logging || !logging->initialized || !description) {
        return false;
    }
    
    // Calculate deviation
    float deviation = 0.0f;
    if (expected != 0.0f) {
        deviation = ((measured - expected) / expected) * 100.0f;
    }
    
    // Create anomaly log entry
    system_anomaly_log_t anomaly = {0};
    anomaly.timestamp_ms = get_system_time_ms();
    anomaly.type = type;
    anomaly.affected_module = module_id;
    anomaly.measured_value = measured;
    anomaly.expected_value = expected;
    anomaly.deviation_percent = deviation;
    anomaly.occurrence_count = 1;
    anomaly.duration_ms = 0;
    strncpy(anomaly.description, description, sizeof(anomaly.description) - 1);
    
    // Determine severity based on deviation
    if (deviation > 50.0f) {
        anomaly.severity_level = 255; // Critical
    } else if (deviation > 25.0f) {
        anomaly.severity_level = 192; // High
    } else if (deviation > 10.0f) {
        anomaly.severity_level = 128; // Medium
    } else {
        anomaly.severity_level = 64;  // Low
    }
    
    return logging_log_anomaly(logging, &anomaly);
}

// Buffer management
uint32_t logging_get_entry_count(const esc_logging_system_t* logging) {
    if (!logging || !logging->initialized) {
        return 0;
    }
    
    return logging->ram_buffer.count;
}

bool logging_get_entry(const esc_logging_system_t* logging, uint32_t index, log_entry_t* entry) {
    if (!logging || !logging->initialized || !entry) {
        return false;
    }
    
    return circular_buffer_read(&logging->ram_buffer, index, entry);
}

bool logging_clear_buffer(esc_logging_system_t* logging) {
    if (!logging || !logging->initialized) {
        return false;
    }
    
    logging->ram_buffer.head = 0;
    logging->ram_buffer.tail = 0;
    logging->ram_buffer.count = 0;
    logging->ram_buffer.full = false;
    
    LOG_INFO(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0006,
             "Log buffer cleared");
    
    return true;
}

uint32_t logging_get_buffer_usage(const esc_logging_system_t* logging) {
    if (!logging || !logging->initialized) {
        return 0;
    }
    
    return (logging->ram_buffer.count * 100) / logging->ram_buffer.size;
}

// Persistent storage implementation
bool logging_flush_to_storage(esc_logging_system_t* logging) {
    if (!logging || !logging->initialized || !logging->storage) {
        return false;
    }
    
    // For demonstration, we'll implement a simple storage format
    // In a real implementation, this would write to actual persistent storage
    
    uint32_t entries_to_flush = logging->ram_buffer.count;
    if (entries_to_flush == 0) {
        return true; // Nothing to flush
    }
    
    LOG_DEBUG(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0007,
              "Flushing %u entries to persistent storage", entries_to_flush);
    
    // Simulate storage write
    if (logging->storage->write) {
        // In a real implementation, would write the actual data
        // Here we just simulate the operation
        bool success = true; // logging->storage->write(address, data, size);
        
        if (success) {
            LOG_DEBUG(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0008,
                     "Successfully flushed %u entries to storage", entries_to_flush);
        } else {
            logging->storage_errors++;
            LOG_ERROR(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x0009,
                     "Failed to flush entries to storage");
            return false;
        }
    }
    
    return true;
}

bool logging_load_from_storage(esc_logging_system_t* logging) {
    if (!logging || !logging->initialized || !logging->storage) {
        return false;
    }
    
    // For demonstration purposes
    LOG_INFO(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_LOGGING, 0x000A,
             "Loading log data from persistent storage");
    
    return true;
}

// Data export functionality
bool logging_export_to_buffer(const esc_logging_system_t* logging, uint8_t* export_buffer,
                              uint32_t buffer_size, uint32_t* bytes_written,
                              uint32_t start_time, uint32_t end_time) {
    if (!logging || !logging->initialized || !export_buffer || !bytes_written) {
        return false;
    }
    
    *bytes_written = 0;
    
    // Create export header
    export_header_t header = {0};
    header.signature = LOG_FILE_SIGNATURE;
    header.version = LOG_FILE_VERSION;
    header.timestamp_start = start_time;
    header.timestamp_end = end_time;
    header.entry_count = 0;
    
    if (buffer_size < sizeof(header)) {
        return false;
    }
    
    // Copy header to buffer
    memcpy(export_buffer, &header, sizeof(header));
    *bytes_written = sizeof(header);
    
    // Export log entries within time range
    uint32_t exported_count = 0;
    for (uint32_t i = 0; i < logging->ram_buffer.count && 
         *bytes_written + sizeof(log_entry_t) <= buffer_size; i++) {
        
        log_entry_t entry;
        if (circular_buffer_read(&logging->ram_buffer, i, &entry)) {
            if (entry.timestamp_ms >= start_time && entry.timestamp_ms <= end_time) {
                memcpy(export_buffer + *bytes_written, &entry, sizeof(entry));
                *bytes_written += sizeof(entry);
                exported_count++;
            }
        }
    }
    
    // Update header with actual count
    ((export_header_t*)export_buffer)->entry_count = exported_count;
    ((export_header_t*)export_buffer)->checksum = 
        calculate_crc32(export_buffer + sizeof(uint32_t), *bytes_written - sizeof(uint32_t));
    
    return true;
}

bool logging_export_interventions(const esc_logging_system_t* logging, uint8_t* export_buffer,
                                  uint32_t buffer_size, uint32_t* bytes_written) {
    if (!logging || !logging->initialized || !export_buffer || !bytes_written) {
        return false;
    }
    
    *bytes_written = 0;
    
    uint32_t max_entries = buffer_size / sizeof(esc_intervention_log_t);
    uint32_t entries_to_export = (logging->intervention_buffer_count < max_entries) ? 
                                 logging->intervention_buffer_count : max_entries;
    
    if (entries_to_export > 0) {
        memcpy(export_buffer, logging->intervention_buffer, 
               entries_to_export * sizeof(esc_intervention_log_t));
        *bytes_written = entries_to_export * sizeof(esc_intervention_log_t);
    }
    
    return true;
}

bool logging_export_anomalies(const esc_logging_system_t* logging, uint8_t* export_buffer,
                              uint32_t buffer_size, uint32_t* bytes_written) {
    if (!logging || !logging->initialized || !export_buffer || !bytes_written) {
        return false;
    }
    
    *bytes_written = 0;
    
    uint32_t max_entries = buffer_size / sizeof(system_anomaly_log_t);
    uint32_t entries_to_export = (logging->anomaly_buffer_count < max_entries) ? 
                                 logging->anomaly_buffer_count : max_entries;
    
    if (entries_to_export > 0) {
        memcpy(export_buffer, logging->anomaly_buffer, 
               entries_to_export * sizeof(system_anomaly_log_t));
        *bytes_written = entries_to_export * sizeof(system_anomaly_log_t);
    }
    
    return true;
}

// Query and filtering functions
uint32_t logging_count_entries_by_level(const esc_logging_system_t* logging, log_level_t level) {
    if (!logging || !logging->initialized) {
        return 0;
    }
    
    uint32_t count = 0;
    for (uint32_t i = 0; i < logging->ram_buffer.count; i++) {
        log_entry_t entry;
        if (circular_buffer_read(&logging->ram_buffer, i, &entry)) {
            if (entry.level == level) {
                count++;
            }
        }
    }
    
    return count;
}

uint32_t logging_count_entries_by_category(const esc_logging_system_t* logging, log_category_t category) {
    if (!logging || !logging->initialized) {
        return 0;
    }
    
    uint32_t count = 0;
    for (uint32_t i = 0; i < logging->ram_buffer.count; i++) {
        log_entry_t entry;
        if (circular_buffer_read(&logging->ram_buffer, i, &entry)) {
            if (entry.category & category) {
                count++;
            }
        }
    }
    
    return count;
}

uint32_t logging_count_interventions_by_type(const esc_logging_system_t* logging, esc_intervention_type_t type) {
    if (!logging || !logging->initialized) {
        return 0;
    }
    
    uint32_t count = 0;
    for (uint32_t i = 0; i < logging->intervention_buffer_count; i++) {
        if (logging->intervention_buffer[i].type == type) {
            count++;
        }
    }
    
    return count;
}

uint32_t logging_count_anomalies_by_type(const esc_logging_system_t* logging, system_anomaly_type_t type) {
    if (!logging || !logging->initialized) {
        return 0;
    }
    
    uint32_t count = 0;
    for (uint32_t i = 0; i < logging->anomaly_buffer_count; i++) {
        if (logging->anomaly_buffer[i].type == type) {
            count++;
        }
    }
    
    return count;
}

// Statistics and reporting
void logging_get_statistics(const esc_logging_system_t* logging, uint32_t* total_entries,
                           uint32_t* interventions, uint32_t* anomalies, uint32_t* storage_errors) {
    if (!logging || !logging->initialized) {
        if (total_entries) *total_entries = 0;
        if (interventions) *interventions = 0;
        if (anomalies) *anomalies = 0;
        if (storage_errors) *storage_errors = 0;
        return;
    }
    
    if (total_entries) *total_entries = logging->ram_buffer.total_entries;
    if (interventions) *interventions = logging->intervention_counter;
    if (anomalies) *anomalies = logging->anomaly_counter;
    if (storage_errors) *storage_errors = logging->storage_errors;
}

void logging_print_summary(const esc_logging_system_t* logging) {
    if (!logging || !logging->initialized) {
        printf("Logging system not initialized\n");
        return;
    }
    
    printf("\n=== ESC Logging System Summary ===\n");
    printf("Total entries logged: %u\n", logging->ram_buffer.total_entries);
    printf("Current buffer entries: %u\n", logging->ram_buffer.count);
    printf("Dropped entries: %u\n", logging->ram_buffer.dropped_entries);
    printf("Buffer usage: %u%%\n", logging_get_buffer_usage(logging));
    printf("ESC interventions: %u\n", logging->intervention_counter);
    printf("System anomalies: %u\n", logging->anomaly_counter);
    printf("Total bytes logged: %u\n", logging->total_bytes_logged);
    printf("Storage errors: %u\n", logging->storage_errors);
    
    // Count by level
    printf("\nEntries by level:\n");
    for (int level = LOG_LEVEL_TRACE; level <= LOG_LEVEL_ANOMALY; level++) {
        uint32_t count = logging_count_entries_by_level(logging, (log_level_t)level);
        if (count > 0) {
            printf("  %s: %u\n", log_level_to_string((log_level_t)level), count);
        }
    }
    
    printf("===================================\n");
}

void logging_print_recent_entries(const esc_logging_system_t* logging, uint32_t count) {
    if (!logging || !logging->initialized) {
        printf("Logging system not initialized\n");
        return;
    }
    
    printf("\n=== Recent Log Entries ===\n");
    
    uint32_t entries_to_show = (count > logging->ram_buffer.count) ? 
                               logging->ram_buffer.count : count;
    
    for (uint32_t i = 0; i < entries_to_show; i++) {
        log_entry_t entry;
        if (circular_buffer_read(&logging->ram_buffer, i, &entry)) {
            printf("[%u] %s %s: %s\n",
                   entry.timestamp_ms,
                   log_level_to_string(entry.level),
                   log_category_to_string(entry.category),
                   entry.message);
        }
    }
    
    printf("==========================\n");
}

void logging_print_interventions(const esc_logging_system_t* logging) {
    if (!logging || !logging->initialized) {
        printf("Logging system not initialized\n");
        return;
    }
    
    printf("\n=== ESC Interventions ===\n");
    printf("Total interventions: %u\n", logging->intervention_counter);
    
    for (uint32_t i = 0; i < logging->intervention_buffer_count; i++) {
        const esc_intervention_log_t* intervention = &logging->intervention_buffer[i];
        printf("Intervention %u:\n", i + 1);
        printf("  Type: %s\n", intervention_type_to_string(intervention->type));
        printf("  Timestamp: %u ms\n", intervention->timestamp_ms);
        printf("  Duration: %u ms\n", intervention->duration_ms);
        printf("  Vehicle Speed: %.1f km/h\n", intervention->vehicle_speed * 3.6f);
        printf("  Yaw Rate: %.2f deg/s\n", intervention->yaw_rate * 180.0f / 3.14159f);
        printf("  Engine Torque Reduction: %.1f%%\n", intervention->engine_torque_reduction);
        printf("\n");
    }
    
    printf("=========================\n");
}

void logging_print_anomalies(const esc_logging_system_t* logging) {
    if (!logging || !logging->initialized) {
        printf("Logging system not initialized\n");
        return;
    }
    
    printf("\n=== System Anomalies ===\n");
    printf("Total anomalies: %u\n", logging->anomaly_counter);
    
    for (uint32_t i = 0; i < logging->anomaly_buffer_count; i++) {
        const system_anomaly_log_t* anomaly = &logging->anomaly_buffer[i];
        printf("Anomaly %u:\n", i + 1);
        printf("  Type: %s\n", anomaly_type_to_string(anomaly->type));
        printf("  Timestamp: %u ms\n", anomaly->timestamp_ms);
        printf("  Affected Module: 0x%02X\n", anomaly->affected_module);
        printf("  Severity: %u/255\n", anomaly->severity_level);
        printf("  Measured: %.2f\n", anomaly->measured_value);
        printf("  Expected: %.2f\n", anomaly->expected_value);
        printf("  Deviation: %.1f%%\n", anomaly->deviation_percent);
        printf("  Description: %s\n", anomaly->description);
        printf("\n");
    }
    
    printf("========================\n");
}

// Utility functions
const char* log_level_to_string(log_level_t level) {
    switch (level) {
        case LOG_LEVEL_TRACE: return "TRACE";
        case LOG_LEVEL_DEBUG: return "DEBUG";
        case LOG_LEVEL_INFO: return "INFO";
        case LOG_LEVEL_WARNING: return "WARNING";
        case LOG_LEVEL_ERROR: return "ERROR";
        case LOG_LEVEL_CRITICAL: return "CRITICAL";
        case LOG_LEVEL_INTERVENTION: return "INTERVENTION";
        case LOG_LEVEL_ANOMALY: return "ANOMALY";
        default: return "UNKNOWN";
    }
}

const char* log_category_to_string(log_category_t category) {
    // Return the first matching category (multiple categories possible)
    if (category & LOG_CATEGORY_INTERVENTION) return "INTERVENTION";
    if (category & LOG_CATEGORY_ANOMALY) return "ANOMALY";
    if (category & LOG_CATEGORY_SAFETY) return "SAFETY";
    if (category & LOG_CATEGORY_CONTROL) return "CONTROL";
    if (category & LOG_CATEGORY_SENSOR) return "SENSOR";
    if (category & LOG_CATEGORY_ACTUATOR) return "ACTUATOR";
    if (category & LOG_CATEGORY_COMMUNICATION) return "COMMUNICATION";
    if (category & LOG_CATEGORY_DIAGNOSTIC) return "DIAGNOSTIC";
    if (category & LOG_CATEGORY_PERFORMANCE) return "PERFORMANCE";
    if (category & LOG_CATEGORY_CALIBRATION) return "CALIBRATION";
    if (category & LOG_CATEGORY_SYSTEM) return "SYSTEM";
    if (category & LOG_CATEGORY_GENERAL) return "GENERAL";
    return "UNKNOWN";
}

const char* intervention_type_to_string(esc_intervention_type_t type) {
    switch (type) {
        case ESC_INTERVENTION_NONE: return "NONE";
        case ESC_INTERVENTION_UNDERSTEER_CORRECTION: return "UNDERSTEER_CORRECTION";
        case ESC_INTERVENTION_OVERSTEER_CORRECTION: return "OVERSTEER_CORRECTION";
        case ESC_INTERVENTION_ROLLOVER_PREVENTION: return "ROLLOVER_PREVENTION";
        case ESC_INTERVENTION_BRAKE_ASSIST: return "BRAKE_ASSIST";
        case ESC_INTERVENTION_ENGINE_TORQUE_REDUCTION: return "ENGINE_TORQUE_REDUCTION";
        case ESC_INTERVENTION_EMERGENCY_BRAKE: return "EMERGENCY_BRAKE";
        case ESC_INTERVENTION_SAFE_STATE_ACTIVATION: return "SAFE_STATE_ACTIVATION";
        case ESC_INTERVENTION_SYSTEM_SHUTDOWN: return "SYSTEM_SHUTDOWN";
        default: return "UNKNOWN";
    }
}

const char* anomaly_type_to_string(system_anomaly_type_t type) {
    switch (type) {
        case ANOMALY_NONE: return "NONE";
        case ANOMALY_SENSOR_DRIFT: return "SENSOR_DRIFT";
        case ANOMALY_SENSOR_NOISE: return "SENSOR_NOISE";
        case ANOMALY_ACTUATOR_DELAY: return "ACTUATOR_DELAY";
        case ANOMALY_COMMUNICATION_TIMEOUT: return "COMMUNICATION_TIMEOUT";
        case ANOMALY_PERFORMANCE_DEGRADATION: return "PERFORMANCE_DEGRADATION";
        case ANOMALY_UNEXPECTED_BEHAVIOR: return "UNEXPECTED_BEHAVIOR";
        case ANOMALY_CALIBRATION_MISMATCH: return "CALIBRATION_MISMATCH";
        case ANOMALY_TIMING_VIOLATION: return "TIMING_VIOLATION";
        case ANOMALY_RESOURCE_EXHAUSTION: return "RESOURCE_EXHAUSTION";
        case ANOMALY_DATA_CORRUPTION: return "DATA_CORRUPTION";
        default: return "UNKNOWN";
    }
}

uint32_t logging_calculate_checksum(const void* data, uint32_t size) {
    return calculate_crc32(data, size);
}

// Default configurations
logging_config_t logging_get_default_config(void) {
    logging_config_t config = {0};
    config.min_level = LOG_LEVEL_INFO;
    config.enabled_categories = LOG_CATEGORY_ALL;
    config.enable_persistent_storage = true;
    config.storage_type = STORAGE_TYPE_EEPROM;
    config.max_file_size = 1024 * 1024; // 1MB
    config.retention_days = 30;
    config.enable_compression = false;
    config.enable_encryption = false;
    return config;
}

logging_config_t logging_get_minimal_config(void) {
    logging_config_t config = {0};
    config.min_level = LOG_LEVEL_WARNING;
    config.enabled_categories = LOG_CATEGORY_INTERVENTION | LOG_CATEGORY_ANOMALY | LOG_CATEGORY_SAFETY;
    config.enable_persistent_storage = false;
    config.storage_type = STORAGE_TYPE_RAM;
    config.max_file_size = 64 * 1024; // 64KB
    config.retention_days = 7;
    config.enable_compression = false;
    config.enable_encryption = false;
    return config;
}

logging_config_t logging_get_debug_config(void) {
    logging_config_t config = {0};
    config.min_level = LOG_LEVEL_TRACE;
    config.enabled_categories = LOG_CATEGORY_ALL;
    config.enable_persistent_storage = true;
    config.storage_type = STORAGE_TYPE_EXTERNAL;
    config.max_file_size = 10 * 1024 * 1024; // 10MB
    config.retention_days = 90;
    config.enable_compression = true;
    config.enable_encryption = false;
    return config;
}

// Internal helper functions
static uint32_t get_system_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static uint32_t calculate_crc32(const void* data, uint32_t size) {
    const uint8_t* bytes = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < size; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CHECKSUM_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc ^ 0xFFFFFFFF;
}

static bool circular_buffer_init(circular_log_buffer_t* buffer, uint32_t size) {
    if (!buffer || size == 0) {
        return false;
    }
    
    buffer->entries = (log_entry_t*)malloc(sizeof(log_entry_t) * size);
    if (!buffer->entries) {
        return false;
    }
    
    buffer->size = size;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
    buffer->full = false;
    buffer->total_entries = 0;
    buffer->dropped_entries = 0;
    
    return true;
}

static void circular_buffer_cleanup(circular_log_buffer_t* buffer) {
    if (!buffer) return;
    
    if (buffer->entries) {
        free(buffer->entries);
        buffer->entries = NULL;
    }
    
    memset(buffer, 0, sizeof(circular_log_buffer_t));
}

static bool circular_buffer_write(circular_log_buffer_t* buffer, const log_entry_t* entry) {
    if (!buffer || !buffer->entries || !entry) {
        return false;
    }
    
    // Copy entry to buffer
    memcpy(&buffer->entries[buffer->head], entry, sizeof(log_entry_t));
    
    // Advance head pointer
    buffer->head = (buffer->head + 1) % buffer->size;
    
    if (buffer->full) {
        // Buffer is full, advance tail (overwrite oldest)
        buffer->tail = (buffer->tail + 1) % buffer->size;
        buffer->dropped_entries++;
    } else {
        buffer->count++;
        if (buffer->count == buffer->size) {
            buffer->full = true;
        }
    }
    
    buffer->total_entries++;
    return true;
}

static bool circular_buffer_read(const circular_log_buffer_t* buffer, uint32_t index, log_entry_t* entry) {
    if (!buffer || !buffer->entries || !entry || index >= buffer->count) {
        return false;
    }
    
    uint32_t actual_index = (buffer->tail + index) % buffer->size;
    memcpy(entry, &buffer->entries[actual_index], sizeof(log_entry_t));
    
    return true;
}

static void create_log_entry(log_entry_t* entry, log_level_t level, log_category_t category,
                            uint8_t module_id, uint32_t event_code, const char* message,
                            const void* data, uint32_t data_size) {
    if (!entry || !message) return;
    
    memset(entry, 0, sizeof(log_entry_t));
    
    entry->timestamp_ms = get_system_time_ms();
    entry->sequence_number = g_logging_system ? g_logging_system->sequence_counter++ : 0;
    entry->level = level;
    entry->category = category;
    entry->module_id = module_id;
    entry->event_code = event_code;
    
    strncpy(entry->message, message, LOG_MAX_MESSAGE_LENGTH - 1);
    entry->message[LOG_MAX_MESSAGE_LENGTH - 1] = '\0';
    
    if (data && data_size > 0) {
        uint32_t copy_size = (data_size > 64) ? 64 : data_size;
        memcpy(entry->data, data, copy_size);
        entry->data_length = copy_size;
    }
    
    entry->checksum = calculate_crc32(entry, sizeof(log_entry_t) - sizeof(uint32_t));
}

// Storage interface implementations (simulation)
static storage_interface_t g_eeprom_storage = {0};
static storage_interface_t g_flash_storage = {0};
static storage_interface_t g_external_storage = {0};

// EEPROM storage simulation
static bool eeprom_init(void) {
    printf("EEPROM storage initialized\n");
    return true;
}

static bool eeprom_write(uint32_t address, const void* data, uint32_t size) {
    printf("EEPROM write: address=0x%08X, size=%u bytes\n", address, size);
    return true;
}

static bool eeprom_read(uint32_t address, void* data, uint32_t size) {
    printf("EEPROM read: address=0x%08X, size=%u bytes\n", address, size);
    return true;
}

static bool eeprom_erase(uint32_t address, uint32_t size) {
    printf("EEPROM erase: address=0x%08X, size=%u bytes\n", address, size);
    return true;
}

static uint32_t eeprom_get_size(void) {
    return 64 * 1024; // 64KB
}

static bool eeprom_sync(void) {
    printf("EEPROM sync\n");
    return true;
}

storage_interface_t* logging_get_eeprom_storage_interface(void) {
    g_eeprom_storage.init = eeprom_init;
    g_eeprom_storage.write = eeprom_write;
    g_eeprom_storage.read = eeprom_read;
    g_eeprom_storage.erase = eeprom_erase;
    g_eeprom_storage.get_size = eeprom_get_size;
    g_eeprom_storage.sync = eeprom_sync;
    return &g_eeprom_storage;
}

// Similar implementations for Flash and External storage
storage_interface_t* logging_get_flash_storage_interface(void) {
    // Implementation similar to EEPROM but for Flash
    return &g_flash_storage;
}

storage_interface_t* logging_get_external_storage_interface(void) {
    // Implementation for external storage (SD card, etc.)
    return &g_external_storage;
}