#include "esc_logging.h"
#include "esc_ecu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

// Demo functions
void demo_basic_logging(esc_logging_system_t* logging);
void demo_intervention_logging(esc_logging_system_t* logging);
void demo_anomaly_detection(esc_logging_system_t* logging);
void demo_data_export(esc_logging_system_t* logging);
void demo_persistent_storage(esc_logging_system_t* logging);

// Simulate ESC system data
esc_sensor_data_t simulate_sensor_data(void) {
    esc_sensor_data_t sensors = {0};
    sensors.timestamp_ms = (uint32_t)(time(NULL) * 1000);
    sensors.vehicle_speed = 25.0f + (rand() % 100) / 10.0f; // 25-35 m/s
    sensors.yaw_rate = -0.1f + (rand() % 20) / 100.0f; // ±0.1 rad/s
    sensors.lateral_acceleration = -1.0f + (rand() % 20) / 10.0f; // ±1 m/s²
    sensors.steering_angle = -0.2f + (rand() % 40) / 100.0f; // ±0.2 rad
    
    for (int i = 0; i < 4; i++) {
        sensors.wheel_speed[i] = 20.0f + (rand() % 50) / 10.0f; // 20-25 rad/s
    }
    
    return sensors;
}

actuator_commands_t simulate_actuator_commands(void) {
    actuator_commands_t commands = {0};
    
    for (int i = 0; i < 4; i++) {
        commands.brake_pressure[i] = (rand() % 100) / 2.0f; // 0-50 bar
    }
    
    commands.engine_torque_reduction = (rand() % 30); // 0-30%
    commands.brake_light_active = (commands.brake_pressure[0] > 10.0f);
    commands.esc_warning_light = (commands.engine_torque_reduction > 5.0f);
    
    return commands;
}

int main(void) {
    printf("ESC Logging System Demonstration\n");
    printf("=================================\n\n");
    
    // Initialize random seed
    srand((unsigned int)time(NULL));
    
    // Create logging system
    esc_logging_system_t logging_system;
    
    // Get default configuration
    logging_config_t config = logging_get_default_config();
    config.min_level = LOG_LEVEL_DEBUG; // Show all messages for demo
    
    // Initialize logging system
    if (!logging_init(&logging_system, &config)) {
        printf("Failed to initialize logging system\n");
        return 1;
    }
    
    // Set up storage interface
    storage_interface_t* storage = logging_get_eeprom_storage_interface();
    logging_set_storage_interface(&logging_system, storage);
    
    printf("\n");
    
    // Run demonstrations
    demo_basic_logging(&logging_system);
    demo_intervention_logging(&logging_system);
    demo_anomaly_detection(&logging_system);
    demo_data_export(&logging_system);
    demo_persistent_storage(&logging_system);
    
    // Print final summary
    printf("\n");
    logging_print_summary(&logging_system);
    
    // Cleanup
    logging_shutdown(&logging_system);
    
    return 0;
}

void demo_basic_logging(esc_logging_system_t* logging) {
    printf("=== Basic Logging Demonstration ===\n");
    
    // Test different log levels and categories
    LOG_INFO(logging, LOG_CATEGORY_SYSTEM, ESC_MODULE_MAIN, 0x1001,
             "ESC system startup completed");
    
    LOG_DEBUG(logging, LOG_CATEGORY_SENSOR, ESC_MODULE_SENSOR, 0x2001,
              "Sensor calibration data loaded");
    
    LOG_WARNING(logging, LOG_CATEGORY_ACTUATOR, ESC_MODULE_BRAKE, 0x3001,
                "Brake actuator response time: %u ms (threshold: %u ms)", 25, 20);
    
    LOG_ERROR(logging, LOG_CATEGORY_COMMUNICATION, ESC_MODULE_CAN, 0x4001,
              "CAN message timeout on ID 0x%03X", 0x123);
    
    LOG_CRITICAL(logging, LOG_CATEGORY_SAFETY, ESC_MODULE_WATCHDOG, 0x5001,
                 "Watchdog timeout detected on task %s", "sensor_acquisition");
    
    // Test binary data logging
    uint8_t sensor_data[] = {0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0xEF, 0x00};
    logging_write_binary(logging, LOG_LEVEL_DEBUG, LOG_CATEGORY_DIAGNOSTIC,
                        ESC_MODULE_SENSOR, 0x2002, "Raw sensor data",
                        sensor_data, sizeof(sensor_data));
    
    printf("Basic logging completed - logged various levels and categories\n\n");
}

void demo_intervention_logging(esc_logging_system_t* logging) {
    printf("=== ESC Intervention Logging Demonstration ===\n");
    
    // Simulate multiple ESC interventions
    for (int i = 0; i < 3; i++) {
        esc_sensor_data_t sensors = simulate_sensor_data();
        actuator_commands_t commands = simulate_actuator_commands();
        
        // Determine intervention type based on simulated conditions
        esc_intervention_type_t intervention_type;
        if (i == 0) {
            intervention_type = ESC_INTERVENTION_UNDERSTEER_CORRECTION;
            sensors.yaw_rate = 0.05f; // Slight understeer
        } else if (i == 1) {
            intervention_type = ESC_INTERVENTION_OVERSTEER_CORRECTION;
            sensors.yaw_rate = -0.08f; // Oversteer condition
        } else {
            intervention_type = ESC_INTERVENTION_ENGINE_TORQUE_REDUCTION;
            commands.engine_torque_reduction = 15.0f;
        }
        
        // Start intervention
        uint32_t sequence_id;
        logging_start_intervention(logging, intervention_type, &sensors, &sequence_id);
        
        // Simulate intervention duration
        usleep(50000); // 50ms
        
        // Create detailed intervention log
        esc_intervention_log_t intervention = {0};
        intervention.timestamp_ms = sensors.timestamp_ms;
        intervention.type = intervention_type;
        intervention.duration_ms = 50 + (rand() % 100); // 50-150ms
        intervention.vehicle_speed = sensors.vehicle_speed;
        intervention.yaw_rate = sensors.yaw_rate;
        intervention.lateral_acceleration = sensors.lateral_acceleration;
        intervention.steering_angle = sensors.steering_angle;
        intervention.engine_torque_reduction = commands.engine_torque_reduction;
        intervention.sequence_id = sequence_id;
        intervention.trigger_conditions = 0x01 << i; // Different trigger conditions
        
        for (int j = 0; j < 4; j++) {
            intervention.brake_pressures[j] = commands.brake_pressure[j];
        }
        
        logging_log_intervention(logging, &intervention);
        
        // End intervention
        logging_end_intervention(logging, sequence_id, &commands);
        
        printf("Logged intervention %d: %s\n", i + 1, 
               intervention_type_to_string(intervention_type));
    }
    
    printf("ESC intervention logging completed\n\n");
}

void demo_anomaly_detection(esc_logging_system_t* logging) {
    printf("=== System Anomaly Detection Demonstration ===\n");
    
    // Simulate various anomalies
    
    // 1. Sensor drift anomaly
    logging_detect_and_log_anomaly(logging, ANOMALY_SENSOR_DRIFT,
                                  ESC_MODULE_SENSOR, 0.15f, 0.0f,
                                  "Yaw rate sensor showing 15% positive drift");
    
    // 2. Actuator delay anomaly
    logging_detect_and_log_anomaly(logging, ANOMALY_ACTUATOR_DELAY,
                                  ESC_MODULE_BRAKE, 25.0f, 10.0f,
                                  "Brake actuator response time exceeded threshold");
    
    // 3. Communication timeout anomaly
    logging_detect_and_log_anomaly(logging, ANOMALY_COMMUNICATION_TIMEOUT,
                                  ESC_MODULE_CAN, 150.0f, 50.0f,
                                  "CAN message response time degraded");
    
    // 4. Performance degradation anomaly
    logging_detect_and_log_anomaly(logging, ANOMALY_PERFORMANCE_DEGRADATION,
                                  ESC_MODULE_CONTROL, 12.5f, 10.0f,
                                  "Control loop execution time increased");
    
    // 5. Create a detailed anomaly manually
    system_anomaly_log_t detailed_anomaly = {0};
    detailed_anomaly.timestamp_ms = (uint32_t)(time(NULL) * 1000);
    detailed_anomaly.type = ANOMALY_CALIBRATION_MISMATCH;
    detailed_anomaly.affected_module = ESC_MODULE_PLAUSIBILITY;
    detailed_anomaly.severity_level = 180; // High severity
    detailed_anomaly.measured_value = 2.15f;
    detailed_anomaly.expected_value = 2.0f;
    detailed_anomaly.deviation_percent = 7.5f;
    detailed_anomaly.occurrence_count = 3;
    detailed_anomaly.duration_ms = 500;
    strncpy(detailed_anomaly.description, 
            "Wheel speed sensor calibration mismatch detected during plausibility check",
            sizeof(detailed_anomaly.description) - 1);
    
    logging_log_anomaly(logging, &detailed_anomaly);
    
    printf("System anomaly detection completed - logged 5 different anomaly types\n\n");
}

void demo_data_export(esc_logging_system_t* logging) {
    printf("=== Data Export Demonstration ===\n");
    
    // Export general log data
    uint8_t export_buffer[8192];
    uint32_t bytes_written;
    
    uint32_t current_time = (uint32_t)(time(NULL) * 1000);
    uint32_t start_time = current_time - 60000; // Last minute
    
    if (logging_export_to_buffer(logging, export_buffer, sizeof(export_buffer),
                                &bytes_written, start_time, current_time)) {
        printf("Exported general log data: %u bytes\n", bytes_written);
        
        // Verify export header
        export_header_t* header = (export_header_t*)export_buffer;
        printf("  File signature: 0x%08X\n", header->signature);
        printf("  Version: 0x%04X\n", header->version);
        printf("  Entry count: %u\n", header->entry_count);
        printf("  Time range: %u - %u ms\n", header->timestamp_start, header->timestamp_end);
    }
    
    // Export intervention data
    if (logging_export_interventions(logging, export_buffer, sizeof(export_buffer),
                                    &bytes_written)) {
        printf("Exported intervention data: %u bytes\n", bytes_written);
        printf("  Intervention entries: %u\n", bytes_written / sizeof(esc_intervention_log_t));
    }
    
    // Export anomaly data
    if (logging_export_anomalies(logging, export_buffer, sizeof(export_buffer),
                                &bytes_written)) {
        printf("Exported anomaly data: %u bytes\n", bytes_written);
        printf("  Anomaly entries: %u\n", bytes_written / sizeof(system_anomaly_log_t));
    }
    
    printf("Data export demonstration completed\n\n");
}

void demo_persistent_storage(esc_logging_system_t* logging) {
    printf("=== Persistent Storage Demonstration ===\n");
    
    // Generate some additional log entries
    for (int i = 0; i < 10; i++) {
        LOG_INFO(logging, LOG_CATEGORY_PERFORMANCE, ESC_MODULE_CONTROL, 0x6000 + i,
                 "Control cycle %d completed in %u ms", i, 8 + (rand() % 5));
    }
    
    // Flush to persistent storage
    printf("Flushing log data to persistent storage...\n");
    if (logging_flush_to_storage(logging)) {
        printf("Successfully flushed log data to storage\n");
    } else {
        printf("Failed to flush log data to storage\n");
    }
    
    // Simulate loading from storage
    printf("Loading log data from persistent storage...\n");
    if (logging_load_from_storage(logging)) {
        printf("Successfully loaded log data from storage\n");
    } else {
        printf("Failed to load log data from storage\n");
    }
    
    printf("Persistent storage demonstration completed\n\n");
}