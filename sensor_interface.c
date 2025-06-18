#include "sensor_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

// Global sensor interface system
static sensor_interface_system_t g_sensor_system = {0};

// Hardware simulation flags (for demo purposes)
#define SIMULATE_HARDWARE 1

// Thread function prototypes
static void* sensor_acquisition_thread(void* arg);
static bool sensor_acquire_sample(sensor_handler_t* handler);
static void sensor_process_sample(sensor_handler_t* handler, float raw_value);
static void sensor_update_status(sensor_handler_t* handler);

bool sensor_interface_init(void) {
    if (g_sensor_system.system_initialized) {
        return true; // Already initialized
    }
    
    memset(&g_sensor_system, 0, sizeof(sensor_interface_system_t));
    
    // Initialize system mutex
    if (pthread_mutex_init(&g_sensor_system.system_mutex, NULL) != 0) {
        return false;
    }
    
    // Initialize each sensor handler
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_handler_t* handler = &g_sensor_system.handlers[i];
        handler->sensor_id = (sensor_type_t)i;
        handler->thread_running = false;
        handler->new_data_available = false;
        handler->current_data.status = SENSOR_STATUS_NO_SIGNAL;
        
        // Initialize mutex for this sensor
        if (pthread_mutex_init(&handler->data_mutex, NULL) != 0) {
            // Cleanup previously initialized mutexes
            for (int j = 0; j < i; j++) {
                pthread_mutex_destroy(&g_sensor_system.handlers[j].data_mutex);
            }
            pthread_mutex_destroy(&g_sensor_system.system_mutex);
            return false;
        }
    }
    
    g_sensor_system.system_initialized = true;
    printf("Sensor interface system initialized\n");
    return true;
}

void sensor_interface_shutdown(void) {
    if (!g_sensor_system.system_initialized) {
        return;
    }
    
    pthread_mutex_lock(&g_sensor_system.system_mutex);
    
    // Stop all acquisition threads
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_stop_acquisition((sensor_type_t)i);
        filter_cleanup(&g_sensor_system.handlers[i].filter_state);
        pthread_mutex_destroy(&g_sensor_system.handlers[i].data_mutex);
    }
    
    g_sensor_system.system_initialized = false;
    pthread_mutex_unlock(&g_sensor_system.system_mutex);
    pthread_mutex_destroy(&g_sensor_system.system_mutex);
    
    printf("Sensor interface system shutdown\n");
}

bool sensor_configure(sensor_type_t sensor_id, const sensor_hw_spec_t* hw_spec, const sensor_calibration_t* calibration) {
    if (!g_sensor_system.system_initialized || sensor_id >= SENSOR_COUNT || !hw_spec || !calibration) {
        return false;
    }
    
    sensor_handler_t* handler = &g_sensor_system.handlers[sensor_id];
    
    pthread_mutex_lock(&handler->data_mutex);
    
    // Copy hardware specification
    memcpy(&handler->hw_spec, hw_spec, sizeof(sensor_hw_spec_t));
    
    // Copy calibration data
    memcpy(&handler->calibration, calibration, sizeof(sensor_calibration_t));
    
    // Initialize digital filter
    if (!filter_init(&handler->filter_state, hw_spec)) {
        pthread_mutex_unlock(&handler->data_mutex);
        return false;
    }
    
    // Reset statistics
    handler->acquisition_errors = 0;
    handler->current_data.sample_count = 0;
    handler->current_data.status = SENSOR_STATUS_NO_SIGNAL;
    
    pthread_mutex_unlock(&handler->data_mutex);
    
    printf("Sensor %s configured\n", sensor_type_to_string(sensor_id));
    return true;
}

bool sensor_start_acquisition(sensor_type_t sensor_id) {
    if (!g_sensor_system.system_initialized || sensor_id >= SENSOR_COUNT) {
        return false;
    }
    
    sensor_handler_t* handler = &g_sensor_system.handlers[sensor_id];
    
    if (handler->thread_running) {
        return true; // Already running
    }
    
    handler->thread_running = true;
    
    // Create acquisition thread
    if (pthread_create(&handler->acquisition_thread, NULL, sensor_acquisition_thread, handler) != 0) {
        handler->thread_running = false;
        return false;
    }
    
    printf("Started acquisition for sensor %s\n", sensor_type_to_string(sensor_id));
    return true;
}

bool sensor_stop_acquisition(sensor_type_t sensor_id) {
    if (!g_sensor_system.system_initialized || sensor_id >= SENSOR_COUNT) {
        return false;
    }
    
    sensor_handler_t* handler = &g_sensor_system.handlers[sensor_id];
    
    if (!handler->thread_running) {
        return true; // Already stopped
    }
    
    handler->thread_running = false;
    
    // Wait for thread to finish
    pthread_join(handler->acquisition_thread, NULL);
    
    pthread_mutex_lock(&handler->data_mutex);
    handler->current_data.status = SENSOR_STATUS_NO_SIGNAL;
    pthread_mutex_unlock(&handler->data_mutex);
    
    printf("Stopped acquisition for sensor %s\n", sensor_type_to_string(sensor_id));
    return true;
}

bool sensor_get_data(sensor_type_t sensor_id, sensor_data_t* data) {
    if (!g_sensor_system.system_initialized || sensor_id >= SENSOR_COUNT || !data) {
        return false;
    }
    
    sensor_handler_t* handler = &g_sensor_system.handlers[sensor_id];
    
    pthread_mutex_lock(&handler->data_mutex);
    memcpy(data, &handler->current_data, sizeof(sensor_data_t));
    handler->new_data_available = false; // Mark as read
    pthread_mutex_unlock(&handler->data_mutex);
    
    return true;
}

bool sensor_set_callback(sensor_type_t sensor_id, void (*callback)(sensor_type_t, const sensor_data_t*)) {
    if (!g_sensor_system.system_initialized || sensor_id >= SENSOR_COUNT) {
        return false;
    }
    
    sensor_handler_t* handler = &g_sensor_system.handlers[sensor_id];
    
    pthread_mutex_lock(&handler->data_mutex);
    handler->callback = callback;
    pthread_mutex_unlock(&handler->data_mutex);
    
    return true;
}

sensor_status_t sensor_get_status(sensor_type_t sensor_id) {
    if (!g_sensor_system.system_initialized || sensor_id >= SENSOR_COUNT) {
        return SENSOR_STATUS_HARDWARE_FAULT;
    }
    
    return g_sensor_system.handlers[sensor_id].current_data.status;
}

// Asynchronous acquisition thread
static void* sensor_acquisition_thread(void* arg) {
    sensor_handler_t* handler = (sensor_handler_t*)arg;
    uint32_t sample_period_us = 1000000 / handler->hw_spec.sample_rate_hz;
    
    printf("Acquisition thread started for sensor %s at %d Hz\n", 
           sensor_type_to_string(handler->sensor_id), handler->hw_spec.sample_rate_hz);
    
    while (handler->thread_running) {
        uint32_t start_time = get_microsecond_timestamp();
        
        // Acquire and process sample
        if (sensor_acquire_sample(handler)) {
            sensor_update_status(handler);
        } else {
            pthread_mutex_lock(&handler->data_mutex);
            handler->acquisition_errors++;
            handler->current_data.status = SENSOR_STATUS_HARDWARE_FAULT;
            pthread_mutex_unlock(&handler->data_mutex);
        }
        
        // Maintain precise timing
        uint32_t elapsed_us = get_microsecond_timestamp() - start_time;
        if (elapsed_us < sample_period_us) {
            usleep(sample_period_us - elapsed_us);
        }
    }
    
    printf("Acquisition thread stopped for sensor %s\n", sensor_type_to_string(handler->sensor_id));
    return NULL;
}

static bool sensor_acquire_sample(sensor_handler_t* handler) {
    float raw_value = 0.0f;
    uint32_t timestamp = get_microsecond_timestamp();
    
    // Hardware-specific acquisition based on interface type
    switch (handler->hw_spec.interface_type) {
        case SENSOR_IF_ANALOG:
            raw_value = sensor_read_analog(handler->sensor_id);
            break;
            
        case SENSOR_IF_DIGITAL:
            raw_value = (float)sensor_read_digital(handler->sensor_id);
            break;
            
        case SENSOR_IF_CAN: {
            uint8_t can_data[8];
            uint8_t length;
            if (sensor_read_can_message(handler->hw_spec.can_id, can_data, &length)) {
                // Extract value from CAN message
                uint16_t raw_int = 0;
                if (handler->hw_spec.can_byte_offset < length) {
                    raw_int = can_data[handler->hw_spec.can_byte_offset];
                    if (handler->hw_spec.can_bit_length > 8 && 
                        handler->hw_spec.can_byte_offset + 1 < length) {
                        raw_int |= (can_data[handler->hw_spec.can_byte_offset + 1] << 8);
                    }
                }
                raw_value = handler->hw_spec.can_signed ? (int16_t)raw_int : raw_int;
            } else {
                return false;
            }
            break;
        }
        
        case SENSOR_IF_SPI: {
            uint8_t tx_data = 0x00;
            uint8_t rx_data[4];
            if (sensor_read_spi_data(handler->sensor_id, &tx_data, rx_data, 2)) {
                raw_value = (rx_data[0] << 8) | rx_data[1];
            } else {
                return false;
            }
            break;
        }
        
        default:
            return false;
    }
    
    // Process the acquired sample
    sensor_process_sample(handler, raw_value);
    
    pthread_mutex_lock(&handler->data_mutex);
    handler->last_acquisition_time_us = timestamp;
    handler->new_data_available = true;
    pthread_mutex_unlock(&handler->data_mutex);
    
    return true;
}

static void sensor_process_sample(sensor_handler_t* handler, float raw_value) {
    pthread_mutex_lock(&handler->data_mutex);
    
    // Store raw value
    handler->current_data.raw_value = raw_value;
    handler->current_data.timestamp_us = get_microsecond_timestamp();
    
    // Apply unit conversion
    float converted_value = raw_value;
    switch (handler->sensor_id) {
        case SENSOR_WHEEL_SPEED_FL:
        case SENSOR_WHEEL_SPEED_FR:
        case SENSOR_WHEEL_SPEED_RL:
        case SENSOR_WHEEL_SPEED_RR:
            converted_value = convert_wheel_speed_to_rads(raw_value, &handler->hw_spec);
            break;
            
        case SENSOR_STEERING_ANGLE:
            converted_value = convert_steering_angle_to_radians(raw_value, &handler->hw_spec);
            break;
            
        case SENSOR_YAW_RATE:
            converted_value = convert_yaw_rate_to_rads(raw_value, &handler->hw_spec);
            break;
            
        case SENSOR_LATERAL_ACCEL:
        case SENSOR_LONGITUDINAL_ACCEL:
            converted_value = convert_acceleration_to_ms2(raw_value, &handler->hw_spec);
            break;
            
        case SENSOR_BRAKE_PRESSURE:
            converted_value = convert_pressure_to_bar(raw_value, &handler->hw_spec);
            break;
            
        default:
            break;
    }
    
    handler->current_data.converted_value = converted_value;
    
    // Apply digital filtering
    handler->current_data.filtered_value = filter_process(
        &handler->filter_state, converted_value, handler->current_data.timestamp_us);
    
    // Update sample count
    handler->current_data.sample_count++;
    
    // Calculate signal quality
    handler->current_data.signal_quality = calculate_signal_quality(
        &handler->current_data, 100); // Use last 100 samples
    
    // Validate data
    handler->current_data.data_valid = validate_sensor_range(
        handler->current_data.filtered_value, &handler->calibration);
    
    pthread_mutex_unlock(&handler->data_mutex);
    
    // Call callback if registered
    if (handler->callback) {
        handler->callback(handler->sensor_id, &handler->current_data);
    }
}

static void sensor_update_status(sensor_handler_t* handler) {
    pthread_mutex_lock(&handler->data_mutex);
    
    sensor_status_t new_status = SENSOR_STATUS_OK;
    
    // Check for out of range
    if (!handler->current_data.data_valid) {
        new_status = SENSOR_STATUS_OUT_OF_RANGE;
    }
    // Check for stuck sensor
    else if (handler->current_data.signal_quality < 0.1f) {
        new_status = SENSOR_STATUS_STUCK;
    }
    // Check for noisy signal
    else if (handler->current_data.signal_quality < 0.5f) {
        new_status = SENSOR_STATUS_NOISY;
    }
    
    handler->current_data.status = new_status;
    
    pthread_mutex_unlock(&handler->data_mutex);
}

// Unit conversion functions
float convert_wheel_speed_to_rads(float raw_value, const sensor_hw_spec_t* hw_spec) {
    // Convert ADC counts to voltage, then to frequency, then to rad/s
    if (hw_spec->interface_type == SENSOR_IF_ANALOG) {
        float voltage = (raw_value / ((1 << hw_spec->resolution_bits) - 1)) * 
                       (hw_spec->voltage_range_max - hw_spec->voltage_range_min) + 
                       hw_spec->voltage_range_min;
        // Assume wheel speed sensor outputs 1V per 10 Hz
        float frequency_hz = voltage * 10.0f;
        // Convert frequency to rad/s (assuming 60 teeth per wheel)
        return (frequency_hz * 2.0f * M_PI) / 60.0f;
    }
    return raw_value; // Direct conversion for other interfaces
}

float convert_steering_angle_to_radians(float raw_value, const sensor_hw_spec_t* hw_spec) {
    if (hw_spec->interface_type == SENSOR_IF_ANALOG) {
        float voltage = (raw_value / ((1 << hw_spec->resolution_bits) - 1)) * 
                       (hw_spec->voltage_range_max - hw_spec->voltage_range_min) + 
                       hw_spec->voltage_range_min;
        // Assume steering angle sensor: 2.5V = 0°, 0.5V = -90°, 4.5V = +90°
        float degrees = (voltage - 2.5f) * (90.0f / 2.0f);
        return degrees * M_PI / 180.0f;
    }
    return raw_value * M_PI / 180.0f; // Assume raw value in degrees
}

float convert_yaw_rate_to_rads(float raw_value, const sensor_hw_spec_t* hw_spec) {
    if (hw_spec->interface_type == SENSOR_IF_ANALOG) {
        float voltage = (raw_value / ((1 << hw_spec->resolution_bits) - 1)) * 
                       (hw_spec->voltage_range_max - hw_spec->voltage_range_min) + 
                       hw_spec->voltage_range_min;
        // Assume yaw rate sensor: 2.5V = 0°/s, sensitivity = 20mV/°/s
        float deg_per_sec = (voltage - 2.5f) / 0.02f;
        return deg_per_sec * M_PI / 180.0f;
    }
    return raw_value * M_PI / 180.0f; // Assume raw value in deg/s
}

float convert_acceleration_to_ms2(float raw_value, const sensor_hw_spec_t* hw_spec) {
    if (hw_spec->interface_type == SENSOR_IF_ANALOG) {
        float voltage = (raw_value / ((1 << hw_spec->resolution_bits) - 1)) * 
                       (hw_spec->voltage_range_max - hw_spec->voltage_range_min) + 
                       hw_spec->voltage_range_min;
        // Assume accelerometer: 2.5V = 0g, sensitivity = 1V/g
        float g_force = (voltage - 2.5f) / 1.0f;
        return g_force * 9.81f; // Convert to m/s²
    }
    return raw_value; // Assume raw value already in m/s²
}

float convert_pressure_to_bar(float raw_value, const sensor_hw_spec_t* hw_spec) {
    if (hw_spec->interface_type == SENSOR_IF_ANALOG) {
        float voltage = (raw_value / ((1 << hw_spec->resolution_bits) - 1)) * 
                       (hw_spec->voltage_range_max - hw_spec->voltage_range_min) + 
                       hw_spec->voltage_range_min;
        // Assume pressure sensor: 0.5V = 0 bar, 4.5V = 250 bar
        return (voltage - 0.5f) * (250.0f / 4.0f);
    }
    return raw_value; // Assume raw value already in bar
}

// Hardware abstraction layer (simulated for demo)
float sensor_read_analog(uint8_t channel) {
#if SIMULATE_HARDWARE
    // Simulate ADC reading with some noise
    static uint32_t seed = 12345;
    seed = seed * 1103515245 + 12345;
    float noise = ((seed >> 16) & 0xFFFF) / 65535.0f * 0.1f - 0.05f;
    
    switch (channel) {
        case SENSOR_WHEEL_SPEED_FL: return 2048 + 500 * sin(get_microsecond_timestamp() / 1000000.0f) + noise * 100;
        case SENSOR_WHEEL_SPEED_FR: return 2048 + 500 * sin(get_microsecond_timestamp() / 1000000.0f + 0.1f) + noise * 100;
        case SENSOR_WHEEL_SPEED_RL: return 2048 + 500 * sin(get_microsecond_timestamp() / 1000000.0f + 0.2f) + noise * 100;
        case SENSOR_WHEEL_SPEED_RR: return 2048 + 500 * sin(get_microsecond_timestamp() / 1000000.0f + 0.3f) + noise * 100;
        case SENSOR_STEERING_ANGLE: return 2048 + 400 * sin(get_microsecond_timestamp() / 2000000.0f) + noise * 50;
        case SENSOR_YAW_RATE: return 2048 + 200 * sin(get_microsecond_timestamp() / 1500000.0f) + noise * 30;
        case SENSOR_LATERAL_ACCEL: return 2048 + 300 * sin(get_microsecond_timestamp() / 1800000.0f) + noise * 40;
        case SENSOR_LONGITUDINAL_ACCEL: return 2048 + 100 * sin(get_microsecond_timestamp() / 3000000.0f) + noise * 20;
        default: return 2048;
    }
#else
    // Real hardware ADC read would go here
    // return adc_read_channel(channel);
    return 0.0f;
#endif
}

uint16_t sensor_read_digital(uint8_t pin) {
#if SIMULATE_HARDWARE
    return (get_microsecond_timestamp() / 100000) % 2; // Toggle every 100ms
#else
    // Real hardware digital read would go here
    // return gpio_read_pin(pin);
    return 0;
#endif
}

bool sensor_read_can_message(uint32_t can_id, uint8_t* data, uint8_t* length) {
#if SIMULATE_HARDWARE
    if (data && length) {
        *length = 8;
        // Simulate CAN data
        static uint32_t counter = 0;
        counter++;
        data[0] = (counter >> 8) & 0xFF;
        data[1] = counter & 0xFF;
        for (int i = 2; i < 8; i++) {
            data[i] = (uint8_t)(sin(get_microsecond_timestamp() / 1000000.0f + i) * 127 + 128);
        }
        return true;
    }
    return false;
#else
    // Real CAN read would go here
    // return can_receive_message(can_id, data, length);
    return false;
#endif
}

bool sensor_read_spi_data(uint8_t device_id, uint8_t* tx_data, uint8_t* rx_data, uint8_t length) {
#if SIMULATE_HARDWARE
    if (rx_data && length >= 2) {
        // Simulate SPI sensor data
        uint16_t value = 32768 + 8192 * sin(get_microsecond_timestamp() / 1000000.0f);
        rx_data[0] = (value >> 8) & 0xFF;
        rx_data[1] = value & 0xFF;
        return true;
    }
    return false;
#else
    // Real SPI transaction would go here
    // return spi_transfer(device_id, tx_data, rx_data, length);
    return false;
#endif
}

uint32_t get_microsecond_timestamp(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

float calculate_signal_quality(const sensor_data_t* data, uint32_t window_samples) {
    // Simplified signal quality calculation
    // In real implementation, would track variance, SNR, etc.
    if (data->sample_count < 10) {
        return 0.5f; // Insufficient data
    }
    
    // Assume good quality if data is within expected range
    return data->data_valid ? 0.9f : 0.1f;
}

bool validate_sensor_range(float value, const sensor_calibration_t* calibration) {
    if (!calibration->calibration_valid) {
        return false;
    }
    
    return (value >= calibration->min_value && value <= calibration->max_value);
}

const char* sensor_status_to_string(sensor_status_t status) {
    switch (status) {
        case SENSOR_STATUS_OK: return "OK";
        case SENSOR_STATUS_NO_SIGNAL: return "NO_SIGNAL";
        case SENSOR_STATUS_OUT_OF_RANGE: return "OUT_OF_RANGE";
        case SENSOR_STATUS_STUCK: return "STUCK";
        case SENSOR_STATUS_NOISY: return "NOISY";
        case SENSOR_STATUS_CALIBRATION_ERROR: return "CALIBRATION_ERROR";
        case SENSOR_STATUS_HARDWARE_FAULT: return "HARDWARE_FAULT";
        default: return "UNKNOWN";
    }
}

const char* sensor_type_to_string(sensor_type_t sensor_type) {
    switch (sensor_type) {
        case SENSOR_WHEEL_SPEED_FL: return "WHEEL_SPEED_FL";
        case SENSOR_WHEEL_SPEED_FR: return "WHEEL_SPEED_FR";
        case SENSOR_WHEEL_SPEED_RL: return "WHEEL_SPEED_RL";
        case SENSOR_WHEEL_SPEED_RR: return "WHEEL_SPEED_RR";
        case SENSOR_STEERING_ANGLE: return "STEERING_ANGLE";
        case SENSOR_YAW_RATE: return "YAW_RATE";
        case SENSOR_LATERAL_ACCEL: return "LATERAL_ACCEL";
        case SENSOR_LONGITUDINAL_ACCEL: return "LONGITUDINAL_ACCEL";
        case SENSOR_BRAKE_PRESSURE: return "BRAKE_PRESSURE";
        default: return "UNKNOWN";
    }
}

bool sensor_calibrate(sensor_type_t sensor_id, float reference_value) {
    if (!g_sensor_system.system_initialized || sensor_id >= SENSOR_COUNT) {
        return false;
    }
    
    sensor_handler_t* handler = &g_sensor_system.handlers[sensor_id];
    
    pthread_mutex_lock(&handler->data_mutex);
    
    // Perform sensor calibration
    if (handler->current_data.sample_count < 100) {
        // Need sufficient samples for calibration
        pthread_mutex_unlock(&handler->data_mutex);
        return false;
    }
    
    // Calculate offset based on current reading vs reference
    float current_reading = handler->current_data.filtered_value;
    float offset_correction = reference_value - current_reading;
    
    // Update calibration offset
    handler->calibration.offset += offset_correction;
    
    // Validate calibration (check if offset is reasonable)
    if (fabsf(handler->calibration.offset) > fabsf(reference_value) * 0.5f) {
        // Offset too large, calibration may be invalid
        handler->calibration.calibration_valid = false;
        pthread_mutex_unlock(&handler->data_mutex);
        return false;
    }
    
    handler->calibration.calibration_valid = true;
    
    // Reset filter to start fresh with new calibration
    filter_reset(&handler->filter_state);
    
    pthread_mutex_unlock(&handler->data_mutex);
    
    printf("Sensor %s calibrated with offset %.3f\n", 
           sensor_type_to_string(sensor_id), handler->calibration.offset);
    
    return true;
}

void sensor_reset_statistics(sensor_type_t sensor_id) {
    if (!g_sensor_system.system_initialized || sensor_id >= SENSOR_COUNT) {
        return;
    }
    
    sensor_handler_t* handler = &g_sensor_system.handlers[sensor_id];
    
    pthread_mutex_lock(&handler->data_mutex);
    
    // Reset acquisition statistics
    handler->acquisition_errors = 0;
    handler->current_data.sample_count = 0;
    
    // Reset filter state
    filter_reset(&handler->filter_state);
    
    pthread_mutex_unlock(&handler->data_mutex);
    
    printf("Statistics reset for sensor %s\n", sensor_type_to_string(sensor_id));
}