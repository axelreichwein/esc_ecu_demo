#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

// Sensor types and identifiers
typedef enum {
    SENSOR_WHEEL_SPEED_FL = 0,
    SENSOR_WHEEL_SPEED_FR = 1,
    SENSOR_WHEEL_SPEED_RL = 2,
    SENSOR_WHEEL_SPEED_RR = 3,
    SENSOR_STEERING_ANGLE = 4,
    SENSOR_YAW_RATE = 5,
    SENSOR_LATERAL_ACCEL = 6,
    SENSOR_LONGITUDINAL_ACCEL = 7,
    SENSOR_BRAKE_PRESSURE = 8,
    SENSOR_COUNT
} sensor_type_t;

// Sensor interface types
typedef enum {
    SENSOR_IF_ANALOG = 0,      // ADC input
    SENSOR_IF_DIGITAL = 1,     // Digital/PWM input
    SENSOR_IF_CAN = 2,         // CAN bus message
    SENSOR_IF_SPI = 3,         // SPI interface
    SENSOR_IF_UART = 4         // UART/Serial interface
} sensor_interface_t;

// Digital filter types
typedef enum {
    FILTER_NONE = 0,
    FILTER_LOW_PASS = 1,
    FILTER_MEDIAN = 2,
    FILTER_MOVING_AVERAGE = 3,
    FILTER_DEBOUNCE = 4,
    FILTER_COMBINED = 5        // Multiple filters in sequence
} filter_type_t;

// Sensor status flags
typedef enum {
    SENSOR_STATUS_OK = 0,
    SENSOR_STATUS_NO_SIGNAL = 1,
    SENSOR_STATUS_OUT_OF_RANGE = 2,
    SENSOR_STATUS_STUCK = 3,
    SENSOR_STATUS_NOISY = 4,
    SENSOR_STATUS_CALIBRATION_ERROR = 5,
    SENSOR_STATUS_HARDWARE_FAULT = 6
} sensor_status_t;

// Sensor calibration data structure
typedef struct {
    float offset;              // Zero offset
    float scale;               // Scale factor
    float min_value;           // Minimum valid value
    float max_value;           // Maximum valid value
    float stuck_threshold;     // Threshold for stuck sensor detection
    float noise_threshold;     // Threshold for excessive noise detection
    uint32_t stuck_time_ms;    // Time to declare sensor stuck
    bool calibration_valid;    // Calibration data validity flag
} sensor_calibration_t;

// Hardware specification structure
typedef struct {
    sensor_interface_t interface_type;
    uint32_t sample_rate_hz;   // Sensor sampling rate
    uint16_t resolution_bits;  // ADC resolution (for analog sensors)
    float voltage_range_min;   // Minimum voltage (for analog sensors)
    float voltage_range_max;   // Maximum voltage (for analog sensors)
    uint32_t can_id;          // CAN message ID (for CAN sensors)
    uint8_t can_byte_offset;  // Byte offset in CAN message
    uint8_t can_bit_length;   // Bit length in CAN message
    bool can_signed;          // Signed/unsigned CAN data
    filter_type_t filter_type;
    float filter_cutoff_hz;   // Low-pass filter cutoff frequency
    uint8_t median_window_size; // Median filter window size
    uint8_t avg_window_size;  // Moving average window size
    uint32_t debounce_time_ms; // Debounce time for digital inputs
} sensor_hw_spec_t;

// Filter state structures
typedef struct {
    float previous_output;
    float alpha;              // Filter coefficient
} low_pass_filter_t;

typedef struct {
    float* buffer;
    uint8_t size;
    uint8_t index;
    bool buffer_full;
} median_filter_t;

typedef struct {
    float* buffer;
    uint8_t size;
    uint8_t index;
    float sum;
    bool buffer_full;
} moving_avg_filter_t;

typedef struct {
    bool current_state;
    bool debounced_state;
    uint32_t last_change_time;
    uint32_t debounce_time_ms;
} debounce_filter_t;

// Combined filter state
typedef struct {
    low_pass_filter_t low_pass;
    median_filter_t median;
    moving_avg_filter_t moving_avg;
    debounce_filter_t debounce;
    filter_type_t active_filter;
} digital_filter_t;

// Sensor data structure with metadata
typedef struct {
    float raw_value;           // Raw sensor value (before conversion)
    float converted_value;     // After unit conversion
    float filtered_value;      // After digital filtering
    sensor_status_t status;    // Sensor health status
    uint32_t timestamp_us;     // Microsecond timestamp
    uint32_t sample_count;     // Number of samples processed
    float signal_quality;      // Signal quality indicator (0-1)
    bool data_valid;          // Data validity flag
} sensor_data_t;

// Asynchronous sensor handler structure
typedef struct {
    sensor_type_t sensor_id;
    sensor_hw_spec_t hw_spec;
    sensor_calibration_t calibration;
    sensor_data_t current_data;
    digital_filter_t filter_state;
    pthread_t acquisition_thread;
    pthread_mutex_t data_mutex;
    volatile bool thread_running;
    volatile bool new_data_available;
    uint32_t acquisition_errors;
    uint32_t last_acquisition_time_us;
    void (*callback)(sensor_type_t sensor_id, const sensor_data_t* data);
} sensor_handler_t;

// Sensor interface management structure
typedef struct {
    sensor_handler_t handlers[SENSOR_COUNT];
    bool system_initialized;
    uint32_t global_timestamp_us;
    pthread_mutex_t system_mutex;
} sensor_interface_system_t;

// Function prototypes for sensor interface
bool sensor_interface_init(void);
void sensor_interface_shutdown(void);
bool sensor_configure(sensor_type_t sensor_id, const sensor_hw_spec_t* hw_spec, const sensor_calibration_t* calibration);
bool sensor_start_acquisition(sensor_type_t sensor_id);
bool sensor_stop_acquisition(sensor_type_t sensor_id);
bool sensor_get_data(sensor_type_t sensor_id, sensor_data_t* data);
bool sensor_set_callback(sensor_type_t sensor_id, void (*callback)(sensor_type_t, const sensor_data_t*));
sensor_status_t sensor_get_status(sensor_type_t sensor_id);
bool sensor_calibrate(sensor_type_t sensor_id, float reference_value);
void sensor_reset_statistics(sensor_type_t sensor_id);

// Hardware abstraction layer functions
float sensor_read_analog(uint8_t channel);
uint16_t sensor_read_digital(uint8_t pin);
bool sensor_read_can_message(uint32_t can_id, uint8_t* data, uint8_t* length);
bool sensor_read_spi_data(uint8_t device_id, uint8_t* tx_data, uint8_t* rx_data, uint8_t length);

// Unit conversion functions
float convert_wheel_speed_to_rads(float raw_value, const sensor_hw_spec_t* hw_spec);
float convert_steering_angle_to_radians(float raw_value, const sensor_hw_spec_t* hw_spec);
float convert_yaw_rate_to_rads(float raw_value, const sensor_hw_spec_t* hw_spec);
float convert_acceleration_to_ms2(float raw_value, const sensor_hw_spec_t* hw_spec);
float convert_pressure_to_bar(float raw_value, const sensor_hw_spec_t* hw_spec);

// Digital filter functions
bool filter_init(digital_filter_t* filter, const sensor_hw_spec_t* hw_spec);
void filter_cleanup(digital_filter_t* filter);
float filter_process(digital_filter_t* filter, float input_value, uint32_t timestamp_us);
void filter_reset(digital_filter_t* filter);

// Individual filter processing functions
float filter_low_pass_process(low_pass_filter_t* lp_filter, float input);
float filter_median_process(median_filter_t* med_filter, float input);
float filter_moving_avg_process(moving_avg_filter_t* avg_filter, float input);
float filter_debounce_process(debounce_filter_t* db_filter, float input, uint32_t timestamp_us);

// Filter utility functions
bool filter_is_stable(const digital_filter_t* filter, float threshold, uint32_t time_window_us);
float filter_get_noise_level(const digital_filter_t* filter);
void filter_adaptive_adjust(digital_filter_t* filter, float signal_quality);

// Utility functions
uint32_t get_microsecond_timestamp(void);
float calculate_signal_quality(const sensor_data_t* data, uint32_t window_samples);
bool validate_sensor_range(float value, const sensor_calibration_t* calibration);
const char* sensor_status_to_string(sensor_status_t status);
const char* sensor_type_to_string(sensor_type_t sensor_type);

#endif // SENSOR_INTERFACE_H