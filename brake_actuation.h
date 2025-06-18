#ifndef BRAKE_ACTUATION_H
#define BRAKE_ACTUATION_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "esc_types.h"

// Brake modulator interface types
typedef enum {
    BRAKE_INTERFACE_PWM = 0,     // PWM-controlled hydraulic modulator
    BRAKE_INTERFACE_CAN = 1,     // CAN-bus controlled electronic modulator
    BRAKE_INTERFACE_SPI = 2,     // SPI-controlled modulator
    BRAKE_INTERFACE_ANALOG = 3   // Analog voltage controlled modulator
} brake_interface_type_t;

// Brake modulator status
typedef enum {
    BRAKE_STATUS_OK = 0,
    BRAKE_STATUS_WARNING = 1,
    BRAKE_STATUS_FAULT = 2,
    BRAKE_STATUS_OFFLINE = 3,
    BRAKE_STATUS_CALIBRATING = 4
} brake_modulator_status_t;

// Individual brake channel configuration
typedef struct {
    brake_interface_type_t interface_type;
    uint8_t channel_id;              // PWM channel or CAN node ID
    float max_pressure;              // Maximum pressure for this channel (bar)
    float min_pressure;              // Minimum pressure for this channel (bar)
    float pressure_resolution;       // Pressure resolution (bar/bit)
    uint16_t pwm_frequency_hz;       // PWM frequency (if PWM interface)
    uint32_t can_id;                 // CAN message ID (if CAN interface)
    float response_time_ms;          // Expected response time
    bool invert_signal;              // Invert control signal
    float deadband_percent;          // Deadband around zero command
} brake_channel_config_t;

// Real-time brake feedback data
typedef struct {
    float actual_pressure;           // Measured brake pressure (bar)
    float target_pressure;           // Commanded brake pressure (bar)
    float pressure_error;            // Target - actual pressure (bar)
    float flow_rate;                 // Brake fluid flow rate (ml/s)
    float temperature;               // Brake fluid temperature (°C)
    uint16_t raw_feedback;           // Raw ADC/sensor value
    brake_modulator_status_t status; // Channel status
    uint32_t last_update_time_ms;    // Last feedback update timestamp
    bool pressure_valid;             // Pressure reading validity
    bool feedback_timeout;           // Feedback timeout flag
} brake_feedback_t;

// Closed-loop PID controller state
typedef struct {
    float kp;                        // Proportional gain
    float ki;                        // Integral gain
    float kd;                        // Derivative gain
    float integral_sum;              // Integral accumulator
    float last_error;                // Previous error for derivative
    float output_min;                // Minimum controller output
    float output_max;                // Maximum controller output
    float integral_limit;            // Integral windup limit
    bool enable_integral;            // Enable integral term
    bool enable_derivative;          // Enable derivative term
} brake_pid_controller_t;

// Brake actuation command
typedef struct {
    float target_pressure[MAX_WHEEL_COUNT];    // Target pressure per wheel (bar)
    float target_force[MAX_WHEEL_COUNT];       // Target force per wheel (N)
    uint16_t pwm_duty[MAX_WHEEL_COUNT];        // PWM duty cycle (0-1000 = 0-100.0%)
    uint8_t can_data[MAX_WHEEL_COUNT][8];      // CAN message data
    bool enable_channel[MAX_WHEEL_COUNT];      // Enable/disable per channel
    bool emergency_brake;                      // Emergency brake flag
    float fade_in_rate;                        // Pressure ramp-up rate (bar/s)
    float fade_out_rate;                       // Pressure ramp-down rate (bar/s)
    uint32_t command_timestamp_ms;             // Command generation timestamp
} brake_command_t;

// Brake actuation system state
typedef struct {
    brake_channel_config_t channel_config[MAX_WHEEL_COUNT];
    brake_feedback_t feedback[MAX_WHEEL_COUNT];
    brake_pid_controller_t pid_controller[MAX_WHEEL_COUNT];
    brake_command_t current_command;
    brake_command_t last_command;
    
    // System-wide status
    brake_modulator_status_t system_status;
    bool system_initialized;
    bool closed_loop_enabled;
    bool emergency_mode;
    uint32_t system_fault_count;
    
    // Performance metrics
    float max_pressure_error[MAX_WHEEL_COUNT];
    float avg_response_time_ms[MAX_WHEEL_COUNT];
    uint32_t command_count;
    uint32_t feedback_timeouts;
    
    // Calibration data
    float pressure_calibration_offset[MAX_WHEEL_COUNT];
    float pressure_calibration_gain[MAX_WHEEL_COUNT];
    bool calibration_valid[MAX_WHEEL_COUNT];
    
    // Threading and timing
    pthread_t control_thread;
    pthread_mutex_t command_mutex;
    bool control_thread_running;
    uint32_t control_loop_frequency_hz;
} brake_actuation_system_t;

// Hardware interface function pointers
typedef struct {
    // PWM interface functions
    bool (*pwm_init)(uint8_t channel, uint16_t frequency_hz);
    bool (*pwm_set_duty)(uint8_t channel, uint16_t duty_cycle);
    uint16_t (*pwm_get_duty)(uint8_t channel);
    
    // CAN interface functions
    bool (*can_init)(uint32_t baudrate);
    bool (*can_send_message)(uint32_t can_id, const uint8_t* data, uint8_t length);
    bool (*can_receive_message)(uint32_t* can_id, uint8_t* data, uint8_t* length);
    
    // Feedback interface functions
    bool (*feedback_init)(uint8_t channel);
    float (*read_pressure)(uint8_t channel);
    float (*read_temperature)(uint8_t channel);
    bool (*feedback_valid)(uint8_t channel);
} brake_hw_interface_t;

// Function prototypes

// System initialization and configuration
bool brake_actuation_init(brake_actuation_system_t* system, const brake_hw_interface_t* hw_interface);
void brake_actuation_shutdown(brake_actuation_system_t* system);
bool brake_configure_channel(brake_actuation_system_t* system, wheel_position_t wheel, 
                            const brake_channel_config_t* config);

// Command interface
bool brake_set_pressure_command(brake_actuation_system_t* system, const float pressures[MAX_WHEEL_COUNT]);
bool brake_set_force_command(brake_actuation_system_t* system, const float forces[MAX_WHEEL_COUNT]);
bool brake_set_emergency_brake(brake_actuation_system_t* system, bool enable);
bool brake_enable_channel(brake_actuation_system_t* system, wheel_position_t wheel, bool enable);

// Control modes
bool brake_enable_closed_loop(brake_actuation_system_t* system, bool enable);
bool brake_configure_pid_controller(brake_actuation_system_t* system, wheel_position_t wheel,
                                   float kp, float ki, float kd);
bool brake_set_control_limits(brake_actuation_system_t* system, wheel_position_t wheel,
                             float min_output, float max_output);

// Real-time operation
void brake_update_control_loop(brake_actuation_system_t* system);
void brake_process_feedback(brake_actuation_system_t* system);
bool brake_execute_commands(brake_actuation_system_t* system);

// Feedback and diagnostics
brake_feedback_t brake_get_feedback(const brake_actuation_system_t* system, wheel_position_t wheel);
brake_modulator_status_t brake_get_system_status(const brake_actuation_system_t* system);
float brake_get_pressure_error(const brake_actuation_system_t* system, wheel_position_t wheel);
bool brake_is_pressure_stable(const brake_actuation_system_t* system, wheel_position_t wheel, float tolerance);

// Calibration functions
bool brake_calibrate_pressure_sensor(brake_actuation_system_t* system, wheel_position_t wheel,
                                    float reference_pressure);
bool brake_auto_calibrate_system(brake_actuation_system_t* system);
bool brake_save_calibration_data(const brake_actuation_system_t* system, const char* filename);
bool brake_load_calibration_data(brake_actuation_system_t* system, const char* filename);

// PWM-specific functions
bool brake_pwm_init_channel(uint8_t channel, uint16_t frequency_hz);
bool brake_pwm_set_pressure(uint8_t channel, float target_pressure, float max_pressure);
uint16_t brake_pressure_to_pwm(float pressure, float max_pressure);
float brake_pwm_to_pressure(uint16_t pwm_duty, float max_pressure);

// CAN-specific functions
bool brake_can_init_interface(uint32_t baudrate);
bool brake_can_send_pressure_command(uint32_t can_id, wheel_position_t wheel, float pressure);
bool brake_can_receive_feedback(uint32_t can_id, brake_feedback_t* feedback);
void brake_can_encode_pressure_message(uint8_t* data, float pressure, uint8_t wheel_id);
float brake_can_decode_feedback_message(const uint8_t* data);

// Utility functions
bool brake_pressure_within_limits(float pressure, const brake_channel_config_t* config);
float brake_apply_pressure_ramp(float current, float target, float rate, float dt);
void brake_reset_pid_controller(brake_pid_controller_t* controller);
float brake_execute_pid_control(brake_pid_controller_t* controller, float error, float dt);

// Safety and monitoring functions
bool brake_safety_check(const brake_actuation_system_t* system);
void brake_emergency_shutdown(brake_actuation_system_t* system);
bool brake_detect_feedback_timeout(const brake_actuation_system_t* system, wheel_position_t wheel);
void brake_handle_system_fault(brake_actuation_system_t* system, wheel_position_t wheel);

// Diagnostic and reporting functions
void brake_print_system_status(const brake_actuation_system_t* system);
void brake_print_feedback_data(const brake_actuation_system_t* system);
void brake_log_performance_metrics(const brake_actuation_system_t* system);
const char* brake_status_to_string(brake_modulator_status_t status);
const char* brake_interface_to_string(brake_interface_type_t interface);

// Default configurations
brake_channel_config_t brake_get_default_pwm_config(wheel_position_t wheel);
brake_channel_config_t brake_get_default_can_config(wheel_position_t wheel);
brake_pid_controller_t brake_get_default_pid_config(void);

#endif // BRAKE_ACTUATION_H