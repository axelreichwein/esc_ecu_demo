#ifndef WATCHDOG_SUPERVISION_H
#define WATCHDOG_SUPERVISION_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "esc_types.h"

// Watchdog supervision timeout types
typedef enum {
    WATCHDOG_TIMEOUT_CRITICAL = 0,  // Critical system functions (10ms)
    WATCHDOG_TIMEOUT_NORMAL = 1,    // Normal system functions (50ms)
    WATCHDOG_TIMEOUT_DIAGNOSTIC = 2, // Diagnostic functions (200ms)
    WATCHDOG_TIMEOUT_COMMUNICATION = 3, // Communication functions (500ms)
    WATCHDOG_TIMEOUT_COUNT
} watchdog_timeout_type_t;

// Watchdog supervision task types
typedef enum {
    WATCHDOG_TASK_MAIN_LOOP = 0,
    WATCHDOG_TASK_SENSOR_ACQUISITION = 1,
    WATCHDOG_TASK_CONTROL_ALGORITHM = 2,
    WATCHDOG_TASK_BRAKE_ACTUATION = 3,
    WATCHDOG_TASK_ENGINE_INTERFACE = 4,
    WATCHDOG_TASK_DIAGNOSTICS = 5,
    WATCHDOG_TASK_COMMUNICATION = 6,
    WATCHDOG_TASK_PLAUSIBILITY = 7,
    WATCHDOG_TASK_COUNT
} watchdog_task_id_t;

// Watchdog supervision status
typedef enum {
    WATCHDOG_STATUS_OK = 0,
    WATCHDOG_STATUS_WARNING = 1,
    WATCHDOG_STATUS_TIMEOUT = 2,
    WATCHDOG_STATUS_CRITICAL_FAULT = 3,
    WATCHDOG_STATUS_RECOVERY = 4,
    WATCHDOG_STATUS_SAFE_STATE = 5
} watchdog_status_t;

// System reset strategy types
typedef enum {
    RESET_STRATEGY_NONE = 0,
    RESET_STRATEGY_SOFT_RESET = 1,      // Software reset without hardware restart
    RESET_STRATEGY_MODULE_RESTART = 2,   // Restart specific module
    RESET_STRATEGY_SAFE_STATE = 3,       // Transition to safe state
    RESET_STRATEGY_HARD_RESET = 4,       // Full system reset
    RESET_STRATEGY_SHUTDOWN = 5          // Emergency shutdown
} reset_strategy_t;

// Safe state configuration
typedef enum {
    SAFE_STATE_LEVEL_1 = 0,  // Minimal safe operation (brake only)
    SAFE_STATE_LEVEL_2 = 1,  // Reduced functionality 
    SAFE_STATE_LEVEL_3 = 2,  // Diagnostic mode
    SAFE_STATE_LEVEL_4 = 3   // Complete shutdown preparation
} safe_state_level_t;

// Individual task monitoring configuration
typedef struct {
    watchdog_task_id_t task_id;
    const char* task_name;
    watchdog_timeout_type_t timeout_type;
    uint32_t timeout_ms;
    uint32_t warning_threshold_ms;
    reset_strategy_t reset_strategy;
    bool enabled;
    bool critical_task;
} watchdog_task_config_t;

// Task monitoring runtime data
typedef struct {
    uint32_t last_kick_time_ms;
    uint32_t kick_count;
    uint32_t timeout_count;
    uint32_t max_execution_time_ms;
    uint32_t avg_execution_time_ms;
    bool timeout_occurred;
    watchdog_status_t status;
} watchdog_task_runtime_t;

// System safe state configuration
typedef struct {
    safe_state_level_t level;
    bool disable_engine_torque;
    bool enable_emergency_brake;
    float emergency_brake_pressure;
    bool disable_esc_intervention;
    bool enable_limp_mode;
    bool shutdown_non_critical_systems;
    uint32_t safe_state_timeout_ms;
    const char* safe_state_reason;
} safe_state_config_t;

// Recovery action configuration
typedef struct {
    reset_strategy_t strategy;
    uint32_t recovery_timeout_ms;
    uint8_t max_recovery_attempts;
    bool require_manual_reset;
    void (*recovery_callback)(watchdog_task_id_t task_id);
    void (*safe_state_callback)(safe_state_level_t level, const char* reason);
} recovery_config_t;

// Main watchdog supervision system
typedef struct {
    // Task monitoring
    watchdog_task_config_t task_configs[WATCHDOG_TASK_COUNT];
    watchdog_task_runtime_t task_runtime[WATCHDOG_TASK_COUNT];
    
    // System state
    watchdog_status_t system_status;
    safe_state_level_t current_safe_state;
    bool watchdog_enabled;
    bool system_initialized;
    
    // Timing and monitoring
    uint32_t system_start_time_ms;
    uint32_t last_global_kick_ms;
    uint32_t global_timeout_ms;
    uint32_t monitoring_frequency_hz;
    
    // Recovery and safe state
    recovery_config_t recovery_config;
    safe_state_config_t safe_state_config;
    uint32_t recovery_attempts[WATCHDOG_TASK_COUNT];
    bool in_recovery_mode;
    bool safe_state_active;
    uint32_t safe_state_entry_time_ms;
    
    // Statistics
    uint32_t total_kicks;
    uint32_t total_timeouts;
    uint32_t total_recoveries;
    uint32_t total_safe_state_entries;
    uint32_t uptime_ms;
    
    // Threading
    pthread_t monitoring_thread;
    pthread_mutex_t watchdog_mutex;
    bool monitoring_active;
    
    // Hardware interface
    bool hardware_watchdog_enabled;
    uint32_t hardware_watchdog_timeout_ms;
    void (*hardware_kick_callback)(void);
    void (*hardware_reset_callback)(void);
} watchdog_supervision_t;

// Hardware abstraction interface
typedef struct {
    bool (*hw_watchdog_init)(uint32_t timeout_ms);
    void (*hw_watchdog_kick)(void);
    void (*hw_watchdog_disable)(void);
    void (*hw_system_reset)(void);
    uint32_t (*get_system_time_ms)(void);
    void (*enter_safe_mode)(safe_state_level_t level);
    void (*emergency_shutdown)(void);
} watchdog_hw_interface_t;

// Function prototypes

// System initialization and configuration
bool watchdog_init(watchdog_supervision_t* watchdog, const watchdog_hw_interface_t* hw_interface);
void watchdog_shutdown(watchdog_supervision_t* watchdog);
bool watchdog_configure_task(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id, 
                            const watchdog_task_config_t* config);
bool watchdog_configure_recovery(watchdog_supervision_t* watchdog, const recovery_config_t* config);
bool watchdog_configure_safe_state(watchdog_supervision_t* watchdog, const safe_state_config_t* config);

// Task monitoring and supervision
bool watchdog_kick_task(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
bool watchdog_kick_global(watchdog_supervision_t* watchdog);
bool watchdog_enable_task(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id, bool enable);
bool watchdog_set_task_timeout(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id, uint32_t timeout_ms);

// Monitoring and status
watchdog_status_t watchdog_get_system_status(const watchdog_supervision_t* watchdog);
watchdog_status_t watchdog_get_task_status(const watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
bool watchdog_is_task_timeout(const watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
uint32_t watchdog_get_task_last_kick(const watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
uint32_t watchdog_get_system_uptime(const watchdog_supervision_t* watchdog);

// Recovery and safe state management
bool watchdog_trigger_recovery(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id, 
                              reset_strategy_t strategy);
bool watchdog_enter_safe_state(watchdog_supervision_t* watchdog, safe_state_level_t level, 
                              const char* reason);
bool watchdog_exit_safe_state(watchdog_supervision_t* watchdog);
bool watchdog_reset_task(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
bool watchdog_reset_system(watchdog_supervision_t* watchdog, reset_strategy_t strategy);

// Internal monitoring functions
void watchdog_monitor_tasks(watchdog_supervision_t* watchdog);
void watchdog_handle_timeout(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
void watchdog_execute_recovery(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
void watchdog_update_statistics(watchdog_supervision_t* watchdog);

// Hardware watchdog integration
bool watchdog_enable_hardware(watchdog_supervision_t* watchdog, uint32_t timeout_ms);
void watchdog_disable_hardware(watchdog_supervision_t* watchdog);
void watchdog_kick_hardware(watchdog_supervision_t* watchdog);

// Utility and helper functions
uint32_t watchdog_get_current_time_ms(void);
bool watchdog_validate_config(const watchdog_task_config_t* config);
void watchdog_calculate_task_statistics(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);

// Diagnostic and reporting functions
void watchdog_print_system_status(const watchdog_supervision_t* watchdog);
void watchdog_print_task_status(const watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
void watchdog_print_statistics(const watchdog_supervision_t* watchdog);
void watchdog_log_timeout(watchdog_task_id_t task_id, uint32_t elapsed_ms);
void watchdog_log_recovery(watchdog_task_id_t task_id, reset_strategy_t strategy, bool success);
void watchdog_log_safe_state_entry(safe_state_level_t level, const char* reason);

// String conversion functions
const char* watchdog_status_to_string(watchdog_status_t status);
const char* watchdog_task_name(watchdog_task_id_t task_id);
const char* reset_strategy_to_string(reset_strategy_t strategy);
const char* safe_state_level_to_string(safe_state_level_t level);

// Default configurations
watchdog_task_config_t watchdog_get_default_task_config(watchdog_task_id_t task_id);
recovery_config_t watchdog_get_default_recovery_config(void);
safe_state_config_t watchdog_get_default_safe_state_config(safe_state_level_t level);

// Timing utilities
static inline uint32_t watchdog_elapsed_time_ms(uint32_t start_time_ms, uint32_t current_time_ms) {
    if (current_time_ms >= start_time_ms) {
        return current_time_ms - start_time_ms;
    } else {
        // Handle overflow
        return (UINT32_MAX - start_time_ms) + current_time_ms + 1;
    }
}

// Task execution timing macros for easy integration
#define WATCHDOG_TASK_START(watchdog, task_id) \
    do { \
        if (watchdog) { \
            watchdog_kick_task(watchdog, task_id); \
        } \
    } while(0)

#define WATCHDOG_TASK_END(watchdog, task_id) \
    do { \
        if (watchdog) { \
            watchdog_kick_task(watchdog, task_id); \
        } \
    } while(0)

#define WATCHDOG_GLOBAL_KICK(watchdog) \
    do { \
        if (watchdog) { \
            watchdog_kick_global(watchdog); \
        } \
    } while(0)

#endif // WATCHDOG_SUPERVISION_H