#include "watchdog_supervision.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

// Constants for watchdog supervision
#define WATCHDOG_MONITORING_FREQUENCY_HZ 100   // 100Hz monitoring
#define WATCHDOG_GLOBAL_TIMEOUT_MS 5000        // 5 second global timeout
#define WATCHDOG_MAX_RECOVERY_ATTEMPTS 3       // Maximum recovery attempts per task
#define WATCHDOG_SAFE_STATE_TIMEOUT_MS 30000   // 30 second safe state timeout
#define WATCHDOG_HARDWARE_KICK_INTERVAL_MS 500 // 500ms hardware kick interval

// Global hardware interface
static const watchdog_hw_interface_t* g_hw_interface = NULL;

// Forward declarations for internal functions
static void* watchdog_monitoring_thread(void* arg);
static void watchdog_handle_critical_timeout(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id);
static void watchdog_execute_safe_state_transition(watchdog_supervision_t* watchdog, safe_state_level_t level);
static void watchdog_emergency_system_shutdown(watchdog_supervision_t* watchdog);
static void watchdog_signal_handler(int signal);

// System initialization and configuration
bool watchdog_init(watchdog_supervision_t* watchdog, const watchdog_hw_interface_t* hw_interface) {
    if (!watchdog) {
        return false;
    }
    
    memset(watchdog, 0, sizeof(watchdog_supervision_t));
    g_hw_interface = hw_interface;
    
    // Initialize system state
    watchdog->system_status = WATCHDOG_STATUS_OK;
    watchdog->current_safe_state = SAFE_STATE_LEVEL_1;
    watchdog->watchdog_enabled = true;
    watchdog->system_initialized = false;
    watchdog->monitoring_frequency_hz = WATCHDOG_MONITORING_FREQUENCY_HZ;
    watchdog->global_timeout_ms = WATCHDOG_GLOBAL_TIMEOUT_MS;
    watchdog->in_recovery_mode = false;
    watchdog->safe_state_active = false;
    
    // Get system start time
    watchdog->system_start_time_ms = watchdog_get_current_time_ms();
    watchdog->last_global_kick_ms = watchdog->system_start_time_ms;
    
    // Initialize mutex for thread safety
    if (pthread_mutex_init(&watchdog->watchdog_mutex, NULL) != 0) {
        printf("Failed to initialize watchdog mutex\\n");
        return false;
    }
    
    // Initialize default task configurations
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        watchdog->task_configs[i] = watchdog_get_default_task_config((watchdog_task_id_t)i);
        watchdog->task_runtime[i].status = WATCHDOG_STATUS_OK;
        watchdog->task_runtime[i].last_kick_time_ms = watchdog->system_start_time_ms;
        watchdog->recovery_attempts[i] = 0;
    }
    
    // Initialize default recovery and safe state configurations
    watchdog->recovery_config = watchdog_get_default_recovery_config();
    watchdog->safe_state_config = watchdog_get_default_safe_state_config(SAFE_STATE_LEVEL_1);
    
    // Initialize hardware watchdog if available
    if (g_hw_interface && g_hw_interface->hw_watchdog_init) {
        watchdog->hardware_watchdog_enabled = g_hw_interface->hw_watchdog_init(
            WATCHDOG_HARDWARE_KICK_INTERVAL_MS * 2); // 2x kick interval for hardware timeout
        watchdog->hardware_watchdog_timeout_ms = WATCHDOG_HARDWARE_KICK_INTERVAL_MS * 2;
        
        if (watchdog->hardware_watchdog_enabled) {
            printf("Hardware watchdog initialized with %u ms timeout\\n", 
                   watchdog->hardware_watchdog_timeout_ms);
        }
    }
    
    // Set up signal handlers for emergency shutdown
    signal(SIGTERM, watchdog_signal_handler);
    signal(SIGINT, watchdog_signal_handler);
    
    // Start monitoring thread
    watchdog->monitoring_active = true;
    if (pthread_create(&watchdog->monitoring_thread, NULL, watchdog_monitoring_thread, watchdog) != 0) {
        printf("Failed to create watchdog monitoring thread\\n");
        watchdog->monitoring_active = false;
        pthread_mutex_destroy(&watchdog->watchdog_mutex);
        return false;
    }
    
    watchdog->system_initialized = true;
    printf("Watchdog supervision system initialized successfully\\n");
    
    return true;
}

void watchdog_shutdown(watchdog_supervision_t* watchdog) {
    if (!watchdog) return;
    
    printf("Watchdog supervision system shutdown initiated\\n");
    
    // Stop monitoring thread
    if (watchdog->monitoring_active) {
        watchdog->monitoring_active = false;
        pthread_join(watchdog->monitoring_thread, NULL);
    }
    
    // Disable hardware watchdog
    if (watchdog->hardware_watchdog_enabled && g_hw_interface && g_hw_interface->hw_watchdog_disable) {
        g_hw_interface->hw_watchdog_disable();
    }
    
    // Exit safe state if active
    if (watchdog->safe_state_active) {
        watchdog_exit_safe_state(watchdog);
    }
    
    // Destroy mutex
    pthread_mutex_destroy(&watchdog->watchdog_mutex);
    
    // Print final statistics
    printf("Watchdog final statistics:\\n");
    printf("  Total uptime: %u ms\\n", watchdog_get_system_uptime(watchdog));
    printf("  Total kicks: %u\\n", watchdog->total_kicks);
    printf("  Total timeouts: %u\\n", watchdog->total_timeouts);
    printf("  Total recoveries: %u\\n", watchdog->total_recoveries);
    printf("  Safe state entries: %u\\n", watchdog->total_safe_state_entries);
    
    memset(watchdog, 0, sizeof(watchdog_supervision_t));
    printf("Watchdog supervision system shutdown complete\\n");
}

bool watchdog_configure_task(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id, 
                            const watchdog_task_config_t* config) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT || !config) {
        return false;
    }
    
    if (!watchdog_validate_config(config)) {
        printf("Invalid watchdog task configuration for task %s\\n", watchdog_task_name(task_id));
        return false;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    
    memcpy(&watchdog->task_configs[task_id], config, sizeof(watchdog_task_config_t));
    watchdog->task_runtime[task_id].status = WATCHDOG_STATUS_OK;
    watchdog->recovery_attempts[task_id] = 0;
    
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    printf("Watchdog task %s configured: timeout=%u ms, strategy=%s\\n",
           config->task_name, config->timeout_ms, reset_strategy_to_string(config->reset_strategy));
    
    return true;
}

bool watchdog_configure_recovery(watchdog_supervision_t* watchdog, const recovery_config_t* config) {
    if (!watchdog || !config) {
        return false;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    memcpy(&watchdog->recovery_config, config, sizeof(recovery_config_t));
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    printf("Watchdog recovery configuration updated\\n");
    return true;
}

bool watchdog_configure_safe_state(watchdog_supervision_t* watchdog, const safe_state_config_t* config) {
    if (!watchdog || !config) {
        return false;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    memcpy(&watchdog->safe_state_config, config, sizeof(safe_state_config_t));
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    printf("Watchdog safe state configuration updated: level=%s\\n", 
           safe_state_level_to_string(config->level));
    return true;
}

// Task monitoring and supervision
bool watchdog_kick_task(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT || !watchdog->watchdog_enabled) {
        return false;
    }
    
    uint32_t current_time = watchdog_get_current_time_ms();
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    
    watchdog_task_runtime_t* runtime = &watchdog->task_runtime[task_id];
    
    // Calculate execution time since last kick
    uint32_t execution_time = watchdog_elapsed_time_ms(runtime->last_kick_time_ms, current_time);
    
    // Update statistics
    runtime->last_kick_time_ms = current_time;
    runtime->kick_count++;
    watchdog->total_kicks++;
    
    // Update execution time statistics
    if (execution_time > runtime->max_execution_time_ms) {
        runtime->max_execution_time_ms = execution_time;
    }
    
    // Update average execution time (simple moving average)
    if (runtime->kick_count > 1) {
        runtime->avg_execution_time_ms = 
            (runtime->avg_execution_time_ms * (runtime->kick_count - 1) + execution_time) / runtime->kick_count;
    } else {
        runtime->avg_execution_time_ms = execution_time;
    }
    
    // Clear timeout flag if it was set
    if (runtime->timeout_occurred) {
        runtime->timeout_occurred = false;
        runtime->status = WATCHDOG_STATUS_OK;
        printf("Watchdog task %s recovered from timeout\\n", watchdog_task_name(task_id));
    }
    
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    return true;
}

bool watchdog_kick_global(watchdog_supervision_t* watchdog) {
    if (!watchdog || !watchdog->watchdog_enabled) {
        return false;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    watchdog->last_global_kick_ms = watchdog_get_current_time_ms();
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    // Kick hardware watchdog
    watchdog_kick_hardware(watchdog);
    
    return true;
}

bool watchdog_enable_task(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id, bool enable) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return false;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    watchdog->task_configs[task_id].enabled = enable;
    
    if (enable) {
        watchdog->task_runtime[task_id].last_kick_time_ms = watchdog_get_current_time_ms();
        watchdog->task_runtime[task_id].status = WATCHDOG_STATUS_OK;
    }
    
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    printf("Watchdog task %s %s\\n", watchdog_task_name(task_id), enable ? "enabled" : "disabled");
    return true;
}

bool watchdog_set_task_timeout(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id, uint32_t timeout_ms) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return false;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    watchdog->task_configs[task_id].timeout_ms = timeout_ms;
    watchdog->task_configs[task_id].warning_threshold_ms = timeout_ms / 2; // Warning at 50%
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    printf("Watchdog task %s timeout updated to %u ms\\n", watchdog_task_name(task_id), timeout_ms);
    return true;
}

// Monitoring and status
watchdog_status_t watchdog_get_system_status(const watchdog_supervision_t* watchdog) {
    if (!watchdog) {
        return WATCHDOG_STATUS_CRITICAL_FAULT;
    }
    return watchdog->system_status;
}

watchdog_status_t watchdog_get_task_status(const watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return WATCHDOG_STATUS_CRITICAL_FAULT;
    }
    return watchdog->task_runtime[task_id].status;
}

bool watchdog_is_task_timeout(const watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return true;
    }
    return watchdog->task_runtime[task_id].timeout_occurred;
}

uint32_t watchdog_get_task_last_kick(const watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return 0;
    }
    return watchdog->task_runtime[task_id].last_kick_time_ms;
}

uint32_t watchdog_get_system_uptime(const watchdog_supervision_t* watchdog) {
    if (!watchdog) {
        return 0;
    }
    return watchdog_elapsed_time_ms(watchdog->system_start_time_ms, watchdog_get_current_time_ms());
}

// Recovery and safe state management
bool watchdog_trigger_recovery(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id, 
                              reset_strategy_t strategy) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return false;
    }
    
    printf("Triggering recovery for task %s with strategy %s\\n",
           watchdog_task_name(task_id), reset_strategy_to_string(strategy));
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    
    // Check recovery attempt limit
    if (watchdog->recovery_attempts[task_id] >= WATCHDOG_MAX_RECOVERY_ATTEMPTS) {
        printf("Maximum recovery attempts exceeded for task %s, entering safe state\\n",
               watchdog_task_name(task_id));
        pthread_mutex_unlock(&watchdog->watchdog_mutex);
        return watchdog_enter_safe_state(watchdog, SAFE_STATE_LEVEL_2, "Max recovery attempts exceeded");
    }
    
    watchdog->recovery_attempts[task_id]++;
    watchdog->total_recoveries++;
    watchdog->in_recovery_mode = true;
    
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    // Execute recovery strategy
    bool recovery_success = false;
    
    switch (strategy) {
        case RESET_STRATEGY_SOFT_RESET:
            // Reset task state
            watchdog_reset_task(watchdog, task_id);
            recovery_success = true;
            break;
            
        case RESET_STRATEGY_MODULE_RESTART:
            // Call recovery callback if available
            if (watchdog->recovery_config.recovery_callback) {
                watchdog->recovery_config.recovery_callback(task_id);
                recovery_success = true;
            }
            break;
            
        case RESET_STRATEGY_SAFE_STATE:
            return watchdog_enter_safe_state(watchdog, SAFE_STATE_LEVEL_2, "Recovery strategy requested");
            
        case RESET_STRATEGY_HARD_RESET:
            return watchdog_reset_system(watchdog, RESET_STRATEGY_HARD_RESET);
            
        case RESET_STRATEGY_SHUTDOWN:
            watchdog_emergency_system_shutdown(watchdog);
            return true;
            
        default:
            break;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    watchdog->in_recovery_mode = false;
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    watchdog_log_recovery(task_id, strategy, recovery_success);
    return recovery_success;
}

bool watchdog_enter_safe_state(watchdog_supervision_t* watchdog, safe_state_level_t level, 
                              const char* reason) {
    if (!watchdog) {
        return false;
    }
    
    printf("ENTERING SAFE STATE: Level %s, Reason: %s\\n", 
           safe_state_level_to_string(level), reason ? reason : "Unknown");
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    
    watchdog->safe_state_active = true;
    watchdog->current_safe_state = level;
    watchdog->safe_state_entry_time_ms = watchdog_get_current_time_ms();
    watchdog->total_safe_state_entries++;
    watchdog->system_status = WATCHDOG_STATUS_SAFE_STATE;
    
    // Update safe state configuration
    watchdog->safe_state_config.level = level;
    watchdog->safe_state_config.safe_state_reason = reason;
    
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    // Execute safe state transition
    watchdog_execute_safe_state_transition(watchdog, level);
    
    // Call safe state callback if available
    if (watchdog->recovery_config.safe_state_callback) {
        watchdog->recovery_config.safe_state_callback(level, reason);
    }
    
    watchdog_log_safe_state_entry(level, reason);
    return true;
}

bool watchdog_exit_safe_state(watchdog_supervision_t* watchdog) {
    if (!watchdog || !watchdog->safe_state_active) {
        return false;
    }
    
    printf("Exiting safe state\\n");
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    
    watchdog->safe_state_active = false;
    watchdog->current_safe_state = SAFE_STATE_LEVEL_1;
    watchdog->system_status = WATCHDOG_STATUS_OK;
    
    // Reset recovery attempts
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        watchdog->recovery_attempts[i] = 0;
        watchdog->task_runtime[i].status = WATCHDOG_STATUS_OK;
        watchdog->task_runtime[i].timeout_occurred = false;
    }
    
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    return true;
}

bool watchdog_reset_task(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return false;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    
    watchdog_task_runtime_t* runtime = &watchdog->task_runtime[task_id];
    runtime->last_kick_time_ms = watchdog_get_current_time_ms();
    runtime->timeout_occurred = false;
    runtime->status = WATCHDOG_STATUS_OK;
    watchdog->recovery_attempts[task_id] = 0;
    
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    printf("Task %s reset successfully\\n", watchdog_task_name(task_id));
    return true;
}

bool watchdog_reset_system(watchdog_supervision_t* watchdog, reset_strategy_t strategy) {
    if (!watchdog) {
        return false;
    }
    
    printf("SYSTEM RESET REQUESTED: Strategy %s\\n", reset_strategy_to_string(strategy));
    
    switch (strategy) {
        case RESET_STRATEGY_HARD_RESET:
            if (g_hw_interface && g_hw_interface->hw_system_reset) {
                printf("Executing hardware system reset\\n");
                g_hw_interface->hw_system_reset();
            } else {
                printf("Hardware reset not available, performing software reset\\n");
                exit(1); // Software reset
            }
            break;
            
        case RESET_STRATEGY_SHUTDOWN:
            watchdog_emergency_system_shutdown(watchdog);
            break;
            
        default:
            printf("Unsupported system reset strategy\\n");
            return false;
    }
    
    return true;
}

// Internal monitoring functions
void watchdog_monitor_tasks(watchdog_supervision_t* watchdog) {
    if (!watchdog || !watchdog->watchdog_enabled) {
        return;
    }
    
    uint32_t current_time = watchdog_get_current_time_ms();
    bool critical_timeout = false;
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    
    // Check global timeout
    uint32_t global_elapsed = watchdog_elapsed_time_ms(watchdog->last_global_kick_ms, current_time);
    if (global_elapsed > watchdog->global_timeout_ms) {
        printf("CRITICAL: Global watchdog timeout (%u ms)\\n", global_elapsed);
        watchdog->system_status = WATCHDOG_STATUS_CRITICAL_FAULT;
        critical_timeout = true;
    }
    
    // Check individual task timeouts
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        watchdog_task_config_t* config = &watchdog->task_configs[i];
        watchdog_task_runtime_t* runtime = &watchdog->task_runtime[i];
        
        if (!config->enabled) {
            continue;
        }
        
        uint32_t elapsed = watchdog_elapsed_time_ms(runtime->last_kick_time_ms, current_time);
        
        // Check for warning threshold
        if (elapsed > config->warning_threshold_ms && runtime->status == WATCHDOG_STATUS_OK) {
            runtime->status = WATCHDOG_STATUS_WARNING;
            printf("WARNING: Task %s approaching timeout (%u ms)\\n", config->task_name, elapsed);
        }
        
        // Check for timeout
        if (elapsed > config->timeout_ms) {
            if (!runtime->timeout_occurred) {
                runtime->timeout_occurred = true;
                runtime->timeout_count++;
                runtime->status = WATCHDOG_STATUS_TIMEOUT;
                watchdog->total_timeouts++;
                
                watchdog_log_timeout((watchdog_task_id_t)i, elapsed);
                
                if (config->critical_task) {
                    critical_timeout = true;
                    runtime->status = WATCHDOG_STATUS_CRITICAL_FAULT;
                }
            }
        }
    }
    
    // Update system status
    if (critical_timeout) {
        watchdog->system_status = WATCHDOG_STATUS_CRITICAL_FAULT;
    } else {
        // Check if any tasks are in timeout
        bool any_timeout = false;
        for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
            if (watchdog->task_runtime[i].timeout_occurred) {
                any_timeout = true;
                break;
            }
        }
        watchdog->system_status = any_timeout ? WATCHDOG_STATUS_TIMEOUT : WATCHDOG_STATUS_OK;
    }
    
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    // Handle timeouts outside of mutex
    if (critical_timeout) {
        for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
            if (watchdog->task_runtime[i].status == WATCHDOG_STATUS_CRITICAL_FAULT) {
                watchdog_handle_critical_timeout(watchdog, (watchdog_task_id_t)i);
            }
        }
    } else {
        // Handle non-critical timeouts
        for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
            if (watchdog->task_runtime[i].timeout_occurred && 
                watchdog->task_runtime[i].status != WATCHDOG_STATUS_CRITICAL_FAULT) {
                watchdog_handle_timeout(watchdog, (watchdog_task_id_t)i);
            }
        }
    }
}

void watchdog_handle_timeout(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return;
    }
    
    watchdog_task_config_t* config = &watchdog->task_configs[task_id];
    
    printf("Handling timeout for task %s\\n", config->task_name);
    
    // Execute recovery based on configured strategy
    watchdog_trigger_recovery(watchdog, task_id, config->reset_strategy);
}

void watchdog_execute_recovery(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return;
    }
    
    watchdog_task_config_t* config = &watchdog->task_configs[task_id];
    
    printf("Executing recovery for task %s\\n", config->task_name);
    
    // Reset task state
    watchdog_reset_task(watchdog, task_id);
    
    // Additional recovery actions can be added here
}

void watchdog_update_statistics(watchdog_supervision_t* watchdog) {
    if (!watchdog) {
        return;
    }
    
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    watchdog->uptime_ms = watchdog_get_system_uptime(watchdog);
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
}

// Hardware watchdog integration
bool watchdog_enable_hardware(watchdog_supervision_t* watchdog, uint32_t timeout_ms) {
    if (!watchdog || !g_hw_interface || !g_hw_interface->hw_watchdog_init) {
        return false;
    }
    
    watchdog->hardware_watchdog_enabled = g_hw_interface->hw_watchdog_init(timeout_ms);
    watchdog->hardware_watchdog_timeout_ms = timeout_ms;
    
    if (watchdog->hardware_watchdog_enabled) {
        printf("Hardware watchdog enabled with %u ms timeout\\n", timeout_ms);
    }
    
    return watchdog->hardware_watchdog_enabled;
}

void watchdog_disable_hardware(watchdog_supervision_t* watchdog) {
    if (!watchdog || !watchdog->hardware_watchdog_enabled) {
        return;
    }
    
    if (g_hw_interface && g_hw_interface->hw_watchdog_disable) {
        g_hw_interface->hw_watchdog_disable();
    }
    
    watchdog->hardware_watchdog_enabled = false;
    printf("Hardware watchdog disabled\\n");
}

void watchdog_kick_hardware(watchdog_supervision_t* watchdog) {
    if (!watchdog || !watchdog->hardware_watchdog_enabled) {
        return;
    }
    
    if (g_hw_interface && g_hw_interface->hw_watchdog_kick) {
        g_hw_interface->hw_watchdog_kick();
    }
}

// Utility and helper functions
uint32_t watchdog_get_current_time_ms(void) {
    if (g_hw_interface && g_hw_interface->get_system_time_ms) {
        return g_hw_interface->get_system_time_ms();
    }
    
    // Fallback to system clock
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

bool watchdog_validate_config(const watchdog_task_config_t* config) {
    if (!config) {
        return false;
    }
    
    if (config->timeout_ms == 0 || config->timeout_ms > 60000) { // Max 1 minute timeout
        return false;
    }
    
    if (config->warning_threshold_ms >= config->timeout_ms) {
        return false;
    }
    
    if (config->task_id >= WATCHDOG_TASK_COUNT) {
        return false;
    }
    
    return true;
}

void watchdog_calculate_task_statistics(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) {
        return;
    }
    
    // Statistics are updated in real-time during kicks
    // This function could be extended for more complex statistics
}

// Internal helper functions
static void* watchdog_monitoring_thread(void* arg) {
    watchdog_supervision_t* watchdog = (watchdog_supervision_t*)arg;
    uint32_t monitoring_interval_ms = 1000 / watchdog->monitoring_frequency_hz;
    
    printf("Watchdog monitoring thread started (interval: %u ms)\\n", monitoring_interval_ms);
    
    while (watchdog->monitoring_active) {
        // Monitor all tasks
        watchdog_monitor_tasks(watchdog);
        
        // Update statistics
        watchdog_update_statistics(watchdog);
        
        // Check for safe state timeout
        if (watchdog->safe_state_active) {
            uint32_t safe_state_elapsed = watchdog_elapsed_time_ms(
                watchdog->safe_state_entry_time_ms, watchdog_get_current_time_ms());
            
            if (safe_state_elapsed > WATCHDOG_SAFE_STATE_TIMEOUT_MS) {
                printf("Safe state timeout exceeded, initiating emergency shutdown\\n");
                watchdog_emergency_system_shutdown(watchdog);
                break;
            }
        }
        
        usleep(monitoring_interval_ms * 1000); // Convert to microseconds
    }
    
    printf("Watchdog monitoring thread terminated\\n");
    return NULL;
}

static void watchdog_handle_critical_timeout(watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    printf("CRITICAL TIMEOUT: Task %s has exceeded critical timeout threshold\\n", 
           watchdog_task_name(task_id));
    
    // For critical timeouts, immediately enter safe state
    watchdog_enter_safe_state(watchdog, SAFE_STATE_LEVEL_3, "Critical task timeout");
}

static void watchdog_execute_safe_state_transition(watchdog_supervision_t* watchdog, safe_state_level_t level) {
    printf("Executing safe state transition to level %s\\n", safe_state_level_to_string(level));
    
    // Call hardware interface for safe mode entry
    if (g_hw_interface && g_hw_interface->enter_safe_mode) {
        g_hw_interface->enter_safe_mode(level);
    }
    
    // Execute safe state actions based on level
    switch (level) {
        case SAFE_STATE_LEVEL_1:
            // Minimal safe operation - basic brake functionality only
            printf("Safe state: Minimal operation mode\\n");
            break;
            
        case SAFE_STATE_LEVEL_2:
            // Reduced functionality
            printf("Safe state: Reduced functionality mode\\n");
            break;
            
        case SAFE_STATE_LEVEL_3:
            // Diagnostic mode
            printf("Safe state: Diagnostic mode\\n");
            break;
            
        case SAFE_STATE_LEVEL_4:
            // Complete shutdown preparation
            printf("Safe state: Shutdown preparation\\n");
            watchdog_emergency_system_shutdown(watchdog);
            break;
    }
}

static void watchdog_emergency_system_shutdown(watchdog_supervision_t* watchdog) {
    printf("EMERGENCY SYSTEM SHUTDOWN INITIATED\\n");
    
    if (g_hw_interface && g_hw_interface->emergency_shutdown) {
        g_hw_interface->emergency_shutdown();
    }
    
    // Set system to critical fault state
    pthread_mutex_lock(&watchdog->watchdog_mutex);
    watchdog->system_status = WATCHDOG_STATUS_CRITICAL_FAULT;
    pthread_mutex_unlock(&watchdog->watchdog_mutex);
    
    // In a real system, this would initiate controlled shutdown
    printf("System entering emergency shutdown state\\n");
}

static void watchdog_signal_handler(int signal) {
    printf("Watchdog received signal %d, initiating clean shutdown\\n", signal);
    // In a real implementation, this would signal the main watchdog to shutdown gracefully
}

// Diagnostic and reporting functions
void watchdog_print_system_status(const watchdog_supervision_t* watchdog) {
    if (!watchdog) return;
    
    printf("\\n=== Watchdog System Status ===\\n");
    printf("System Status: %s\\n", watchdog_status_to_string(watchdog->system_status));
    printf("Watchdog Enabled: %s\\n", watchdog->watchdog_enabled ? "YES" : "NO");
    printf("Hardware Watchdog: %s\\n", watchdog->hardware_watchdog_enabled ? "ENABLED" : "DISABLED");
    printf("Safe State Active: %s\\n", watchdog->safe_state_active ? "YES" : "NO");
    
    if (watchdog->safe_state_active) {
        printf("Current Safe State: %s\\n", safe_state_level_to_string(watchdog->current_safe_state));
        printf("Safe State Reason: %s\\n", watchdog->safe_state_config.safe_state_reason);
    }
    
    printf("System Uptime: %u ms\\n", watchdog_get_system_uptime(watchdog));
    printf("Recovery Mode: %s\\n", watchdog->in_recovery_mode ? "ACTIVE" : "INACTIVE");
    printf("==============================\\n");
}

void watchdog_print_task_status(const watchdog_supervision_t* watchdog, watchdog_task_id_t task_id) {
    if (!watchdog || task_id >= WATCHDOG_TASK_COUNT) return;
    
    const watchdog_task_config_t* config = &watchdog->task_configs[task_id];
    const watchdog_task_runtime_t* runtime = &watchdog->task_runtime[task_id];
    
    printf("\\n=== Task %s Status ===\\n", config->task_name);
    printf("Enabled: %s\\n", config->enabled ? "YES" : "NO");
    printf("Critical: %s\\n", config->critical_task ? "YES" : "NO");
    printf("Status: %s\\n", watchdog_status_to_string(runtime->status));
    printf("Timeout: %u ms\\n", config->timeout_ms);
    printf("Reset Strategy: %s\\n", reset_strategy_to_string(config->reset_strategy));
    printf("Kick Count: %u\\n", runtime->kick_count);
    printf("Timeout Count: %u\\n", runtime->timeout_count);
    printf("Max Execution Time: %u ms\\n", runtime->max_execution_time_ms);
    printf("Avg Execution Time: %u ms\\n", runtime->avg_execution_time_ms);
    printf("Recovery Attempts: %u\\n", watchdog->recovery_attempts[task_id]);
    
    uint32_t last_kick_elapsed = watchdog_elapsed_time_ms(runtime->last_kick_time_ms, watchdog_get_current_time_ms());
    printf("Last Kick: %u ms ago\\n", last_kick_elapsed);
    printf("==========================\\n");
}

void watchdog_print_statistics(const watchdog_supervision_t* watchdog) {
    if (!watchdog) return;
    
    printf("\\n=== Watchdog Statistics ===\\n");
    printf("Total Kicks: %u\\n", watchdog->total_kicks);
    printf("Total Timeouts: %u\\n", watchdog->total_timeouts);
    printf("Total Recoveries: %u\\n", watchdog->total_recoveries);
    printf("Safe State Entries: %u\\n", watchdog->total_safe_state_entries);
    printf("System Uptime: %u ms\\n", watchdog->uptime_ms);
    
    // Calculate uptime percentage for each task
    printf("\\nTask Statistics:\\n");
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        const watchdog_task_runtime_t* runtime = &watchdog->task_runtime[i];
        if (runtime->kick_count > 0) {
            printf("  %s: %u kicks, %u timeouts\\n", 
                   watchdog_task_name((watchdog_task_id_t)i), 
                   runtime->kick_count, runtime->timeout_count);
        }
    }
    printf("===========================\\n");
}

void watchdog_log_timeout(watchdog_task_id_t task_id, uint32_t elapsed_ms) {
    printf("WATCHDOG TIMEOUT: Task %s exceeded timeout by %u ms\\n", 
           watchdog_task_name(task_id), elapsed_ms);
}

void watchdog_log_recovery(watchdog_task_id_t task_id, reset_strategy_t strategy, bool success) {
    printf("WATCHDOG RECOVERY: Task %s, Strategy %s, Result: %s\\n",
           watchdog_task_name(task_id), reset_strategy_to_string(strategy),
           success ? "SUCCESS" : "FAILED");
}

void watchdog_log_safe_state_entry(safe_state_level_t level, const char* reason) {
    printf("WATCHDOG SAFE STATE: Entered level %s, Reason: %s\\n",
           safe_state_level_to_string(level), reason ? reason : "Unknown");
}

// String conversion functions
const char* watchdog_status_to_string(watchdog_status_t status) {
    switch (status) {
        case WATCHDOG_STATUS_OK: return "OK";
        case WATCHDOG_STATUS_WARNING: return "WARNING";
        case WATCHDOG_STATUS_TIMEOUT: return "TIMEOUT";
        case WATCHDOG_STATUS_CRITICAL_FAULT: return "CRITICAL_FAULT";
        case WATCHDOG_STATUS_RECOVERY: return "RECOVERY";
        case WATCHDOG_STATUS_SAFE_STATE: return "SAFE_STATE";
        default: return "UNKNOWN";
    }
}

const char* watchdog_task_name(watchdog_task_id_t task_id) {
    switch (task_id) {
        case WATCHDOG_TASK_MAIN_LOOP: return "MAIN_LOOP";
        case WATCHDOG_TASK_SENSOR_ACQUISITION: return "SENSOR_ACQUISITION";
        case WATCHDOG_TASK_CONTROL_ALGORITHM: return "CONTROL_ALGORITHM";
        case WATCHDOG_TASK_BRAKE_ACTUATION: return "BRAKE_ACTUATION";
        case WATCHDOG_TASK_ENGINE_INTERFACE: return "ENGINE_INTERFACE";
        case WATCHDOG_TASK_DIAGNOSTICS: return "DIAGNOSTICS";
        case WATCHDOG_TASK_COMMUNICATION: return "COMMUNICATION";
        case WATCHDOG_TASK_PLAUSIBILITY: return "PLAUSIBILITY";
        default: return "UNKNOWN_TASK";
    }
}

const char* reset_strategy_to_string(reset_strategy_t strategy) {
    switch (strategy) {
        case RESET_STRATEGY_NONE: return "NONE";
        case RESET_STRATEGY_SOFT_RESET: return "SOFT_RESET";
        case RESET_STRATEGY_MODULE_RESTART: return "MODULE_RESTART";
        case RESET_STRATEGY_SAFE_STATE: return "SAFE_STATE";
        case RESET_STRATEGY_HARD_RESET: return "HARD_RESET";
        case RESET_STRATEGY_SHUTDOWN: return "SHUTDOWN";
        default: return "UNKNOWN";
    }
}

const char* safe_state_level_to_string(safe_state_level_t level) {
    switch (level) {
        case SAFE_STATE_LEVEL_1: return "LEVEL_1_MINIMAL";
        case SAFE_STATE_LEVEL_2: return "LEVEL_2_REDUCED";
        case SAFE_STATE_LEVEL_3: return "LEVEL_3_DIAGNOSTIC";
        case SAFE_STATE_LEVEL_4: return "LEVEL_4_SHUTDOWN";
        default: return "UNKNOWN_LEVEL";
    }
}

// Default configurations
watchdog_task_config_t watchdog_get_default_task_config(watchdog_task_id_t task_id) {
    watchdog_task_config_t config = {0};
    
    config.task_id = task_id;
    config.task_name = watchdog_task_name(task_id);
    config.enabled = true;
    
    switch (task_id) {
        case WATCHDOG_TASK_MAIN_LOOP:
            config.timeout_type = WATCHDOG_TIMEOUT_CRITICAL;
            config.timeout_ms = 20;         // 20ms for main loop
            config.warning_threshold_ms = 10;
            config.reset_strategy = RESET_STRATEGY_SAFE_STATE;
            config.critical_task = true;
            break;
            
        case WATCHDOG_TASK_SENSOR_ACQUISITION:
            config.timeout_type = WATCHDOG_TIMEOUT_CRITICAL;
            config.timeout_ms = 15;         // 15ms for sensor acquisition
            config.warning_threshold_ms = 8;
            config.reset_strategy = RESET_STRATEGY_MODULE_RESTART;
            config.critical_task = true;
            break;
            
        case WATCHDOG_TASK_CONTROL_ALGORITHM:
            config.timeout_type = WATCHDOG_TIMEOUT_CRITICAL;
            config.timeout_ms = 10;         // 10ms for control algorithm
            config.warning_threshold_ms = 5;
            config.reset_strategy = RESET_STRATEGY_SAFE_STATE;
            config.critical_task = true;
            break;
            
        case WATCHDOG_TASK_BRAKE_ACTUATION:
            config.timeout_type = WATCHDOG_TIMEOUT_CRITICAL;
            config.timeout_ms = 25;         // 25ms for brake actuation
            config.warning_threshold_ms = 12;
            config.reset_strategy = RESET_STRATEGY_MODULE_RESTART;
            config.critical_task = true;
            break;
            
        case WATCHDOG_TASK_ENGINE_INTERFACE:
            config.timeout_type = WATCHDOG_TIMEOUT_NORMAL;
            config.timeout_ms = 100;        // 100ms for engine interface
            config.warning_threshold_ms = 50;
            config.reset_strategy = RESET_STRATEGY_SOFT_RESET;
            config.critical_task = false;
            break;
            
        case WATCHDOG_TASK_DIAGNOSTICS:
            config.timeout_type = WATCHDOG_TIMEOUT_DIAGNOSTIC;
            config.timeout_ms = 500;        // 500ms for diagnostics
            config.warning_threshold_ms = 250;
            config.reset_strategy = RESET_STRATEGY_SOFT_RESET;
            config.critical_task = false;
            break;
            
        case WATCHDOG_TASK_COMMUNICATION:
            config.timeout_type = WATCHDOG_TIMEOUT_COMMUNICATION;
            config.timeout_ms = 1000;       // 1000ms for communication
            config.warning_threshold_ms = 500;
            config.reset_strategy = RESET_STRATEGY_MODULE_RESTART;
            config.critical_task = false;
            break;
            
        case WATCHDOG_TASK_PLAUSIBILITY:
            config.timeout_type = WATCHDOG_TIMEOUT_NORMAL;
            config.timeout_ms = 50;         // 50ms for plausibility
            config.warning_threshold_ms = 25;
            config.reset_strategy = RESET_STRATEGY_MODULE_RESTART;
            config.critical_task = true;
            break;
            
        default:
            config.timeout_ms = 100;
            config.warning_threshold_ms = 50;
            config.reset_strategy = RESET_STRATEGY_SOFT_RESET;
            config.critical_task = false;
            break;
    }
    
    return config;
}

recovery_config_t watchdog_get_default_recovery_config(void) {
    recovery_config_t config = {
        .strategy = RESET_STRATEGY_SOFT_RESET,
        .recovery_timeout_ms = 5000,        // 5 second recovery timeout
        .max_recovery_attempts = WATCHDOG_MAX_RECOVERY_ATTEMPTS,
        .require_manual_reset = false,
        .recovery_callback = NULL,
        .safe_state_callback = NULL
    };
    
    return config;
}

safe_state_config_t watchdog_get_default_safe_state_config(safe_state_level_t level) {
    safe_state_config_t config = {0};
    
    config.level = level;
    config.safe_state_timeout_ms = WATCHDOG_SAFE_STATE_TIMEOUT_MS;
    
    switch (level) {
        case SAFE_STATE_LEVEL_1:
            config.disable_engine_torque = false;
            config.enable_emergency_brake = false;
            config.emergency_brake_pressure = 0.0f;
            config.disable_esc_intervention = false;
            config.enable_limp_mode = false;
            config.shutdown_non_critical_systems = false;
            config.safe_state_reason = "Minimal safe operation";
            break;
            
        case SAFE_STATE_LEVEL_2:
            config.disable_engine_torque = true;
            config.enable_emergency_brake = true;
            config.emergency_brake_pressure = 30.0f;
            config.disable_esc_intervention = false;
            config.enable_limp_mode = true;
            config.shutdown_non_critical_systems = true;
            config.safe_state_reason = "Reduced functionality";
            break;
            
        case SAFE_STATE_LEVEL_3:
            config.disable_engine_torque = true;
            config.enable_emergency_brake = true;
            config.emergency_brake_pressure = 50.0f;
            config.disable_esc_intervention = true;
            config.enable_limp_mode = true;
            config.shutdown_non_critical_systems = true;
            config.safe_state_reason = "Diagnostic mode";
            break;
            
        case SAFE_STATE_LEVEL_4:
            config.disable_engine_torque = true;
            config.enable_emergency_brake = true;
            config.emergency_brake_pressure = 80.0f;
            config.disable_esc_intervention = true;
            config.enable_limp_mode = true;
            config.shutdown_non_critical_systems = true;
            config.safe_state_reason = "Shutdown preparation";
            break;
    }
    
    return config;
}