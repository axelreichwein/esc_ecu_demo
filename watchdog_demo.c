#include "watchdog_supervision.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

// Demo hardware interface functions
static bool demo_watchdog_hw_init(uint32_t timeout_ms) {
    printf("Demo hardware watchdog initialized with %u ms timeout\n", timeout_ms);
    return true;
}

static void demo_watchdog_hw_kick(void) {
    static uint32_t kick_count = 0;
    kick_count++;
    if (kick_count % 50 == 0) {
        printf("Hardware watchdog kicked (%u times)\n", kick_count);
    }
}

static void demo_watchdog_hw_disable(void) {
    printf("Hardware watchdog disabled\n");
}

static void demo_watchdog_system_reset(void) {
    printf("SYSTEM RESET: Hardware reset requested\n");
    exit(2);
}

static uint32_t demo_get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void demo_enter_safe_mode(safe_state_level_t level) {
    printf("ENTERING SAFE MODE: Level %s\n", safe_state_level_to_string(level));
    
    switch (level) {
        case SAFE_STATE_LEVEL_1:
            printf("Demo: Minimal safe operation mode activated\n");
            break;
        case SAFE_STATE_LEVEL_2:
            printf("Demo: Reduced functionality mode activated\n");
            break;
        case SAFE_STATE_LEVEL_3:
            printf("Demo: Diagnostic mode activated\n");
            break;
        case SAFE_STATE_LEVEL_4:
            printf("Demo: Emergency shutdown preparation\n");
            break;
    }
}

static void demo_emergency_shutdown(void) {
    printf("EMERGENCY SHUTDOWN: System shutdown initiated\n");
    exit(3);
}

static const watchdog_hw_interface_t demo_hw_interface = {
    .hw_watchdog_init = demo_watchdog_hw_init,
    .hw_watchdog_kick = demo_watchdog_hw_kick,
    .hw_watchdog_disable = demo_watchdog_hw_disable,
    .hw_system_reset = demo_watchdog_system_reset,
    .get_system_time_ms = demo_get_time_ms,
    .enter_safe_mode = demo_enter_safe_mode,
    .emergency_shutdown = demo_emergency_shutdown
};

// Demo recovery callback
static void demo_recovery_callback(watchdog_task_id_t task_id) {
    printf("RECOVERY CALLBACK: Attempting recovery for task %s\n", watchdog_task_name(task_id));
    
    switch (task_id) {
        case WATCHDOG_TASK_SENSOR_ACQUISITION:
            printf("Recovery: Reinitializing sensor subsystem\n");
            break;
        case WATCHDOG_TASK_CONTROL_ALGORITHM:
            printf("Recovery: Resetting control algorithm state\n");
            break;
        case WATCHDOG_TASK_BRAKE_ACTUATION:
            printf("Recovery: Resetting brake system\n");
            break;
        default:
            printf("Recovery: Generic system reset\n");
            break;
    }
}

// Demo safe state callback
static void demo_safe_state_callback(safe_state_level_t level, const char* reason) {
    printf("SAFE STATE CALLBACK: Level %s, Reason: %s\n", 
           safe_state_level_to_string(level), reason ? reason : "Unknown");
}

void demo_basic_watchdog_operation(void) {
    printf("\n=== Basic Watchdog Operation Demo ===\n");
    
    watchdog_supervision_t watchdog;
    
    // Initialize watchdog system
    if (!watchdog_init(&watchdog, &demo_hw_interface)) {
        printf("Failed to initialize watchdog system\n");
        return;
    }
    
    // Configure recovery callbacks
    recovery_config_t recovery_config = watchdog_get_default_recovery_config();
    recovery_config.recovery_callback = demo_recovery_callback;
    recovery_config.safe_state_callback = demo_safe_state_callback;
    watchdog_configure_recovery(&watchdog, &recovery_config);
    
    printf("\nDemonstrating normal watchdog operation:\n");
    
    // Simulate normal task execution with proper kicks
    for (int cycle = 0; cycle < 20; cycle++) {
        printf("Cycle %d: ", cycle + 1);
        
        // Kick main loop task
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_MAIN_LOOP);
        usleep(5000); // 5ms simulated work
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_MAIN_LOOP);
        
        // Kick sensor acquisition task
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
        usleep(3000); // 3ms simulated work
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
        
        // Global kick
        WATCHDOG_GLOBAL_KICK(&watchdog);
        
        printf("All tasks completed successfully\n");
        usleep(10000); // 10ms cycle time
    }
    
    watchdog_print_system_status(&watchdog);
    watchdog_shutdown(&watchdog);
}

void demo_timeout_detection_and_recovery(void) {
    printf("\n=== Timeout Detection and Recovery Demo ===\n");
    
    watchdog_supervision_t watchdog;
    
    if (!watchdog_init(&watchdog, &demo_hw_interface)) {
        printf("Failed to initialize watchdog system\n");
        return;
    }
    
    // Configure recovery callbacks
    recovery_config_t recovery_config = watchdog_get_default_recovery_config();
    recovery_config.recovery_callback = demo_recovery_callback;
    recovery_config.safe_state_callback = demo_safe_state_callback;
    recovery_config.max_recovery_attempts = 2;
    watchdog_configure_recovery(&watchdog, &recovery_config);
    
    // Configure a task with short timeout for demo
    watchdog_task_config_t sensor_config = watchdog_get_default_task_config(WATCHDOG_TASK_SENSOR_ACQUISITION);
    sensor_config.timeout_ms = 50;  // Very short timeout
    sensor_config.warning_threshold_ms = 25;
    sensor_config.reset_strategy = RESET_STRATEGY_MODULE_RESTART;
    watchdog_configure_task(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION, &sensor_config);
    
    printf("\nSimulating task timeout scenario:\n");
    
    // Normal operation first
    for (int i = 0; i < 5; i++) {
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
        usleep(20000); // 20ms - within timeout
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
        printf("Cycle %d: Normal operation\n", i + 1);
        usleep(30000);
    }
    
    // Now simulate a timeout
    printf("\nSimulating sensor acquisition timeout...\n");
    WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
    // Don't kick the watchdog - simulate hung task
    usleep(100000); // 100ms - exceeds timeout
    
    // Check status
    watchdog_status_t status = watchdog_get_task_status(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
    printf("Task status after timeout: %s\n", watchdog_status_to_string(status));
    
    // Resume normal operation
    printf("\nResuming normal operation:\n");
    for (int i = 0; i < 3; i++) {
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
        usleep(20000); // Back to normal timing
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
        printf("Recovery cycle %d: Normal operation\n", i + 1);
        usleep(30000);
    }
    
    watchdog_print_statistics(&watchdog);
    watchdog_shutdown(&watchdog);
}

void demo_safe_state_transition(void) {
    printf("\n=== Safe State Transition Demo ===\n");
    
    watchdog_supervision_t watchdog;
    
    if (!watchdog_init(&watchdog, &demo_hw_interface)) {
        printf("Failed to initialize watchdog system\n");
        return;
    }
    
    // Configure recovery callbacks
    recovery_config_t recovery_config = watchdog_get_default_recovery_config();
    recovery_config.recovery_callback = demo_recovery_callback;
    recovery_config.safe_state_callback = demo_safe_state_callback;
    recovery_config.max_recovery_attempts = 1; // Only 1 attempt before safe state
    watchdog_configure_recovery(&watchdog, &recovery_config);
    
    // Configure critical task with aggressive timeout
    watchdog_task_config_t control_config = watchdog_get_default_task_config(WATCHDOG_TASK_CONTROL_ALGORITHM);
    control_config.timeout_ms = 30;  // Short timeout
    control_config.reset_strategy = RESET_STRATEGY_SAFE_STATE;
    control_config.critical_task = true;
    watchdog_configure_task(&watchdog, WATCHDOG_TASK_CONTROL_ALGORITHM, &control_config);
    
    // Configure safe state
    safe_state_config_t safe_state = watchdog_get_default_safe_state_config(SAFE_STATE_LEVEL_2);
    safe_state.enable_emergency_brake = true;
    safe_state.emergency_brake_pressure = 50.0f;
    safe_state.disable_engine_torque = true;
    watchdog_configure_safe_state(&watchdog, &safe_state);
    
    printf("\nSimulating critical task failure leading to safe state:\n");
    
    // Normal operation first
    for (int i = 0; i < 3; i++) {
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_CONTROL_ALGORITHM);
        usleep(15000); // 15ms - within timeout
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_CONTROL_ALGORITHM);
        printf("Cycle %d: Normal control operation\n", i + 1);
        usleep(25000);
    }
    
    // Simulate critical failure
    printf("\nSimulating critical control algorithm failure...\n");
    WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_CONTROL_ALGORITHM);
    // Don't kick - simulate complete failure
    usleep(80000); // 80ms - well beyond timeout
    
    // Check if we're in safe state
    if (watchdog.safe_state_active) {
        printf("System successfully entered safe state: %s\n", 
               safe_state_level_to_string(watchdog.current_safe_state));
    }
    
    // Try to exit safe state
    printf("\nAttempting to exit safe state...\n");
    if (watchdog_exit_safe_state(&watchdog)) {
        printf("Successfully exited safe state\n");
    }
    
    watchdog_print_system_status(&watchdog);
    watchdog_shutdown(&watchdog);
}

void demo_multiple_task_monitoring(void) {
    printf("\n=== Multiple Task Monitoring Demo ===\n");
    
    watchdog_supervision_t watchdog;
    
    if (!watchdog_init(&watchdog, &demo_hw_interface)) {
        printf("Failed to initialize watchdog system\n");
        return;
    }
    
    // Configure recovery callbacks
    recovery_config_t recovery_config = watchdog_get_default_recovery_config();
    recovery_config.recovery_callback = demo_recovery_callback;
    recovery_config.safe_state_callback = demo_safe_state_callback;
    watchdog_configure_recovery(&watchdog, &recovery_config);
    
    printf("\nMonitoring multiple tasks simultaneously:\n");
    
    // Simulate realistic ESC operation with multiple tasks
    for (int cycle = 0; cycle < 15; cycle++) {
        printf("Cycle %d: ", cycle + 1);
        
        // Main loop task
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_MAIN_LOOP);
        
        // Sensor acquisition
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
        usleep(2000); // 2ms sensor work
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_SENSOR_ACQUISITION);
        
        // Control algorithm
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_CONTROL_ALGORITHM);
        usleep(3000); // 3ms control work
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_CONTROL_ALGORITHM);
        
        // Brake actuation
        WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_BRAKE_ACTUATION);
        usleep(4000); // 4ms brake work
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_BRAKE_ACTUATION);
        
        // Diagnostics (less frequent)
        if (cycle % 3 == 0) {
            WATCHDOG_TASK_START(&watchdog, WATCHDOG_TASK_DIAGNOSTICS);
            usleep(8000); // 8ms diagnostic work
            WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_DIAGNOSTICS);
        }
        
        WATCHDOG_TASK_END(&watchdog, WATCHDOG_TASK_MAIN_LOOP);
        WATCHDOG_GLOBAL_KICK(&watchdog);
        
        printf("All tasks completed\n");
        usleep(5000); // 5ms remaining cycle time
    }
    
    // Print individual task statistics
    printf("\nTask-specific status:\n");
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        watchdog_task_id_t task_id = (watchdog_task_id_t)i;
        if (watchdog.task_configs[i].enabled) {
            watchdog_print_task_status(&watchdog, task_id);
        }
    }
    
    watchdog_shutdown(&watchdog);
}

int main(void) {
    printf("Watchdog Supervision System Demonstration\n");
    printf("=========================================\n");
    
    demo_basic_watchdog_operation();
    demo_timeout_detection_and_recovery();
    demo_safe_state_transition();
    demo_multiple_task_monitoring();
    
    printf("\nWatchdog supervision demonstration completed.\n");
    return 0;
}