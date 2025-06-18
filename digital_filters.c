#include "sensor_interface.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Digital filter implementation functions

bool filter_init(digital_filter_t* filter, const sensor_hw_spec_t* hw_spec) {
    if (!filter || !hw_spec) {
        return false;
    }
    
    memset(filter, 0, sizeof(digital_filter_t));
    filter->active_filter = hw_spec->filter_type;
    
    switch (hw_spec->filter_type) {
        case FILTER_NONE:
            // No initialization needed
            break;
            
        case FILTER_LOW_PASS:
            // Initialize low-pass filter
            if (hw_spec->filter_cutoff_hz > 0 && hw_spec->sample_rate_hz > 0) {
                float dt = 1.0f / hw_spec->sample_rate_hz;
                float rc = 1.0f / (2.0f * M_PI * hw_spec->filter_cutoff_hz);
                filter->low_pass.alpha = dt / (rc + dt);
                filter->low_pass.previous_output = 0.0f;
            } else {
                return false;
            }
            break;
            
        case FILTER_MEDIAN:
            // Initialize median filter
            if (hw_spec->median_window_size > 0 && hw_spec->median_window_size <= 32) {
                filter->median.size = hw_spec->median_window_size;
                filter->median.buffer = (float*)calloc(filter->median.size, sizeof(float));
                if (!filter->median.buffer) {
                    return false;
                }
                filter->median.index = 0;
                filter->median.buffer_full = false;
            } else {
                return false;
            }
            break;
            
        case FILTER_MOVING_AVERAGE:
            // Initialize moving average filter
            if (hw_spec->avg_window_size > 0 && hw_spec->avg_window_size <= 64) {
                filter->moving_avg.size = hw_spec->avg_window_size;
                filter->moving_avg.buffer = (float*)calloc(filter->moving_avg.size, sizeof(float));
                if (!filter->moving_avg.buffer) {
                    return false;
                }
                filter->moving_avg.index = 0;
                filter->moving_avg.sum = 0.0f;
                filter->moving_avg.buffer_full = false;
            } else {
                return false;
            }
            break;
            
        case FILTER_DEBOUNCE:
            // Initialize debounce filter
            filter->debounce.current_state = false;
            filter->debounce.debounced_state = false;
            filter->debounce.last_change_time = 0;
            filter->debounce.debounce_time_ms = hw_spec->debounce_time_ms;
            break;
            
        case FILTER_COMBINED:
            // Initialize multiple filters in sequence
            // First initialize low-pass
            if (hw_spec->filter_cutoff_hz > 0 && hw_spec->sample_rate_hz > 0) {
                float dt = 1.0f / hw_spec->sample_rate_hz;
                float rc = 1.0f / (2.0f * M_PI * hw_spec->filter_cutoff_hz);
                filter->low_pass.alpha = dt / (rc + dt);
                filter->low_pass.previous_output = 0.0f;
            }
            
            // Then initialize median filter
            if (hw_spec->median_window_size > 0 && hw_spec->median_window_size <= 32) {
                filter->median.size = hw_spec->median_window_size;
                filter->median.buffer = (float*)calloc(filter->median.size, sizeof(float));
                if (!filter->median.buffer) {
                    return false;
                }
                filter->median.index = 0;
                filter->median.buffer_full = false;
            }
            break;
            
        default:
            return false;
    }
    
    return true;
}

void filter_cleanup(digital_filter_t* filter) {
    if (!filter) {
        return;
    }
    
    // Free allocated memory
    if (filter->median.buffer) {
        free(filter->median.buffer);
        filter->median.buffer = NULL;
    }
    
    if (filter->moving_avg.buffer) {
        free(filter->moving_avg.buffer);
        filter->moving_avg.buffer = NULL;
    }
    
    memset(filter, 0, sizeof(digital_filter_t));
}

float filter_process(digital_filter_t* filter, float input_value, uint32_t timestamp_us) {
    if (!filter) {
        return input_value;
    }
    
    float output = input_value;
    
    switch (filter->active_filter) {
        case FILTER_NONE:
            output = input_value;
            break;
            
        case FILTER_LOW_PASS:
            output = filter_low_pass_process(&filter->low_pass, input_value);
            break;
            
        case FILTER_MEDIAN:
            output = filter_median_process(&filter->median, input_value);
            break;
            
        case FILTER_MOVING_AVERAGE:
            output = filter_moving_avg_process(&filter->moving_avg, input_value);
            break;
            
        case FILTER_DEBOUNCE:
            output = filter_debounce_process(&filter->debounce, input_value, timestamp_us);
            break;
            
        case FILTER_COMBINED:
            // Apply filters in sequence: median -> low-pass
            output = filter_median_process(&filter->median, input_value);
            output = filter_low_pass_process(&filter->low_pass, output);
            break;
            
        default:
            output = input_value;
            break;
    }
    
    return output;
}

void filter_reset(digital_filter_t* filter) {
    if (!filter) {
        return;
    }
    
    switch (filter->active_filter) {
        case FILTER_LOW_PASS:
            filter->low_pass.previous_output = 0.0f;
            break;
            
        case FILTER_MEDIAN:
            if (filter->median.buffer) {
                memset(filter->median.buffer, 0, filter->median.size * sizeof(float));
                filter->median.index = 0;
                filter->median.buffer_full = false;
            }
            break;
            
        case FILTER_MOVING_AVERAGE:
            if (filter->moving_avg.buffer) {
                memset(filter->moving_avg.buffer, 0, filter->moving_avg.size * sizeof(float));
                filter->moving_avg.index = 0;
                filter->moving_avg.sum = 0.0f;
                filter->moving_avg.buffer_full = false;
            }
            break;
            
        case FILTER_DEBOUNCE:
            filter->debounce.current_state = false;
            filter->debounce.debounced_state = false;
            filter->debounce.last_change_time = 0;
            break;
            
        case FILTER_COMBINED:
            // Reset all active sub-filters
            filter->low_pass.previous_output = 0.0f;
            if (filter->median.buffer) {
                memset(filter->median.buffer, 0, filter->median.size * sizeof(float));
                filter->median.index = 0;
                filter->median.buffer_full = false;
            }
            break;
            
        default:
            break;
    }
}

// Individual filter implementations

float filter_low_pass_process(low_pass_filter_t* lp_filter, float input) {
    if (!lp_filter) {
        return input;
    }
    
    // First-order low-pass filter: y[n] = α * x[n] + (1-α) * y[n-1]
    float output = lp_filter->alpha * input + (1.0f - lp_filter->alpha) * lp_filter->previous_output;
    lp_filter->previous_output = output;
    
    return output;
}

float filter_median_process(median_filter_t* med_filter, float input) {
    if (!med_filter || !med_filter->buffer) {
        return input;
    }
    
    // Add new value to circular buffer
    med_filter->buffer[med_filter->index] = input;
    med_filter->index = (med_filter->index + 1) % med_filter->size;
    
    if (!med_filter->buffer_full && med_filter->index == 0) {
        med_filter->buffer_full = true;
    }
    
    // Create working copy for sorting
    float working_buffer[32]; // Maximum window size
    uint8_t samples_to_process = med_filter->buffer_full ? med_filter->size : med_filter->index;
    
    if (samples_to_process == 0) {
        return input;
    }
    
    memcpy(working_buffer, med_filter->buffer, samples_to_process * sizeof(float));
    
    // Simple bubble sort for median calculation
    for (int i = 0; i < samples_to_process - 1; i++) {
        for (int j = 0; j < samples_to_process - i - 1; j++) {
            if (working_buffer[j] > working_buffer[j + 1]) {
                float temp = working_buffer[j];
                working_buffer[j] = working_buffer[j + 1];
                working_buffer[j + 1] = temp;
            }
        }
    }
    
    // Return median value
    if (samples_to_process % 2 == 1) {
        return working_buffer[samples_to_process / 2];
    } else {
        return (working_buffer[samples_to_process / 2 - 1] + working_buffer[samples_to_process / 2]) / 2.0f;
    }
}

float filter_moving_avg_process(moving_avg_filter_t* avg_filter, float input) {
    if (!avg_filter || !avg_filter->buffer) {
        return input;
    }
    
    // Remove old value from sum if buffer is full
    if (avg_filter->buffer_full) {
        avg_filter->sum -= avg_filter->buffer[avg_filter->index];
    }
    
    // Add new value
    avg_filter->buffer[avg_filter->index] = input;
    avg_filter->sum += input;
    
    // Update index
    avg_filter->index = (avg_filter->index + 1) % avg_filter->size;
    
    if (!avg_filter->buffer_full && avg_filter->index == 0) {
        avg_filter->buffer_full = true;
    }
    
    // Calculate average
    uint8_t sample_count = avg_filter->buffer_full ? avg_filter->size : avg_filter->index;
    return avg_filter->sum / sample_count;
}

float filter_debounce_process(debounce_filter_t* db_filter, float input, uint32_t timestamp_us) {
    if (!db_filter) {
        return input;
    }
    
    // Convert to boolean for debouncing (assume threshold of 0.5)
    bool new_state = (input > 0.5f);
    
    if (new_state != db_filter->current_state) {
        db_filter->current_state = new_state;
        db_filter->last_change_time = timestamp_us;
    }
    
    // Check if debounce time has elapsed
    uint32_t time_since_change = timestamp_us - db_filter->last_change_time;
    if (time_since_change >= (db_filter->debounce_time_ms * 1000)) {
        db_filter->debounced_state = db_filter->current_state;
    }
    
    return db_filter->debounced_state ? 1.0f : 0.0f;
}

// Additional filter utility functions

bool filter_is_stable(const digital_filter_t* filter, float threshold, uint32_t time_window_us) {
    // Check if filter output has been stable within threshold for given time
    // This would require additional state tracking in a real implementation
    (void)filter;
    (void)threshold;
    (void)time_window_us;
    return true; // Simplified implementation
}

float filter_get_noise_level(const digital_filter_t* filter) {
    // Estimate noise level based on filter type and recent history
    // This would require additional statistics tracking in a real implementation
    (void)filter;
    return 0.01f; // Simplified implementation
}

void filter_adaptive_adjust(digital_filter_t* filter, float signal_quality) {
    // Adaptively adjust filter parameters based on signal quality
    if (!filter || signal_quality < 0.0f || signal_quality > 1.0f) {
        return;
    }
    
    switch (filter->active_filter) {
        case FILTER_LOW_PASS:
            // Reduce cutoff frequency for noisy signals
            if (signal_quality < 0.5f) {
                filter->low_pass.alpha *= 0.9f; // Make filter more aggressive
            } else if (signal_quality > 0.8f) {
                filter->low_pass.alpha *= 1.1f; // Make filter less aggressive
                if (filter->low_pass.alpha > 1.0f) {
                    filter->low_pass.alpha = 1.0f;
                }
            }
            break;
            
        default:
            // Adaptive adjustment not implemented for other filter types
            break;
    }
}