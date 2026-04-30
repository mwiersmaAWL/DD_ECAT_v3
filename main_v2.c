// main_optimized.c - Optimized main control loop sections
// Key optimizations for servo control performance
#include "synapticon_servo_tuning.h"

// Enhanced configuration for better servo response
#define MAIN_LOOP_FREQUENCY_HZ 1000     // Increased to 1kHz for better servo response
#define CYCLE_TIME_NS (1000000000L / MAIN_LOOP_FREQUENCY_HZ)
#define POSITION_FILTER_ALPHA 0.1f      // Low-pass filter for position
#define VELOCITY_CALCULATION_SAMPLES 5  // Samples for velocity calculation
#define EMERGENCY_RECOVERY_TIME_MS 100  // Fast emergency recovery

// Optimized position and velocity calculation
typedef struct {
    float position_history[VELOCITY_CALCULATION_SAMPLES];
    struct timespec time_history[VELOCITY_CALCULATION_SAMPLES];
    int history_index;
    int history_filled;
    float position_filtered;
    float velocity_calculated;
    float velocity_filtered;
} position_tracker_t;

static position_tracker_t pos_tracker = {0};

// Optimized position system update with better velocity calculation
static void update_position_system_optimized(app_state_t *state) {
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    // Get raw position from SOEM (encoder counts)
    state->current_position_raw = soem_interface_get_current_position();
    
    pthread_mutex_lock(&position_global_mutex);
    
    // Update global position
    global_current_position = state->current_position_raw;
    
    // Initialize center position on first run
    if (!position_system_initialized) {
        global_center_position = state->current_position_raw;
        position_system_initialized = 1;
        
        // Initialize position tracker
        for (int i = 0; i < VELOCITY_CALCULATION_SAMPLES; i++) {
            pos_tracker.position_history[i] = state->current_position_raw;
            pos_tracker.time_history[i] = current_time;
        }
        pos_tracker.position_filtered = state->current_position_raw;
        
        printf("Main: Optimized position system initialized for Synapticon 16-bit encoder\n");
        printf("       High-frequency control: %d Hz for improved servo response\n", MAIN_LOOP_FREQUENCY_HZ);
    }
    
    // Calculate relative position (in encoder counts)
    state->current_position_relative = global_current_position - global_center_position;
    
    pthread_mutex_unlock(&position_global_mutex);
    
    // Apply position filtering for noise reduction
    pos_tracker.position_filtered = POSITION_FILTER_ALPHA * state->current_position_relative + 
                                   (1.0f - POSITION_FILTER_ALPHA) * pos_tracker.position_filtered;
    
    // Store current position and time in circular buffer
    pos_tracker.position_history[pos_tracker.history_index] = pos_tracker.position_filtered;
    pos_tracker.time_history[pos_tracker.history_index] = current_time;
    
    // Calculate velocity using multiple samples for better accuracy
    if (pos_tracker.history_filled) {
        int oldest_index = (pos_tracker.history_index + 1) % VELOCITY_CALCULATION_SAMPLES;
        float position_delta = pos_tracker.position_filtered - pos_tracker.position_history[oldest_index];
        
        struct timespec *newest_time = &pos_tracker.time_history[pos_tracker.history_index];
        struct timespec *oldest_time = &pos_tracker.time_history[oldest_index];
        
        float time_delta = (newest_time->tv_sec - oldest_time->tv_sec) + 
                          (newest_time->tv_nsec - oldest_time->tv_nsec) * 1e-9f;
        
        if (time_delta > 0.0f) {
            // Convert encoder counts/sec to degrees/sec
            float velocity_counts_per_sec = position_delta / time_delta;
            pos_tracker.velocity_calculated = (velocity_counts_per_sec / ENCODER_COUNTS_PER_REV) * 360.0f;
            
            // Apply velocity filtering
            float alpha = 0.3f; // More aggressive filtering for velocity
            pos_tracker.velocity_filtered = alpha * pos_tracker.velocity_calculated + 
                                          (1.0f - alpha) * pos_tracker.velocity_filtered;
        }
    }
    
    // Update circular buffer index
    pos_tracker.history_index = (pos_tracker.history_index + 1) % VELOCITY_CALCULATION_SAMPLES;
    if (!pos_tracker.history_filled && pos_tracker.history_index == 0) {
        pos_tracker.history_filled = 1;
    }
    
    // Update state with filtered/calculated values
    state->current_position_relative = pos_tracker.position_filtered;
    state->current_angle_degrees = (state->current_position_relative / ENCODER_COUNTS_PER_REV) * 360.0f;
    state->current_velocity = pos_tracker.velocity_filtered;
    state->normalized_position = normalize_position_for_hid(state->current_angle_degrees);
}

// Optimized safety checks with predictive monitoring
static int apply_safety_checks_optimized(app_state_t *state) {
    static int comm_loss_count = 0;
    static int torque_spike_count = 0;
    static float torque_history[10] = {0};
    static int torque_history_index = 0;
    
    // Predictive torque spike detection
    torque_history[torque_history_index] = state->desired_torque;
    torque_history_index = (torque_history_index + 1) % 10;
    
    // Calculate torque rate of change
    float torque_rate = 0.0f;
    for (int i = 1; i < 10; i++) {
        int prev_idx = (torque_history_index - i - 1 + 10) % 10;
        int curr_idx = (torque_history_index - i + 10) % 10;
        torque_rate += fabsf(torque_history[curr_idx] - torque_history[prev_idx]);
    }
    torque_rate /= 9.0f; // Average rate of change
    
    // Emergency stop for excessive torque or rapid changes
    if (fabsf(state->desired_torque) > EMERGENCY_STOP_THRESHOLD || torque_rate > 2000.0f) {
        if (!emergency_stop) {
            printf("EMERGENCY STOP: Torque %.1f (rate: %.1f) exceeds safety limits\n",
                   state->desired_torque, torque_rate);
            emergency_stop = 1;
            state->stats.emergency_stops++;
        }
        state->desired_torque = 0.0f;
        return 1;
    }
    
    // Fast emergency recovery
    if (emergency_stop && fabsf(state->desired_torque) < MAX_TORQUE_LIMIT * 0.2f && torque_rate < 100.0f) {
        static struct timespec emergency_start_time = {0};
        // Bug fix: tv_sec==0 is not a safe sentinel (CLOCK_MONOTONIC can start near 0).
        // Use a separate flag instead.
        static int emergency_timer_active = 0;
        if (!emergency_timer_active) {
            clock_gettime(CLOCK_MONOTONIC, &emergency_start_time);
            emergency_timer_active = 1;
        }
        
        struct timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        long elapsed_ms = (current_time.tv_sec - emergency_start_time.tv_sec) * 1000 +
                         (current_time.tv_nsec - emergency_start_time.tv_nsec) / 1000000;
        
        if (elapsed_ms > EMERGENCY_RECOVERY_TIME_MS) {
            emergency_stop = 0;
            emergency_timer_active = 0;
            printf("Emergency stop cleared after %ld ms\n", elapsed_ms);
        }
    }
    
    // Optimized communication monitoring
    if (!state->ethercat_status) {
        if (++comm_loss_count > 10) { // 10ms at 1kHz = very fast detection
            state->desired_torque = 0.0f;
            if (comm_loss_count == 11) { // Print only once
                printf("SAFETY: EtherCAT communication lost, zero torque applied\n");
            }
            return 1;
        }
    } else {
        comm_loss_count = 0;
    }
    
    // Adaptive torque limiting based on position
    float position_factor = 1.0f;
    if (fabsf(state->current_angle_degrees) > MAX_STEERING_ANGLE * 0.8f) {
        // Reduce maximum torque near steering limits
        position_factor = 1.0f - (fabsf(state->current_angle_degrees) - MAX_STEERING_ANGLE * 0.8f) / 
                                 (MAX_STEERING_ANGLE * 0.2f) * 0.5f;
        position_factor = fmaxf(position_factor, 0.5f); // Minimum 50% torque
    }
    
    float adjusted_max_torque = MAX_TORQUE_LIMIT * position_factor;
    if (fabsf(state->desired_torque) > adjusted_max_torque) {
        state->desired_torque = (state->desired_torque > 0) ? adjusted_max_torque : -adjusted_max_torque;
    }
    
    return 0;
}

// Optimized main loop timing with adaptive scheduling
static void maintain_loop_timing_optimized(const struct timespec *start_time, const struct timespec *end_time) {
    static int late_warning_count = 0;
    static int timing_adjustment = 0;
    static float avg_execution_time = 0.0f;
    static int timing_sample_count = 0;
    
    long elapsed_ns = (end_time->tv_sec - start_time->tv_sec) * 1000000000L + 
                      (end_time->tv_nsec - start_time->tv_nsec);
    long sleep_ns = CYCLE_TIME_NS - elapsed_ns;
    
    // Track average execution time for adaptive tuning
    avg_execution_time = (avg_execution_time * timing_sample_count + elapsed_ns) / (timing_sample_count + 1);
    if (timing_sample_count < 1000) timing_sample_count++;
    
    // Adaptive timing adjustment
    if (timing_sample_count > 100 && avg_execution_time > CYCLE_TIME_NS * 0.8f) {
        // If consistently using >80% of cycle time, add small buffer
        timing_adjustment = (int)(CYCLE_TIME_NS * 0.05f); // 5% buffer
    }
    
    sleep_ns -= timing_adjustment;
    
    if (sleep_ns > 0) {
        struct timespec sleep_time = {
            .tv_sec = sleep_ns / 1000000000L,
            .tv_nsec = sleep_ns % 1000000000L
        };
        
        // Use high-resolution sleep for better timing accuracy
        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
    } else if (sleep_ns < -100000 && late_warning_count < 10) { // 0.1ms tolerance
        printf("Warning: Loop %.3fms late (exec: %.3fms, target: %.3fms)\n",
               -sleep_ns / 1000000.0, elapsed_ns / 1000000.0, CYCLE_TIME_NS / 1000000.0);
        late_warning_count++;
    }
}

// Enhanced performance statistics with servo-specific metrics
static void print_servo_performance_stats(const performance_stats_t *stats) {
    if (stats->loop_count == 0) return;
    
    long avg_time_ns = stats->total_time_ns / stats->loop_count;
    float servo_update_rate = 1000000000.0f / avg_time_ns;
    
    printf("=== Optimized Servo Performance Statistics ===\n");
    printf("Servo Control: Rate=%.1fHz, Target=%dHz, Efficiency=%.1f%%\n",
           servo_update_rate, MAIN_LOOP_FREQUENCY_HZ, 
           (servo_update_rate / MAIN_LOOP_FREQUENCY_HZ) * 100.0f);
    printf("Loop timing: Min=%.3fms, Max=%.3fms, Avg=%.3fms\n",
           stats->min_time_ns / 1000000.0, stats->max_time_ns / 1000000.0, avg_time_ns / 1000000.0);
    printf("Position: Current=%.1f°, Velocity=%.1f°/s (filtered)\n",
           pos_tracker.position_filtered / ENCODER_COUNTS_PER_REV * 360.0f, pos_tracker.velocity_filtered);

    // FFB Calculator stats — get last_torque first so it can be used in the torque print below
    uint32_t calc_count, effect_changes;
    float last_torque;
    ffb_calculator_get_stats(&calc_count, &effect_changes, &last_torque);

    printf("Torque: Current=%.1f, Avg=%.1f, Max=%.1f, Rate-limited=%s\n",
           last_torque, stats->avg_torque, stats->max_torque,
           (avg_time_ns < CYCLE_TIME_NS * 0.9f) ? "No" : "Yes");
    
    // FFB Calculator stats
    printf("FFB Calculator: %u calculations, %u effect changes, last torque=%.1f\n",
           calc_count, effect_changes, last_torque);
    
    printf("System: Emergency stops=%d, Communication errors=%d\n",
           stats->emergency_stops, stats->communication_errors);
    printf("Encoder: 16-bit Synapticon, %.4f°/count precision\n", 360.0f / ENCODER_COUNTS_PER_REV);
}

// Optimized main control loop structure (key sections)
// Note: This replaces the main while loop in your existing main.c

// Enhanced main loop with 1kHz servo control
static void optimized_main_control_loop(app_state_t *app_state) {
    static int comm_check_counter = 0;
    static int hid_update_counter = 0;
    static int stats_counter = 0;
    static int log_batch_counter = 0;
    
    // Pre-allocated variables to avoid stack allocations in loop
    ffb_motor_effect_t *effect_ptr;
    float calculated_torque;
    int safety_violation;
    
    printf("Starting optimized servo control loop at %d Hz...\n", MAIN_LOOP_FREQUENCY_HZ);
    
    while (running) {
        clock_gettime(CLOCK_MONOTONIC, &app_state->loop_start_time);
        
        // 1. Fast keyboard input check (every cycle, optimized)
        check_ctrl_combinations();
        
        if (pause_control) {
            usleep(1000); // 1ms sleep when paused
            continue;
        }
        
        // 2. Communication status (check every 10 cycles = 10ms)
        if (++comm_check_counter >= 10) {
            comm_check_counter = 0;
            app_state->ethercat_status = soem_interface_get_communication_status();
            if (app_state->ethercat_status != app_state->last_ethercat_status) {
                printf("EtherCAT status: %s\n", app_state->ethercat_status ? "OK" : "LOST");
                if (!app_state->ethercat_status) app_state->stats.communication_errors++;
                app_state->last_ethercat_status = app_state->ethercat_status;
            }
        }
        
        // 3. High-frequency position and velocity update (every cycle)
        update_position_system_optimized(app_state);
        
        // 4. FFB command processing (every cycle for responsiveness)
        app_state->effect_available = hid_interface_get_ffb_effect(&app_state->current_ffb_effect);
        effect_ptr = app_state->effect_available ? &app_state->current_ffb_effect : NULL;
        
        // 5. High-frequency torque calculation (every cycle)
        calculated_torque = ffb_calculator_calculate_torque(
            effect_ptr, app_state->current_position_relative, app_state->current_velocity);
        app_state->desired_torque = calculated_torque;
        
        // 6. Safety checks with emergency response (every cycle)
        safety_violation = apply_safety_checks_optimized(app_state);
        
        // 7. Critical: Send torque to servo (every cycle - highest priority)
        if (app_state->ethercat_status && !emergency_stop) {
            soem_interface_send_and_receive_pdo(app_state->desired_torque);
        } else {
            soem_interface_send_and_receive_pdo(0.0f);
        }
        
        // 8. HID updates (every 10 cycles = 10ms, sufficient for HID)
        if (++hid_update_counter >= 10) {
            hid_update_counter = 0;
            app_state->hid_status = hid_interface_get_connection_status();
            app_state->button_states = read_button_states();
            
            if (app_state->hid_status) {
                hid_interface_send_gamepad_report(app_state->normalized_position, app_state->button_states);
            }
        }
        
        // 9. Logging (batched every 10 cycles for efficiency)
        if (++log_batch_counter >= 10) {
            log_batch_counter = 0;
            log_ffb_data(effect_ptr, app_state->current_angle_degrees, 
                        app_state->current_velocity, app_state->desired_torque, 
                        app_state->effect_available);
        }
        
        // 10. Performance monitoring
        clock_gettime(CLOCK_MONOTONIC, &app_state->loop_end_time);
        update_performance_stats(app_state);
        
        // 11. Statistics (every 1000 cycles = 1 second)
        if (++stats_counter >= 1000) {
            stats_counter = 0;
            printf("Servo: Pos=%.1f°, Vel=%.1f°/s, Torque=%.1f, Rate=%dHz, EtherCAT=%s\n",
                   app_state->current_angle_degrees, app_state->current_velocity, 
                   app_state->desired_torque, MAIN_LOOP_FREQUENCY_HZ,
                   app_state->ethercat_status ? "OK" : "LOST");
        }
        
        // 12. Precise timing control (every cycle)
        maintain_loop_timing_optimized(&app_state->loop_start_time, &app_state->loop_end_time);
    }
}

// Additional optimization: Servo-specific EtherCAT configuration
static int configure_servo_ethercat_optimized(void) {
    printf("Configuring EtherCAT for optimized servo performance...\n");
    
    // Set EtherCAT cycle time to match our loop frequency
    int ethercat_cycle_us = 1000; // 1ms = 1000µs for 1kHz
    
    // Configure distributed clocks for synchronization
    // This ensures servo updates are precisely timed
    if (ec_configdc() > 0) {
        printf("EtherCAT: Distributed clocks configured for %d µs cycle\n", ethercat_cycle_us);
    }
    
    // Set servo to high-performance mode if supported
    // Many servo drives have performance vs efficiency modes
    uint16_t slave_idx = 1;
    
    // Example: Set interpolation time to match our cycle
    uint8_t interpolation_time = 1; // 1ms
    int8_t time_index = -3; // 10^-3 = milliseconds
    
    if (soem_interface_write_sdo(slave_idx, 0x60C2, 0x01, sizeof(interpolation_time), &interpolation_time) == 0) {
        printf("Servo: Interpolation time set to %d ms for smooth operation\n", interpolation_time);
    }
    
    // Enable high-resolution feedback if available
    uint32_t feedback_resolution = 16; // 16-bit resolution
    if (soem_interface_write_sdo(slave_idx, 0x608F, 0x01, sizeof(feedback_resolution), &feedback_resolution) == 0) {
        printf("Servo: Feedback resolution optimized for 16-bit encoder\n");
    }
    
    return 0;
}

// Memory optimization: Pre-allocate critical structures
typedef struct {
    // Pre-allocated FFB effect buffer to avoid malloc in loop
    ffb_motor_effect_t effect_buffer[10];
    int effect_buffer_index;
    
    // Pre-allocated timing structures
    struct timespec timing_buffer[100];
    int timing_buffer_index;
    
    // Statistics buffers
    float torque_history[1000];
    float position_history[1000];
    int history_index;
    
} servo_optimization_context_t;

static servo_optimization_context_t servo_context = {0};

// Performance tuning function
static void tune_servo_performance(void) {
    // Set CPU governor to performance mode for consistent timing
    system("echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor");
    
    // Disable CPU frequency scaling for real-time performance
    system("echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo");
    
    // Set higher priority for our process
    struct sched_param param;
    param.sched_priority = 80; // High priority for servo control
    if (sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
        printf("Process priority set to real-time level %d\n", param.sched_priority);
    }
    
    // Pin process to specific CPU core to avoid context switching
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset); // Use CPU core 2 (adjust based on your Pi)
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0) {
        printf("Process pinned to CPU core 2 for consistent performance\n");
    }
    
    printf("Servo performance tuning completed\n");
}