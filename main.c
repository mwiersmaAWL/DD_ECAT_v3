// main.c - Final version for Synapticon 16-bit absolute encoder with 540° range
// Added FFB command and torque logging functionality
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <pthread.h>
#include <errno.h>
#include <sched.h>
#include <termios.h>

// Application libraries
#include "hid_interface.h"
#include "ffb_calculator.h"
#include "soem_interface.h"
#include "ffb_types.h"

// Configuration constants
#define MAIN_LOOP_FREQUENCY_HZ 100  // 100 Hz (10ms cycle time)
#define CYCLE_TIME_NS (1000000000L / MAIN_LOOP_FREQUENCY_HZ)
#define MAX_STEERING_ANGLE 1080.0f  // ±540 degrees (3 full turns)
#define MAX_TORQUE_LIMIT 5000.0f   // Maximum torque in appropriate units
#define STATS_PRINT_INTERVAL 100   // Print stats every 100 loops (1 second at 100Hz)
#define MAX_LATE_WARNINGS 20       // Reduced warnings
#define EMERGENCY_STOP_THRESHOLD 10000.0f // Emergency torque threshold

// **SYNAPTICON 16-BIT ENCODER SPECIFICATIONS**
// 16-bit absolute encoder = 65,536 counts per revolution
// This provides very high precision: 360° / 65536 = 0.0055° per count
#define ENCODER_COUNTS_PER_REV 65536.0f  // 2^16 = 65,536 counts per revolution
#define MAX_STEERING_REVOLUTIONS 1.5f    // ±1.5 revolutions = ±540 degrees

// **FFB LOGGING CONFIGURATION**
#define LOG_FILENAME_FORMAT "ffb_log_%Y%m%d_%H%M%S.csv"
#define LOG_BUFFER_SIZE 1024
#define LOG_FLUSH_INTERVAL 50  // Flush every 50 loops (0.5 seconds)

// Global flags and state
static volatile int running = 1;
static volatile int emergency_stop = 0;
static volatile int pause_control = 0;

// Position handling - centralized approach
static float global_current_position = 0.0f;
static float global_center_position = 0.0f;
static pthread_mutex_t position_global_mutex = PTHREAD_MUTEX_INITIALIZER;
static int position_system_initialized = 0;

// FFB Logging system
static FILE *log_file = NULL;
static pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
static int log_counter = 0;
static int logging_enabled = 1;  // Can be toggled

//Keyboard inputs
struct termios orig_termios;

// FFB Logging functions
static int init_ffb_logging(void);
static void cleanup_ffb_logging(void);
static void log_ffb_data(const ffb_motor_effect_t *ffb_effect, float position_deg, float velocity, float torque_out, int effect_available);
static void toggle_logging(void);

// Initialize FFB logging system
static int init_ffb_logging(void) {
    char filename[256];
    time_t rawtime;
    struct tm *timeinfo;
    
    // Generate timestamp-based filename
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(filename, sizeof(filename), LOG_FILENAME_FORMAT, timeinfo);
    
    // Open log file
    log_file = fopen(filename, "w");
    if (!log_file) {
        perror("Failed to create FFB log file");
        return -1;
    }
    
    // Write CSV header
    fprintf(log_file, "timestamp_ms,loop_count,position_deg,velocity_deg_s,effect_available,");
    fprintf(log_file, "report_id,effect_type_enum,effect_type_raw,magnitude,direction_deg,duration_ms,");
    fprintf(log_file, "spring_coeff,damper_coeff,friction_coeff,inertia_coeff,center_pos,dead_band,");
    fprintf(log_file, "attack_level,attack_time_ms,fade_level,fade_time_ms,");
    fprintf(log_file, "torque_output,emergency_stop,ethercat_status\n");
    
    fflush(log_file);
    
    printf("FFB logging initialized: %s\n", filename);
    return 0;
}

// Cleanup FFB logging
static void cleanup_ffb_logging(void) {
    pthread_mutex_lock(&log_mutex);
    if (log_file) {
        fflush(log_file);
        fclose(log_file);
        log_file = NULL;
        printf("FFB logging closed\n");
    }
    pthread_mutex_unlock(&log_mutex);
}

// Log FFB data to CSV file
static void log_ffb_data(const ffb_motor_effect_t *ffb_effect, float position_deg, float velocity, float torque_out, int effect_available) {
    if (!logging_enabled) return;
    
    pthread_mutex_lock(&log_mutex);
    
    if (!log_file) {
        pthread_mutex_unlock(&log_mutex);
        return;
    }
    
    // Get current timestamp in milliseconds
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    long timestamp_ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    
    // Log basic data
    fprintf(log_file, "%ld,%d,%.3f,%.3f,%d,", 
            timestamp_ms, log_counter, position_deg, velocity, effect_available);
    
    // Log FFB effect data (if available)
    if (effect_available && ffb_effect) {
        fprintf(log_file, "%d,%d,%d,%.3f,%.3f,%d,",
                ffb_effect->report_id,
                (int)ffb_effect->type,
                ffb_effect->effect_type,
                ffb_effect->magnitude,
                ffb_effect->direction,
                ffb_effect->duration_ms);
        
        // Log coefficient parameters
        fprintf(log_file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                ffb_effect->spring_coefficient,
                ffb_effect->damper_coefficient,
                ffb_effect->friction_coefficient,
                ffb_effect->inertia_coefficient,
                ffb_effect->center_position,
                ffb_effect->dead_band);
        
        // Log envelope parameters
        fprintf(log_file, "%.3f,%d,%.3f,%d,",
                ffb_effect->attack_level,
                ffb_effect->attack_time_ms,
                ffb_effect->fade_level,
                ffb_effect->fade_time_ms);
    } else {
        // No effect available - log zeros/empty values for all FFB fields
        fprintf(log_file, "0,0,0,0.0,0.0,0,");           // basic effect data
        fprintf(log_file, "0.0,0.0,0.0,0.0,0.0,0.0,");   // coefficients
        fprintf(log_file, "0.0,0,0.0,0,");               // envelope
    }
    
    // Log output torque and status
    fprintf(log_file, "%.3f,%d,%d\n", 
            torque_out, emergency_stop ? 1 : 0, 
            soem_interface_get_communication_status() ? 1 : 0);
    
    log_counter++;
    
    // Flush periodically for real-time monitoring
    if (log_counter % LOG_FLUSH_INTERVAL == 0) {
        fflush(log_file);
    }
    
    pthread_mutex_unlock(&log_mutex);
}

// Toggle logging on/off
static void toggle_logging(void) {
    logging_enabled = !logging_enabled;
    printf("FFB logging %s\n", logging_enabled ? "enabled" : "disabled");
}

int check_ctrl_combinations() {
    char ch;
    if (read(STDIN_FILENO, &ch, 1) == 1) {
        switch (ch) {
            case 3:  // Ctrl+C (disabled by raw mode ISIG flag, handle manually)
                printf("Ctrl+C pressed - initiating graceful shutdown!\n");
                running = 0;
                return 1;
            case 18: // Ctrl+R
                printf("Ctrl+R pressed - recentering wheel!\n");
                // Recenter using our centralized system
                pthread_mutex_lock(&position_global_mutex);
                global_center_position = global_current_position;
                pthread_mutex_unlock(&position_global_mutex);
                printf("Main: Wheel recentered to position: %.2f encoder counts\n", global_center_position);
                return 1;
            case 12: // Ctrl+L
                printf("Ctrl+L pressed - toggling FFB logging!\n");
                toggle_logging();
                return 1;
        }
    }
    return 0;
}

// Performance monitoring structure
typedef struct {
    long min_time_ns;
    long max_time_ns;
    long total_time_ns;
    int loop_count;
    int late_warnings;
    int emergency_stops;
    int communication_errors;
    int ffb_effects_processed;
    double avg_torque;
    double max_torque;
} performance_stats_t;

// Application state structure
typedef struct {
    // Control variables
    float current_position_raw;      // Raw encoder position from SOEM
    float current_position_relative; // Position relative to center (encoder counts)
    float current_angle_degrees;     // Position in degrees
    float current_velocity;
    float desired_torque;
    float normalized_position;       // Normalized for HID (-1.0 to 1.0)
    unsigned int button_states;
    
    // FFB state
    ffb_motor_effect_t current_ffb_effect;
    int effect_available;
    
    // Communication status
    int ethercat_status;
    int hid_status;
    int last_ethercat_status;
    
    // Performance monitoring
    performance_stats_t stats;
    
    // Timing
    struct timespec loop_start_time;
    struct timespec loop_end_time;
} app_state_t;

// Function prototypes
static void setup_real_time_scheduling(void);
static void setup_signal_handlers(void);
static void cleanup_and_exit(int exit_code);
static void sigint_handler(int signum);
static void sigusr1_handler(int signum);
static void sigusr2_handler(int signum);
static float normalize_position_for_hid(float position_degrees);
static unsigned int read_button_states(void);
static void update_performance_stats(app_state_t *state);
static void print_performance_stats(const performance_stats_t *stats);
static void reset_performance_stats(performance_stats_t *stats);
static int apply_safety_checks(app_state_t *state);
static void maintain_loop_timing(const struct timespec *start_time, const struct timespec *end_time);
static long timespec_diff_ns(const struct timespec *start, const struct timespec *end);
static void update_position_system(app_state_t *state);

// Signal handlers
static void sigint_handler(int signum) {
    printf("\nCaught SIGINT (Ctrl+C), initiating graceful shutdown...\n");
    running = 0;
}

static void sigusr1_handler(int signum) {
    printf("\nCaught SIGUSR1, pausing control loop...\n");
    pause_control = 1;
}

static void sigusr2_handler(int signum) {
    printf("\nCaught SIGUSR2, resuming control loop...\n");
    pause_control = 0;
}

// Setup real-time scheduling and memory locking
static void setup_real_time_scheduling(void) {
    struct sched_param param;
    int ret;

    // Lock memory to prevent page faults
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (ret != 0) {
        perror("Warning: Failed to lock memory");
    }

    // Use a moderate real-time priority
    param.sched_priority = 50;
    ret = sched_setscheduler(0, SCHED_FIFO, &param);
    if (ret != 0) {
        perror("Warning: Failed to set real-time scheduling");
        printf("Running with normal scheduling. Consider running with sudo for real-time priority.\n");
    } else {
        printf("Real-time scheduling enabled with priority %d\n", param.sched_priority);
    }

    // Set process priority
    ret = setpriority(PRIO_PROCESS, 0, -10);
    if (ret != 0) {
        perror("Warning: Failed to set process priority");
    }
}

// Setup Keyboard input for centering wheel
void disable_raw_mode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disable_raw_mode);
    
    struct termios raw = orig_termios;
    raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    raw.c_oflag &= ~(OPOST);
    raw.c_cflag |= (CS8);
    raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;
    
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// Setup signal handlers
static void setup_signal_handlers(void) {
    struct sigaction sa;
    
    // SIGINT handler (Ctrl+C)
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGINT, &sa, NULL);
    
    // SIGUSR1 handler (pause control)
    sa.sa_handler = sigusr1_handler;
    sigaction(SIGUSR1, &sa, NULL);
    
    // SIGUSR2 handler (resume control)
    sa.sa_handler = sigusr2_handler;
    sigaction(SIGUSR2, &sa, NULL);
    
    // Block SIGPIPE (broken pipe)
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}

// Cleanup and exit function
static void cleanup_and_exit(int exit_code) {
    printf("\nInitiating cleanup sequence...\n");
    
    // Stop all subsystems
    running = 0;
    
    // Cleanup FFB logging first
    cleanup_ffb_logging();
    
    // Stop EtherCAT first to ensure safe torque shutdown
    printf("Stopping EtherCAT master...\n");
    soem_interface_stop_master();
    
    // Stop HID interface
    printf("Stopping HID interface...\n");
    hid_interface_stop();
    
    // Unlock memory
    munlockall();
    
    printf("Cleanup completed successfully.\n");
    exit(exit_code);
}

// Convert position to normalized range for HID report
static float normalize_position_for_hid(float position_degrees) {
    // Clamp to range ±540 degrees
    if (position_degrees > MAX_STEERING_ANGLE) position_degrees = MAX_STEERING_ANGLE;
    if (position_degrees < -MAX_STEERING_ANGLE) position_degrees = -MAX_STEERING_ANGLE;
    
    // Normalize to [-1.0, 1.0]
    return position_degrees / MAX_STEERING_ANGLE;
}

// Read button states (placeholder implementation)
static unsigned int read_button_states(void) {
    return 0;
}

// Time difference calculation in nanoseconds
static long timespec_diff_ns(const struct timespec *start, const struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000000000L + (end->tv_nsec - start->tv_nsec);
}

// Centralized position system update - FIXED FOR SYNAPTICON 16-BIT ENCODER
static void update_position_system(app_state_t *state) {
    // Get raw position from SOEM (encoder counts)
    state->current_position_raw = soem_interface_get_current_position();
    
    pthread_mutex_lock(&position_global_mutex);
    
    // Update global position
    global_current_position = state->current_position_raw;
    
    // Initialize center position on first run
    if (!position_system_initialized) {
        global_center_position = state->current_position_raw;
        position_system_initialized = 1;
        printf("Main: Position system initialized for Synapticon 16-bit encoder\n");
        printf("       Encoder resolution: %.0f counts/revolution (16-bit precision)\n", ENCODER_COUNTS_PER_REV);
        printf("       Precision: %.4f degrees per count\n", 360.0f / ENCODER_COUNTS_PER_REV);
        printf("       Center position: %.2f encoder counts\n", global_center_position);
        printf("       Max steering range: ±%.0f degrees (±%.1f revolutions)\n", 
               MAX_STEERING_ANGLE, MAX_STEERING_REVOLUTIONS);
    }
    
    // Calculate relative position (in encoder counts)
    state->current_position_relative = global_current_position - global_center_position;
    
    pthread_mutex_unlock(&position_global_mutex);
    
    // Convert to degrees using correct Synapticon 16-bit encoder resolution
    // Formula: degrees = (encoder_counts / counts_per_revolution) * 360°
    state->current_angle_degrees = (state->current_position_relative / ENCODER_COUNTS_PER_REV) * 360.0f;
    
    // Normalize for HID (-1.0 to +1.0 for ±540°)
    state->normalized_position = normalize_position_for_hid(state->current_angle_degrees);
    
    // Enhanced debug output (reduced frequency)
    static int debug_counter = 0;
    if (++debug_counter % 50 == 0) {  // Every 50 loops (0.5 seconds)
        printf("Position: Raw=%.0f, Center=%.0f, Rel=%.0f, Deg=%.1f°, Norm=%.4f, Rev=%.3f\n",
               state->current_position_raw, global_center_position, 
               state->current_position_relative, state->current_angle_degrees, 
               state->normalized_position, state->current_angle_degrees / 360.0f);
    }
}

// Update performance statistics
static void update_performance_stats(app_state_t *state) {
    long elapsed_ns = timespec_diff_ns(&state->loop_start_time, &state->loop_end_time);
    performance_stats_t *stats = &state->stats;
    
    if (elapsed_ns < stats->min_time_ns) stats->min_time_ns = elapsed_ns;
    if (elapsed_ns > stats->max_time_ns) stats->max_time_ns = elapsed_ns;
    stats->total_time_ns += elapsed_ns;
    stats->loop_count++;
    
    // Update torque statistics
    float abs_torque = fabs(state->desired_torque);
    stats->avg_torque = (stats->avg_torque * (stats->loop_count - 1) + abs_torque) / stats->loop_count;
    if (abs_torque > stats->max_torque) stats->max_torque = abs_torque;
    
    if (state->effect_available) {
        stats->ffb_effects_processed++;
    }
}

// Print performance statistics
static void print_performance_stats(const performance_stats_t *stats) {
    if (stats->loop_count == 0) return;
    
    long avg_time_ns = stats->total_time_ns / stats->loop_count;
    
    printf("=== Performance Statistics ===\n");
    printf("Loop timing: Min=%.3fms, Max=%.3fms, Avg=%.3fms\n",
           stats->min_time_ns / 1000000.0, stats->max_time_ns / 1000000.0, avg_time_ns / 1000000.0);
    printf("Loop count: %d, Late warnings: %d, Emergency stops: %d\n",
           stats->loop_count, stats->late_warnings, stats->emergency_stops);
    printf("Communication errors: %d, FFB effects processed: %d\n",
           stats->communication_errors, stats->ffb_effects_processed);
    printf("Torque: Avg=%.1f, Max=%.1f\n", stats->avg_torque, stats->max_torque);
    printf("Encoder: Synapticon 16-bit absolute, %.0f counts/rev, ±%.0f° range\n", 
           ENCODER_COUNTS_PER_REV, MAX_STEERING_ANGLE);
    printf("Precision: %.4f degrees per encoder count\n", 360.0f / ENCODER_COUNTS_PER_REV);
    printf("FFB Logging: %s, Records logged: %d\n", 
           logging_enabled ? "Enabled" : "Disabled", log_counter);
    
    // HID statistics
    int hid_write_errors, hid_read_errors, hid_reconnects;
    hid_interface_get_stats(&hid_write_errors, &hid_read_errors, &hid_reconnects);
    printf("HID: Write errors=%d, Read errors=%d, Reconnects=%d, Connected=%s\n",
           hid_write_errors, hid_read_errors, hid_reconnects,
           hid_interface_get_connection_status() ? "Yes" : "No");
}

// Reset performance statistics
static void reset_performance_stats(performance_stats_t *stats) {
    memset(stats, 0, sizeof(performance_stats_t));
    stats->min_time_ns = LONG_MAX;
}

// Apply safety checks and emergency procedures
static int apply_safety_checks(app_state_t *state) {
    // Check for emergency stop conditions
    if (fabs(state->desired_torque) > EMERGENCY_STOP_THRESHOLD) {
        if (!emergency_stop) {
            printf("EMERGENCY STOP: Torque %.1f exceeds threshold %.1f\n",
                   state->desired_torque, EMERGENCY_STOP_THRESHOLD);
            emergency_stop = 1;
            state->stats.emergency_stops++;
        }
        state->desired_torque = 0.0f;
        return 1;
    }
    
    // Check for communication loss
    // Bug fix: counter was reset to 0 on detection, allowing non-zero torque for the
    // next 100 cycles. Now: keep torque at zero until comms is confirmed restored.
    static int comm_loss_count = 0;
    static int comm_loss_logged = 0;
    if (!state->ethercat_status) {
        comm_loss_count++;
        if (comm_loss_count > 100) {
            if (!comm_loss_logged) {
                printf("SAFETY: EtherCAT communication lost, setting zero torque\n");
                comm_loss_logged = 1;
                state->stats.communication_errors++;
            }
            state->desired_torque = 0.0f;
            return 1;
        }
    } else {
        comm_loss_count = 0;
        comm_loss_logged = 0;
    }
    
    // Apply torque limits
    if (fabs(state->desired_torque) > MAX_TORQUE_LIMIT) {
        state->desired_torque = (state->desired_torque > 0) ? MAX_TORQUE_LIMIT : -MAX_TORQUE_LIMIT;
    }
    
    // Check for excessive steering angle (beyond ±540°)
    if (fabs(state->current_angle_degrees) > MAX_STEERING_ANGLE * 1.1f) {
        printf("WARNING: Steering angle %.1f° exceeds safe range (±%.0f°)\n", 
               state->current_angle_degrees, MAX_STEERING_ANGLE);
    }
    
    return 0;
}

// Maintain loop timing
static void maintain_loop_timing(const struct timespec *start_time, const struct timespec *end_time) {
    static int late_warning_count = 0;
    
    long elapsed_ns = timespec_diff_ns(start_time, end_time);
    long sleep_ns = CYCLE_TIME_NS - elapsed_ns;
    
    if (sleep_ns > 0) {
        struct timespec sleep_time = {
            .tv_sec = sleep_ns / 1000000000L,
            .tv_nsec = sleep_ns % 1000000000L
        };
        nanosleep(&sleep_time, NULL);
    } else if (sleep_ns < -1000000 && late_warning_count < MAX_LATE_WARNINGS) {
        printf("Warning: Loop running %.3fms late (target: %.3fms, actual: %.3fms)\n",
               -sleep_ns / 1000000.0, CYCLE_TIME_NS / 1000000.0, elapsed_ns / 1000000.0);
        late_warning_count++;
    }
}

// Main function
int main(int argc, char *argv[]) {
    printf("=== Raspberry Pi FFB Steering Wheel Application ===\n");
    printf("Synapticon 16-bit Absolute Encoder Version with FFB Logging\n");
    printf("Encoder: %d counts/revolution (%.4f° precision), ±%.0f° steering range\n", 
           (int)ENCODER_COUNTS_PER_REV, 360.0f / ENCODER_COUNTS_PER_REV, MAX_STEERING_ANGLE);
    printf("Controls: Ctrl+C=Exit, Ctrl+R=Recenter wheel, Ctrl+L=Toggle FFB logging\n");
    printf("\n");
    
    // Initialize application state
    app_state_t app_state;
    memset(&app_state, 0, sizeof(app_state));
    reset_performance_stats(&app_state.stats);
    
    // Setup real-time environment
    setup_real_time_scheduling();
    setup_signal_handlers();
    enable_raw_mode();
    
    // Initialize FFB logging
    if (init_ffb_logging() != 0) {
        fprintf(stderr, "Warning: FFB logging initialization failed, continuing without logging\n");
        logging_enabled = 0;
    }
    
    // --- Subsystem Initialization ---
    printf("\n=== Initializing Subsystems ===\n");
    
    // Initialize FFB calculator
    printf("Initializing FFB calculator...\n");
    ffb_calculator_init();
    
    // Initialize EtherCAT
    const char *ethercat_ifname = (argc > 1) ? argv[1] : "eth1";
    printf("Initializing EtherCAT master on interface %s...\n", ethercat_ifname);
    
    if (soem_interface_init_enhanced(ethercat_ifname) != 0) {
        fprintf(stderr, "Failed to initialize EtherCAT master.\n");
        cleanup_and_exit(EXIT_FAILURE);
    }
    
    printf("EtherCAT master initialized and stabilized.\n"); 
    
    // Wait for SOEM to stabilize before initializing HID
    printf("Waiting for SOEM to stabilize...\n");
    sleep(2);
    
    // Initialize HID interface
    printf("Initializing HID interface...\n");
    if (hid_interface_init() != 0) {
        fprintf(stderr, "Failed to initialize HID interface.\n");
        cleanup_and_exit(EXIT_FAILURE);
    }

    if (hid_interface_start() != 0) {
        fprintf(stderr, "Failed to start HID interface.\n");
        cleanup_and_exit(EXIT_FAILURE);
    }
    
    // --- Main Control Loop ---
    printf("\n=== Starting Main Control Loop ===\n");
    printf("Loop frequency: %d Hz (target cycle time: %.1f ms)\n", 
           MAIN_LOOP_FREQUENCY_HZ, (float)CYCLE_TIME_NS / 1000000.0);
    printf("Steering range: ±%.0f degrees (%.1f full rotations)\n",
           MAX_STEERING_ANGLE, MAX_STEERING_REVOLUTIONS);
    printf("High precision: %.4f degrees per encoder step\n", 360.0f / ENCODER_COUNTS_PER_REV);
    printf("FFB Logging: %s\n", logging_enabled ? "Enabled" : "Disabled");
    printf("Ready! Turn your wheel and enjoy the full 540° range.\n\n");
    
    while (running) {
        clock_gettime(CLOCK_MONOTONIC, &app_state.loop_start_time);
        
        // Check for keyboard input (non-blocking)
        check_ctrl_combinations();
        
        // Skip control if paused
        if (pause_control) {
            usleep(10000); // Sleep 10ms when paused
            continue;
        }
        
        // 1. Update communication status
        app_state.ethercat_status = soem_interface_get_communication_status();
        app_state.hid_status = hid_interface_get_connection_status();
        
        if (app_state.ethercat_status != app_state.last_ethercat_status) {
            printf("EtherCAT status changed: %s\n", 
                   app_state.ethercat_status ? "OK" : "LOST");
            if (!app_state.ethercat_status) {
                app_state.stats.communication_errors++;
            }
            app_state.last_ethercat_status = app_state.ethercat_status;
        }
        
        // 2. Update position system (centralized with correct 16-bit encoder handling)
        update_position_system(&app_state);
        
        // 3. Get velocity from servo
        app_state.current_velocity = soem_interface_get_current_velocity();
        
        // 4. Get FFB commands from PC
        app_state.effect_available = hid_interface_get_ffb_effect(&app_state.current_ffb_effect);
        const ffb_motor_effect_t *effect_ptr = app_state.effect_available ? &app_state.current_ffb_effect : NULL;
        
        // 5. Calculate desired torque (use relative position in encoder counts)
        app_state.desired_torque = ffb_calculator_calculate_torque(
            effect_ptr, app_state.current_position_relative, app_state.current_velocity);
        
        // 6. Apply safety checks
        apply_safety_checks(&app_state);
        
        // 7. Log FFB data before sending to motor
        log_ffb_data(effect_ptr, app_state.current_angle_degrees, app_state.current_velocity, 
                     app_state.desired_torque, app_state.effect_available);
        
        // 8. Send torque command to servo (only if EtherCAT is connected)
        if (app_state.ethercat_status && !emergency_stop) {
            soem_interface_send_and_receive_pdo(app_state.desired_torque);
        } else {
            // Send zero torque if communication is lost or emergency stop is active
            soem_interface_send_and_receive_pdo(0.0f);
        }
        
        // 9. Read button states
        app_state.button_states = read_button_states();
        
        // 10. Send gamepad report to PC (use normalized position)
        if (app_state.hid_status) {
            hid_interface_send_gamepad_report(app_state.normalized_position, app_state.button_states);
        }
        
        // 11. Update performance statistics
        clock_gettime(CLOCK_MONOTONIC, &app_state.loop_end_time);
        update_performance_stats(&app_state);
        
        // 12. Print periodic statistics
        if (app_state.stats.loop_count % STATS_PRINT_INTERVAL == 0) {
            printf("Status: Deg=%.1f° (%.3f rev), Norm=%.4f, Vel=%.1f°/s, Torque=%.1f, EtherCAT=%s, HID=%s, Emergency=%s, Log=%d\n",
                   app_state.current_angle_degrees, app_state.current_angle_degrees / 360.0f,
                   app_state.normalized_position, app_state.current_velocity, app_state.desired_torque,
                   app_state.ethercat_status ? "OK" : "LOST",
                   app_state.hid_status ? "OK" : "LOST",
                   emergency_stop ? "STOP" : "OK",
                   log_counter);
        }
        
        // 13. Maintain loop timing
        maintain_loop_timing(&app_state.loop_start_time, &app_state.loop_end_time);
        
        // Clear emergency stop if torque is back to normal
        if (emergency_stop && fabs(app_state.desired_torque) < MAX_TORQUE_LIMIT * 0.5f) {
            emergency_stop = 0;
            printf("Emergency stop cleared\n");
        }
    }
    
    // --- Final Statistics and Cleanup ---
    printf("\n=== Final Statistics ===\n");
    print_performance_stats(&app_state.stats);
    
    cleanup_and_exit(EXIT_SUCCESS);
    return EXIT_SUCCESS; // Never reached
}