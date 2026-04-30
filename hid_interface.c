// hid_interface.c - Simplified version with position handling removed (handled in main.c)
#include "hid_interface.h"
#include "soem_interface.h"
#include "ffb_types.h"

#include <math.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sched.h> // For CPU affinity

#define HID_DEVICE_PATH "/dev/hidg0"
// Optimized send interval for better USB performance
#define HID_SEND_INTERVAL_MS 10  // Send reports every 10ms (100Hz)
#define USB_RECONNECT_DELAY_MS 1000  // Wait 1 second before trying to reconnect
#define MAX_STEERING_ANGLE 1080.0f // Adjusted to 540 degrees as per main.c

// Reduced retries and timeouts for better performance
#define MAX_WRITE_RETRIES 2
#define WRITE_RETRY_DELAY_US 500
#define WRITE_SELECT_TIMEOUT_US 5000  // Reduced to 5ms

// Structure matching standard HID gamepad Report ID 1
typedef struct {
    uint8_t report_id;      // Report ID = 1
    int16_t x_axis;         // 16-bit signed X-axis (-32768 to 32767)
    uint16_t buttons;       // 16-bit button field
} __attribute__((packed)) gamepad_report_t;

// Fixed FFB Input Report structures matching your HID descriptor
typedef struct {
    uint8_t report_id;      // Report ID = 2
    uint8_t effect_block_index;  // Effect block index
    uint8_t effect_type;    // Effect type
} __attribute__((packed)) ffb_pid_pool_report_t;

typedef struct {
    uint8_t report_id;      // Report ID = 3
    uint8_t magnitude;      // Effect magnitude
    uint8_t offset;         // Effect offset
    uint8_t phase;          // Phase
    uint8_t period;         // Period
    uint8_t duration_low;   // Duration low byte
    uint8_t duration_high;  // Duration high byte
} __attribute__((packed)) ffb_set_effect_report_t;

// Generic FFB report for any report ID
typedef struct {
    uint8_t report_id;
    uint8_t data[16];  // Maximum data size based on your largest report
} __attribute__((packed)) ffb_generic_report_t;

// FFB Effect Queue
#define FFB_EFFECT_QUEUE_SIZE 16
static ffb_motor_effect_t ffb_effect_queue[FFB_EFFECT_QUEUE_SIZE];
static int queue_head = 0;
static int queue_tail = 0;
static int queue_count = 0;
static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;

// Thread running flag
volatile int hid_running = 0;

static int hidg_fd = -1;
static pthread_mutex_t hidg_fd_mutex = PTHREAD_MUTEX_INITIALIZER;

// USB connection state tracking
static volatile int usb_connected = 0;
static int consecutive_write_failures = 0;
static struct timespec last_reconnect_attempt = {0, 0};

// Error tracking
static int total_write_errors = 0;
static int total_read_errors = 0;
static int reconnect_count = 0;

// Debugging - reduced frequency to avoid spam
static int report_counter = 0;
static int debug_every_n_reports = 1000;  // Debug every 1000 reports

// Threads
static pthread_t ffb_reception_thread;
static pthread_t gamepad_report_thread;

// Add this helper function for time comparison
static long timespec_diff_ms(struct timespec *start, struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000 + (end->tv_nsec - start->tv_nsec) / 1000000;
}

// Check if HID device exists and is accessible
static int check_hid_device_exists() {
    struct stat st;
    if (stat(HID_DEVICE_PATH, &st) != 0) {
        return 0;
    }
    return S_ISCHR(st.st_mode);
}

// Safe function to close HID device (acquires mutex itself, use from OUTSIDE locked sections)
static void safe_close_hid_device() {
    pthread_mutex_lock(&hidg_fd_mutex);
    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1; // Invalidate the file descriptor
        usb_connected = 0;
        printf("HIDInterface: Closed HID device\n");
    }
    pthread_mutex_unlock(&hidg_fd_mutex);
}

// Internal close: call ONLY while hidg_fd_mutex is already held to avoid deadlock
static void _close_hid_device_nolock(void) {
    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1;
        usb_connected = 0;
        printf("HIDInterface: Closed HID device\n");
    }
}

// Attempt to open/reopen HID device
static int try_open_hid_device() {
    pthread_mutex_lock(&hidg_fd_mutex);
    
    // Close existing connection if any before trying to reopen
    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1;
    }
    
    // Check if device exists
    if (!check_hid_device_exists()) {
        pthread_mutex_unlock(&hidg_fd_mutex);
        return -1;
    }
    
    // Try to open the device
    hidg_fd = open(HID_DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (hidg_fd < 0) {
        perror("HIDInterface: Failed to open HID device");
        pthread_mutex_unlock(&hidg_fd_mutex);
        return -1;
    }
    
    usb_connected = 1;
    consecutive_write_failures = 0;
    reconnect_count++;
    
    printf("HIDInterface: Successfully opened HID device (reconnect #%d)\n", reconnect_count);
    
    pthread_mutex_unlock(&hidg_fd_mutex);
    return 0;
}

// Check if we should attempt reconnection
static int should_attempt_reconnect() {
    if (usb_connected) return 0;
    
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    if (last_reconnect_attempt.tv_sec != 0 || last_reconnect_attempt.tv_nsec != 0) {
        long elapsed_ms = timespec_diff_ms(&last_reconnect_attempt, &current_time);
        if (elapsed_ms < USB_RECONNECT_DELAY_MS) {
            return 0;
        }
    }
    
    last_reconnect_attempt = current_time;
    return 1;
}

// Updated parse FFB reports from PC with proper structure handling
static int parse_ffb_report(uint8_t *report, size_t len, ffb_motor_effect_t *effect) {
    if (len < 2) return 0;
    
    uint8_t report_id = report[0];
    
    // Initialize effect with default values
    memset(effect, 0, sizeof(ffb_motor_effect_t));
    effect->report_id = report_id;
    clock_gettime(CLOCK_REALTIME, &effect->received_time);
    
    switch (report_id) {
        case 2: // PID Pool Report
            if (len >= sizeof(ffb_pid_pool_report_t)) {
                ffb_pid_pool_report_t *pid_report = (ffb_pid_pool_report_t *)report;
                effect->effect_type = pid_report->effect_type;
                return 1;
            }
            break;
            
        case 3: // Set Effect Report
            if (len >= sizeof(ffb_set_effect_report_t)) {
                ffb_set_effect_report_t *set_effect = (ffb_set_effect_report_t *)report;
                effect->magnitude = (set_effect->magnitude - 128) / 128.0f;
                effect->duration_ms = (set_effect->duration_high << 8) | set_effect->duration_low;
                return 1;
            }
            break;
            
        // Add other report types as needed
        default:
            // Unknown report type
            return 0;
    }
    
    return 0;
}

// FFB reception thread with improved error handling
static void* _usb_ffb_reception_thread(void* arg) {
    (void)arg;
    
    // Set CPU affinity for this thread to core 1
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        perror("FFB Reception Thread: Failed to set CPU affinity");
    }

    ffb_generic_report_t ffb_report;
    int read_failures = 0;

    printf("FFB: Reception thread started\n");
    
    while (hid_running) {
        // Check if we need to reconnect
        if (!usb_connected && should_attempt_reconnect()) {
            if (try_open_hid_device() == 0) {
                read_failures = 0;
            }
        }
        
        // Only try to read if we're connected
        if (usb_connected) {
            pthread_mutex_lock(&hidg_fd_mutex);
            int fd = hidg_fd;
            
            if (fd >= 0) {
                fd_set read_fds;
                struct timeval tv;
                FD_ZERO(&read_fds);
                FD_SET(fd, &read_fds);
                tv.tv_sec = 0;
                tv.tv_usec = 5000; // 5ms timeout

                int retval = select(fd + 1, &read_fds, NULL, NULL, &tv);

                if (retval == -1) {
                    perror("FFB Reception Thread: select error");
                    read_failures++;
                    total_read_errors++;
                    if (read_failures > 10) {
                        // Bug fix: safe_close_hid_device() takes the mutex — deadlock here.
                        // Use nolock variant since hidg_fd_mutex is already held.
                        _close_hid_device_nolock();
                        read_failures = 0;
                    }
                } else if (retval) {
                    ssize_t len = read(fd, &ffb_report, sizeof(ffb_report));
                    if (len > 0) {
                        read_failures = 0;
                        
                        ffb_motor_effect_t new_effect;
                        if (parse_ffb_report((uint8_t*)&ffb_report, (size_t)len, &new_effect)) {
                            pthread_mutex_lock(&queue_mutex);
                            if (queue_count < FFB_EFFECT_QUEUE_SIZE) {
                                ffb_effect_queue[queue_tail] = new_effect;
                                queue_tail = (queue_tail + 1) % FFB_EFFECT_QUEUE_SIZE;
                                queue_count++;
                                pthread_cond_signal(&queue_cond);
                            }
                            pthread_mutex_unlock(&queue_mutex);
                        }
                    } else if (len < 0) {
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            perror("FFB Reception Thread: read error");
                            read_failures++;
                            total_read_errors++;
                            if (read_failures > 10) {
                                // Bug fix: deadlock — use nolock variant
                                _close_hid_device_nolock();
                                read_failures = 0;
                            }
                        }
                    }
                }
            }
            pthread_mutex_unlock(&hidg_fd_mutex);
        }
        
        // Small delay to prevent busy waiting
        usleep(1000); // 1ms
    }

    printf("FFB: Reception thread stopped\n");
    return NULL;
}

// Simplified gamepad report sending thread
static void* _gamepad_report_loop(void* arg) {
    (void)arg;

    // Bug fix: both threads were pinned to core 1, competing for the same CPU.
    // Assign gamepad thread to core 2, FFB reception thread stays on core 1.
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        perror("Gamepad Report Thread: Failed to set CPU affinity");
    }

    printf("HIDInterface: Gamepad report thread started\n");

    struct timespec loop_start_time, loop_end_time;

    while (hid_running) {
        clock_gettime(CLOCK_MONOTONIC, &loop_start_time);

        // The main loop will call hid_interface_send_gamepad_report() directly
        // This thread just maintains the timing for potential future use
        
        // Periodic debugging output
        report_counter++;
        if (report_counter % debug_every_n_reports == 0) {
            printf("HID Thread Status: Connected=%s, WriteErr=%d, ReadErr=%d\n",
                   usb_connected ? "YES" : "NO", total_write_errors, total_read_errors);
        }
        
        // Enforce the send interval
        clock_gettime(CLOCK_MONOTONIC, &loop_end_time);
        long elapsed_ms = timespec_diff_ms(&loop_start_time, &loop_end_time);
        long sleep_time_ms = HID_SEND_INTERVAL_MS - elapsed_ms;

        if (sleep_time_ms > 0) {
            usleep(sleep_time_ms * 1000);
        }
    }

    printf("HIDInterface: Gamepad report thread stopped\n");
    return NULL;
}

/**
 * @brief Initializes the HID interface.
 */
int hid_interface_init() {
    printf("HIDInterface: Initializing HID interface...\n");
    
    // Try to open HID device initially
    if (try_open_hid_device() != 0) {
        printf("HIDInterface: Warning - Could not open HID device initially. Will try to reconnect later.\n");
    } else {
        // Give USB host time to recognize the device
        usleep(20000); // 20ms delay
    }

    printf("HIDInterface: HID interface initialized.\n");
    return 0;
}

/**
 * @brief Starts the HID communication threads.
 */
int hid_interface_start() {
    hid_running = 1;
    
    if (pthread_create(&ffb_reception_thread, NULL, _usb_ffb_reception_thread, NULL) != 0) {
        perror("HIDInterface: Failed to create FFB reception thread");
        return -1;
    }
    
    if (pthread_create(&gamepad_report_thread, NULL, _gamepad_report_loop, NULL) != 0) {
        perror("HIDInterface: Failed to create gamepad report thread");
        pthread_cancel(ffb_reception_thread);
        pthread_join(ffb_reception_thread, NULL);
        return -1;
    }
    
    printf("HIDInterface: Started FFB reception and gamepad report threads.\n");
    return 0;
}

/**
 * @brief Stops the HID communication and cleans up resources.
 */
void hid_interface_stop() {
    hid_running = 0;
    
    // Wait for threads to finish
    if (ffb_reception_thread) {
        pthread_join(ffb_reception_thread, NULL);
    }
    
    if (gamepad_report_thread) {
        pthread_join(gamepad_report_thread, NULL);
    }

    safe_close_hid_device();

    printf("HIDInterface: Stopped. Stats - Write errors: %d, Read errors: %d, Reconnects: %d\n", 
           total_write_errors, total_read_errors, reconnect_count);
}

/**
 * @brief Retrieves the latest FFB effect from the queue.
 */
int hid_interface_get_ffb_effect(ffb_motor_effect_t *effect_out) {
    pthread_mutex_lock(&queue_mutex);
    if (queue_count > 0) {
        *effect_out = ffb_effect_queue[queue_head];
        queue_head = (queue_head + 1) % FFB_EFFECT_QUEUE_SIZE;
        queue_count--;
        pthread_mutex_unlock(&queue_mutex);
        return 1;
    }
    pthread_mutex_unlock(&queue_mutex);
    return 0;
}

/**
 * @brief Sends a gamepad report with the position provided by main.c
 */
int hid_interface_send_gamepad_report(float normalized_position, unsigned int buttons) {
    if (!hid_running) {
        return -1;
    }

    // Check connection and try to reconnect if needed
    if (!usb_connected && should_attempt_reconnect()) {
        if (try_open_hid_device() != 0) {
            return 0; // Temporary failure, couldn't reconnect yet
        }
    }
    
    // If still not connected after potential reconnect attempt, return
    if (!usb_connected) {
        return 0; 
    }

    // Clamp position to valid range
    if (normalized_position > 1.0f) normalized_position = 1.0f;
    if (normalized_position < -1.0f) normalized_position = -1.0f;
    
    // Create report
    gamepad_report_t report = {0};
    report.report_id = 1;
    
    // Convert normalized float position [-1.0, 1.0] to int16_t [-32768, 32767]
    if (normalized_position >= 0.0f) {
        report.x_axis = (int16_t)(normalized_position * 32767.0f);
    } else {
        // For negative values, map -1.0 to -32768
        report.x_axis = (int16_t)(normalized_position * 32768.0f);
    }
    
    report.buttons = (uint16_t)(buttons & 0xFFFF);

    // Debug print (reduced frequency)
    static int send_debug_counter = 0;
    if (++send_debug_counter % 1000 == 0) {  // Every 1000 sends
        printf("HIDInterface: Sending - Normalized: %.4f, X-axis: %d, Buttons: %u\n",
               normalized_position, report.x_axis, report.buttons);
    }
    
    // Lock the mutex for the entire write operation
    pthread_mutex_lock(&hidg_fd_mutex);
    int fd = hidg_fd;

    if (fd < 0) {
        pthread_mutex_unlock(&hidg_fd_mutex);
        return -1; // Device not open
    }

    int success = 0;

    fd_set write_fds;
    struct timeval tv;
    
    FD_ZERO(&write_fds);
    FD_SET(fd, &write_fds);
    tv.tv_sec = 0;
    tv.tv_usec = WRITE_SELECT_TIMEOUT_US; // 5ms timeout

    int retval = select(fd + 1, NULL, &write_fds, NULL, &tv);

    if (retval == -1) {
        perror("HIDInterface: select error during write");
        consecutive_write_failures++;
        total_write_errors++;
        if (consecutive_write_failures > 5) {
            // Bug fix: safe_close_hid_device() takes the mutex — deadlock here.
            // Use nolock variant since hidg_fd_mutex is already held.
            _close_hid_device_nolock();
            pthread_mutex_unlock(&hidg_fd_mutex);
            return -1;
        }
        pthread_mutex_unlock(&hidg_fd_mutex);
        return 0;
    } else if (retval == 0) {
        // Timeout - buffer not ready, skip sending this report
        consecutive_write_failures++;
        pthread_mutex_unlock(&hidg_fd_mutex);
        return 0;
    } else {
        // Buffer ready, attempt write
        ssize_t bytes_written = write(fd, &report, sizeof(report));
        
        if (bytes_written == sizeof(report)) {
            consecutive_write_failures = 0;
            success = 1;
        } else if (bytes_written < 0) {
            consecutive_write_failures++;
            total_write_errors++;

            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Buffer full, skip this report
            } else if (errno == EBADF || errno == ENODEV || errno == EPIPE) {
                // Critical error: device lost
                perror("HIDInterface: Critical write error (device lost)");
                // Bug fix: safe_close_hid_device() takes the mutex — deadlock here.
                _close_hid_device_nolock();
                pthread_mutex_unlock(&hidg_fd_mutex);
                return -1;
            } else {
                perror("HIDInterface: write error");
            }
            success = 0; 
        } else {
            // Partial write
            printf("HIDInterface: Partial write (%zd/%zu bytes)\n", bytes_written, sizeof(report));
            consecutive_write_failures++;
            total_write_errors++;
            success = 0;
        }
    }
    
    pthread_mutex_unlock(&hidg_fd_mutex);
    return success;
}

/**
 * @brief Get connection status
 */
int hid_interface_get_connection_status() {
    return usb_connected;
}

/**
 * @brief Get error statistics
 */
void hid_interface_get_stats(int *write_errors, int *read_errors, int *reconnects) {
    if (write_errors) *write_errors = total_write_errors;
    if (read_errors) *read_errors = total_read_errors;
    if (reconnects) *reconnects = reconnect_count;
}