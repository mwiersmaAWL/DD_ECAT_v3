// hid_interface.c - Simplified version with position handling removed (handled in main.c)
#include "hid_interface.h"
#include "soem_interface.h"
#include "ffb_types.h"

#include <math.h>
#include <stdio.h>
#include <pthread.h>
#include <stdatomic.h>
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

/*
 * Standard USB HID PID output report structures.
 * All layouts must stay in sync with the HID descriptor in create_ffb_gadget.sh.
 */

/* Report ID 1 (Output): Set Effect — creates or modifies an effect in a slot. */
typedef struct {
    uint8_t  report_id;               /* = 1 */
    uint8_t  effect_block_index;      /* 1–FFB_EFFECT_SLOTS */
    uint8_t  effect_type;             /* array value: 1=Constant, 2=Ramp,
                                       * 3=Square, 4=Sine, 5=Triangle,
                                       * 6=SawUp, 7=SawDown, 8=Spring,
                                       * 9=Damper, 10=Inertia, 11=Friction */
    uint16_t duration;                /* ms; 0xFFFF = infinite */
    uint16_t trigger_repeat_interval; /* ms */
    uint8_t  axes_dir_pad;            /* bit0=axes_enable(X), bit1=direction_enable,
                                       * bits2-7=padding */
    uint8_t  gain;                    /* 0–255 */
    uint8_t  trigger_button;          /* 0–255, 0 = no trigger */
} __attribute__((packed)) pid_set_effect_report_t;

/* Report ID 5 (Output): Set Constant Force — magnitude for a constant-force effect. */
typedef struct {
    uint8_t  report_id;           /* = 5 */
    uint8_t  effect_block_index;
    int16_t  magnitude;           /* -32768..32767 */
} __attribute__((packed)) pid_set_constant_force_report_t;

/* Report ID 4 (Output): Set Periodic — parameters for periodic (oscillating) effects. */
typedef struct {
    uint8_t  report_id;           /* = 4 */
    uint8_t  effect_block_index;
    uint16_t magnitude;           /* 0–65535 */
    int16_t  offset;              /* -32768..32767 */
    uint16_t phase;               /* 0–65535 */
    uint16_t period;              /* ms */
} __attribute__((packed)) pid_set_periodic_report_t;

/* Report ID 10 / 0x0A (Output): Effect Operation — start or stop an effect. */
typedef struct {
    uint8_t report_id;            /* = 10 */
    uint8_t effect_block_index;
    uint8_t operation;            /* 1=Start, 2=StartSolo, 3=Stop */
    uint8_t loop_count;           /* 0–255; 0xFF = infinite */
} __attribute__((packed)) pid_effect_operation_report_t;

/* Report ID 11 / 0x0B (Output): PID Block Free — releases an effect slot. */
typedef struct {
    uint8_t report_id;            /* = 11 */
    uint8_t effect_block_index;
} __attribute__((packed)) pid_block_free_report_t;

/* Report ID 12 / 0x0C (Output): PID Device Control — device-wide commands. */
typedef struct {
    uint8_t report_id;            /* = 12 */
    uint8_t device_control;       /* 1=EnableActuators, 2=DisableActuators,
                                   * 3=StopAll, 4=DeviceReset, 5=Pause, 6=Continue */
} __attribute__((packed)) pid_device_control_report_t;

/* Generic read buffer — sized for the largest output report (10 bytes). */
typedef struct {
    uint8_t report_id;
    uint8_t data[16];  /* Sufficient for the 9-byte max PID output report payload. */
} __attribute__((packed)) ffb_generic_report_t;

// FFB Effect Slot Table — persistent storage keyed by USB PID effect block index.
// Each slot lives until explicitly stopped or its duration elapses, replacing the
// single-cycle queue that caused all effects to die after one loop iteration.
typedef struct {
    ffb_motor_effect_t effect;
    int occupied;           /* 1 once a Set Effect report has populated this slot */
    int running;            /* 1 while the effect is currently active */
    struct timespec start_time; /* CLOCK_MONOTONIC timestamp when the effect started running */
} effect_slot_t;

static effect_slot_t effect_slots[FFB_EFFECT_SLOTS];
static int last_block_index = 0; /* 0-based index of the most recently referenced slot */
static pthread_mutex_t slot_mutex = PTHREAD_MUTEX_INITIALIZER;

// Thread running flag
volatile int hid_running = 0;

static int hidg_fd = -1;
static pthread_mutex_t hidg_fd_mutex = PTHREAD_MUTEX_INITIALIZER;

// USB connection state tracking
// _Atomic so the main thread and the reception thread can read/write these
// flags without holding hidg_fd_mutex (which is now only held briefly to
// snapshot the file descriptor, not across blocking I/O).  Issue #16.
static _Atomic int usb_connected = 0;
static _Atomic int consecutive_write_failures = 0;
static struct timespec last_reconnect_attempt = {0, 0};
// Protects last_reconnect_attempt which is a non-atomic struct timespec shared
// between the reception thread and the main-thread send path.  Issue #16.
static pthread_mutex_t reconnect_mutex = PTHREAD_MUTEX_INITIALIZER;

// Error tracking
static int total_write_errors = 0;
static int total_read_errors = 0;
static int reconnect_count = 0;

// Debugging - reduced frequency to avoid spam
static int report_counter = 0;
static int debug_every_n_reports = 1000;  // Debug every 1000 reports

// Threads
static pthread_t ffb_reception_thread;

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
        atomic_store_explicit(&usb_connected, 0, memory_order_release);
        printf("HIDInterface: Closed HID device\n");
    }
    pthread_mutex_unlock(&hidg_fd_mutex);
}

// Internal close: call ONLY while hidg_fd_mutex is already held to avoid deadlock
static void _close_hid_device_nolock(void) {
    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1;
        atomic_store_explicit(&usb_connected, 0, memory_order_release);
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
    
    atomic_store_explicit(&usb_connected, 1, memory_order_release);
    atomic_store_explicit(&consecutive_write_failures, 0, memory_order_relaxed);
    reconnect_count++;
    
    printf("HIDInterface: Successfully opened HID device (reconnect #%d)\n", reconnect_count);
    
    pthread_mutex_unlock(&hidg_fd_mutex);
    return 0;
}

// Check if we should attempt reconnection
static int should_attempt_reconnect() {
    /* Fast atomic check first — avoids taking reconnect_mutex on every call
     * from the hot send path.  Issue #16.                                    */
    if (atomic_load_explicit(&usb_connected, memory_order_relaxed)) return 0;

    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    pthread_mutex_lock(&reconnect_mutex);
    int result = 0;
    if (last_reconnect_attempt.tv_sec == 0 && last_reconnect_attempt.tv_nsec == 0) {
        last_reconnect_attempt = current_time;
        result = 1;
    } else {
        long elapsed_ms = timespec_diff_ms(&last_reconnect_attempt, &current_time);
        if (elapsed_ms >= USB_RECONNECT_DELAY_MS) {
            last_reconnect_attempt = current_time;
            result = 1;
        }
    }
    pthread_mutex_unlock(&reconnect_mutex);
    return result;
}

/**
 * @brief Maps a raw USB HID PID effect-type usage value to the internal enum.
 *
 * Args:
 *     raw: Effect type byte from the HID report (USB HID Usage Tables, PID section).
 *
 * Returns:
 *     Corresponding ffb_effect_type_t; falls back to FFB_EFFECT_CONSTANT_FORCE for
 *     unknown values so the effect is always reachable rather than silently lost.
 */
static ffb_effect_type_t map_raw_effect_type(uint8_t raw) {
    switch (raw) {
        case 0x01: return FFB_EFFECT_CONSTANT_FORCE;
        case 0x02: return FFB_EFFECT_RAMP;
        case 0x03: /* square   — fall-through */
        case 0x04: /* sine     — fall-through */
        case 0x05: /* triangle — fall-through */
        case 0x06: /* saw-up   — fall-through */
        case 0x07: return FFB_EFFECT_PERIODIC; /* saw-down */
        case 0x08: return FFB_EFFECT_SPRING;
        case 0x09: return FFB_EFFECT_DAMPER;
        case 0x0A: return FFB_EFFECT_INERTIA;
        case 0x0B: return FFB_EFFECT_FRICTION;
        default:   return FFB_EFFECT_CONSTANT_FORCE;
    }
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
        case 1: { /* Set Effect — defines type, duration, and gain for an effect slot. */
            if (len < sizeof(pid_set_effect_report_t)) return 0;
            const pid_set_effect_report_t *r = (const pid_set_effect_report_t *)report;
            effect->effect_type = r->effect_type;
            effect->type        = map_raw_effect_type(r->effect_type);
            /* duration 0xFFFF means infinite; map to duration_ms == 0 for that convention */
            effect->duration_ms = (r->duration == 0xFFFF) ? 0 : (int)r->duration;
            return 1;
        }
        case 5: { /* Set Constant Force — magnitude for ID_CONSTANT_FORCE slots. */
            if (len < sizeof(pid_set_constant_force_report_t)) return 0;
            const pid_set_constant_force_report_t *r =
                (const pid_set_constant_force_report_t *)report;
            effect->magnitude = (float)r->magnitude / 32767.0f;
            return 1;
        }
        case 4: { /* Set Periodic — magnitude, offset, phase, period for periodic slots. */
            if (len < sizeof(pid_set_periodic_report_t)) return 0;
            const pid_set_periodic_report_t *r = (const pid_set_periodic_report_t *)report;
            effect->magnitude = (float)r->magnitude / 65535.0f;
            effect->direction = (float)r->offset; /* direction reused as offset per ffb_types.h */
            return 1;
        }
        case 10: /* Effect Operation — start/stop handled entirely in thread */
        case 11: /* PID Block Free — slot release handled in thread */
        case 12: /* PID Device Control — device-wide command handled in thread */
            return 1;
        default:
            return 0;
    }
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
        if (atomic_load_explicit(&usb_connected, memory_order_acquire)) {
            /* Issue #12: Snapshot the fd under a brief lock, then release the
             * mutex before the blocking select()/read().  If the fd is closed
             * concurrently, select()/read() return EBADF which is handled as a
             * recoverable error below.  This prevents the main thread from
             * waiting up to 5 ms to acquire hidg_fd_mutex on every cycle.     */
            pthread_mutex_lock(&hidg_fd_mutex);
            int fd = hidg_fd;
            pthread_mutex_unlock(&hidg_fd_mutex);

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
                        safe_close_hid_device(); /* acquires hidg_fd_mutex internally */
                        read_failures = 0;
                    }
                } else if (retval) {
                    ssize_t len = read(fd, &ffb_report, sizeof(ffb_report));
                    if (len > 0) {
                        read_failures = 0;
                        
                        ffb_motor_effect_t new_effect;
                        if (parse_ffb_report((uint8_t*)&ffb_report, (size_t)len, &new_effect)) {
                            uint8_t report_id = new_effect.report_id;

                            /* PID Device Control (ID 12) has no effect_block_index; handle
                             * separately so it doesn't corrupt last_block_index. */
                            if (report_id == 12) {
                                if (len >= (ssize_t)sizeof(pid_device_control_report_t)) {
                                    const pid_device_control_report_t *ctrl =
                                        (const pid_device_control_report_t *)&ffb_report;
                                    pthread_mutex_lock(&slot_mutex);
                                    switch (ctrl->device_control) {
                                        case 3: /* Stop All Effects */
                                            for (int s = 0; s < FFB_EFFECT_SLOTS; s++) {
                                                effect_slots[s].running = 0;
                                            }
                                            break;
                                        case 4: /* Device Reset */
                                            for (int s = 0; s < FFB_EFFECT_SLOTS; s++) {
                                                effect_slots[s].running  = 0;
                                                effect_slots[s].occupied = 0;
                                            }
                                            break;
                                        default:
                                            break; /* Pause/Continue/Enable/Disable: no slot change */
                                    }
                                    pthread_mutex_unlock(&slot_mutex);
                                }
                            } else {
                                /* All other recognised PID output reports carry
                                 * effect_block_index as the first data byte. */
                                int block_idx = (int)(ffb_report.data[0] % FFB_EFFECT_SLOTS);
                                last_block_index = block_idx;

                                pthread_mutex_lock(&slot_mutex);
                                effect_slot_t *slot = &effect_slots[block_idx];

                                switch (report_id) {
                                    case 1: /* Set Effect: initialise or overwrite the slot */
                                        slot->effect   = new_effect;
                                        slot->occupied = 1;
                                        /* Effect only starts when Effect Operation Start arrives */
                                        slot->running  = 0;
                                        break;
                                    case 5: /* Set Constant Force */
                                    case 4: /* Set Periodic */
                                        if (slot->occupied) {
                                            ffb_effect_type_t saved_type = slot->effect.type;
                                            slot->effect.magnitude = new_effect.magnitude;
                                            slot->effect.direction = new_effect.direction;
                                            slot->effect.type      = saved_type;
                                        }
                                        break;
                                    case 10: { /* Effect Operation */
                                        if (slot->occupied &&
                                            len >= (ssize_t)sizeof(pid_effect_operation_report_t)) {
                                            const pid_effect_operation_report_t *op =
                                                (const pid_effect_operation_report_t *)&ffb_report;
                                            switch (op->operation) {
                                                case 1: /* Start */
                                                    slot->running = 1;
                                                    clock_gettime(CLOCK_MONOTONIC, &slot->start_time);
                                                    break;
                                                case 2: /* Start Solo: stop all others first */
                                                    for (int s = 0; s < FFB_EFFECT_SLOTS; s++) {
                                                        if (s != block_idx) {
                                                            effect_slots[s].running = 0;
                                                        }
                                                    }
                                                    slot->running = 1;
                                                    clock_gettime(CLOCK_MONOTONIC, &slot->start_time);
                                                    break;
                                                case 3: /* Stop */
                                                    slot->running = 0;
                                                    break;
                                            }
                                        }
                                        break;
                                    }
                                    case 11: /* PID Block Free */
                                        slot->occupied = 0;
                                        slot->running  = 0;
                                        break;
                                }

                                pthread_mutex_unlock(&slot_mutex);
                            }
                        }
                    } else if (len < 0) {
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            perror("FFB Reception Thread: read error");
                            read_failures++;
                            total_read_errors++;
                            if (read_failures > 10) {
                                safe_close_hid_device(); /* acquires hidg_fd_mutex internally */
                                read_failures = 0;
                            }
                        }
                    }
                }
            }
        }
        
        // Small delay to prevent busy waiting
        usleep(1000); // 1ms
    }

    printf("FFB: Reception thread stopped\n");
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

    /* Issue #13: The gamepad report thread was an empty timing shell that did
     * nothing except print stats.  Reports are sent directly from the main
     * loop via hid_interface_send_gamepad_report().  Thread removed.         */
    if (pthread_create(&ffb_reception_thread, NULL, _usb_ffb_reception_thread, NULL) != 0) {
        perror("HIDInterface: Failed to create FFB reception thread");
        return -1;
    }

    printf("HIDInterface: Started FFB reception thread.\n");
    return 0;
}

/**
 * @brief Stops the HID communication and cleans up resources.
 */
void hid_interface_stop() {
    hid_running = 0;

    if (ffb_reception_thread) {
        pthread_join(ffb_reception_thread, NULL);
    }

    safe_close_hid_device();

    printf("HIDInterface: Stopped. Stats - Write errors: %d, Read errors: %d, Reconnects: %d\n", 
           total_write_errors, total_read_errors, reconnect_count);
}

/**
 * @brief Returns all currently running effects from the persistent slot table.
 *
 * Effects that have exceeded their duration_ms are automatically expired.
 * Effects with duration_ms == 0 run indefinitely until overwritten.
 *
 * Args:
 *     effect_array: Buffer to receive the active effect copies.
 *     max_count:    Maximum number of entries to write.
 *
 * Returns:
 *     Number of active effects written to effect_array.
 */
int hid_interface_get_active_effects(ffb_motor_effect_t *effect_array, int max_count) {
    int count = 0;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    pthread_mutex_lock(&slot_mutex);
    for (int i = 0; i < FFB_EFFECT_SLOTS && count < max_count; i++) {
        effect_slot_t *slot = &effect_slots[i];
        if (!slot->occupied || !slot->running) continue;

        /* Expire timed effects whose duration has elapsed */
        if (slot->effect.duration_ms > 0) {
            long elapsed_ms = (now.tv_sec  - slot->start_time.tv_sec)  * 1000L +
                              (now.tv_nsec - slot->start_time.tv_nsec) / 1000000L;
            if (elapsed_ms >= (long)slot->effect.duration_ms) {
                slot->running = 0;
                continue;
            }
        }

        effect_array[count++] = slot->effect;
    }
    pthread_mutex_unlock(&slot_mutex);
    return count;
}

/**
 * @brief Retrieves the latest FFB effect from the slot table (convenience wrapper).
 *
 * Returns the first active effect found.  Prefer hid_interface_get_active_effects()
 * when the caller needs to handle multiple simultaneous effects.
 *
 * Args:
 *     effect_out: Pointer to receive the effect data.
 *
 * Returns:
 *     1 if an active effect was found, 0 otherwise.
 */
int hid_interface_get_ffb_effect(ffb_motor_effect_t *effect_out) {
    return (hid_interface_get_active_effects(effect_out, 1) == 1) ? 1 : 0;
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
    return atomic_load_explicit(&usb_connected, memory_order_relaxed);
}

/**
 * @brief Get error statistics
 */
void hid_interface_get_stats(int *write_errors, int *read_errors, int *reconnects) {
    if (write_errors) *write_errors = total_write_errors;
    if (read_errors) *read_errors = total_read_errors;
    if (reconnects) *reconnects = reconnect_count;
}