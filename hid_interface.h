// hid_interface.h - Updated header with new functions for better error handling
#ifndef HID_INTERFACE_H
#define HID_INTERFACE_H

#include "ffb_types.h"

/* Maximum number of concurrent USB PID effect slots (indexed by effect block index). */
#define FFB_EFFECT_SLOTS 16

// Global running flag
extern volatile int hid_running;

// Core function prototypes
int hid_interface_init();
int hid_interface_start();
void hid_interface_stop();
int hid_interface_get_ffb_effect(ffb_motor_effect_t *effect_out);
int hid_interface_get_active_effects(ffb_motor_effect_t *effect_array, int max_count);
int hid_interface_send_gamepad_report(float position, unsigned int buttons);
//void hid_interface_send_gamepad_report(float position, unsigned int buttons);
float hid_interface_get_relative_position(void);

// Configuration functions
void hid_interface_set_rate_limiting(int enable);
void hid_interface_set_report_interval(int interval_ms);
void hid_interface_recenter_wheel(void);

// Status and diagnostics functions
int hid_interface_get_connection_status();
void hid_interface_get_stats(int *write_errors, int *read_errors, int *reconnects);


#endif // HID_INTERFACE_H