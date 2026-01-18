// synapticon_servo_tuning.h - Header file for Synapticon servo configuration
#ifndef SYNAPTICON_SERVO_TUNING_H
#define SYNAPTICON_SERVO_TUNING_H

#include <stdint.h>

// Synapticon-specific encoder configuration
// Update these values based on your actual encoder specification
#define SYNAPTICON_ENCODER_RESOLUTION 65536  // 16-bit = 2^16 counts/revolution
#define SYNAPTICON_ENCODER_BITS 16           // Bit resolution
#define DEGREES_PER_COUNT (360.0f / SYNAPTICON_ENCODER_RESOLUTION)

// Synapticon servo tuning parameters structure
typedef struct {
    // Motion control parameters
    uint32_t max_velocity_rpm;          // Maximum velocity in RPM
    uint32_t max_acceleration_rpm_s;    // Max acceleration in RPM/s
    uint32_t max_deceleration_rpm_s;    // Max deceleration in RPM/s
    
    // Torque control parameters
    uint16_t rated_torque_per_mille;    // Rated torque in per mille (1000 = 100%)
    uint16_t max_torque_per_mille;      // Maximum torque limit
    uint32_t torque_slope_per_mille_s;  // Torque ramp rate
    
    // Position control parameters
    uint32_t position_window_counts;    // Position tolerance window
    uint16_t position_time_ms;          // Time to stay in position window
    
    // Encoder and feedback parameters
    uint32_t encoder_resolution;        // Encoder counts per revolution
    uint16_t interpolation_time_ms;     // Interpolation period
    
    // Safety parameters
    uint16_t quick_stop_option;         // Quick stop behavior
    uint32_t following_error_window;    // Following error limit
    
} synapticon_tuning_params_t;

// Function declarations
int configure_synapticon_for_steering_wheel(uint16_t slave_idx);
void print_synapticon_status(uint16_t slave_idx);
void monitor_steering_wheel_performance(float current_angle, float current_velocity, float current_torque);
const synapticon_tuning_params_t* get_steering_wheel_params(void);

// Utility functions
float synapticon_counts_to_degrees(int32_t counts);
int32_t synapticon_degrees_to_counts(float degrees);
float synapticon_rpm_to_degrees_per_sec(uint32_t rpm);
uint32_t synapticon_degrees_per_sec_to_rpm(float deg_per_sec);

#endif // SYNAPTICON_SERVO_TUNING_H