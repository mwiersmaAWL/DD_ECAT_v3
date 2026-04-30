// ffb_calculator.c
#include "ffb_calculator.h"
#include <stdio.h> // For printf (for debugging)
#include <math.h>  // For fmax, fmin, sin, cos
#include <stdint.h>
#include <sys/time.h> // For gettimeofday
#include <time.h>

// Define pi value 
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Internal state for time-based effects
static struct timeval start_time;
static int time_initialized = 0;

// Helper function to get current time in milliseconds
static uint32_t get_current_time_ms() {
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    
    if (!time_initialized) {
        start_time = current_time;
        time_initialized = 1;
        return 0;
    }
    
    return (uint32_t)((current_time.tv_sec - start_time.tv_sec) * 1000 + 
                      (current_time.tv_usec - start_time.tv_usec) / 1000);
}

/**
 * @brief Initializes the FFB calculator.
 */
void ffb_calculator_init() {
    printf("FFB_Calculator: Initialized.\n");
    time_initialized = 0;
    // Any setup for internal state variables can go here
}

/**
 * @brief Calculates the desired torque based on the FFB effect and current wheel position.
 * @param effect Pointer to the current FFB effect. Can be NULL if no effect.
 * @param current_position The current angular position of the steering wheel.
 * @param current_velocity The current angular velocity of the steering wheel (for damper/inertia).
 * @return The calculated desired torque value.
 */
float ffb_calculator_calculate_torque(const ffb_motor_effect_t *effect, float current_position, float current_velocity) {
    float desired_torque = 0.0f;

    if (effect != NULL) {
        uint32_t current_time = get_current_time_ms();
        
        switch (effect->type) {
            case FFB_EFFECT_CONSTANT_FORCE:
                // Apply a constant force based on magnitude
                // Scale factor (e.g., 1000) converts normalized magnitude to motor units
                desired_torque = effect->magnitude * 1000.0f;
                break;
                
            case FFB_EFFECT_SPRING:
                // Spring effect: Torque proportional to displacement from a center point.
                // Use spring coefficient if available, otherwise use magnitude
                float center_position = 0.0f; // Assuming center is 0
                float spring_strength = (effect->spring_coefficient > 0) ? effect->spring_coefficient : effect->magnitude;
                float spring_gain = 0.8f * spring_strength;
                desired_torque = -(current_position - center_position) * spring_gain * 500.0f;
                break;
                
            case FFB_EFFECT_DAMPER:
                // Damper effect: Torque proportional to velocity, opposing motion.
                float damper_strength = (effect->damper_coefficient > 0) ? effect->damper_coefficient : effect->magnitude;
                float damper_gain = 1.0f * damper_strength;
                desired_torque = -current_velocity * damper_gain * 200.0f;
                break;
                
            case FFB_EFFECT_INERTIA:
                // Inertia effect: Resistance to acceleration (simplified as velocity-based)
                float inertia_strength = (effect->inertia_coefficient > 0) ? effect->inertia_coefficient : effect->magnitude;
                float inertia_gain = 0.5f * inertia_strength;
                desired_torque = -current_velocity * inertia_gain * 300.0f;
                break;
                
            case FFB_EFFECT_FRICTION:
                // Friction effect: Constant resistance to motion
                float friction_strength = (effect->friction_coefficient > 0) ? effect->friction_coefficient : effect->magnitude;
                float friction_force = friction_strength * 400.0f;
                
                // Apply friction opposite to motion direction
                if (current_velocity > 0.01f) {
                    desired_torque = -friction_force;
                } else if (current_velocity < -0.01f) {
                    desired_torque = friction_force;
                } else {
                    desired_torque = 0.0f; // No friction when stationary
                }
                break;
                
            case FFB_EFFECT_PERIODIC:
                // Periodic effect: Sine wave, square wave, etc.
                // Unpack parameters from the stored fields
                uint8_t waveform = (uint8_t)effect->start_delay;
                uint8_t frequency = (uint8_t)(effect->timestamp >> 16);
                uint8_t phase = (uint8_t)(effect->timestamp >> 8);
                uint8_t offset = (uint8_t)effect->timestamp;
                
                float time_sec = current_time / 1000.0f;
                float angular_freq = 2.0f * M_PI * frequency;
                float phase_rad = phase * M_PI / 128.0f; // Convert to radians
                float offset_norm = (offset - 128) / 128.0f; // Normalize offset
                
                float wave_value = 0.0f;
                switch (waveform) {
                    case 0: // Square wave
                        wave_value = (sinf(angular_freq * time_sec + phase_rad) >= 0) ? 1.0f : -1.0f;
                        break;
                    case 1: // Sine wave
                        wave_value = sinf(angular_freq * time_sec + phase_rad);
                        break;
                    case 2: // Triangle wave
                        {
                            float t = fmodf(angular_freq * time_sec + phase_rad, 2.0f * M_PI) / (2.0f * M_PI);
                            wave_value = (t < 0.5f) ? (4.0f * t - 1.0f) : (3.0f - 4.0f * t);
                        }
                        break;
                    case 3: // Sawtooth up
                        wave_value = 2.0f * fmodf(angular_freq * time_sec + phase_rad, 2.0f * M_PI) / (2.0f * M_PI) - 1.0f;
                        break;
                    case 4: // Sawtooth down
                        wave_value = 1.0f - 2.0f * fmodf(angular_freq * time_sec + phase_rad, 2.0f * M_PI) / (2.0f * M_PI);
                        break;
                    default:
                        wave_value = sinf(angular_freq * time_sec + phase_rad); // Default to sine
                        break;
                }
                
                desired_torque = (wave_value * effect->magnitude + offset_norm) * 800.0f;
                break;
                
            case FFB_EFFECT_RAMP:
                // Ramp effect: Linear interpolation between start and end magnitude
                if (effect->duration > 0) {
                    float start_magnitude = effect->magnitude;
                    float end_magnitude = effect->direction; // Reused field
                    float elapsed_time = current_time - effect->timestamp;
                    float progress = elapsed_time / effect->duration;
                    
                    if (progress < 0.0f) progress = 0.0f;
                    if (progress > 1.0f) progress = 1.0f;
                    
                    float current_magnitude = start_magnitude + (end_magnitude - start_magnitude) * progress;
                    desired_torque = current_magnitude * 1000.0f;
                } else {
                    desired_torque = effect->magnitude * 1000.0f;
                }
                break;
                
            default:
                // Unknown effect type, apply no force
                desired_torque = 0.0f;
                printf("FFB_Calculator: Unknown effect type: %d\n", effect->type);
                break;
        }
    }

    // Clamp torque to motor limits
    float max_torque_output = 5000.0f; // Example max motor command units
    float min_torque_output = -5000.0f; // Example min motor command units
    desired_torque = fmaxf(min_torque_output, fminf(max_torque_output, desired_torque));

    return desired_torque;
}

// Stub implementation: ffb_calculator_v2.c has a full version with counters.
// This stub satisfies the declaration for builds using ffb_calculator.c.
void ffb_calculator_get_stats(uint32_t *calc_count, uint32_t *effect_changes, float *last_torque) {
    if (calc_count)   *calc_count    = 0;
    if (effect_changes) *effect_changes = 0;
    if (last_torque)  *last_torque   = 0.0f;
}