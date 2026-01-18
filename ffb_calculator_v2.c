// ffb_calculator_optimized.c - Optimized servo control logic
#include "ffb_calculator.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Enhanced configuration constants
#define MAX_TORQUE_OUTPUT 5000.0f
#define MIN_TORQUE_OUTPUT -5000.0f
#define TORQUE_RATE_LIMIT 50000.0f      // Max torque change per second
#define VELOCITY_FILTER_ALPHA 0.3f      // Low-pass filter for velocity
#define POSITION_DEADBAND 0.5f          // Encoder counts deadband
#define MIN_VELOCITY_THRESHOLD 0.1f     // Minimum velocity for friction/damping

// Improved scaling factors based on typical sim racing feel
#define CONSTANT_FORCE_SCALE 800.0f     // Reduced from 1000 for smoother feel
#define SPRING_SCALE 300.0f             // Reduced from 500 for less harsh centering
#define DAMPER_SCALE 150.0f             // Reduced from 200 for smoother damping
#define INERTIA_SCALE 200.0f            // Reduced from 300 for less sluggish feel
#define FRICTION_SCALE 250.0f           // Reduced from 400 for smoother operation
#define PERIODIC_SCALE 600.0f           // Reduced from 800 for less jarring effects
#define RAMP_SCALE 800.0f               // Consistent with constant force

// Performance optimization: Pre-calculated values
static const float TWO_PI = 2.0f * M_PI;
static const float PI_OVER_128 = M_PI / 128.0f;
static const float HALF_PI = M_PI / 2.0f;

// Internal state for optimized calculations
typedef struct {
    struct timeval start_time;
    int time_initialized;
    
    // Filters and smoothing
    float velocity_filtered;
    float torque_filtered;
    float last_torque_output;
    
    // Rate limiting
    struct timespec last_update_time;
    float torque_rate_limiter;
    
    // Effect state caching
    uint8_t last_effect_id;
    ffb_effect_type_t last_effect_type;
    uint32_t last_effect_timestamp;
    
    // Performance counters
    uint32_t calculation_count;
    uint32_t effect_changes;
} ffb_state_t;

static ffb_state_t ffb_state = {0};

// Fast time functions
static inline uint32_t get_current_time_ms(void) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    
    if (!ffb_state.time_initialized) {
        ffb_state.start_time = current_time;
        ffb_state.time_initialized = 1;
        return 0;
    }
    
    return (uint32_t)((current_time.tv_sec - ffb_state.start_time.tv_sec) * 1000 + 
                      (current_time.tv_usec - ffb_state.start_time.tv_usec) / 1000);
}

// Fast math approximations for periodic effects
static inline float fast_sin(float x) {
    // Normalize to [-PI, PI]
    while (x > M_PI) x -= TWO_PI;
    while (x < -M_PI) x += TWO_PI;
    
    // Taylor series approximation (good for performance)
    float x2 = x * x;
    return x * (1.0f - x2 * (1.0f/6.0f - x2 * (1.0f/120.0f)));
}

static inline float fast_triangle_wave(float phase) {
    // Optimized triangle wave generation
    float t = phase / TWO_PI;
    t = t - floorf(t); // Get fractional part
    return (t < 0.5f) ? (4.0f * t - 1.0f) : (3.0f - 4.0f * t);
}

// Optimized low-pass filter
static inline float apply_low_pass_filter(float new_value, float old_value, float alpha) {
    return alpha * new_value + (1.0f - alpha) * old_value;
}

// Torque rate limiter for smooth transitions
static float apply_torque_rate_limiting(float desired_torque, float dt) {
    if (dt <= 0.0f || dt > 0.1f) return desired_torque; // Skip if invalid dt
    
    float max_change = TORQUE_RATE_LIMIT * dt;
    float torque_diff = desired_torque - ffb_state.last_torque_output;
    
    if (torque_diff > max_change) {
        return ffb_state.last_torque_output + max_change;
    } else if (torque_diff < -max_change) {
        return ffb_state.last_torque_output - max_change;
    }
    
    return desired_torque;
}

// Optimized spring effect with deadband and progressive response
static float calculate_spring_effect(const ffb_motor_effect_t *effect, float current_position) {
    float center_position = effect->center_position;
    float displacement = current_position - center_position;
    
    // Apply deadband
    if (fabsf(displacement) < POSITION_DEADBAND) {
        return 0.0f;
    }
    
    // Progressive spring response (softer near center, firmer at extremes)
    float abs_displacement = fabsf(displacement);
    float spring_strength = effect->spring_coefficient > 0 ? effect->spring_coefficient : effect->magnitude;
    
    // Progressive scaling: linear near center, quadratic at extremes
    float response_factor = (abs_displacement < 1000.0f) ? 
        spring_strength : 
        spring_strength * (1.0f + (abs_displacement - 1000.0f) / 5000.0f);
    
    return -displacement * response_factor * SPRING_SCALE;
}

// Optimized damper with velocity-dependent scaling
static float calculate_damper_effect(const ffb_motor_effect_t *effect, float current_velocity) {
    if (fabsf(current_velocity) < MIN_VELOCITY_THRESHOLD) {
        return 0.0f; // No damping for very small velocities
    }
    
    float damper_strength = effect->damper_coefficient > 0 ? effect->damper_coefficient : effect->magnitude;
    
    // Velocity-dependent damping (stronger at higher speeds)
    float velocity_factor = 1.0f + fabsf(current_velocity) / 1000.0f;
    velocity_factor = fminf(velocity_factor, 3.0f); // Cap at 3x
    
    return -current_velocity * damper_strength * velocity_factor * DAMPER_SCALE;
}

// Optimized periodic effect with waveform caching
static float calculate_periodic_effect(const ffb_motor_effect_t *effect, uint32_t current_time) {
    // Unpack parameters efficiently
    uint8_t waveform = (uint8_t)effect->start_delay;
    uint8_t frequency = (uint8_t)(effect->timestamp >> 16);
    uint8_t phase = (uint8_t)(effect->timestamp >> 8);
    uint8_t offset = (uint8_t)effect->timestamp;
    
    if (frequency == 0) return 0.0f; // Avoid division by zero
    
    float time_sec = current_time * 0.001f; // Convert to seconds
    float angular_freq = TWO_PI * frequency;
    float phase_rad = phase * PI_OVER_128;
    float offset_norm = (offset - 128) * (1.0f / 128.0f);
    
    float phase_total = angular_freq * time_sec + phase_rad;
    float wave_value;
    
    // Optimized waveform generation
    switch (waveform) {
        case 0: // Square wave
            wave_value = (fast_sin(phase_total) >= 0.0f) ? 1.0f : -1.0f;
            break;
        case 1: // Sine wave
            wave_value = fast_sin(phase_total);
            break;
        case 2: // Triangle wave
            wave_value = fast_triangle_wave(phase_total);
            break;
        case 3: // Sawtooth up
            wave_value = 2.0f * fmodf(phase_total, TWO_PI) / TWO_PI - 1.0f;
            break;
        case 4: // Sawtooth down
            wave_value = 1.0f - 2.0f * fmodf(phase_total, TWO_PI) / TWO_PI;
            break;
        default:
            wave_value = fast_sin(phase_total);
            break;
    }
    
    return (wave_value * effect->magnitude + offset_norm) * PERIODIC_SCALE;
}

// Main optimized calculation function
void ffb_calculator_init(void) {
    printf("FFB_Calculator: Optimized version initialized.\n");
    memset(&ffb_state, 0, sizeof(ffb_state));
    clock_gettime(CLOCK_MONOTONIC, &ffb_state.last_update_time);
}

float ffb_calculator_calculate_torque(const ffb_motor_effect_t *effect, float current_position, float current_velocity) {
    float desired_torque = 0.0f;
    uint32_t current_time = get_current_time_ms();
    
    // Calculate delta time for rate limiting
    struct timespec current_update_time;
    clock_gettime(CLOCK_MONOTONIC, &current_update_time);
    float dt = (current_update_time.tv_sec - ffb_state.last_update_time.tv_sec) + 
               (current_update_time.tv_nsec - ffb_state.last_update_time.tv_nsec) * 1e-9f;
    ffb_state.last_update_time = current_update_time;
    
    // Apply velocity filtering for smoother damping effects
    ffb_state.velocity_filtered = apply_low_pass_filter(current_velocity, ffb_state.velocity_filtered, VELOCITY_FILTER_ALPHA);
    
    // Track effect changes for optimization
    if (effect && (effect->report_id != ffb_state.last_effect_id || effect->type != ffb_state.last_effect_type)) {
        ffb_state.effect_changes++;
        ffb_state.last_effect_id = effect->report_id;
        ffb_state.last_effect_type = effect->type;
    }
    
    if (effect != NULL) {
        switch (effect->type) {
            case FFB_EFFECT_CONSTANT_FORCE:
                desired_torque = effect->magnitude * CONSTANT_FORCE_SCALE;
                break;
                
            case FFB_EFFECT_SPRING:
                desired_torque = calculate_spring_effect(effect, current_position);
                break;
                
            case FFB_EFFECT_DAMPER:
                desired_torque = calculate_damper_effect(effect, ffb_state.velocity_filtered);
                break;
                
            case FFB_EFFECT_INERTIA:
                // Simplified inertia as velocity-based resistance
                if (fabsf(ffb_state.velocity_filtered) > MIN_VELOCITY_THRESHOLD) {
                    float inertia_strength = effect->inertia_coefficient > 0 ? effect->inertia_coefficient : effect->magnitude;
                    desired_torque = -ffb_state.velocity_filtered * inertia_strength * INERTIA_SCALE;
                }
                break;
                
            case FFB_EFFECT_FRICTION:
                // Improved friction with stiction model
                if (fabsf(ffb_state.velocity_filtered) > MIN_VELOCITY_THRESHOLD) {
                    float friction_strength = effect->friction_coefficient > 0 ? effect->friction_coefficient : effect->magnitude;
                    float friction_force = friction_strength * FRICTION_SCALE;
                    
                    // Coulomb friction model
                    desired_torque = (ffb_state.velocity_filtered > 0.0f) ? -friction_force : friction_force;
                } else {
                    // Static friction (stiction) - resist small movements
                    float static_friction = effect->magnitude * FRICTION_SCALE * 0.5f;
                    desired_torque = -current_position * static_friction * 0.1f; // Very small centering force
                }
                break;
                
            case FFB_EFFECT_PERIODIC:
                desired_torque = calculate_periodic_effect(effect, current_time);
                break;
                
            case FFB_EFFECT_RAMP:
                if (effect->duration > 0) {
                    float start_magnitude = effect->magnitude;
                    float end_magnitude = effect->direction;
                    float elapsed_time = current_time - effect->timestamp;
                    float progress = elapsed_time / effect->duration;
                    
                    progress = fmaxf(0.0f, fminf(1.0f, progress)); // Clamp [0,1]
                    
                    float current_magnitude = start_magnitude + (end_magnitude - start_magnitude) * progress;
                    desired_torque = current_magnitude * RAMP_SCALE;
                } else {
                    desired_torque = effect->magnitude * RAMP_SCALE;
                }
                break;
                
            default:
                desired_torque = 0.0f;
                break;
        }
    }
    
    // Apply rate limiting for smoother torque transitions
    desired_torque = apply_torque_rate_limiting(desired_torque, dt);
    
    // Apply final torque limits with soft limiting near boundaries
    if (desired_torque > MAX_TORQUE_OUTPUT * 0.9f) {
        // Soft limiting in the last 10%
        float excess = desired_torque - MAX_TORQUE_OUTPUT * 0.9f;
        float max_excess = MAX_TORQUE_OUTPUT * 0.1f;
        float soft_factor = 1.0f - (excess / max_excess) * 0.5f; // 50% reduction at limit
        desired_torque = MAX_TORQUE_OUTPUT * 0.9f + excess * soft_factor;
        desired_torque = fminf(desired_torque, MAX_TORQUE_OUTPUT);
    } else if (desired_torque < MIN_TORQUE_OUTPUT * 0.9f) {
        float excess = desired_torque - MIN_TORQUE_OUTPUT * 0.9f;
        float max_excess = MIN_TORQUE_OUTPUT * 0.1f;
        float soft_factor = 1.0f - (-excess / -max_excess) * 0.5f;
        desired_torque = MIN_TORQUE_OUTPUT * 0.9f + excess * soft_factor;
        desired_torque = fmaxf(desired_torque, MIN_TORQUE_OUTPUT);
    }
    
    // Update state
    ffb_state.last_torque_output = desired_torque;
    ffb_state.calculation_count++;
    
    return desired_torque;
}

// Diagnostic function for performance monitoring
void ffb_calculator_get_stats(uint32_t *calc_count, uint32_t *effect_changes, float *last_torque) {
    if (calc_count) *calc_count = ffb_state.calculation_count;
    if (effect_changes) *effect_changes = ffb_state.effect_changes;
    if (last_torque) *last_torque = ffb_state.last_torque_output;
}