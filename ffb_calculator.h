#ifndef FFB_CALCULATOR_H
#define FFB_CALCULATOR_H

#include "ffb_types.h"

// FFB calculator function prototypes
void ffb_calculator_init(void);
void ffb_calculator_update(float position, float velocity, float acceleration);
void ffb_calculator_process_effect(const ffb_motor_effect_t *effect);
float ffb_calculator_get_torque(void);
void ffb_calculator_set_gains(float spring_gain, float damper_gain, float inertia_gain);

/**
 * @brief Returns diagnostic statistics from the FFB calculator.
 * Used by main_v2.c for performance monitoring.
 */
void ffb_calculator_get_stats(uint32_t *calc_count, uint32_t *effect_changes, float *last_torque);

/**
 * @brief Initializes the FFB calculator.
 */
void ffb_calculator_init();

/**
 * @brief Calculates the desired torque based on the FFB effect and current wheel position.
 * @param effect Pointer to the current FFB effect. Can be NULL if no effect.
 * @param current_position The current angular position of the steering wheel.
 * @param current_velocity The current angular velocity of the steering wheel (for damper/inertia).
 * @return The calculated desired torque value.
 */
float ffb_calculator_calculate_torque(const ffb_motor_effect_t *effect, float current_position, float current_velocity);

#endif // FFB_CALCULATOR_H