// synapticon_servo_tuning.c - Optimized parameters for Synapticon servo drives
// Configure for direct-drive steering wheel application

#include "synapticon_servo_tuning.h"
#include "soem_interface.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>

// Optimized parameters for direct-drive steering wheel
static const synapticon_tuning_params_t steering_wheel_params = {
    // Motion limits (conservative for safety)
    .max_velocity_rpm = 300,            // 300 RPM = 1800°/sec (very fast for steering)
    .max_acceleration_rpm_s = 1000,     // 1000 RPM/s acceleration
    .max_deceleration_rpm_s = 2000,     // 2000 RPM/s deceleration (faster for safety)
    
    // Torque settings for realistic steering feel
    .rated_torque_per_mille = 1000,     // 100% of rated torque available
    .max_torque_per_mille = 800,        // Limit to 80% for safety margin
    .torque_slope_per_mille_s = 50000,  // Fast torque ramp for responsive feel
    
    // Position control (loose for free movement)
    .position_window_counts = 100,      // ±100 counts tolerance
    .position_time_ms = 50,             // 50ms settle time
    
    // High-resolution feedback
    .encoder_resolution = SYNAPTICON_ENCODER_RESOLUTION,
    .interpolation_time_ms = 1,         // 1ms interpolation for 1kHz control
    
    // Safety configuration
    .quick_stop_option = 2,             // Immediate torque off
    .following_error_window = 5000,     // 5000 counts following error limit
};

// Function to configure Synapticon servo for steering wheel application
int configure_synapticon_for_steering_wheel(uint16_t slave_idx) {
    int result = 0;
    
    printf("Synapticon: Configuring servo drive for direct-drive steering wheel...\n");
    printf("Synapticon: Encoder resolution: %u counts/revolution (%.4f°/count)\n", 
           SYNAPTICON_ENCODER_RESOLUTION, DEGREES_PER_COUNT);
    
    // 1. Set operation mode to torque control
    int8_t operation_mode = 4;  // Torque mode
    if (soem_interface_write_sdo(slave_idx, 0x6060, 0x00, sizeof(operation_mode), &operation_mode) != 0) {
        fprintf(stderr, "Synapticon: Failed to set torque mode\n");
        result = -1;
    } else {
        printf("Synapticon: Set to torque control mode\n");
    }
    
    // 2. Configure motion limits
    if (soem_interface_write_sdo(slave_idx, 0x607F, 0x00, 
                                sizeof(steering_wheel_params.max_velocity_rpm), 
                                &steering_wheel_params.max_velocity_rpm) == 0) {
        printf("Synapticon: Max velocity set to %u RPM\n", steering_wheel_params.max_velocity_rpm);
    }
    
    if (soem_interface_write_sdo(slave_idx, 0x6083, 0x00, 
                                sizeof(steering_wheel_params.max_acceleration_rpm_s), 
                                &steering_wheel_params.max_acceleration_rpm_s) == 0) {
        printf("Synapticon: Max acceleration set to %u RPM/s\n", steering_wheel_params.max_acceleration_rpm_s);
    }
    
    if (soem_interface_write_sdo(slave_idx, 0x6084, 0x00, 
                                sizeof(steering_wheel_params.max_deceleration_rpm_s), 
                                &steering_wheel_params.max_deceleration_rpm_s) == 0) {
        printf("Synapticon: Max deceleration set to %u RPM/s\n", steering_wheel_params.max_deceleration_rpm_s);
    }
    
    // 3. Configure torque parameters
    if (soem_interface_write_sdo(slave_idx, 0x6072, 0x00, 
                                sizeof(steering_wheel_params.max_torque_per_mille), 
                                &steering_wheel_params.max_torque_per_mille) == 0) {
        printf("Synapticon: Max torque set to %u per mille\n", steering_wheel_params.max_torque_per_mille);
    }
    
    if (soem_interface_write_sdo(slave_idx, 0x6087, 0x00, 
                                sizeof(steering_wheel_params.torque_slope_per_mille_s), 
                                &steering_wheel_params.torque_slope_per_mille_s) == 0) {
        printf("Synapticon: Torque slope set to %u per mille/s\n", steering_wheel_params.torque_slope_per_mille_s);
    }
    
    // 4. Configure high-resolution encoder feedback
    uint32_t encoder_increments = steering_wheel_params.encoder_resolution;
    if (soem_interface_write_sdo(slave_idx, 0x608F, 0x01, sizeof(encoder_increments), &encoder_increments) == 0) {
        printf("Synapticon: Encoder resolution confirmed: %u increments/revolution\n", encoder_increments);
    }
    
    // 5. Set interpolation time for smooth operation
    uint8_t interpolation_time = steering_wheel_params.interpolation_time_ms;
    int8_t interpolation_index = -3;  // 10^-3 = milliseconds
    if (soem_interface_write_sdo(slave_idx, 0x60C2, 0x01, sizeof(interpolation_time), &interpolation_time) == 0 &&
        soem_interface_write_sdo(slave_idx, 0x60C2, 0x02, sizeof(interpolation_index), &interpolation_index) == 0) {
        printf("Synapticon: Interpolation time set to %u ms\n", interpolation_time);
    }
    
    // 6. Configure safety parameters
    if (soem_interface_write_sdo(slave_idx, 0x605A, 0x00, 
                                sizeof(steering_wheel_params.quick_stop_option), 
                                &steering_wheel_params.quick_stop_option) == 0) {
        printf("Synapticon: Quick stop configured for immediate torque off\n");
    }
    
    if (soem_interface_write_sdo(slave_idx, 0x6065, 0x00, 
                                sizeof(steering_wheel_params.following_error_window), 
                                &steering_wheel_params.following_error_window) == 0) {
        printf("Synapticon: Following error window set to %u counts\n", steering_wheel_params.following_error_window);
    }
    
    // 7. Configure position window for loose control (allows free movement)
    if (soem_interface_write_sdo(slave_idx, 0x6067, 0x00, 
                                sizeof(steering_wheel_params.position_window_counts), 
                                &steering_wheel_params.position_window_counts) == 0) {
        printf("Synapticon: Position window set to %u counts (±%.2f degrees)\n", 
               steering_wheel_params.position_window_counts,
               steering_wheel_params.position_window_counts * DEGREES_PER_COUNT);
    }
    
    // 8. Set gear ratio (1:1 for direct drive)
    uint32_t gear_numerator = 1;
    uint32_t gear_denominator = 1;
    soem_interface_write_sdo(slave_idx, 0x608F, 0x02, sizeof(gear_numerator), &gear_numerator);
    soem_interface_write_sdo(slave_idx, 0x6091, 0x01, sizeof(gear_denominator), &gear_denominator);
    printf("Synapticon: Gear ratio set to 1:1 (direct drive)\n");
    
    // 9. Enable digital inputs/outputs if needed for buttons/LEDs
    // This is optional and depends on your specific Synapticon model
    
    // 10. Verify configuration by reading back key parameters
    printf("Synapticon: Verifying configuration...\n");
    
    int8_t current_mode = 0;
    if (soem_interface_read_sdo(slave_idx, 0x6061, 0x00, sizeof(current_mode), &current_mode) == 0) {
        if (current_mode == 4) {
            printf("Synapticon: Configuration verified - Operating in torque mode\n");
        } else {
            printf("Synapticon: Warning - Mode is %d, expected 4 (torque mode)\n", current_mode);
        }
    }
    
    // Wait for parameters to be processed
    usleep(100000); // 100ms
    
    printf("Synapticon: Servo configuration completed\n");
    printf("Synapticon: Ready for high-frequency force feedback control\n");
    printf("Synapticon: Steering range: ±%.0f degrees (±%.1f revolutions)\n", 
           MAX_STEERING_ANGLE, MAX_STEERING_ANGLE / 360.0f);
    printf("Synapticon: Position resolution: %.4f degrees per encoder count\n", DEGREES_PER_COUNT);
    
    return result;
}

// Function to read and display current Synapticon status
void print_synapticon_status(uint16_t slave_idx) {
    printf("\n=== Synapticon Servo Status ===\n");
    
    // Read current operation mode
    int8_t current_mode = 0;
    if (soem_interface_read_sdo(slave_idx, 0x6061, 0x00, sizeof(current_mode), &current_mode) == 0) {
        const char* mode_names[] = {"", "Profile Position", "Velocity", "Profile Velocity", 
                                   "Torque", "Reserved", "Homing", "Interpolated Position"};
        printf("Operation Mode: %d (%s)\n", current_mode, 
               (current_mode >= 1 && current_mode <= 7) ? mode_names[current_mode] : "Unknown");
    }
    
    // Read current limits
    uint32_t max_vel = 0;
    if (soem_interface_read_sdo(slave_idx, 0x607F, 0x00, sizeof(max_vel), &max_vel) == 0) {
        printf("Max Velocity: %u RPM (%.1f degrees/sec)\n", max_vel, max_vel * 6.0f);
    }
    
    uint16_t max_torque = 0;
    if (soem_interface_read_sdo(slave_idx, 0x6072, 0x00, sizeof(max_torque), &max_torque) == 0) {
        printf("Max Torque: %u per mille (%.1f%%)\n", max_torque, max_torque / 10.0f);
    }
    
    // Read encoder configuration
    uint32_t encoder_res = 0;
    if (soem_interface_read_sdo(slave_idx, 0x608F, 0x01, sizeof(encoder_res), &encoder_res) == 0) {
        printf("Encoder Resolution: %u counts/rev (%.4f°/count)\n", 
               encoder_res, 360.0f / encoder_res);
    }
    
    printf("==============================\n\n");
}

// Utility functions for unit conversions
float synapticon_counts_to_degrees(int32_t counts) {
    return counts * DEGREES_PER_COUNT;
}

int32_t synapticon_degrees_to_counts(float degrees) {
    return (int32_t)(degrees / DEGREES_PER_COUNT);
}

float synapticon_rpm_to_degrees_per_sec(uint32_t rpm) {
    return rpm * 6.0f; // RPM * 360°/60sec = RPM * 6
}

uint32_t synapticon_degrees_per_sec_to_rpm(float deg_per_sec) {
    return (uint32_t)(deg_per_sec / 6.0f);
}

// Get the steering wheel parameters (read-only access)
const synapticon_tuning_params_t* get_steering_wheel_params(void) {
    return &steering_wheel_params;
}

// Performance monitoring specific to steering wheel application
void monitor_steering_wheel_performance(float current_angle, float current_velocity, float current_torque) {
    static float max_angle = 0.0f;
    static float max_velocity = 0.0f;
    static float max_torque = 0.0f;
    static uint32_t sample_count = 0;
    static float angle_sum = 0.0f;
    static float velocity_sum = 0.0f;
    static float torque_sum = 0.0f;
    
    sample_count++;
    angle_sum += fabsf(current_angle);
    velocity_sum += fabsf(current_velocity);
    torque_sum += fabsf(current_torque);
    
    if (fabsf(current_angle) > max_angle) max_angle = fabsf(current_angle);
    if (fabsf(current_velocity) > max_velocity) max_velocity = fabsf(current_velocity);
    if (fabsf(current_torque) > max_torque) max_torque = fabsf(current_torque);
    
    // Print statistics every 10 seconds (10000 samples at 1kHz)
    if (sample_count % 10000 == 0) {
        printf("=== Steering Wheel Performance (10s window) ===\n");
        printf("Angle: Current=%.1f°, Max=%.1f°, Avg=%.1f°, Range utilization=%.1f%%\n",
               current_angle, max_angle, angle_sum / sample_count, 
               (max_angle / 540.0f) * 100.0f); // Using 540° as max steering angle
        printf("Velocity: Current=%.1f°/s, Max=%.1f°/s, Avg=%.1f°/s\n",
               current_velocity, max_velocity, velocity_sum / sample_count);
        printf("Torque: Current=%.1f, Max=%.1f, Avg=%.1f, Utilization=%.1f%%\n",
               current_torque, max_torque, torque_sum / sample_count,
               (max_torque / 5000.0f) * 100.0f); // Using 5000 as max torque limit
        printf("Encoder precision utilized: %.1f counts/degree\n", 
               1.0f / DEGREES_PER_COUNT);
        printf("===============================================\n");
        
        // Reset maximums for next window
        max_angle = max_velocity = max_torque = 0.0f;
        angle_sum = velocity_sum = torque_sum = 0.0f;
        sample_count = 0;
    }
}