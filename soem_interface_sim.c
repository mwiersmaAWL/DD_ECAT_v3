// soem_interface_sim.c - Software simulation of the SOEM EtherCAT + Synapticon servo layer.
//
// Build with: -DSIM_MODE (set automatically by the 'sim' Makefile target)
//
// Physics model: 1-DOF rotational system
//   I * alpha = target_torque - b * omega
// where:
//   I = moment of inertia (SIM_INERTIA, kg*m^2)
//   b = viscous damping (SIM_DAMPING, Nms/rad)
//   omega = angular velocity (rad/s)
//   alpha = angular acceleration (rad/s^2)
//
// Position is reported in encoder counts (65536 counts/revolution, matching main.c).
// Velocity is reported in encoder counts/second.
//
// Usage: replace soem_interface.c in the build (see Makefile_v2 'sim' target).

#ifndef PACKED
#define PACKED __attribute__((__packed__))
#endif

// Must include before soem_interface.h which uses SIM_MODE guard
#include "soem_interface.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Simulation physics constants
// ---------------------------------------------------------------------------
#define SIM_INERTIA         0.05f       // kg*m^2 — typical direct-drive steering wheel
#define SIM_DAMPING         2.0f        // Nms/rad — bearing friction
#define SIM_DT              0.001f      // Simulation time step: 1 ms (1 kHz)
#define SIM_ENCODER_CPR     65536.0f    // Counts per revolution (matches main.c)

// Scale from CiA402 per-mille torque back to SI torque (Nm).
// Assumption: 1000 per-mille = rated torque = 10 Nm (adjust to your motor spec).
#define SIM_RATED_TORQUE_NM 10.0f
#define SIM_PERMILLE_TO_NM  (SIM_RATED_TORQUE_NM / 1000.0f)

// Steering soft-stop: resist at ±540 degrees (same limit as MAX_STEERING_ANGLE in main.c)
#define SIM_MAX_ANGLE_DEG   540.0f
#define SIM_MAX_ANGLE_RAD   (SIM_MAX_ANGLE_DEG * (float)M_PI / 180.0f)
#define SIM_STOP_STIFFNESS  50.0f       // Nm/rad — virtual end-stop spring

// ---------------------------------------------------------------------------
// Simulation state
// ---------------------------------------------------------------------------
static struct {
    float position_rad;         // Current shaft angle in radians
    float velocity_rad_s;       // Current angular velocity in rad/s
    float target_torque_pm;     // Last commanded torque in per-mille of rated
} sim_state = {0.0f, 0.0f, 0.0f};

static pthread_mutex_t sim_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t sim_thread;
static volatile int sim_running = 0;

// ---------------------------------------------------------------------------
// Physics thread
// ---------------------------------------------------------------------------
static void *sim_physics_loop(void *arg) {
    (void)arg;

    struct timespec next_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

    printf("SIM: Physics thread started at %.0f Hz\n", 1.0f / SIM_DT);

    while (sim_running) {
        // Advance deadline
        long dt_ns = (long)(SIM_DT * 1e9f);
        next_wakeup.tv_nsec += dt_ns;
        if (next_wakeup.tv_nsec >= 1000000000L) {
            next_wakeup.tv_sec++;
            next_wakeup.tv_nsec -= 1000000000L;
        }

        pthread_mutex_lock(&sim_mutex);

        // Convert commanded torque from per-mille to Nm
        float torque_nm = sim_state.target_torque_pm * SIM_PERMILLE_TO_NM;

        // Soft end-stop: add restoring spring torque near steering limits
        float pos = sim_state.position_rad;
        if (pos > SIM_MAX_ANGLE_RAD) {
            torque_nm -= SIM_STOP_STIFFNESS * (pos - SIM_MAX_ANGLE_RAD);
        } else if (pos < -SIM_MAX_ANGLE_RAD) {
            torque_nm -= SIM_STOP_STIFFNESS * (pos + SIM_MAX_ANGLE_RAD);
        }

        // Euler integration:  alpha = (torque - b*omega) / I
        float omega = sim_state.velocity_rad_s;
        float alpha = (torque_nm - SIM_DAMPING * omega) / SIM_INERTIA;
        sim_state.velocity_rad_s += alpha * SIM_DT;
        sim_state.position_rad   += sim_state.velocity_rad_s * SIM_DT;

        pthread_mutex_unlock(&sim_mutex);

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
    }

    printf("SIM: Physics thread stopped\n");
    return NULL;
}

// ---------------------------------------------------------------------------
// Helper: convert radians to encoder counts
// ---------------------------------------------------------------------------
static inline float rad_to_counts(float rad) {
    return (rad / (2.0f * (float)M_PI)) * SIM_ENCODER_CPR;
}

// ---------------------------------------------------------------------------
// Public API — implements all functions declared in soem_interface.h
// ---------------------------------------------------------------------------

int soem_interface_init_enhanced(const char *ifname) {
    (void)ifname;
    printf("SIM: soem_interface_init_enhanced() — simulation mode, no real hardware\n");
    printf("SIM: Motor model: I=%.3f kg*m^2, b=%.1f Nms/rad, rated=%.1f Nm\n",
           SIM_INERTIA, SIM_DAMPING, SIM_RATED_TORQUE_NM);
    printf("SIM: Encoder: %.0f counts/rev, steering limit ±%.0f°\n",
           SIM_ENCODER_CPR, SIM_MAX_ANGLE_DEG);

    sim_running = 1;
    if (pthread_create(&sim_thread, NULL, sim_physics_loop, NULL) != 0) {
        perror("SIM: Failed to create physics thread");
        return -1;
    }

    // Short stabilisation delay to match real init timing
    usleep(100000);
    printf("SIM: Initialisation complete — simulation running\n");
    return 0;
}

void soem_interface_send_and_receive_pdo(float target_torque) {
    // target_torque arrives as FFB output in [-5000, +5000] range.
    // Convert to CiA402 per-mille using same scale as soem_interface.c.
    float pm = target_torque * (1000.0f / 5000.0f);
    pthread_mutex_lock(&sim_mutex);
    sim_state.target_torque_pm = pm;
    pthread_mutex_unlock(&sim_mutex);
}

float soem_interface_get_current_position(void) {
    pthread_mutex_lock(&sim_mutex);
    float pos = rad_to_counts(sim_state.position_rad);
    pthread_mutex_unlock(&sim_mutex);
    return pos;
}

float soem_interface_get_current_velocity(void) {
    pthread_mutex_lock(&sim_mutex);
    // Return velocity in encoder counts/second (same unit as real interface)
    float vel = (sim_state.velocity_rad_s / (2.0f * (float)M_PI)) * SIM_ENCODER_CPR;
    pthread_mutex_unlock(&sim_mutex);
    return vel;
}

int soem_interface_get_communication_status(void) {
    return sim_running ? 1 : 0;
}

cia402_state_t soem_interface_get_cia402_state(void) {
    return CIA402_STATE_OPERATION_ENABLED;
}

uint16_t soem_interface_get_statusword(void) {
    // Simulate statusword for "Operation Enabled" (bits: OE|QS|VE|SO|RTSO = 0x0037)
    return 0x0037;
}

void soem_interface_stop_master(void) {
    printf("SIM: soem_interface_stop_master() called\n");
    sim_running = 0;
    if (sim_thread) {
        pthread_join(sim_thread, NULL);
    }
    printf("SIM: Physics thread joined, simulation stopped\n");
}

int soem_interface_write_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex,
                              uint16_t data_size, void *data) {
    (void)slave_idx; (void)index; (void)subindex; (void)data_size; (void)data;
    // Silently accept all SDO writes in simulation
    return 0;
}

int soem_interface_read_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex,
                             uint16_t data_size, void *data) {
    (void)slave_idx; (void)index; (void)subindex;
    // Return zeroed data — callers check return code, not value for most SDOs
    if (data && data_size > 0) {
        memset(data, 0, data_size);
    }
    return 0;
}

int soem_interface_set_ethercat_state(uint16_t slave_idx, ec_state desired_state) {
    (void)slave_idx; (void)desired_state;
    return 0;
}

int soem_interface_configure_pdo_mapping_enhanced(uint16_t slave_idx,
                                                   uint16_t pdo_assign_idx,
                                                   uint16_t pdo_map_idx,
                                                   uint32_t *mapped_objects,
                                                   uint8_t num_mapped_objects) {
    (void)slave_idx; (void)pdo_assign_idx; (void)pdo_map_idx;
    (void)mapped_objects; (void)num_mapped_objects;
    return 0;
}

// CiA 402 helpers (also declared in soem_interface.h, needed by callers)
cia402_state_t get_cia402_state(uint16_t statusword) {
    (void)statusword;
    return CIA402_STATE_OPERATION_ENABLED;
}

const char *get_cia402_state_name(cia402_state_t state) {
    switch (state) {
        case CIA402_STATE_NOT_READY:            return "NOT_READY";
        case CIA402_STATE_SWITCH_ON_DISABLED:   return "SWITCH_ON_DISABLED";
        case CIA402_STATE_READY_TO_SWITCH_ON:   return "READY_TO_SWITCH_ON";
        case CIA402_STATE_SWITCHED_ON:          return "SWITCHED_ON";
        case CIA402_STATE_OPERATION_ENABLED:    return "OPERATION_ENABLED";
        case CIA402_STATE_QUICK_STOP_ACTIVE:    return "QUICK_STOP_ACTIVE";
        case CIA402_STATE_FAULT_REACTION_ACTIVE: return "FAULT_REACTION_ACTIVE";
        case CIA402_STATE_FAULT:                return "FAULT";
        default:                                return "UNKNOWN";
    }
}
