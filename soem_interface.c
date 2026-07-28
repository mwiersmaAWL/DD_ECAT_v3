// soem_interface.c - Fixed for Synapticon 16-bit absolute encoder
#include "soem_interface.h" 
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <sched.h>

// --- SOEM Library Includes ---
#include "ethercat.h"  
#include "ethercattype.h" 

#ifndef PACKED
#define PACKED __attribute__((__packed__))
#endif

// --- CiA 402 State Machine Definitions ---
#define CIA402_STATUSWORD_RTSO          0x0001  // Ready to switch on
#define CIA402_STATUSWORD_SO            0x0002  // Switched on
#define CIA402_STATUSWORD_OE            0x0004  // Operation enabled
#define CIA402_STATUSWORD_FAULT         0x0008  // Fault
#define CIA402_STATUSWORD_VE            0x0010  // Voltage enabled
#define CIA402_STATUSWORD_QS            0x0020  // Quick stop
#define CIA402_STATUSWORD_SOD           0x0040  // Switch on disabled
#define CIA402_STATUSWORD_WARNING       0x0080  // Warning
#define CIA402_STATUSWORD_REMOTE        0x0200  // Remote
#define CIA402_STATUSWORD_TARGET        0x0400  // Target reached
#define CIA402_STATUSWORD_INTERNAL      0x0800  // Internal limit active

// --- Controlwords
#define CIA402_CONTROLWORD_SO           0x0001  // Switch on
#define CIA402_CONTROLWORD_EV           0x0002  // Enable voltage
#define CIA402_CONTROLWORD_QS           0x0004  // Quick stop
#define CIA402_CONTROLWORD_EO           0x0008  // Enable operation
#define CIA402_CONTROLWORD_FAULT_RESET  0x0080  // Fault reset

// **SYNAPTICON 16-BIT ENCODER CONFIGURATION** (matches main.c and synapticon_servo_tuning.h)
// 16-bit absolute encoder = 65,536 counts per revolution
#define ENCODER_COUNTS_PER_REV          65536.0f // 2^16 counts per revolution

// Scale factor: FFB calculator outputs [-5000, +5000]; CiA402 target_torque is per-mille [-1000, +1000]
#define TORQUE_PERMILLE_SCALE           (1000.0f / 5000.0f)

// --- SOEM Global Variables ---
char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
ec_ODlistt ODlist;
ec_groupt DCgroup;
int wkc;
int expectedWKC;
ec_timet tmo;
// Cycle time matches main loop (1kHz = 1ms, see main_v2.c MAIN_LOOP_FREQUENCY_HZ)
int cycle_time = 1000; // 1 ms in microseconds

// Use minimal essential mapping to avoid size issues
uint32_t rxpdo_mapping[] = {
    0x60400010,  // Controlword (16-bit)
    0x60600008,  // Modes of operation (8-bit)
    0x60710010,  // Target torque (16-bit)
    0x607A0020   // Target position (32-bit)
    };
    
uint32_t txpdo_mapping[] = {
    0x60410010,  // Statusword (16-bit)
    0x60610008,  // Modes of operation display (8-bit)
    0x60640020,  // Position actual value (32-bit)
    0x606C0020,  // Velocity actual value (32-bit) — was missing, caused somanet_tx_pdo_enhanced_t misalignment
    0x60770010   // Torque actual value (16-bit)
    };
    
    // Calculate and verify sizes
    uint16_t rxpdo_size_bits = 16 + 8 + 16 + 32; // 72 bits = 9 bytes
    uint16_t txpdo_size_bits = 16 + 8 + 32 + 32 + 16; // 104 bits = 13 bytes

// Pointers to the PDO data in the IOmap
somanet_rx_pdo_enhanced_t *somanet_outputs; 
somanet_tx_pdo_enhanced_t *somanet_inputs; 

// Mutex for protecting PDO data access
static pthread_mutex_t pdo_mutex = PTHREAD_MUTEX_INITIALIZER;

// Thread for EtherCAT communication
static pthread_t ecat_thread;
static volatile int ecat_thread_running = 0;
static volatile int master_initialized = 0;
/* Issue #16: written by ecat_loop, read by soem_interface_get_communication_status()
 * without mutex coverage — use _Atomic to make the cross-thread read safe.        */
static _Atomic int communication_ok = 0;

// Global variables to hold current PDO values
static float target_torque_f = 0.0f;
static float current_position_f = 0.0f;
static float current_velocity_f = 0.0f;
static cia402_state_t current_cia402_state = CIA402_STATE_NOT_READY;
static uint16_t current_statusword = 0;
static uint16_t current_controlword = 0;

/**
 * @brief PI controller to align the local EtherCAT cyclic thread to the DC sync0 boundary.
 *
 * After ec_receive_processdata(), ec_DCtime holds the distributed clock reference
 * time in nanoseconds.  This function computes a small nanosecond offset to add to
 * the next absolute sleep deadline so that the thread gradually synchronises to the
 * DC master clock.  Based on the canonical SOEM dc_rtex example.
 *
 * Args:
 *     reftime:      Current DC reference clock (ec_DCtime) in nanoseconds.
 *     cycletime_ns: Nominal cycle period in nanoseconds.
 *     integral:     Persistent integrator state; caller must zero-initialise once.
 *
 * Returns:
 *     Nanosecond correction to add to the next wakeup timestamp.
 */
static int64_t compute_dc_offset(int64_t reftime, int64_t cycletime_ns, int64_t *integral) {
    /* Phase error: distance of the reference clock from the nearest cycle boundary,
     * shifted 50 µs early so the thread wakes slightly before the sync pulse.     */
    int64_t delta = (reftime - 50000LL) % cycletime_ns;
    if (delta > cycletime_ns / 2LL) delta -= cycletime_ns;
    /* PI: proportional term + slow integrator to eliminate steady-state offset.    */
    *integral += (delta > 0LL) ? 1LL : -1LL;
    return -(delta / 100LL) - (*integral / 20LL);
}

// --- CiA 402 State Machine Functions ---
cia402_state_t get_cia402_state(uint16_t statusword) {
    // Extract relevant bits for state determination
    uint16_t state_bits = statusword & 0x006F;
    
    switch (state_bits) {
        case 0x0000: return CIA402_STATE_NOT_READY;
        case 0x0040: return CIA402_STATE_SWITCH_ON_DISABLED;
        case 0x0021: return CIA402_STATE_READY_TO_SWITCH_ON;
        case 0x0023: return CIA402_STATE_SWITCHED_ON;
        case 0x0027: return CIA402_STATE_OPERATION_ENABLED;
        case 0x0007: return CIA402_STATE_QUICK_STOP_ACTIVE;
        case 0x000F: return CIA402_STATE_FAULT_REACTION_ACTIVE;
        case 0x0008: return CIA402_STATE_FAULT;
        default: 
            // Check for fault bit
            if (statusword & CIA402_STATUSWORD_FAULT) {
                return CIA402_STATE_FAULT;
            }
            return CIA402_STATE_NOT_READY;
    }
}

const char* get_cia402_state_name(cia402_state_t state) {
    switch (state) {
        case CIA402_STATE_NOT_READY: return "NOT_READY";
        case CIA402_STATE_SWITCH_ON_DISABLED: return "SWITCH_ON_DISABLED";
        case CIA402_STATE_READY_TO_SWITCH_ON: return "READY_TO_SWITCH_ON";
        case CIA402_STATE_SWITCHED_ON: return "SWITCHED_ON";
        case CIA402_STATE_OPERATION_ENABLED: return "OPERATION_ENABLED";
        case CIA402_STATE_QUICK_STOP_ACTIVE: return "QUICK_STOP_ACTIVE";
        case CIA402_STATE_FAULT_REACTION_ACTIVE: return "FAULT_REACTION_ACTIVE";
        case CIA402_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

uint16_t get_cia402_controlword_for_transition(cia402_state_t current_state, cia402_state_t target_state) {
    switch (current_state) {
        case CIA402_STATE_NOT_READY:
        case CIA402_STATE_SWITCH_ON_DISABLED:
            if (target_state == CIA402_STATE_READY_TO_SWITCH_ON) {
                return 0x0006; // Shutdown: Enable voltage + Quick stop
            }
            break;
            
        case CIA402_STATE_READY_TO_SWITCH_ON:
            if (target_state == CIA402_STATE_SWITCHED_ON) {
                return 0x0007; // Switch on: Enable voltage + Quick stop + Switch on
            }
            break;
            
        case CIA402_STATE_SWITCHED_ON:
            if (target_state == CIA402_STATE_OPERATION_ENABLED) {
                return 0x000F; // Enable operation: All control bits set
            }
            break;
            
        case CIA402_STATE_FAULT:
            return 0x0080; // Fault reset
            
        default:
            break;
    }
    
    // Default: maintain current state
    return 0x0006;
}

// Function to initialize CiA 402 parameters via SDO - CONFIGURED FOR SYNAPTICON
int initialize_cia402_parameters(uint16_t slave_idx) {
    printf("SOEM_Interface: Initializing CiA 402 parameters for Synapticon 16-bit encoder...\n");
    
    // Set modes of operation to torque mode (4)
    int8_t torque_mode = 4;
    if (soem_interface_write_sdo(slave_idx, 0x6060, 0x00, sizeof(torque_mode), &torque_mode) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set modes of operation to torque mode\n");
        return -1;
    }
    printf("SOEM_Interface: Set modes of operation to torque mode (4)\n");
    
    // Set motor rated current (example: 3000 mA = 3A)
    // Adjust this value according to your motor specifications
    uint32_t motor_rated_current = 3000; // mA
    if (soem_interface_write_sdo(slave_idx, 0x6075, 0x00, sizeof(motor_rated_current), &motor_rated_current) != 0) {
        printf("SOEM_Interface: Warning: Failed to set motor rated current (may not be supported)\n");
    } else {
        printf("SOEM_Interface: Set motor rated current to %u mA\n", motor_rated_current);
    }
    
    // Set max torque (example: 1000 per mille = 100% of rated torque)
    uint16_t max_torque = 1000; // per mille
    if (soem_interface_write_sdo(slave_idx, 0x6072, 0x00, sizeof(max_torque), &max_torque) != 0) {
        printf("SOEM_Interface: Warning: Failed to set max torque\n");
    } else {
        printf("SOEM_Interface: Set max torque to %u per mille\n", max_torque);
    }
    
    // Set torque slope (acceleration/deceleration limit)
    uint32_t torque_slope = 10000;
    if (soem_interface_write_sdo(slave_idx, 0x6087, 0x00, sizeof(torque_slope), &torque_slope) != 0) {
        printf("SOEM_Interface: Warning: Failed to set torque slope\n");
    } else {
        printf("SOEM_Interface: Set torque slope to %u per mille/s\n", torque_slope);
    }
    
    // **SYNAPTICON-SPECIFIC: Set encoder resolution to match 16-bit encoder**
    // Set position encoder resolution (65536 increments per revolution for 16-bit)
    uint32_t encoder_increments = (uint32_t)65536; // 2^16 = 65536 for 16-bit encoder
    if (soem_interface_write_sdo(slave_idx, 0x608F, 0x01, sizeof(encoder_increments), &encoder_increments) != 0) {
        printf("SOEM_Interface: Warning: Failed to set encoder increments (may use internal default)\n");
    } else {
        printf("SOEM_Interface: Set encoder increments to %u per revolution (16-bit)\n", encoder_increments);
    }
    
    // Set gear ratio to 1:1 (direct drive)
    uint32_t gear_ratio_num = 1;   // Numerator
    uint32_t gear_ratio_den = 1;   // Denominator
    if (soem_interface_write_sdo(slave_idx, 0x608F, 0x02, sizeof(gear_ratio_num), &gear_ratio_num) != 0) {
        printf("SOEM_Interface: Info: Gear ratio numerator setting not available\n");
    } else {
        printf("SOEM_Interface: Set gear ratio numerator to %u\n", gear_ratio_num);
    }
    
    // Set interpolation time period — must match the EtherCAT PDO cycle time (1 ms at 1 kHz)
    uint8_t interpolation_time_period = 1; // 1 ms to match 1 kHz EtherCAT PDO cycle
    int8_t interpolation_time_index = -3;  // 10^-3 seconds (milliseconds)
    if (soem_interface_write_sdo(slave_idx, 0x60C2, 0x01, sizeof(interpolation_time_period), &interpolation_time_period) != 0) {
        printf("SOEM_Interface: Warning: Failed to set interpolation time period\n");
    } else {
        printf("SOEM_Interface: Set interpolation time period to %u ms\n", interpolation_time_period);
    }
    
    if (soem_interface_write_sdo(slave_idx, 0x60C2, 0x02, sizeof(interpolation_time_index), &interpolation_time_index) != 0) {
        printf("SOEM_Interface: Warning: Failed to set interpolation time index\n");
    } else {
        printf("SOEM_Interface: Set interpolation time index to %d\n", interpolation_time_index);
    }
    
    // Wait for parameters to be processed
    usleep(100000); // 100ms delay
    
    // Verify modes of operation was set correctly
    int8_t current_mode = 0;
    if (soem_interface_read_sdo(slave_idx, 0x6061, 0x00, sizeof(current_mode), &current_mode) == 0) {
        printf("SOEM_Interface: Current modes of operation display: %d\n", current_mode);
        if (current_mode == torque_mode) {
            printf("SOEM_Interface: Mode verification successful\n");
        } else {
            printf("SOEM_Interface: Warning: Mode not yet active (expected %d, got %d)\n", torque_mode, current_mode);
        }
    } else {
        printf("SOEM_Interface: Warning: Could not verify modes of operation\n");
    }
    
    printf("SOEM_Interface: Synapticon 16-bit encoder initialization completed\n");
    return 0;
}

// --- Helper function to check if slave is in operational state ---
int is_slave_operational(int slave_idx) {
    uint16 actual_state = ec_slave[slave_idx].state & 0x0F;
    return actual_state == EC_STATE_OPERATIONAL;
}

// --- Helper function to get readable state name ---
const char* get_state_name(uint16 state) {
    uint16 actual_state = state & 0x0F;
    switch(actual_state) {
        case EC_STATE_INIT: return "INIT";
        case EC_STATE_PRE_OP: return "PRE_OP";
        case EC_STATE_BOOT: return "BOOT";
        case EC_STATE_SAFE_OP: return "SAFE_OP";
        case EC_STATE_OPERATIONAL: return "OPERATIONAL";
        default: return "UNKNOWN";
    }
}

// --- Helper functions for SDO operations ---
int soem_interface_write_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data) {
    int wkc_sdo;
    wkc_sdo = ec_SDOwrite(slave_idx, index, subindex, FALSE, data_size, data, 50000); // 50ms timeout
    if (wkc_sdo == 0) {
        fprintf(stderr, "SOEM_Interface: SDO write failed for slave %u, index 0x%04X:%02X\n", slave_idx, index, subindex);
        return -1;
    }
    return 0;
}

int soem_interface_read_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data) {
    int wkc_sdo;
    int actual_size = data_size;
    wkc_sdo = ec_SDOread(slave_idx, index, subindex, FALSE, &actual_size, data, 50000); // 50ms timeout
    if (wkc_sdo == 0) {
        fprintf(stderr, "SOEM_Interface: SDO read failed for slave %u, index 0x%04X:%02X\n", slave_idx, index, subindex);
        return -1;
    }
    return 0;
}

// --- Function to set EtherCAT slave state ---
int soem_interface_set_ethercat_state(uint16_t slave_idx, ec_state desired_state) {
    int max_retries = 10;
    int retry_count = 0;
    
    while (retry_count < max_retries) {
        printf("SOEM_Interface: Attempt %d/%d - Setting slave %u to state %s...\n", 
               retry_count + 1, max_retries, slave_idx, get_state_name(desired_state));
        
        ec_slave[slave_idx].ALstatuscode = 0;
        ec_slave[slave_idx].state = desired_state;
        ec_writestate(slave_idx);
        usleep(50000); // 50ms delay
        
        int wkc_state = ec_statecheck(slave_idx, desired_state, 100000); // 100ms timeout
        
        if (wkc_state > 0 && (ec_slave[slave_idx].state & 0x0F) == desired_state) {
            printf("SOEM_Interface: Slave %u successfully transitioned to state %s\n", 
                   slave_idx, get_state_name(ec_slave[slave_idx].state));
            return 0;
        }
        
        printf("SOEM_Interface: State transition failed - Current state: %s, ALstatuscode: 0x%04X\n",
               get_state_name(ec_slave[slave_idx].state), ec_slave[slave_idx].ALstatuscode);
        
        // Try intermediate states if stuck in PRE_OP
        if (desired_state == EC_STATE_OPERATIONAL && (ec_slave[slave_idx].state & 0x0F) == EC_STATE_PRE_OP) {
            printf("SOEM_Interface: Trying intermediate SAFE_OP transition...\n");
            
            ec_slave[slave_idx].state = EC_STATE_SAFE_OP;
            ec_writestate(slave_idx);
            usleep(200000);
            
            if (ec_statecheck(slave_idx, EC_STATE_SAFE_OP, 200000) > 0) {
                printf("SOEM_Interface: Intermediate SAFE_OP successful\n");
                ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
                ec_writestate(slave_idx);
                usleep(200000);
                
                if (ec_statecheck(slave_idx, EC_STATE_OPERATIONAL, 200000) > 0) {
                    printf("SOEM_Interface: Final OPERATIONAL transition successful\n");
                    return 0;
                }
            }
        }
        
        retry_count++;
        if (retry_count < max_retries) {
            printf("SOEM_Interface: Retrying in 100ms...\n");
            usleep(100000);
        }
    }
    
    fprintf(stderr, "SOEM_Interface: Failed to set slave %u to state %s after %d attempts\n",
            slave_idx, get_state_name(desired_state), max_retries);
    return -1;
}

// --- Function to perform CiA 402 state machine transition ---
int perform_cia402_transition_to_operational(uint16_t slave_idx) {
    printf("SOEM_Interface: Starting CiA 402 state machine transition to operational...\n");
    
    int max_attempts = 100;
    int attempt = 0;
    
    while (attempt < max_attempts) {
        // Read current statusword
        if (somanet_inputs) {
            current_statusword = somanet_inputs->statusword;
        } else {
            fprintf(stderr, "SOEM_Interface: somanet_inputs not available for status reading.\n");
            return -1;
        }
        
        current_cia402_state = get_cia402_state(current_statusword);
        
        printf("SOEM_Interface: CiA 402 State: %s (statusword: 0x%04X)\n", 
               get_cia402_state_name(current_cia402_state), current_statusword);
        
        // Check if we're in operational state
        if (current_cia402_state == CIA402_STATE_OPERATION_ENABLED) {
            printf("SOEM_Interface: Successfully reached Operation Enabled state!\n");
            return 0;
        }
        
        // Handle fault state
        if (current_cia402_state == CIA402_STATE_FAULT) {
            printf("SOEM_Interface: Device in fault state. Attempting fault reset...\n");
            current_controlword = CIA402_CONTROLWORD_FAULT_RESET;
        } else {
            // Determine next transition
            switch (current_cia402_state) {
                case CIA402_STATE_NOT_READY:
                case CIA402_STATE_SWITCH_ON_DISABLED:
                    current_controlword = 0x0006; // Shutdown
                    break;
                case CIA402_STATE_READY_TO_SWITCH_ON:
                    current_controlword = 0x0007; // Switch on
                    break;
                case CIA402_STATE_SWITCHED_ON:
                    current_controlword = 0x000F; // Enable operation
                    break;
                case CIA402_STATE_QUICK_STOP_ACTIVE:
                    current_controlword = 0x0006; // Shutdown
                    break;
                default:
                    current_controlword = 0x0006; // Default to shutdown
                    break;
            }
        }
        
        // Apply controlword
        if (somanet_outputs) {
            somanet_outputs->controlword = current_controlword;
            somanet_outputs->modes_of_operation = 4; // Torque mode
        }
        
        // Send PDO data to apply controlword
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        
        printf("SOEM_Interface: Applied controlword: 0x%04X, WKC: %d/%d\n", current_controlword, wkc, expectedWKC);
        
        attempt++;
        usleep(10000); // 10ms delay between attempts
    }
    
    fprintf(stderr, "SOEM_Interface: Failed to reach operational state after %d attempts.\n", max_attempts);
    return -1;
}

// --- SOEM Thread Function ---
void *ecat_loop(void *ptr) {
    int slave_idx = 1;
    int state_machine_initialized = 0;

    /* Issue #14: Pin this thread to core 3 (isolated by isolcpus=3 kernel
     * parameter) with SCHED_FIFO priority 80 to minimise jitter in the
     * EtherCAT cyclic exchange.  The main loop runs on core 2 at priority 50.*/
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        perror("SOEM_Interface: EtherCAT thread: failed to set affinity to core 3");
    }
    struct sched_param rt_param = { .sched_priority = 80 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &rt_param) != 0) {
        perror("SOEM_Interface: EtherCAT thread: failed to set SCHED_FIFO priority 80");
    }

    printf("SOEM_Interface: EtherCAT thread started (%d µs cycle time).\n", cycle_time);

    while (!master_initialized && ecat_thread_running) {
        usleep(10000); // Wait for master to be initialized
    }

    if (!master_initialized) {
        printf("SOEM_Interface: Master not initialized, exiting thread.\n");
        return NULL;
    }

    printf("SOEM_Interface: Entering EtherCAT cyclic loop.\n");

    // Use absolute-time sleeping to prevent cumulative drift from relative nanosleep
    struct timespec next_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

    /* Issue #17: integrator state for the DC sync PI controller.
     * Must be zero-initialised once before the loop.                         */
    int64_t dc_integral = 0LL;

    while (ecat_thread_running) {
        // Advance absolute deadline at the top of the loop
        next_wakeup.tv_nsec += (long)cycle_time * 1000L;
        if (next_wakeup.tv_nsec >= 1000000000L) {
            next_wakeup.tv_sec++;
            next_wakeup.tv_nsec -= 1000000000L;
        }

        // Update output PDO data
        pthread_mutex_lock(&pdo_mutex);
        if (somanet_outputs) {
            // Only update torque if we're in operational state
            if (current_cia402_state == CIA402_STATE_OPERATION_ENABLED) {
                // Bug fix: FFB outputs [-5000,+5000]; CiA402 target_torque is per-mille [-1000,+1000]
                // Previous code multiplied by 1000 causing int16_t overflow at any FFB value > 32
                somanet_outputs->target_torque = (int16_t)(target_torque_f * TORQUE_PERMILLE_SCALE);
            } else {
                somanet_outputs->target_torque = 0; // Safe value
            }
            
            // Keep controlword updated for state machine
            somanet_outputs->controlword = current_controlword;
            somanet_outputs->modes_of_operation = 4; // Torque mode
        }
        pthread_mutex_unlock(&pdo_mutex);

        // Exchange process data
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc < expectedWKC) {
            printf("SOEM_Interface: Working counter too low: %d < %d\n", wkc, expectedWKC);
            communication_ok = 0;
        } else {
            communication_ok = 1;
            
            // Update input PDO data - This is where we get the 14-bit encoder position
            pthread_mutex_lock(&pdo_mutex);
            if (somanet_inputs) {
                current_statusword = somanet_inputs->statusword;
                current_cia402_state = get_cia402_state(current_statusword);
                
                // **CRITICAL: Position from 14-bit encoder comes via position_actual_value**
                current_position_f = (float)somanet_inputs->position_actual_value;
                current_velocity_f = (float)somanet_inputs->velocity_actual_value;
                
                // Debug: Print encoder position periodically (every 1000 cycles = ~10 seconds)
                static int debug_counter = 0;
                if (++debug_counter % 1000 == 0) {
                    printf("SOEM_Interface: 16-bit Encoder Position: %d (%.2f), Velocity: %d (%.2f)\n", 
                           somanet_inputs->position_actual_value, current_position_f,
                           somanet_inputs->velocity_actual_value, current_velocity_f);
                }
            }
            pthread_mutex_unlock(&pdo_mutex);

            // Initialize state machine once after successful PDO exchange
            if (!state_machine_initialized && somanet_inputs && somanet_outputs) {
                printf("SOEM_Interface: Attempting CiA 402 state machine initialization...\n");
                if (perform_cia402_transition_to_operational(slave_idx) == 0) {
                    state_machine_initialized = 1;
                    printf("SOEM_Interface: CiA 402 state machine initialized to Operation Enabled.\n");
                    printf("SOEM_Interface: 16-bit encoder ready, resolution: %.0f counts/rev\n", ENCODER_COUNTS_PER_REV);
                } else {
                    fprintf(stderr, "SOEM_Interface: Failed to initialize CiA 402 state machine.\n");
                    communication_ok = 0; 
                }
            }
        }

        // Check EtherCAT slave state periodically
        if (!is_slave_operational(slave_idx)) {
            ec_statecheck(slave_idx, EC_STATE_OPERATIONAL, 100000);
        }

        /* Issue #17: Align next wakeup to the DC sync0 boundary.
         * ec_DCtime is updated by ec_receive_processdata() and holds the
         * distributed clock reference time in nanoseconds.  The PI controller
         * (compute_dc_offset) nudges next_wakeup so the thread gradually
         * phase-locks to the DC master, eliminating the accumulated drift that
         * ec_configdc() enables but free-running CLOCK_MONOTONIC sleep ignores. */
        int64_t toff = compute_dc_offset((int64_t)ec_DCtime,
                                          (int64_t)cycle_time * 1000LL,
                                          &dc_integral);
        next_wakeup.tv_nsec += (long)toff;
        if (next_wakeup.tv_nsec >= 1000000000L) {
            next_wakeup.tv_sec++;
            next_wakeup.tv_nsec -= 1000000000L;
        } else if (next_wakeup.tv_nsec < 0L) {
            next_wakeup.tv_sec--;
            next_wakeup.tv_nsec += 1000000000L;
        }

        // Sleep until absolute deadline — prevents accumulated drift from relative nanosleep
        int sleep_ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
        if (sleep_ret != 0 && sleep_ret != EINTR) {
            static int late_warnings = 0;
            if (late_warnings < 10) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                long behind_ns = (now.tv_sec - next_wakeup.tv_sec) * 1000000000L +
                                 (now.tv_nsec - next_wakeup.tv_nsec);
                if (behind_ns > 0) {
                    printf("SOEM_Interface: EtherCAT thread running %.3fms late\n", behind_ns / 1000000.0);
                    late_warnings++;
                }
            }
        }
    }

    printf("SOEM_Interface: EtherCAT thread stopping.\n");
    return NULL;
}

// Enhanced SOEM initialization (simplified for key parts)
int soem_interface_init_enhanced(const char *ifname) {
    int i;
    int slave_idx = 1;

    printf("SOEM_Interface: Enhanced initialization for Synapticon 16-bit encoder on %s...\n", ifname);

    if (!ec_init(ifname)) {
        fprintf(stderr, "SOEM_Interface: ec_init failed on %s\n", ifname);
        return -1;
    }

    printf("SOEM_Interface: ec_init succeeded\n");

    if (ec_config_init(FALSE) <= 0) {
        fprintf(stderr, "SOEM_Interface: No slaves found during config_init\n");
        return -1;
    }

    printf("SOEM_Interface: Found %d slaves\n", ec_slavecount);

    if (ec_slavecount == 0) {
        fprintf(stderr, "SOEM_Interface: No EtherCAT slaves found!\n");
        return -1;
    }

    // Print detailed slave information
    for (i = 1; i <= ec_slavecount; i++) {
        printf("SOEM_Interface: Slave %d: %s\n", i, ec_slave[i].name);
        printf("  - Vendor ID: 0x%08X, Product Code: 0x%08X\n", 
               ec_slave[i].eep_id, ec_slave[i].eep_pdi);
        printf("  - Output: %d bits (%d bytes), Input: %d bits (%d bytes)\n",
               ec_slave[i].Obits, ec_slave[i].Obits/8, ec_slave[i].Ibits, ec_slave[i].Ibits/8);
        printf("  - State: %s, ALstatuscode: 0x%04X\n", 
               get_state_name(ec_slave[i].state), ec_slave[i].ALstatuscode);
    }

    // Configure distributed clocks
    ec_configdc();

    // Map the IO
    printf("SOEM_Interface: Mapping IO...\n");
    if (ec_config_map(&IOmap) == 0) {
        fprintf(stderr, "SOEM_Interface: ec_config_map failed\n");
        return -1;
    }

    // Print actual mapped sizes
    printf("SOEM_Interface: IO mapping completed\n");
    for (i = 1; i <= ec_slavecount; i++) {
        printf("SOEM_Interface: Slave %d mapped - Output: %d bytes, Input: %d bytes\n",
               i, ec_slave[i].Obits/8, ec_slave[i].Ibits/8);
    }

    // Assign PDO pointers
    if (ec_slave[slave_idx].outputs > 0) {
        somanet_outputs = (somanet_rx_pdo_enhanced_t *)(ec_slave[slave_idx].outputs);
        printf("SOEM_Interface: somanet_outputs mapped successfully\n");
    } else {
        fprintf(stderr, "SOEM_Interface: No output PDO data available!\n");
        return -1;
    }
    
    if (ec_slave[slave_idx].inputs > 0) {
        somanet_inputs = (somanet_tx_pdo_enhanced_t *)(ec_slave[slave_idx].inputs);
        printf("SOEM_Interface: somanet_inputs mapped successfully (16-bit encoder data)\n");
    } else {
        fprintf(stderr, "SOEM_Interface: No input PDO data available!\n");
        return -1;
    }

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("SOEM_Interface: Expected WKC: %d\n", expectedWKC);

    // Initialize CiA 402 parameters in PRE_OP
    if (initialize_cia402_parameters(slave_idx) != 0) {
        printf("SOEM_Interface: CiA 402 initialization had issues, continuing anyway\n");
    }
    
    // Initialize safe values — zero the entire slave output buffer, not just the struct
    // prefix (issue 7: somanet_rx_pdo_enhanced_t is 5 bytes but the SM is up to 35 bytes).
    memset(ec_slave[slave_idx].outputs, 0, (size_t)(ec_slave[slave_idx].Obits / 8));
    if (somanet_outputs) {
        somanet_outputs->controlword = 0x0006; // Shutdown
        somanet_outputs->modes_of_operation = 4; // Torque mode
    }

    // Transition to Safe-Operational
    printf("SOEM_Interface: Transitioning to Safe-Operational...\n");
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_SAFE_OP) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to reach Safe-Operational state\n");
        return -1;
    }

    // Send initial PDO data
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    usleep(10000);
    
    // Send multiple cycles to establish communication
    for (int wd_cycles = 0; wd_cycles < 10; wd_cycles++) {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        usleep(cycle_time);
    }

    // Transition to Operational
    printf("SOEM_Interface: Transitioning to Operational...\n");
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_OPERATIONAL) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to reach Operational state\n");
        return -1;
    }

    // Verify all slaves are operational
    for (i = 1; i <= ec_slavecount; i++) {
        if (!is_slave_operational(i)) {
            fprintf(stderr, "SOEM_Interface: Slave %d not operational after transition\n", i);
            return -1;
        }
    }

    printf("SOEM_Interface: All slaves operational, starting communication thread...\n");
    
    // Start communication thread immediately
    master_initialized = 1;
    ecat_thread_running = 1;
    
    if (pthread_create(&ecat_thread, NULL, ecat_loop, NULL) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to create EtherCAT thread\n");
        return -1;
    }
    
    // Give the thread time to start
    usleep(50000);
    
    // Verify slaves are still operational
    for (i = 1; i <= ec_slavecount; i++) {
        if (!is_slave_operational(i)) {
            fprintf(stderr, "SOEM_Interface: Slave %d not operational after thread startup\n", i);
            return -1;
        }
    }
    
    printf("SOEM_Interface: Synapticon 16-bit encoder initialization completed successfully\n");
    return 0;
}

void soem_interface_send_and_receive_pdo(float target_torque) {
    if (!master_initialized) return;
    
    pthread_mutex_lock(&pdo_mutex);
    target_torque_f = target_torque;
    pthread_mutex_unlock(&pdo_mutex);
}

float soem_interface_get_current_position(void) {
    pthread_mutex_lock(&pdo_mutex);
    float raw_pos = current_position_f;
    pthread_mutex_unlock(&pdo_mutex);
    return raw_pos;
}

float soem_interface_get_current_velocity() {
    float velocity;
    pthread_mutex_lock(&pdo_mutex);
    velocity = current_velocity_f;
    pthread_mutex_unlock(&pdo_mutex);
    return velocity;
}

int soem_interface_get_communication_status() {
    return atomic_load_explicit(&communication_ok, memory_order_relaxed);
}

cia402_state_t soem_interface_get_cia402_state() {
    /* Issue #16: current_cia402_state is written under pdo_mutex but the
     * getter had no mutex coverage.  Lock pdo_mutex for the read.          */
    pthread_mutex_lock(&pdo_mutex);
    cia402_state_t state = current_cia402_state;
    pthread_mutex_unlock(&pdo_mutex);
    return state;
}

uint16_t soem_interface_get_statusword() {
    /* Issue #16: same as above — add pdo_mutex coverage for the read.      */
    pthread_mutex_lock(&pdo_mutex);
    uint16_t sw = current_statusword;
    pthread_mutex_unlock(&pdo_mutex);
    return sw;
}

void soem_interface_stop_master() {
    if (master_initialized) {
        printf("SOEM_Interface: Stopping EtherCAT master...\n");

        ecat_thread_running = 0;
        if (ecat_thread) {
            pthread_join(ecat_thread, NULL);
        }

        // Safe shutdown: disable operation
        if (somanet_outputs) {
            somanet_outputs->target_torque = 0;
            somanet_outputs->controlword = 0x0006; // Shutdown
        }
        ec_send_processdata();
        
        usleep(10000); // Wait 10ms

        // Transition to Safe-Op then Init
        soem_interface_set_ethercat_state(0, EC_STATE_SAFE_OP);
        soem_interface_set_ethercat_state(0, EC_STATE_INIT);

        ec_close();
        master_initialized = 0;
        printf("SOEM_Interface: EtherCAT master stopped.\n");
    }
}