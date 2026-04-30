// soem_interface.h - Improved header for SOEM EtherCAT interface
#ifndef SOEM_INTERFACE_H
#define SOEM_INTERFACE_H

#include <stdint.h>

// Allow building without SOEM installed (simulation mode)
#ifndef SIM_MODE
#include "ethercat.h" // Include for ec_state
#else
typedef int ec_state;  // Stub for simulation builds
#endif

// Ensure PACKED is defined (soem_interface.h is self-contained)
#ifndef PACKED
#define PACKED __attribute__((__packed__))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// --- Constants ---

// Scaling factors for unit conversion
#define SOEM_POSITION_SCALE_FACTOR (360.0f / 1000000.0f)  // Convert from encoder counts to degrees
#define SOEM_VELOCITY_SCALE_FACTOR (1.0f / 1000.0f)       // Convert from internal units to deg/s
#define SOEM_TORQUE_SCALE_FACTOR   (1000.0f)              // Convert from float torque to motor units

// EtherCAT state machine control words
#define SOEM_CONTROLWORD_SHUTDOWN           0x06  // Ready to switch on
#define SOEM_CONTROLWORD_SWITCH_ON          0x07  // Switch on
#define SOEM_CONTROLWORD_OPERATION_ENABLED  0x0F  // Operation enabled
#define SOEM_CONTROLWORD_FAULT_RESET        0x80  // Fault reset

// EtherCAT status word masks
#define SOEM_STATUSWORD_FAULT_MASK          0x08  // Fault bit mask
#define SOEM_STATUSWORD_OPERATION_ENABLED   0x37  // Operation enabled state

// Modes of operation
#define SOEM_MODE_CYCLIC_SYNC_TORQUE        0x0A  // Cyclic Synchronous Torque mode

// --- Error Codes ---
#define SOEM_SUCCESS                         0
#define SOEM_ERROR_INIT_FAILED              -1
#define SOEM_ERROR_NO_SLAVES                -2
#define SOEM_ERROR_CONFIG_FAILED            -3
#define SOEM_ERROR_STATE_FAILED             -4
#define SOEM_ERROR_THREAD_FAILED            -5

// --- CiA 402 State Machine Definitions ---
typedef enum {
    CIA402_STATE_NOT_READY = 0,
    CIA402_STATE_SWITCH_ON_DISABLED,
    CIA402_STATE_READY_TO_SWITCH_ON,
    CIA402_STATE_SWITCHED_ON,
    CIA402_STATE_OPERATION_ENABLED,
    CIA402_STATE_QUICK_STOP_ACTIVE,
    CIA402_STATE_FAULT_REACTION_ACTIVE,
    CIA402_STATE_FAULT
} cia402_state_t;

// --- PDO Structures for Synapticon ACTILINK-S (Slave 1) ---
// These structures define the expected PDO layout based on PDO_mapping.md.
// They are crucial for correct data alignment and size matching with the EtherCAT slave.
typedef struct PACKED
{
    uint16_t controlword;         // 0x6040:0x00 (16 bits)
    int8_t   modes_of_operation;  // 0x6060:0x00 (8 bits)
    int16_t  target_torque;       // 0x6071:0x00 (16 bits)
    //int32  target_position;     // 0x607A:0x00 (32 bits)
    //int32  target_velocity;     // 0x60FF:0x00 (32 bits)
    //int16  torque_offset;       // 0x60B2:0x00 (16 bits)
    //uint32 tuning_command;      // 0x2701:0x00 (32 bits)
    //uint32 physical_outputs;    // 0x60FE:0x01 (32 bits)
    //uint32 bit_mask;            // 0x60FE:0x02 (32 bits) - Assuming this is the 'Bit mask' from doc
    //uint32 user_mosi;           // 0x2703:0x00 (32 bits)
    //int32  velocity_offset;     // 0x60B1:0x00 (32 bits)
} somanet_rx_pdo_enhanced_t;

typedef struct PACKED
{
    uint16_t statusword;                  // 0x6041:0x00 (16 bits)
    int8_t   modes_of_operation_display;  // 0x6061:0x00 (8 bits)
    int32_t  position_actual_value;       // 0x6064:0x00 (32 bits)
    int32_t  velocity_actual_value;       // 0x606C:0x00 (32 bits)
    int16_t  torque_actual_value;         // 0x6077:0x00 (16 bits)
    //uint16 analog_input_1;              // 0x2401:0x00 (16 bits)
    //uint16 analog_input_2;              // 0x2402:0x00 (16 bits)
    //uint16 analog_input_3;              // 0x2403:0x00 (16 bits)
    //uint16 analog_input_4;              // 0x2404:0x00 (16 bits)
    //uint32 tuning_status;               // 0x2702:0x00 (32 bits)
    //uint32 digital_inputs;              // 0x60FD:0x00 (32 bits)
    //uint32 user_miso;                   // 0x2704:0x00 (32 bits)
    //uint32 timestamp;                   // 0x20F0:0x00 (32 bits)
    //int32  position_demand_internal_value; // 0x60FC:0x00 (32 bits)
    //int32  velocity_demand_value;       // 0x606B:0x00 (32 bits)
    //int16  torque_demand;               // 0x6074:0x00 (16 bits)
} somanet_tx_pdo_enhanced_t;

// --- Function Prototypes ---

/**
 * @brief Initializes the SOEM master and discovers slaves.
 * @param ifname The network interface name (e.g., "eth0").
 * @return 0 on success, -1 on failure.
 */
int soem_interface_init_enhanced(const char *ifname);

/**
 * @brief Sends target torque to the EtherCAT thread.
 * The actual PDO communication happens in the EtherCAT thread.
 * @param target_torque The desired torque to send to the servo motor.
 */
void soem_interface_send_and_receive_pdo(float target_torque);

/**
 * @brief Returns the last known position from the servo motor.
 * @return The current angular position in degrees.
 */
float soem_interface_get_current_position(void);


/**
 * @brief Returns the last known velocity from the servo motor.
 * @return The current angular velocity in degrees/second.
 */
float soem_interface_get_current_velocity(void);

/**
 * @brief Returns the communication status.
 * @return 1 if communication is OK, 0 if there are communication errors.
 */
int soem_interface_get_communication_status(void);

/**
 * @brief Returns the current CiA 402 state of the servo motor.
 * @return The current CiA 402 state.
 */
cia402_state_t soem_interface_get_cia402_state(void);

/**
 * @brief Returns the current statusword from the servo motor.
 * @return The current statusword value.
 */
uint16_t soem_interface_get_statusword(void);

/**
 * @brief Stops the SOEM master and cleans up resources.
 */
void soem_interface_stop_master(void);

/**
 * @brief Helper function to perform an SDO write operation to a specific slave.
 * @param slave_idx The index of the slave (1-based).
 * @param index The 16-bit object dictionary index.
 * @param subindex The 8-bit object dictionary subindex.
 * @param data_size The size of the data to write in bytes.
 * @param data Pointer to the data to write.
 * @return 0 on success, -1 on failure.
 */
int soem_interface_write_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data);

/**
 * @brief Helper function to perform an SDO read operation from a specific slave.
 * @param slave_idx The index of the slave (1-based).
 * @param index The 16-bit object dictionary index.
 * @param subindex The 8-bit object dictionary subindex.
 * @param data_size The size of the data to read in bytes.
 * @param data Pointer to the buffer to store the read data.
 * @return 0 on success, -1 on failure.
 */
int soem_interface_read_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data);

/**
 * @brief Attempts to set a specific EtherCAT slave to a desired state.
 * @param slave_idx The index of the slave (0 for all slaves, 1-based for specific).
 * @param desired_state The target EtherCAT state (e.g., EC_STATE_PRE_OP, EC_STATE_OPERATIONAL).
 * @return 0 on success, -1 on failure.
 */
int soem_interface_set_ethercat_state(uint16_t slave_idx, ec_state desired_state);

/**
 * @brief Configures PDO mapping dynamically for a specific slave.
 * This function implements the steps for dynamic PDO remapping as per Synapticon documentation.
 * @param slave_idx The index of the slave (1-based).
 * @param pdo_assign_idx The index of the PDO Assign object (e.g., 0x1C12 for RxPDO, 0x1C13 for TxPDO).
 * @param pdo_map_idx The index of the PDO Mapping object (e.g., 0x1600 for RxPDO, 0x1A00 for TxPDO).
 * @param mapped_objects An array of 32-bit values, where each value represents an object to map:
 * (Index << 16 | Subindex << 8 | LengthInBits).
 * @param num_mapped_objects The number of objects in the mapped_objects array.
 * @return 0 on success, -1 on failure.
 */ 
int soem_interface_configure_pdo_mapping_enhanced(uint16_t slave_idx, uint16_t pdo_assign_idx, uint16_t pdo_map_idx, uint32_t *mapped_objects, uint8_t num_mapped_objects);

// --- CiA 402 State Machine Helper Functions ---

/**
 * @brief Determines CiA 402 state from statusword.
 * @param statusword The statusword value from the servo motor.
 * @return The corresponding CiA 402 state.
 */
cia402_state_t get_cia402_state(uint16_t statusword);

/**
 * @brief Returns a human-readable string for the CiA 402 state.
 * @param state The CiA 402 state.
 * @return A string representation of the state.
 */
const char* get_cia402_state_name(cia402_state_t state);

/**
 * @brief Determines the required controlword for a state transition.
 * @param current_state The current CiA 402 state.
 * @param target_state The desired CiA 402 state.
 * @return The controlword value needed for the transition.
 */
uint16_t get_cia402_controlword_for_transition(cia402_state_t current_state, cia402_state_t target_state);

/**
 * @brief Checks if a specific slave is in operational state.
 * @param slave_idx The index of the slave to check.
 * @return 1 if operational, 0 if not.
 */
int is_slave_operational(int slave_idx);

/**
 * @brief Returns a human-readable string for EtherCAT state.
 * @param state The EtherCAT state value.
 * @return A string representation of the state.
 */
const char* get_state_name(uint16_t state);

/**
 * @brief Initializes CiA 402 parameters for a specific slave.
 * @param slave_idx The index of the slave to initialize.
 * @return 0 on success, -1 on failure.
 */
int initialize_cia402_parameters(uint16_t slave_idx);

/**
 * @brief Performs CiA 402 state machine transition to operational state.
 * @param slave_idx The index of the slave.
 * @return 0 on success, -1 on failure.
 */
int perform_cia402_transition_to_operational(uint16_t slave_idx);

#ifdef __cplusplus
}
#endif

#endif // SOEM_INTERFACE_H
