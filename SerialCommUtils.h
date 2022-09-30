#ifndef SERIALCOMMS_H_
#define SERIALCOMMS_H_

#define FROM_MCU_MSG_LENGTH  25  // search for MCU_MSG_LENGTH in _def.py
#define TO_MCU_CMD_LENGTH    15  // search for MCU_CMD_LENGTH in _def.py
#define CMD_START_BYTE      '>'  // All incoming serial commands will start with this byte
// command sets - these are commands from the computer
// each of the commands may break down to multiple internal programs in the MCU
// search for class CMD_SET in _def.py
#define CLEAR   0
#define REMOVE_MEDIUM   1
#define ADD_MEDIUM   2
#define SET_SELECTOR_VALVE   10
#define SET_10MM_SOLENOID_VALVE   11
#define SET_SOLENOID_VALVE_B   12
#define SET_SOLENOID_VALVE_C   13
#define SET_MANUAL_CONTROL   20
#define ENABLE_PRESSURE_CONTROL_LOOP   30
#define SET_PRESSURE_CONTROL_SETPOINT_PSI   31
#define SET_PRESSURE_CONTROL_LOOP_P_COEFFICIENT   32
#define SET_PRESSURE_CONTROL_LOOP_I_COEFFICIENT   33
#define PREUSE_CHECK_PRESSURE   40
#define PREUSE_CHECK_VACUUM   41
// defs for parsing the commands
#define CMD_ENABLE  1
#define CMD_DISABLE 0

// command parameters
// search for class MCU_CMD_PARAMETERS in _def.py
#define CONSTANT_POWER   0
#define CONSTANT_PRESSURE   1
#define CONSTANT_FLOW   2
#define VOLUME_CONTROL   3

// command execution status constants
// search for class CMD_EXECUTION_STATUS in _def.py
#define COMPLETED_WITHOUT_ERRORS   0
#define IN_PROGRESS   1
#define CMD_CHECKSUM_ERROR   2
#define CMD_INVALID   3
#define CMD_EXECUTION_ERROR   4
#define ERROR_CODE_EMPTYING_THE_FLUDIIC_LINE_FAILED   100
#define ERROR_CODE_PREUSE_CHECK_FAILED   110


#endif /* SERIALCOMMS_H_ */
