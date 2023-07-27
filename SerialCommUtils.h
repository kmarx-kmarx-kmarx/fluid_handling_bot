#ifndef SERIALCOMMS_H_
#define SERIALCOMMS_H_
//
#include <Arduino.h>
//#include "IOdefs.h"
#include <stdint.h>

// communication with the python software
/*
#########################################################
#########   MCU -> Computer message structure   #########
#########################################################
byte 0-1  : computer -> MCU CMD counter (UID)
byte 2    : cmd from host computer (error checking through check sum => no need to transmit back the parameters associated with the command)
        <see below for command set>
byte 3    : status of the command
        - 1: in progress
        - 0: completed without errors
        - 2: error in cmd check sum
        - 3: invalid cmd
        - 4: error during execution
byte 4    : MCU internal program being executed
        - 0: idle
          <see below for command set>
byte 5    : state of valve A1,A2,B1,B2,bubble_sensor_1,bubble_sensor_2,x,x
byte 6    : state of valve C1-C7, manual input bit
byte 7-8  : state of valve D1-D16
byte 9    : state of selector valve
byte 10-11  : pump power
byte 12-13  : pressure sensor 1 reading (vacuum)
byte 14-15  : pressure sensor 2 reading (pressure)
byte 16-17  : flow sensor 1 reading (downstream)
byte 18-19  : flow sensor 2 reading (upstream)
byte 20     : elapsed time since the start of the last internal program (in seconds)
byte 21-22  : volume (ul), range: 0 - 5000
byte 23-24  : reserved
*/

#define FROM_MCU_MSG_LENGTH  25  // search for MCU_MSG_LENGTH in _def.py
#define TO_MCU_CMD_LENGTH    15  // search for MCU_CMD_LENGTH in _def.py
// command sets - these are commands from the computer
// each of the commands may break down to multiple internal programs in the MCU
// search for class CMD_SET in _def.py
#define CLEAR   0
#define INITIALIZE_DISC_PUMP 1
#define INITIALIZE_PRESSURE_SENSORS 2
#define INITIALIZE_FLOW_SENSOR 3
#define INITIALIZE_BUBBLE_SENSORS 4
#define INITIALIZE_BANG_BANG_PARAMS 5
#define PRETEST_PRESSURE_START 6
#define PRETEST_VACUUM_START 7
#define LOAD_MEDIUM_START 8
#define UNLOAD_MEDIUM_START 9
#define CLEAR_MEDIUM_START 10
#define SET_SOLENOID_VALVES   11
#define LOAD_MEDIUM_VOLUME_START 12
#define UNLOAD_MEDIUM_VOLUME_START 16
#define VENT_VB0 13
#define INITIALIZE_SELECTOR_VALVE 14
#define SET_SELECTOR_VALVE 15

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
#define CMD_INVALID   2
#define CMD_EXECUTION_ERROR   3

//uint8_t current_serial_command;
void send_serial_data(float ttp_max_pwr, float VOLUME_UL_MAX, uint8_t command_execution_status, uint8_t internal_program, bool liquid_present_0, bool liquid_present_1, uint8_t valveset, uint16_t NXP33996_state, int16_t pressure_0_raw, int16_t pressure_1_raw, int16_t flowrate_0_raw, uint8_t time_elapsed_s, float volume_ul, uint8_t selectorset, float disc_pump_power);
bool read_serial_command(byte payloads[TO_MCU_CMD_LENGTH - 3], uint8_t &cmd);

#endif /* SERIALCOMMS_H_ */
