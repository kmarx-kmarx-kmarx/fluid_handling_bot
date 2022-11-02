/*
   STATES.h
   Contains PID and main control loop state machine definitions and functions

    Created on: 10/7/2022
        Author: Kevin Marx
*/


#ifndef STATES_H_
#define STATES_H_

#include <stdint.h>
#include <Arduino.h>
#include "TTP.h"

// MCU internal program state definitions
#define INTERNAL_PROGRAM_IDLE   0
#define INTERNAL_PROGRAM_LOAD_RESERVOIR   1
#define INTERNAL_PROGRAM_LOAD_RESERVOIR_START 2
#define INTENRAL_PROGRAM_LOAD_FLOW 3
#define INTERNAL_PROGRAM_UNLOAD_START 4
#define INTERNAL_PROGRAM_UNLOAD 5
#define INTERNAL_PROGRAM_PREUSE_CHECK_PRESSURE   6
#define INTERNAL_PROGRAM_PREUSE_CHECK_VACUUM   7
#define INTERNAL_PROGRAM_INITIAL 8
#define INTERNAL_PROGRAM_CLEAR_LINES_START 9
#define INTERNAL_PROGRAM_CLEAR_LINES 10

// PID loop definitions
// PID control parameters and variables
#define IDLE_LOOP     0
#define VACUUM_LOOP   1
#define PRESSURE_LOOP 2
#define FLOWRATE_LOOP 3

// bang-bang params
#define UPPER_FLOW_THRESH 500
#define LOWER_FLOW_THRESH 200

// Function headers
void pid_loop(HardwareSerial &S, uint8_t mode, float flow_reading, float vacuum_reading, float pressure_reading, int8_t sign, float setpoint, float &measurement, float &disc_pump_power);

void pid_reset();

#endif /* STATES_H_ */
