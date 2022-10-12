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
#define INTERNAL_PROGRAM_LOAD_RESERVIOR   1
#define INTERNAL_PROGRAM_LOAD_RESERVIOR_START 2
#define INTERNAL_PROGRAM_PREUSE_CHECK_PRESSURE   5
#define INTERNAL_PROGRAM_PREUSE_CHECK_VACUUM   6
#define INTERNAL_PROGRAM_INITIAL 7

// PID loop definitions
// PID control parameters and variables
#define IDLE_LOOP     0
#define PRESSURE_LOOP 1
#define FLOWRATE_LOOP 2

// Function headers
void pid_loop(HardwareSerial &S, uint8_t mode, float flow_reading, float pressure_reading, float setpoint, float &measurement, float &disc_pump_power);

#endif /* STATES_H_ */
