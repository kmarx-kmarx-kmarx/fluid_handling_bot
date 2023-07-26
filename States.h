/*
   STATES.h
   Contains bang-bang and main control loop state machine definitions and functions

    Created on: 10/7/2022
        Author: Kevin Marx
*/


#ifndef STATES_H_
#define STATES_H_

#include <stdint.h>
#include <Arduino.h>
#include "TTP.h"

// MCU internal program state definitions
#define INTERNAL_STATE_IDLE                        0
#define INTERNAL_STATE_LOAD_MEDIUM                 2
#define INTERNAL_STATE_LOAD_MEDIUM_START           1
#define INTERNAL_STATE_VENT_VB0                    3
#define INTERNAL_STATE_UNLOAD_START                4
#define INTERNAL_STATE_CLEAR_START                 5
//#define INTERNAL_STATE_PREUSE_CHECK_PRESSURE       6
//#define INTERNAL_STATE_PREUSE_CHECK_VACUUM         7

#define INTERNAL_STATE_BUBBLE_START              8
#define INTERNAL_STATE_BUBBLE_FINISH             9

// bang-bang def
#define IDLE_LOOP     0
#define BANG_BANG_FLOWRATE 1

// bang-bang params
#define UPPER_FLOW_THRESH 2000
#define LOWER_FLOW_THRESH 200
#define PUMP_PWR_mW_GO     96 // 50   

// Function headers
void bang_bang_flowrate(HardwareSerial &S, uint8_t mode, float flow_reading, int8_t sign, float &measurement, float &disc_pump_power);

void set_bang_bang_params(float lower_thresh, float upper_thresh, float min_pwr, float max_pwr);

#endif /* STATES_H_ */
