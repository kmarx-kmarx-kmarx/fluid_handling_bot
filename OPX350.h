/*
   OPX350.h
   Contains utilities for running the OPX350 optical bubble sensor.

    Created on: 9/14/2022
        Author: Kevin Marx
*/

#ifndef OPX350_H_
#define OPX350_H_

#include <stddef.h>
#include <stdint.h>
#include <Arduino.h>

// Constants for operation
#define CALIB_LOW_TIME_MS  700
#define CALIB_TIMEOUT_MS   1000

// Function headers
void OPX350_init(uint8_t logic_pin, uint8_t calib_pin);
bool OPX350_calib(uint8_t logic_pin, uint8_t calib_pin);
bool OPX350_read(uint8_t logic_pin);


#endif /* OPX350_H_ */
