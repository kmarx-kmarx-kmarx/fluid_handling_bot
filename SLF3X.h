/*
   SLF3X.h
   Contains utilities for running the SLF3X flow sensor.

    Created on: 9/13/2022
        Author: Kevin Marx
*/

#ifndef SLF3X_H_
#define SLF3X_H_

#include <Wire.h>
#include <stddef.h>
#include <stdint.h>

// Constants for operation
#define SLF3x_ADDRESS       0x08  // hard-coded address in sensor
#define GEN_RST_ADDRESS     0x00  // send reset signal to this address
#define GEN_RST_CMD         0x06  // command to reset
#define STOP_CTS_MEAS_HIGH  0x3F  // High byte of command to stop continuous measurement
#define STOP_CTS_MEAS_LOW   0xF9  // Low byte
#define START_CTS_MEAS      0x36  // Send this byte followed by one of the two medium select bytes to begin continuous measurement
#define MEDIUM_WATER        0x08  // Use calibration values for water flow measurements
#define MEDIUM_IPA          0x15  // Use calibration values for isopropyl alcohol (IPA)
#define CRC_POLYNOMIAL      0x31  // polynomial for cyclic redundancy check (CRC)

#define SCALE_FACTOR_TEMP   200.0 // 200 per degrees Celsius
#define SCALE_FACTOR_FLOW   10.0  // 10 per (microliter per minute)

#define FLOW_IDX            0     // Where data is stored in readings array
#define TEMP_IDX            1
#define FLAG_IDX            2

// Function headers
bool    SLF3X_init(uint16_t n_tries, TwoWire &W, uint8_t medium);
uint8_t SLF3X_read(bool do_crc, TwoWire &W, int16_t *readings);
float   SLF3X_to_celsius(int16_t raw_temp);
float   SLF3X_to_uLmin(int16_t raw_flow);

#endif /* SLF3X_H_ */
