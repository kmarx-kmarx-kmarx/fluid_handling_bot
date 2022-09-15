/*
   SSCX.h
   Contains utilities for running the SSCX pressure sensor.

    Created on: 9/13/2022
        Author: Kevin Marx
*/

#ifndef SSCX_H_
#define SSCX_H_

#include <Wire.h>
#include <stddef.h>
#include <stdint.h>

// Constants for operation
#define SSCX_ADDRESS       0x28  // hard-coded address in sensor - SSCMRRV015PD2A3 is I2C, address hard-coded
#define SSCX_OUT_MIN     1638.0  // 10% of 2^14, we do not expect to see counts below this
#define SSCX_OUT_MAX    14745.0  // 90% of 2^14, we do not expect to see counts above this
#define SSCX_PSI_MAX       15.0  // We are using the SSCMRRV015PD2A3; +- 15 PSI
#define SSCX_PSI_MIN      -15.0
#define SSCX_OUT_SCALE    ((SSCX_PSI_MAX - SSCX_PSI_MIN)/(SSCX_OUT_MAX - SSCX_OUT_MIN)) // macro for counts-to-PSI scale factor
#define SSCX_TEMP_SCALE   (200.0 / 2047.0) // conversion factor for getting the temperature in degrees C
#define SSCX_TEMP_OFFSET  -50.0  // offset factor for getting the temperature in degrees C 
#define SSCX_FLAG_MASK    0b11000000 // mask for the status bits
#define SSCX_LOWTEMP_MASK 0b11100000 // mask for the lower temperature bits
#define SSCX_LOWTEMP_SHIFT         5 // bits to shift the low byte
#define SSCX_HITEMP_SHIFT (8 - SSCX_LOWTEMP_SHIFT)

// Function headers
uint8_t SSCX_read(TwoWire &W, int16_t *readings);
float   SSCX_to_celsius(int16_t raw_temp);
float   SSCX_to_psi(int16_t raw_press);

#endif /* SSCX_H_ */
