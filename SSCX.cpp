/*
    SSCX.cpp
    This file contains functions for initializing and reading from the pressure sensor.

    uint8_t SSCX_read: Read a value from the sensor on a given I2C bus into the shared readings variable. Returns flags indicating specific failures in the reading process.
      Arguments: TwoWire &W, int16_t *readings
    float SSCX_to_celsius: Convert from raw to degrees Celsius
      Arguments: int16_t raw_temp
    float SSCX_to_psi: Convert from raw to pounds per square inch
      Arguments: int16_t raw_press
*/
#include "SSCX.h"

void SSCX_init(TwoWire &W){
  W.begin();
  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: SSCX_read() reads the raw pressure, temperature, and flags from the sensor over I2C. The flags are returned in a uint8_t and the pressure and temperature are written to a shared array

  OPERATION:   We request 4 uint8_ts from the sensor, status and pressure_high, pressure_low, temp_high, and temp_low. The status bits are returned as part of the flags while the pressure and temperature data is stored in the shared readings array.

  ARGUMENTS:
      TwoWire &W:        streaming class to read from (e.g. Wire, Wire1, etc. Must be I2C)
      int16_t *readings: Pointer to array for storing data. The data at this array will be overwritten.

  RETURNS:
      uint8_t error: 0 if the reading succeeded, bits set if there was an error
        bit 0 set    - couldn't get any readings
        bits 1..6    - no meaning
        bits 7,6 set - 00: valid data
                       01: command mode, invalid
                       10: stale data (data already been fetched)
                       11: diagnostic condition

  INPUTS / OUTPUTS: The I2C lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
     TwoWire &W:          I2C class
     int16_t readings[2]: Used to store the data. We are writing to this array.
        readings[0]: raw pressure value
        readings[1]: raw temperature value

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint8_t SSCX_read(TwoWire &W, int16_t *readings) {
  uint8_t    rx[4];   // store raw data
  uint8_t    err = 0; // assume no error

  // put default values in readings[]
  for (uint8_t i = 0; i < 2; i++) {
    readings[i] = INT16_MAX;
  }

  W.requestFrom(SSCX_ADDRESS, 4);
  // Return with error if we fail to read all the uint8_ts
  if (W.available() < 4) {
    err |= (1 << 0);
    return err;
  }

  // Read all the data
  for (uint8_t i = 0; i < 4; i++) {
    rx[i] = W.read();
  }

  // format the err bits
  err |= (rx[0] & SSCX_FLAG_MASK);
  // get the raw pressure
  readings[0] = ((rx[0] & ~SSCX_FLAG_MASK) << 8 );
  readings[0] |= rx[1];
  // get the raw temperature
  readings[1] = rx[2] << SSCX_HITEMP_SHIFT;
  rx[3] = rx[3] >> SSCX_LOWTEMP_SHIFT;
  readings[1] |= rx[3];

  return err;
}


/*
  -----------------------------------------------------------------------------
  DESCRIPTION: SSCX_to_celsius() converts from a raw sensor reading to degrees Celsius

  OPERATION:   We multiply the raw value by the scale value, add an offset, and return it

  ARGUMENTS:
      int16_t  raw_temp: raw temperature value from the sensor

  RETURNS:
      float temp:         temperature in degrees Celsius

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
float SSCX_to_celsius(int16_t raw_temp) {
  return (raw_temp * SSCX_TEMP_SCALE) + SSCX_TEMP_OFFSET;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: SSCX_to_psi() converts from a raw sensor reading to pounds per square inch

  OPERATION:   We offset the raw value, scale it, then offset it again

  ARGUMENTS:
      int16_t  raw_press: raw pressure value from the sensor

  RETURNS:
      float pressure:     pressure in PSI

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
float SSCX_to_psi(int16_t raw_press) {
  float pressure = constrain(raw_press, SSCX_OUT_MIN, SSCX_OUT_MAX);
  return (pressure - SSCX_OUT_MIN) * SSCX_OUT_SCALE + SSCX_PSI_MIN;
}
