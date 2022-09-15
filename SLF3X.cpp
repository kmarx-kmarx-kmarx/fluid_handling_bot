/*
    SLF3X.cpp
    This file contains functions for initializing and reading from the flow sensor.

    bool SLF3X_init: Initialize sensor on given I2C bus, returns false if sensor not present
      Arguments: uint16_t n_tries, TwoWire &W, uint8_t medium
    uint8_t SLF3X_read: Read a value from the sensor on a given I2C bus into the shared readings variable and optionally perform the CRC. Returns flags indicating specific failures in the reading process.
      Arguments: bool do_crc, TwoWire &W, int16_t *readings
    float SLF3X_to_celsius: Convert from raw to degrees Celsius
      Arguments: int16_t raw_temp
    float SLF3X_to_uLmin: Convert from raw to microliters per minute
      Arguments: int16_t raw_flow
    
    static uint8_t crc: Perform a CRC
      Arguments: uint8_t *data
*/

#include "SLF3X.h"

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: calculate_crc() takes a two-uint8_t array as an argument and returns the CRC-8

  OPERATION:   We pass the data array by reference and perform a (non-destructive) CRC calculating operation using the CRC parameters defined in the sensor's datasheet, then we return the array.

  ARGUMENTS:
      uint8_t *data: pointer to data array

  RETURNS:
      uint8_t calc_crc: the calculated CRC

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: 
      uint8_t *data: The array is read and is not written to

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
static uint8_t calculate_crc(uint8_t *dat) {
  uint8_t calc_crc = 0xFF;
  for (uint8_t b = 0; b < 2; ++b) {
    calc_crc ^= (dat[b]);
    for (uint8_t i = 8; i > 0; --i) {
      if (calc_crc & 0x80) {
        calc_crc = (calc_crc << 1) ^ CRC_POLYNOMIAL;
      } else {
        calc_crc = (calc_crc << 1);
      }
    }
  }

  return calc_crc;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: SLF3X_init() initializes the sensor on a given I2C bus. It returns true if it was initialized successfully and false otherwise.

  OPERATION:   We first try sending reset signals until the chip is properly reset, then we send requests to put it in continuous mode. If the number of tries exceeds n_tries, give up and return false. Otherwise, the sensor was initialized properly and return true.

  ARGUMENTS:
      uint16_t n_tries:  number of times to try resetting and reading from the sensor before giving up
      TwoWire &W:        streaming class to read from (e.g. Wire, Wire1, etc. Must be I2C)
      uint8_t medium:    Select whether to use water ior IPA calibration data

  RETURNS:
      bool sensor_connected: true if the sensor returns valid data, false otherwise

  INPUTS / OUTPUTS: The I2C lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
bool SLF3X_init(uint16_t n_tries, TwoWire &W, uint8_t medium) {
  uint16_t n = 0;
  int8_t ret = 0;

  n_tries++; // Account for guaranteed 2 tries (1 resetting, 1 reading)

  do {
    n++; // Count number of times we have tried resetting
    // Send reset signal
    W.beginTransmission(GEN_RST_ADDRESS);
    W.write(GEN_RST_CMD);
    ret = W.endTransmission();
    delay(50);
  } while (n < n_tries && ret != 0);

  // If we have not successfully reset, return out of this function
  if (n >= n_tries) {
    return false;
  }

  // Set the sensor to cts mode
  n_tries++;
  do {
    n++; // Additionally count number of tries setting the sensor
    W.beginTransmission(SLF3X_ADDRESS);
    W.write(START_CTS_MEAS);
    W.write(medium);
    ret = W.endTransmission();
    delay(50);
  } while (n < n_tries && ret != 0);

  // If we have not successfully set, return out of this function
  if (n >= n_tries) {
    return false;
  }

  // If we get here, that means setup was a success
  delay(100); // at least 60 ms needed for reliable measurements to begin
  return true;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: SLF3X_read() reads the raw flow rate, temperature, and flags from the sensor over I2C and optionally calculates the CRC. If there is a CRC mismatch, we return INT16_MAX in the array instead of the actual value

  OPERATION:   We request 9 uint8_ts from the sensor, flow_high, flow_low, flow_crc, temp_high, temp_low, temp_crc, flags_high, flags_low, flags_crc. We read the data into a shared array and optionally perform the CRC; if the CRC fails or if there is some other error, we return false. If the read is successfull, return true.

  ARGUMENTS:
      bool do_crc:       Set true if the CRC should be performed, set false otherwise
      TwoWire &W:        streaming class to read from (e.g. Wire, Wire1, etc. Must be I2C)
      int16_t *readings: Pointer to array for storing data. The data at this array will be overwritten.
      
  RETURNS:
      uint8_t error: 0 if the reading succeeded, bits set if there was an error
        bit 0 set - couldn't get any readings
        bit 1 set - flow CRC failed
        bit 2 set - temperature CRC failed
        bit 3 set - flags CRC failed

  INPUTS / OUTPUTS: The I2C lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
     int16_t readings[3]: Used to store the data. We are writing to this array.
        readings[0]: raw flow value
        readings[1]: raw temp value
        readings[3]: flags from the SLF3X

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint8_t SLF3X_read(bool do_crc, TwoWire &W, int16_t *readings) {
  uint8_t    crc[3];  // store CRC
  uint8_t    rx[6];   // store raw data
  uint8_t    err = 0; // assume no error
  uint8_t    crc_calc;

  // put default values in readings[]
  for (uint8_t i = 0; i < 3; i++) {
    readings[i] = INT16_MAX;
  }

  W.requestFrom(SLF3X_ADDRESS, 9);
  // Return with error if we fail to read all the uint8_ts
  if (W.available() < 9) {
    err |= (1 << 0);
    return err;
  }

  // Read all the data
  for (uint8_t i = 0; i < 3; i++) {
    rx[(2 * i)] = W.read(); // read the MSB from the sensor
    rx[(2 * i) + 1] = W.read(); // read the LSB from the sensor
    crc[i]    = W.read();
    readings[i]  = rx[(2 * i)] << 8;
    readings[i] |= rx[(2 * i) + 1];
  }
  // If we aren't doing the CRC, we're done here
  if (!do_crc) {
    return err;
  }
  // Perform the CRC
  for (uint8_t i = 0; i < 3; i++) {
    crc_calc = calculate_crc(rx + (2 * i * sizeof(rx[0])));
    // Set the error flags if there was a problem reading
    if (crc_calc != crc[i]) {
      err |= (1 << (i + 1));
    }
  }

  return err;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: SLF3X_to_celsius() converts from a raw sensor reading to degrees Celsius

  OPERATION:   We divide the raw value by the scale value and return it

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
float SLF3X_to_celsius(int16_t raw_temp){
  return ((float)raw_temp)/SCALE_FACTOR_TEMP;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: SLF3X_to_uLmin() converts from a raw sensor reading to microliters per minute

  OPERATION:   We divide the raw value by the scale value and return it

  ARGUMENTS:
      int16_t  raw_flow: raw flow value from the sensor

  RETURNS:
      float flow:        flow rate in units microliters per minute

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
float SLF3X_to_uLmin(int16_t raw_flow){
  return ((float)raw_flow)/SCALE_FACTOR_FLOW;
}
