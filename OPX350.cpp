/*
    OPX350.cpp
    This file contains functions for initializing and reading from the bubble sensor.

    void OPX350_init:  Initialize bubble sensor
      Arguments: uint8_t logic_pin, uint8_t calib_pin
    bool OPX350_calib: Calibrate the bubble sensor. Return false if calib failed
      Arguments: uint8_t logic_pin, uint8_t calib_pin
    bool OPX350_read:  Read the bubble sensor; return true if bubbles are present
      Arguments: uint8_t logic_pin
*/

#include "OPX350.h"


/*
  -----------------------------------------------------------------------------
  DESCRIPTION: OPX350_init() initializes the bubble sensor

  OPERATION:   We set the logic_pin pin as input, calib_pin pin as output, and set calib_pin high (default value)

  ARGUMENTS:
      uint8_t logic_pin: pin number of the logic-level bubble signal
      uint8_t calib_pin: pin number of the logic-level calibration signal

  RETURNS: None

  INPUTS / OUTPUTS: calib_pin is set high.

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
void OPX350_init(uint8_t logic_pin, uint8_t calib_pin) {
  pinMode(logic_pin, INPUT);
  pinMode(calib_pin, OUTPUT);

  digitalWrite(calib_pin, HIGH);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: OPX350_calib() calibrates the bubble sensor. The tube must have be in place and have no bubbles for calibration to be effective.

  OPERATION:   We set calib_pin low for a few milliseconds and wait for the LED current to increase until the logic pin goes low

  ARGUMENTS:
      uint8_t logic_pin: pin number of the logic-level bubble signal
      uint8_t calib_pin: pin number of the logic-level calibration signal

  RETURNS: None

  INPUTS / OUTPUTS: calib_pin is pulsed and we read from logic_pin

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
bool OPX350_calib(uint8_t logic_pin, uint8_t calib_pin) {
  bool reading;
  // Set calib_pin low for a few ms to begin calibration
  digitalWrite(calib_pin, LOW);
  delay(CALIB_LOW_TIME_MS);
  digitalWrite(calib_pin, LOW);
  // get current time for timeout
  uint32_t t_init = millis();
  // wait until the reading goes low or until timeout
  do {
    reading = digitalRead(logic_pin);
  } while (reading != LOW && t_init - millis() < CALIB_TIMEOUT_MS);

  // debugging - get estimate of time it takes for calibration to succeed
  Serial.println(t_init - millis());

  // If calibration failed, reading is false
  return reading;

}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: OPX350_read() returns true if bubbles are present and false otherwise

  OPERATION:   Return opposite of logic_pin's state

  ARGUMENTS:
      uint8_t logic_pin: pin number of the logic-level bubble signal

  RETURNS: None

  INPUTS / OUTPUTS: we read from logic_pin

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
bool OPX350_read(uint8_t logic_pin) {
  return !digitalRead(logic_pin);
}
