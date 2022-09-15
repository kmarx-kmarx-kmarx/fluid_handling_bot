/*
    teensy41_fluidhandling.ino:
        This project will incrementally implement features necessary for the Prakash Lab's new fluid handling robot.

     DESCRIPTION: This file sets up and runs the main control loop.
        setup(): Initialize Serial and communications with the motor controller IC
        loop():  Read commands over Serial and perform them

        set_read_sensors_flag(): set a global bool indicating the sensors should be read
        set_send_update_flag():  set a global bool indicating the status should be sent over serial

    Shared Variables:
        bool    SLF3X_present          - set true when the flow sensor is initialized properly
        int16_t SLF3X_readings[3]      - stores flow, temperature, and flag readings from the flow sensor
        uint8_t SLF3X_err;             - stores data on how the reading went
        bool    OCB350_0_present       - set true when bubble sensor 0 is calibrated properly
        bool    OCB350_0_reading       - set true when bubbles are present in sensor 0
        bool    OCB350_1_present
        bool    OCB350_1_reading
        bool    SSCX_0_PRESENT         - set true when pressure sensor 0 is iniialized properly
        int16_t SSCX_0_readings[2]     - stores pressure and temperature readings from the pressure sensor
        uint8_t SSCX_0_err;            - stores data on how the reading went
        bool    SSCX_1_PRESENT
        int16_t SSCX_1_readings[2]
        uint8_t SSCX_1_err
        volatile bool flag_read_sensors - indicates the sensors should be read during the next loop
        volatile bool flag_send_update  - indicates debug data should be sent during the next loop


    Dependencies:
        SLF3X.h     : Functions for initializing and reading from the flow sensor
        SSCX.h      : Functions for reading from the pressure sensor
        OPX350.h    : Functions for initializing and reading from the bubble sensor
        Wire.h      : For I2C communication

    Author: Kevin Marx
    Created on: 9/13/2022
*/

#include <Wire.h>
#include "SLF3X.h"
#include "SSCX.h"
#include "OPX350.h"

// SLF3X flow sensor parameters
#define W_SLF3X      Wire1
#define SLF3X_NTRIES 10
#define PERFORM_CRC  true
// SLF3X flow sensor variables
bool    SLF3X_present = false;
int16_t SLF3X_readings[3];
uint8_t SLF3X_err;

// OPX350 bubble sensor parameters
// bubble sensor 0
#define OCB350_0_CALIB  29
#define OCB350_0_LOGIC  30
bool    OCB350_0_present = false;
bool    OCB350_0_reading;
// bubble sensor 1
#define OCB350_1_CALIB  31
#define OCB350_1_LOGIC  32
bool    OCB350_1_present = false;
bool    OCB350_1_reading;

// SSCX pressure sensor parameters
#define W_SSCX           Wire1
// SSCX pressure sensor variables
// pressure sensor 0
bool    SSCX_0_PRESENT = false;
int16_t SSCX_0_readings[2];
uint8_t SSCX_0_err;
// pressure sensor 1
bool    SSCX_1_PRESENT = false;
int16_t SSCX_1_readings[2];
uint8_t SSCX_1_err;

// Timer parameters
// Set flag to read the sensors every 5 ms
#define READ_SENSORS_INTERVAL_US 5000
volatile bool flag_read_sensors = false;
IntervalTimer Timer_read_sensors_input;

// Set flag to send updates every 20 ms
#define SEND_UPDATE_INTERVAL_US 20000
volatile bool flag_send_update = false;
IntervalTimer Timer_send_update_input;

void setup() {
  // Initialize Serial to communicate with the computer
  Serial.begin(2000000);

  // Initialize flow sensor
  Serial.print("Initializing flow sensor... ");
  SLF3X_present = SLF3X_init(SLF3X_NTRIES, W_SLF3X, MEDIUM_WATER);
  if (SLF3X_present) {
    Serial.println("initialized");
  }
  else {
    Serial.println("not detected");
  }

  // Initialize and calibrate the bubble sensors
  Serial.print("Initializing bubble sensor 0. Make sure logic output A is connected... ");
  OPX350_init(OCB350_0_LOGIC, OCB350_0_CALIB);
  OCB350_0_present =  OPX350_calib(OCB350_0_LOGIC, OCB350_0_CALIB);
  if (OCB350_0_present) {
    Serial.println("calibrated");
  }
  else {
    Serial.println("calibration failed");
  }
  Serial.print("Initializing bubble sensor 1. Make sure logic output A is connected... ");
  OPX350_init(OCB350_1_LOGIC, OCB350_1_CALIB);
  OCB350_1_present =  OPX350_calib(OCB350_1_LOGIC, OCB350_1_CALIB);
  if (OCB350_1_present) {
    Serial.println("calibrated");
  }
  else {
    Serial.println("calibration failed");
  }

  // Initialize pressure sensor
  SSCX_init(W_SSCX);

  // Initialize timed interrupts
  // When they trigger, set a flag to indicate something should be done the next loop cycle
  Timer_read_sensors_input.begin(set_read_sensors_flag, READ_SENSORS_INTERVAL_US);
  Timer_send_update_input.begin(set_send_update_flag, SEND_UPDATE_INTERVAL_US);
}

void loop() {
  // flag to indicate fresh data in the shared variables
  bool sensors_read = false;

  // Handle the timer flags first
  if (flag_read_sensors) {
    flag_read_sensors = false;
    sensors_read = true;
    if (SLF3X_present) {
      SLF3X_err = SLF3X_read(PERFORM_CRC, W_SLF3X, SLF3X_readings);
    }
    if (OCB350_0_present) {
      OCB350_0_reading = OPX350_read(OCB350_0_LOGIC);
    }
    if (OCB350_1_present) {
      OCB350_1_reading = OPX350_read(OCB350_1_LOGIC);
    }

    SSCX_0_err = SSCX_read(W_SSCX, SSCX_0_readings);
    SSCX_1_err = SSCX_read(W_SSCX, SSCX_1_readings);
  }
  if (flag_send_update) {
    flag_send_update = false;
    Serial.println("~~~~~~~~~~~~~~~~~~~");
    if (SLF3X_present) {
      Serial.print("SLF3X Error:   ");
      Serial.println(SLF3X_err, BIN);
      Serial.print("Flow (uL/min): ");
      Serial.println(SLF3X_to_uLmin(SLF3X_readings[SLF3X_FLOW_IDX]));
      Serial.print("Temp (deg C):  ");
      Serial.println(SLF3X_to_celsius(SLF3X_readings[SLF3X_TEMP_IDX]));
      Serial.print("Flags:       ");
      Serial.println(SLF3X_readings[SLF3X_FLAG_IDX], BIN);
    }
    if (OCB350_0_present) {
      Serial.print("OCB350_0 Bubbles Present: ");
      Serial.println(OCB350_0_reading, BIN);
    }
    if (OCB350_1_present) {
      Serial.print("OCB350_1 Bubbles Present: ");
      Serial.println(OCB350_1_reading, BIN);
    }
    Serial.print("SSCX_0 Error:    ");
    Serial.println(SSCX_0_err, BIN);
    Serial.print("Pressure (psi): ");
    Serial.println(SSCX_to_psi(SSCX_0_readings[SSCX_PRESS_IDX]));
    Serial.print("Temp (deg C):  ");
    Serial.println(SSCX_to_celsius(SSCX_0_readings[SSCX_TEMP_IDX]));
    Serial.print("SSCX_1 Error:    ");
    Serial.println(SSCX_1_err, BIN);
    Serial.print("Pressure (psi): ");
    Serial.println(SSCX_to_psi(SSCX_1_readings[SSCX_PRESS_IDX]));
    Serial.print("Temp (deg C):  ");
    Serial.println(SSCX_to_celsius(SSCX_1_readings[SSCX_TEMP_IDX]));
    Serial.println(millis());
  }
}

void set_read_sensors_flag() {
  flag_read_sensors = true;
  return;
}
void set_send_update_flag() {
  flag_send_update = true;
  return;
}
