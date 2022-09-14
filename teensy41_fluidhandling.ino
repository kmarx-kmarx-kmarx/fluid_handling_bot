/*
    teensy41_fluidhandling.ino:
        This project will incrementally implement features necessary for the Prakash Lab's new fluid handling robot.

     DESCRIPTION: This file sets up and runs the main control loop.
        setup(): Initialize Serial and communications with the motor controller IC
        loop():  Read commands over Serial and perform them

        set_read_sensors_flag(): set a global bool indicating the sensors should be read
        set_send_update_flag():  set a global bool indicating the status should be sent over serial

    Shared Variables:
        None

    Dependencies:
        SLF3X.h     : Functions for initializing and reading from the flow sensor
        Wire.h      : For I2C communication

    Author: Kevin Marx
    Created on: 9/13/2022
*/

#include <Wire.h>
#include "SLF3X.h"

// SLF3X flow sensor parameters
#define W_SLF3X      Wire1
#define NTRIES_SLF3X 10
#define PERFORM_CRC  true
// SLF3X flow sensor variables
bool    flow_sensor_present = false;
int16_t SLF3X_readings[3];
uint8_t SLF3X_err;

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
  // Initialize Serial
  Serial.begin(2000000);
  // Initialize flow sensor
  int16_t n_tries = -1;
  do {
    flow_sensor_present = SLF3X_init(NTRIES_SLF3X, W_SLF3X, MEDIUM_WATER);
    n_tries++;
  } while (n_tries < NTRIES_SLF3X && flow_sensor_present != true);

  // Initialize timed interrupts
  // When they trigger, set a flag to indicate something should be done the next loop cycle
  Timer_read_sensors_input.begin(set_read_sensors_flag, READ_SENSORS_INTERVAL_US);
  Timer_send_update_input.begin(set_send_update_flag, SEND_UPDATE_INTERVAL_US);

}

void loop() {
  // Handle the timer flags first
  if (flag_read_sensors) {
    flag_read_sensors = false;
    if (flow_sensor_present) {
      SLF3X_err = SLF3X_read(PERFORM_CRC, W_SLF3X, SLF3X_readings);
    }
  }
  if (flag_send_update) {
    flag_send_update = false;
    if (flow_sensor_present) {
      Serial.println("~~~~~~~~~~~~~~~~~~~");
      Serial.print("SLF3X Error:   ");
      Serial.println(SLF3X_err, BIN);
      Serial.print("Flow (uL/min): ");
      Serial.println(SLF3X_to_uLmin(SLF3X_readings[SLF3X_FLOW_IDX]));
      Serial.print("Temp (deg C):  ");
      Serial.println(SLF3X_to_celsius(SLF3X_readings[SLF3X_TEMP_IDX]));
      Serial.print("Flags:       ");
      Serial.println(SLF3X_readings[SLF3X_FLAG_IDX], BIN);
    }
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
