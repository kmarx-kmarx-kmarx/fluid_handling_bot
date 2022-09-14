/*
    teensy41_fluidhandling.ino:
        This project will incrementally implement features necessary for the Prakash Lab's new fluid handling robot.

     DESCRIPTION: This file sets up and runs the main control loop.
        setup(): Initialize Serial and communications with the motor controller IC
        loop():  Read commands over Serial and perform them

    Shared Variables:
        None

    Motor Parameters:


    Dependencies:
        SLF3X.h     : Functions for initializing and reading from the flow sensor
        Wire.h      : For I2C communication

    Author: Kevin Marx
    Created on: 9/13/2022
*/

#include <Wire.h>
#include "SLF3X.h"

#define W_SLF3X      Wire1
#define NTRIES_SLF3X 10

void setup() {
  Serial.begin(2000000);
  while (!SLF3X_init(NTRIES_SLF3X, W_SLF3X, MEDIUM_WATER)) {
    Serial.println("Initializing SLF3X");
  }
}

void loop() {
  int16_t readings[3];
  uint8_t err;

  err = SLF3X_read(true, W_SLF3X, readings);

  Serial.print("Error: ");
  Serial.println(err, BIN);
  Serial.print("Flow:  ");
  Serial.println(SLF3X_to_uLmin(readings[FLOW_IDX]));
  Serial.print("Temp:  ");
  Serial.println(SLF3X_to_celsius(readings[TEMP_IDX]));
  Serial.print("Flags: ");
  Serial.println(readings[FLAG_IDX], BIN);


}
