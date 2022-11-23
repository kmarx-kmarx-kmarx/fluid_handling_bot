/*
    States.cpp
    This file contains functions for running the PID control loop and main control loop

    PID Functions and Shared Variables:
    Shared Variables:
    // positive pressure PID params:
    float pres_pos_p, float pres_pos_i, float pres_pos_d: PID parameters when generating a positive pressure
    // negative pressure PID params:
    float pres_neg_p, float pres_neg_i, float pres_neg_d: PID for negative pressures (using vacuum bottle)
    // positive flowrate PID params:
    float flow_pos_p, float flow_pos_i, float flow_pos_d: PID params for positive flowrate
    // negative flowrate PID params:
    float flow_neg_p, float flow_neg_i, float flow_neg_d: PID for negative flowrate (using vacuum bottle)
    float pid_i_error:    integral error
    float pid_prev_error: previous error for calculating derivative
    int16_t dt:           previous cycle time for calculating derivative

    Functions:
    float pid_loop: Set the output power using a PI controller, setting either the pressure or the flowrate.
      Arguments: HardwareSerial &S, uint8_t mode, float flow_reading, float vacuum_reading, float setpoint

*/

#include "States.h"

// Store PID parameters - can be modified later
// positive pressure PID params:
float pres_pos_p = 0;
float pres_pos_i = 0;
float pres_pos_d = 0;
// negative pressure PID params:
float pres_neg_p = 0.5;
float pres_neg_i = .005;
float pres_neg_d = .25;
// positive flowrate PID params:
float flow_pos_p = 0.06;
float flow_pos_i = 0.0004;
float flow_pos_d = 240;
// negative flowrate PID params:
float flow_neg_p = 0.1;
float flow_neg_i = 0.05;
float flow_neg_d = 8;

// Store error terms
float pid_i_error = 0;
float pid_prev_error = 0;
// store elapsed time
int16_t dt = 0;

// bang-bang param
float prev_power = 0;

/*  -----------------------------------------------------------------------------
  DESCRIPTION: pid_loop() runs position-integral control over either pressure or flowrate.

  OPERATION:   We first assume the valves are set properly to achieve the setpoint (e.g. no blockages, fluid flowing in the correct direction)
               Operation then depends on the mode.
               IDLE_LOOP - set pump power to 0 and clear the integral error terms
               VACUUM_LOOP - calculate difference between measurement and target and negate it if our target is negative. Assuming our valves are set correctly, the only way to achieve negative pressures is to run the pump harder and the negation lets this happen. We then run PI controls, implementing wind-up limit and constraining the output.
               FLOWRATE_LOOP - same as VACUUM_LOOP but with flow measurement instead of pressure measurement.
               Thenm the disc pump power is bounded and sent to the disc pump

  ARGUMENTS:
      HardwareSerial &S:      stream class to read from Serial, Serial1, etc.
      uint8_t mode:           either IDLE_LOOP, VACUUM_LOOP, or FLOWRATE_LOOP; sets whether the PID loop should be idling, controlling pressure, or controlling flowrate
      float flow_reading:     flowrate reading in units uL/min
      float vacuum_reading: pressure reading in units PSI
      float setpoint:         target value in either units uL/min or PSI depending on mode
      float &measurement:     Overwritten with one of the readings depending on mode
      float &disc_pump_power: Overwritten with the disc pump power in units mW

  RETURNS: None

  INPUTS: None

  OUTPUTS: The disc pump power is set

  SHARED VARIABLES:
      // positive pressure PID params:
      float pres_pos_p, float pres_pos_i, float pres_pos_d: PID parameters when generating a positive pressure
      // negative pressure PID params:
      float pres_neg_p, float pres_neg_i, float pres_neg_d: PID for negative pressures (using vacuum bottle)
      // positive flowrate PID params:
      float flow_pos_p, float flow_pos_i, float flow_pos_d: PID params for positive flowrate
      // negative flowrate PID params:
      float flow_neg_p, float flow_neg_i, float flow_neg_d: PID for negative flowrate (using vacuum bottle)

      HardwareSerial &S:                    stream class to read from Serial, Serial1, etc.
      float &measurement, &disc_pump_power: These values are overwritten with new updated values

  GLOBAL VARIABLES: None

  DEPENDENCIES:
      TTP.h:   for setting the disc pump power and power macros
  -----------------------------------------------------------------------------
*/
void pid_loop(HardwareSerial &S, uint8_t mode, float flow_reading, float vacuum_reading, float pressure_reading, int8_t sign, float setpoint, float &measurement, float &disc_pump_power) {
  // bang-bang implementation - placeholder while debugging pid
  // we only care about managing flowrate - prevent the flow sensor from saturating
  measurement = flow_reading * sign;
  if (measurement > UPPER_FLOW_THRESH) {
    disc_pump_power = TTP_MIN_PWR;
  }
  else if (measurement < LOWER_FLOW_THRESH) {
    disc_pump_power = PUMP_PWR_mW_GO;
  }
  else {
    disc_pump_power = prev_power;
  }
  prev_power = disc_pump_power;


  //  float error; // difference between setpoint and measurement
  //  float pid_p_coeff;
  //  float pid_i_coeff;
  //  float pid_d_coeff;
  //
  //  switch (mode) {
  //    // set parameters
  //    case VACUUM_LOOP:
  //      measurement = vacuum_reading;
  //      error = setpoint - measurement;
  //      // if we want negative pressure, invert the error (assuming the valves were set properly, more disc pump power -> more negative pressure)
  //      // we need different PID parameters in this case
  //      if (sign < 0) {
  //        error = -error;
  //        pid_p_coeff = pres_neg_p;
  //        pid_i_coeff = pres_neg_i;
  //        pid_d_coeff = pres_neg_d;
  //      }
  //      else {
  //        pid_p_coeff = pres_pos_p;
  //        pid_i_coeff = pres_pos_i;
  //        pid_d_coeff = pres_pos_d;
  //      }
  //      break;
  //    // handle flowrate loop case
  //    case FLOWRATE_LOOP:
  //      measurement = flow_reading;
  //      error = (setpoint - measurement) / 1000; // scale error to account for large flowrates in units uL/min
  //      // if we want negative flowrate, invert the error (assuming the valves were set properly, more disc pump power -> more negative flowrate)
  //      // we need different PID parameters in this case
  //      if (sign < 0) {
  //        error = -error;
  //        pid_p_coeff = flow_neg_p;
  //        pid_i_coeff = flow_neg_i;
  //        pid_d_coeff = flow_neg_d;
  //      }
  //      else {
  //        pid_p_coeff = flow_pos_p;
  //        pid_i_coeff = flow_pos_i;
  //        pid_d_coeff = flow_pos_d;
  //      }
  //      break;
  //    // handle default/IDLE_LOOP case
  //    default:
  //      pid_reset();
  //      pid_p_coeff = 0;
  //      pid_i_coeff = 0;
  //      pid_d_coeff = 0;
  //      error = 0;
  //      break;
  //  }
  //  // perform the PID loop
  //  // add integral error
  //  pid_i_error += error;
  //  // integral wind-up limit
  //  pid_i_error = constrain(pid_i_error, 0, (1 / pid_i_coeff));
  //  // weighted sum of error coefficients scaled by max power
  //  disc_pump_power = ((pid_i_error * pid_i_coeff + error * pid_p_coeff + pid_d_coeff * (error - pid_prev_error) / (millis() - dt))) * TTP_MAX_PWR;
  //  // current err is now prev err
  //  pid_prev_error = error;
  //  // store time of previous loop
  //  dt = millis();
  // limit power
  disc_pump_power = constrain(disc_pump_power, TTP_MIN_PWR, TTP_MAX_PWR);
  // set disc pump power
  if (mode != IDLE_LOOP) {
    TTP_set_target(S, disc_pump_power);
  }
  return;
}

void pid_reset() {
  // clear error terms
  pid_i_error = 0;
  pid_prev_error = 0;
  // clear elapsed time
  dt = 0;

  return;
}
