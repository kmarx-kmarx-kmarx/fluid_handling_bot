/*
    States.cpp
    This file contains functions for running the PID control loop and main control loop

    PID Functions and Shared Variables:
    Shared Variables:
    float pres_pid_p_coeff:   Pressure control p coefficient
    float flow_pid_p_coeff:   Flowrate control p coefficient
    float pres_pid_i_error:   Pressure integral error
    float flow_pid_i_error:   Flowrate integral error
    float pres_pid_i_coeff:   Pressure control i coefficient
    float flow_pid_i_coeff:   Flowrate control i coefficient

    Functions:
    float pid_loop: Set the output power using a PI controller, setting either the pressure or the flowrate.
      Arguments: HardwareSerial &S, uint8_t mode, float flow_reading, float pressure_reading, float setpoint

*/

#include "States.h"

// Store PID parameters - can be modified later
float pres_pid_p_coeff = 1;
float flow_pid_p_coeff = 0.5;
float pres_pid_i_coeff = 0;
float flow_pid_i_coeff = 0.0001;
// Store error terms
float pres_pid_i_error = 0;
float flow_pid_i_error = 0;

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: pid_loop() runs position-integral control over either pressure or flowrate.

  OPERATION:   We first assume the valves are set properly to achieve the setpoint (e.g. no blockages, fluid flowing in the correct direction)
               Operation then depends on the mode.
               IDLE_LOOP - set pump power to 0 and clear the integral error terms
               PRESSURE_LOOP - calculate difference between measurement and target and negate it if our target is negative. Assuming our valves are set correctly, the only way to achieve negative pressures is to run the pump harder and the negation lets this happen. We then run PI controls, implementing wind-up limit and constraining the output.
               FLOWRATE_LOOP - same as PRESSURE_LOOP but with flow measurement instead of pressure measurement.
               Thenm the disc pump power is bounded and sent to the disc pump

  ARGUMENTS:
      HardwareSerial &S:      stream class to read from Serial, Serial1, etc.
      uint8_t mode:           either IDLE_LOOP, PRESSURE_LOOP, or FLOWRATE_LOOP; sets whether the PID loop should be idling, controlling pressure, or controlling flowrate
      float flow_reading:     flowrate reading in units uL/min
      float pressure_reading: pressure reading in units PSI
      float setpoint:         target value in either units uL/min or PSI depending on mode
      float &measurement:     Overwritten with one of the readings depending on mode
      float &disc_pump_power: Overwritten with the disc pump power in units mW

  RETURNS: None

  INPUTS: None

  OUTPUTS: The disc pump power is set

  LOCAL VARIABLES:
      float pres_pid_p_coeff, flow_pid_p_coeff, pres_pid_i_coeff, flow_pid_i_coeff: PID parameters
      float pres_pid_i_error flow_pid_i_error:                                      integral error terms

  SHARED VARIABLES:
      HardwareSerial &S:                    stream class to read from Serial, Serial1, etc.
      float &measurement, &disc_pump_power: These values are overwritten with new updated values

  GLOBAL VARIABLES: None

  DEPENDENCIES:
      TTP.h:   for setting the disc pump power and power macros
  -----------------------------------------------------------------------------
*/
void pid_loop(HardwareSerial &S, uint8_t mode, float flow_reading, float pressure_reading, float setpoint, float &measurement, float &disc_pump_power) {
  float error; // difference between setpoint and measurement

  switch (mode) {
    // handle pressure loop case
    case PRESSURE_LOOP:
      measurement = pressure_reading;
      error = setpoint - measurement;
      // if we want negative pressure, invert the error (assuming the valves were set properly, more disc pump power -> more negative pressure)
      if (setpoint < 0) {
        error = -error;
      }
      // add integral error
      pres_pid_i_error += error;
      // integral wind-up limit
      pres_pid_i_error = constrain(pres_pid_i_error, 0, (1 / pres_pid_i_coeff));
      // weighted sum of error coefficients scaled by max power
      disc_pump_power = ((pres_pid_i_error * pres_pid_i_coeff + error * pres_pid_p_coeff) * TTP_MAX_PWR);
      break;
    // handle flowrate loop case
    case FLOWRATE_LOOP:
      measurement = flow_reading;
      error = (setpoint - measurement) / 1000; // scale error to account for large flowrates in units uL/min
      // if we want negative flowrate, invert the error (assuming the valves were set properly, more disc pump power -> more negative flowrate)
      if (setpoint < 0) {
        error = -error;
      }
      // add integral error
      flow_pid_i_error += error;
      // integral wind up limit
      flow_pid_i_error = constrain(flow_pid_i_error, 0, 1 / flow_pid_i_coeff);
      // weighted sum of error coefficients scaled by max power
      disc_pump_power = ((flow_pid_i_error * flow_pid_i_coeff + error * flow_pid_p_coeff) * TTP_MAX_PWR);
      break;
    // handle default/IDLE_LOOP case
    default:
      // reset integral errors
      pres_pid_i_error = 0;
      flow_pid_i_error = 0;
      // no power to pump
      disc_pump_power = 0;
      // no relevant measurements
      measurement = 0;
      break;
  }
  // limit power
  disc_pump_power = constrain(disc_pump_power, TTP_MIN_PWR, TTP_MAX_PWR);
  // set disc pump power
  if(mode != IDLE_LOOP){
    TTP_set_target(S, disc_pump_power);
  }
  return;
}
