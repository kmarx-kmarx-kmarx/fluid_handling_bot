/*
    States.cpp
    This file contains functions for running the bang-bang flowrate control loop and main control loop

    Bang-Bang Functions and Shared Variables:
    Shared Variables:
    prev_power: float, previous power setpoint

    Functions:
    float bang_bang_flowrate: Set the output power using a bang-bang controller using the flowrate
      Arguments: HardwareSerial &S, uint8_t mode, float flow_reading, float vacuum_reading, float pressure_reading, int8_t sign, float setpoint, float &measurement, float &disc_pump_power

*/

#include "States.h"

// bang-bang param
float prev_power = 0;
float bb_lower_thresh = LOWER_FLOW_THRESH;
float bb_upper_thresh = UPPER_FLOW_THRESH;
float bb_min_pwr = TTP_MIN_PWR;
float bb_max_pwr = PUMP_PWR_mW_GO;

/*  -----------------------------------------------------------------------------
  DESCRIPTION: bang_bang_flowrate() runs bang-bang control on the flowrate

  OPERATION:   We first assume the valves are set properly to achieve the setpoint (e.g. no blockages, fluid flowing in the correct direction)
               The target flowrate can either be positive or negative; we multiply by sign so increasing the pump power always increases the flowrate.
               Next, if the measurement is above our upper threshold, we decarese the pump power to its minimum value. Otherwise, if the
               measurement falls below the minimum threshold, turn on the pump

  ARGUMENTS:
      HardwareSerial &S:      stream class to read from Serial, Serial1, etc.
      uint8_t mode:           either IDLE_LOOP, VACUUM_LOOP, or FLOWRATE_LOOP; sets whether the PID loop should be idling, controlling pressure, or controlling flowrate
      float flow_reading:     flowrate reading in units uL/min


      float &measurement:     Overwritten with one of the readings depending on mode
      float &disc_pump_power: Overwritten with the disc pump power in units mW

  RETURNS: None

  INPUTS: None

  OUTPUTS: The disc pump power is set

  SHARED VARIABLES:
      HardwareSerial &S:                    stream class to read from Serial, Serial1, etc.
      float &measurement, &disc_pump_power: These values are overwritten with new updated values

  GLOBAL VARIABLES: None

  DEPENDENCIES:
      TTP.h:   for setting the disc pump power and power macros
  -----------------------------------------------------------------------------
*/
void bang_bang_flowrate(HardwareSerial &S, uint8_t mode, float flow_reading, int8_t sign, float &measurement, float &disc_pump_power) {
  // bang-bang implementation - placeholder while debugging pid
  // we only care about managing flowrate - prevent the flow sensor from saturating
  measurement = flow_reading * sign;
  if (measurement > bb_upper_thresh) {
    disc_pump_power = bb_min_pwr;
  }
  else if (measurement < bb_lower_thresh) {
    disc_pump_power = bb_max_pwr;
  }
  else {
    disc_pump_power = prev_power;
  }
  prev_power = disc_pump_power;

  // set disc pump power
  if (mode == BANG_BANG_FLOWRATE) {
    TTP_set_target(S, disc_pump_power);
  }
  
  return;
}

void set_bang_bang_params(float lower_thresh, float upper_thresh, float min_pwr, float max_pwr) {
  // Set the shared variables
  bb_lower_thresh = constrain(lower_thresh, TTP_MIN_PWR, TTP_MAX_PWR);
  bb_upper_thresh = constrain(upper_thresh, TTP_MIN_PWR, TTP_MAX_PWR);
  bb_min_pwr = min_pwr;
  bb_max_pwr = max_pwr;
  return;
}
