/*
   notes - for the OPX350 - use both outputs to detect bubbles and fluid; just using one is insufficient
    teensy41_fluidhandling.ino:
        This project will incrementally implement features necessary for the Prakash Lab's new fluid handling robot.

     DESCRIPTION: This file sets up and runs the main control loop.
        setup(): Initialize Serial and communications with the motor controller IC
        loop():  Read commands over Serial and perform them

        set_read_sensors_flag(): set a global bool indicating the sensors should be read
        set_send_update_flag():  set a global bool indicating the status should be sent over serial
        set_check_manual_input_flag(): set global bool indicating manual controls should be read

        stop_current_enable_manual(): stop the PID loop and enable manual control
        set_valves_suction():         set valves so the disc pump pulls fluid from the open chamber into vacuum bottle 1
        set_valves_pressure():        set valves so the pump pushes fluid from the reservoir into the chamber
        set_valves_vacuum():          set valves so the pump sucks fluid from the containers into the reservoir
        reset_valves():               de-energize all the valves
        float vol_inc():              integrates flow sensor to return current volume

    Shared Variables:
      Command Management:
        uint8_t  command_execution_status  - indicates the progress of a command. See SerialCommUtils.h for more information
        uint8_t  INTERNAL_STATE            - indicates state of internal state machine. See States.h
        float    vol_load_uL               - volume target
        uint16_t t0                        - used to track global elapsed time
        uint16_t cmd_time                  - used to track command elapsed time
        float setpoint                     - generic setpoint

      Bang-Bang Control -
        int8_t  bang_bang_sign
        uint8_t bang_bang_mode = IDLE_LOOP;


      Time Flags:
        volatile bool flag_read_sensors - indicates the sensors should be read during the next loop
        volatile bool flag_send_update  - indicates debug data should be sent during the next loop

      Measurement:
        float   SLF3X_0_volume_uL      - total volume that passed through the flow sensor
        int32_t SLF3X_0_dt             - timestamp of last read

    Dependencies:
        SLF3X.h     : Functions for initializing and reading from the flow sensor
        SSCX.h      : Functions for initializing and reading from the pressure sensor
        OPX350.h    : Functions for initializing and reading from the bubble sensor
        TTP.h       : Functions for initializing, reading from, and writing to the disc pump
        NXP33996.h  : For interacting with the SPI GPIO expander
        TITAN.h     : Functions for initializing, reading from, and writing to the selector valve
        SerialCommUtils.h: Handles serial commands and output.
        Wire.h      : For I2C communication
        IOdefs.h    : Pin defintions
        States.h    : State definitions and PID controller

    Author: Kevin Marx
    Created on: 9/13/2022
*/

#include "SerialCommUtils.h"
#include "IOdefs.h"
#include "TTP.h"
#include "SLF3X.h"
#include "SSCX.h"
#include "OPX350.h"
#include "States.h"
#include "TITAN.h"

// System parameters

// SLF3X flow sensor parameters
#define W_SLF3X      Wire1
#define PERFORM_CRC  true
// SSCX pressure sensor parameters
#define W_SSCX           Wire1
// filter parameters (for pressure value LPF)
#define DECAY  0.05
float prev_p0 = 0;
float prev_p1 = 0;
float pr0 = 0;
float pr1 = 0;
float peak_pressure = 0;
bool past_peak = false;

// OPX350 bubble sensor parameters
// bubble sensor 0
#define OCB350_0_CALIB  29
#define OCB350_0_LOGIC  30
// bubble sensor 1
#define OCB350_1_CALIB  31
#define OCB350_1_LOGIC  32
// Disc pump parameters
#define UART_TTP             Serial8
#define TTP_SELECTED_MODE    TTP_MODE_MANUAL
#define TTP_SELECTED_STREAM  TTP_STREAM_DISABLE
#define TTP_SELECTED_SRC     TTP_SRC_SETVAL
#define TTP_PWR_LIM_mW       TTP_MAX_PWR

// IDEX selector valve
#define UART_Titan Serial5
uint8_t set_position = 0;

// command management
uint8_t  command_execution_status = COMPLETED_WITHOUT_ERRORS;
uint32_t t0 = 0;
uint8_t  internal_state = INTERNAL_STATE_IDLE;
uint32_t cmd_time = 0;
uint32_t operation_time = 0;
float    setpoint = 0;

// fluid management
float   SLF3X_0_volume_uL  = 0;
int32_t SLF3X_0_dt  = 0;
float   vol_load_uL = 0;      // volume target
bool vol_integrate_flag = false;
// bang-bang controller
int8_t bang_bang_sign = 0;
uint8_t bang_bang_mode = IDLE_LOOP;
#define VOL_uL_MAX 5000

// Timer parameters
// Set flag to read the sensors every 5 ms
#define READ_SENSORS_INTERVAL_US 5000
volatile bool flag_read_sensors = false;
IntervalTimer Timer_read_sensors_input;
// Set flag to send updates every 60 ms
#define SEND_UPDATE_INTERVAL_US 60000
volatile bool flag_send_update = false;
IntervalTimer Timer_send_update_input;

void setup() {
  // Initialize Serial to communicate with the computer
  Serial.begin(2000000);

  // initialize pins
  pinMode(pin_manual_control_enable, INPUT_PULLUP); // manual ctrl enable button
  pinMode(pin_pressure_vacuum, INPUT);              // pressure/vacuum selector switch
  pinMode(pin_analog_in, INPUT);                    // potentiometer setpoint

  pinMode(pin_valve_0, OUTPUT); // initialize pins
  pinMode(pin_valve_1, OUTPUT);
  pinMode(pin_valve_2, OUTPUT);
  pinMode(pin_valve_3, OUTPUT);
  pinMode(pin_valve_4, OUTPUT);
  pinMode(pin_valve_5, OUTPUT);

  // Initialize I2C selector
  pinMode(PIN_SENSOR_SELECT, OUTPUT);

  analogWriteResolution(10);

  while (!Serial) {
    delay(10);
  }

  Timer_read_sensors_input.begin(set_read_sensors_flag, READ_SENSORS_INTERVAL_US);
  Timer_send_update_input.begin(set_send_update_flag, SEND_UPDATE_INTERVAL_US);
  // Initialize pressure and flowrate sensors
  SSCX_init(W_SSCX);
  digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
  delay(10);
  SLF3X_init(W_SLF3X, MEDIUM_WATER);

}


void loop() {
  // Init variables
  byte payloads[TO_MCU_CMD_LENGTH - 3]; // incoming serial payload
  uint8_t serial_command;               // which command to run
  bool init_result;
  // manage readings
  int16_t  SLF3X_0_readings[3];  // flowrate, temperature, and flags from the flow sensor
  uint8_t  SLF3X_0_err;          // indicates whether the read succeeded
  bool     OCB350_0_reading;     // readings from the bubble sensors
  bool     OCB350_1_reading;
  int16_t  SSCX_0_readings[2];   // raw pressure and temperature
  uint8_t  SSCX_0_err;           // indicates whether the reading succeeded
  int16_t  SSCX_1_readings[2];
  uint8_t  SSCX_1_err;
  int bubbles_present = 0;
  vol_integrate_flag = true;

  // first, make sure we aren't already executing a command
  if (command_execution_status != IN_PROGRESS) {
    // try reading a command. read_serial_command overwrites payloads[] and serial_command
    if (read_serial_command(payloads, serial_command)) {
      // We have a command in progress!
      command_execution_status = IN_PROGRESS; //unfactor this
      switch (serial_command) {
        // Set the valve pins given by payload 0
        case SET_SOLENOID_VALVES:
          digitalWrite(pin_valve_0, (((payloads[0] >> 0) & 0x01) == 1));
          digitalWrite(pin_valve_1, (((payloads[0] >> 1) & 0x01) == 1));
          digitalWrite(pin_valve_2, (((payloads[0] >> 2) & 0x01) == 1));
          digitalWrite(pin_valve_3, (((payloads[0] >> 3) & 0x01) == 1));
          digitalWrite(pin_valve_4, (((payloads[0] >> 4) & 0x01) == 1));
          digitalWrite(pin_valve_5, (((payloads[0] >> 5) & 0x01) == 1));

          command_execution_status = COMPLETED_WITHOUT_ERRORS;
          break;
        // clear the command UID. Handled in the read_serial_command
        case CLEAR:
          command_execution_status = COMPLETED_WITHOUT_ERRORS;
          break;
        // Init the disc pump
        case INITIALIZE_DISC_PUMP:
          init_result = TTP_init(UART_TTP, TTP_PWR_LIM_mW, TTP_SELECTED_SRC, TTP_SELECTED_MODE, TTP_SELECTED_STREAM);
          init_result &= TTP_enable(UART_TTP, true);
          init_result &= TTP_set_target(UART_TTP, 0);
          if (init_result) {
            command_execution_status = COMPLETED_WITHOUT_ERRORS;
          }
          else
            command_execution_status = CMD_EXECUTION_ERROR;
          break;
        // Init the pressure sensors
        case INITIALIZE_PRESSURE_SENSORS:
          // only need to initialize once for both sensors
          SSCX_init(W_SSCX);
          command_execution_status = COMPLETED_WITHOUT_ERRORS;
          break;
        // Init the flow sensor
        case INITIALIZE_FLOW_SENSOR:
          digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
          init_result = SLF3X_init(W_SLF3X, MEDIUM_WATER);
          if (init_result)
            command_execution_status = COMPLETED_WITHOUT_ERRORS;
          else
            command_execution_status = CMD_EXECUTION_ERROR;
          break;
        // Init the bubble sensors - blocking
        case INITIALIZE_BUBBLE_SENSORS:
          // Try to read from the disc pump - disc pump must be initialized first to ensure the lines are clear for the bubble sensor
          cmd_time = (uint32_t(payloads[2]) << 8) + uint32_t(payloads[3]);

          set_valves_vacuum();
          TTP_set_target(UART_TTP, PUMP_PWR_mW_GO);
          t0 = millis();
          internal_state = INTERNAL_STATE_BUBBLE_START;

          break;
        // Set up bang-bang control - 0 and 1 are the lower and upper flowrate bounds (0-255), 2-3, 6-7 is the lower power and upper power setpoints
        case INITIALIZE_BANG_BANG_PARAMS:
          set_bang_bang_params(payloads[0], payloads[1],
                               float((uint32_t(payloads[2]) << 8) + uint32_t(payloads[3]))*float(TTP_MAX_PWR) / float(UINT16_MAX),
                               float((uint32_t(payloads[6]) << 8) + uint32_t(payloads[7]))*float(TTP_MAX_PWR) / float(UINT16_MAX));
          command_execution_status = COMPLETED_WITHOUT_ERRORS;
          break;
        // Check if fittings are tight by pressurizing the system
        case PRETEST_PRESSURE_START:
          break;
        // Check if fittings are tight by pulling a vacuum in VB0
        case PRETEST_VACUUM_START:
          break;
        // vent VB0. Run before loading a medium
        case VENT_VB0:
          cmd_time = (uint32_t(payloads[2]) << 8) + uint32_t(payloads[3]);
          // setpoint represents the desired pressure setpoint, should be negative here to represent vacuum. value ranges from 0 to -1
          setpoint = -1 * float((uint32_t(payloads[6]) << 8) + uint32_t(payloads[7])) / float(UINT16_MAX);
          // turn off the pump
          TTP_set_target(UART_TTP, 0);
          // set valves to prevent fluid flow while venting
          set_valves_vacuum();
          digitalWrite(pin_valve_1, LOW);
          // Connect VB0 to the pressure sensor
          digitalWrite(pin_valve_4, HIGH);
          // start venting
          digitalWrite(pin_valve_5, HIGH);
          t0 = millis();
          // Monitor pressure over time - prev_p1
          internal_state = INTERNAL_STATE_VENT_VB0;
          break;
        // Load a specified volume into the reservoir - run once LOAD_MEDIUM_VOLUME_START completed
        case UNLOAD_MEDIUM_VOLUME_START:
          // Set valves
          set_valves_to_vb1();
          // Reset integration
          SLF3X_0_volume_uL = 0;
          // Load the values from the payload
          // vol_load_uL ranges from 0 to VOL_uL_MAX, set by constant.
          vol_load_uL = (float((uint32_t(payloads[6]) << 8) + uint32_t(payloads[7])) / float(UINT16_MAX)) * VOL_uL_MAX;
          // timeout time
          cmd_time = (uint32_t(payloads[2]) << 8) + uint32_t(payloads[3]);
          t0 = millis();
          // Start bang-bang ctrl
          bang_bang_sign = 1; // flow is in the + dirn wrt the flow sensor
          bang_bang_mode = BANG_BANG_FLOWRATE;
          internal_state = INTERNAL_STATE_LOAD_MEDIUM;
          break;
        // Load a specified volume into the reservoir - run once LOAD_MEDIUM_START completed
        case LOAD_MEDIUM_VOLUME_START:
          // Set valves
          set_valves_vacuum();
          // Load the values from the payload
          // vol_load_uL ranges from 0 to VOL_uL_MAX, set by constant.
          vol_load_uL = (float((uint32_t(payloads[6]) << 8) + uint32_t(payloads[7])) / float(UINT16_MAX)) * VOL_uL_MAX;
          // timeout time
          cmd_time = (uint32_t(payloads[2]) << 8) + uint32_t(payloads[3]);
          t0 = millis();
          // Start bang-bang ctrl
          bang_bang_sign = -1; // flow is in the - dirn wrt the flow sensor
          bang_bang_mode = BANG_BANG_FLOWRATE;
          internal_state = INTERNAL_STATE_LOAD_MEDIUM;
          break;
        // Bring medium right up to the flow sensor
        case LOAD_MEDIUM_START:
          // Set the valves
          set_valves_vacuum();
          // Start pumping open-loop until we hit the first bubble sensor near the flow sensor - can't use bang-bang if there is no valid flow reading
          bang_bang_mode = IDLE_LOOP;
          TTP_set_target(UART_TTP, PUMP_PWR_mW_GO);
          cmd_time = (uint32_t(payloads[2]) << 8) + uint32_t(payloads[3]);
          t0 = millis();
          internal_state = INTERNAL_STATE_LOAD_MEDIUM_START;
          break;
        // Eject all into an open chamber (VB1) (must be open so pressure drop can be used to detect when all the fluid is ejected)
        case UNLOAD_MEDIUM_START:
          // reset peak flag
          past_peak = false;
          peak_pressure=0;
          // Set valves
          set_valves_to_vb1();
          // Start pumping open loop
          bang_bang_mode = IDLE_LOOP;
          TTP_set_target(UART_TTP, (uint32_t(payloads[6]) << 8) + uint32_t(payloads[7]));;
          // Go to state where we track filtered pressure 0
          internal_state = INTERNAL_STATE_UNLOAD_START;
          // timeout time
          cmd_time = (uint32_t(payloads[2]) << 8) + uint32_t(payloads[3]);
          t0 = millis();
          operation_time = millis();
          // use setpoint to track the max pressure
          setpoint = 0;
          break;
        // clear the fluid in the reservoir without changing the fluid in the closed chamber
        case CLEAR_MEDIUM_START:
          // reset peak flag
          past_peak = false;
          peak_pressure=0;
          // Set valves
          set_valves_vacuum();
          // Start pumping open loop
          bang_bang_mode = IDLE_LOOP;
          TTP_set_target(UART_TTP, (uint32_t(payloads[6]) << 8) + uint32_t(payloads[7]));
          // Go to state where we track bubble sensors
          internal_state = INTERNAL_STATE_CLEAR_START;
          // timeout time in seconds
          cmd_time = (uint32_t(payloads[2]) << 8) + uint32_t(payloads[3]);
          // payloads[1] is post-bubble wait time in seconds
          t0 = millis();
          operation_time = t0;
          break;
        case INITIALIZE_SELECTOR_VALVE:
          if (TITAN_init(UART_Titan))
            command_execution_status = COMPLETED_WITHOUT_ERRORS;
          else
            command_execution_status = CMD_EXECUTION_ERROR;
          break;
        case SET_SELECTOR_VALVE:
          if (set_selector_valve_position_blocking(UART_Titan, payloads[0])) {
            set_position = payloads[0];
            command_execution_status = COMPLETED_WITHOUT_ERRORS;
          }
          else
            command_execution_status = CMD_EXECUTION_ERROR;
          break;

        // If we have a command that isn't any of the ones above, return an error
        default:
          command_execution_status = CMD_INVALID;
          break;
      }
    }
  }

  switch (internal_state) {
    case INTERNAL_STATE_UNLOAD_START:
    case INTERNAL_STATE_CLEAR_START:
      // if we didn't time out AND we haven't had enough time pass yet, check bubble state and break
      if (((millis() - t0) / 1000 < cmd_time) && ((millis() - operation_time) / 1000 < payloads[1])) {
        if(abs(prev_p1) > peak_pressure){
          peak_pressure= abs(prev_p1);
          TTP_set_target(UART_TTP, (uint32_t(payloads[6]) << 8) + uint32_t(payloads[7]));
          past_peak = false;
        }
        
        // check if pressure has fallen below threshold
        if((abs(prev_p1) <= (peak_pressure * payloads[0] / 255)) && (past_peak==false)){
          past_peak = true;
          TTP_set_target(UART_TTP, 1+(uint32_t(payloads[6]) << 8) + uint32_t(payloads[7]));
        }
        if(past_peak==false)
          operation_time = millis();
        break;
      }
      TTP_set_target(UART_TTP, 0);
      internal_state = INTERNAL_STATE_IDLE;
      // prevent fluid flow through valves
      digitalWrite(pin_valve_0, HIGH);
      digitalWrite(pin_valve_1, LOW);
      digitalWrite(pin_valve_2, LOW);
      digitalWrite(pin_valve_3, LOW);
      digitalWrite(pin_valve_4, HIGH);
      SLF3X_0_volume_uL = 0;
      // check if we timed out
      if ((millis() - t0) / 1000 >= cmd_time) {
        command_execution_status = CMD_EXECUTION_ERROR;
      }
      // if operation_time elapsed without seeing bubbles, we are done
      else if ((millis() - operation_time) / 1000 >= payloads[1]) {
        command_execution_status = COMPLETED_WITHOUT_ERRORS;
      }
      break;
    case INTERNAL_STATE_VENT_VB0:
      // if we didn't timeout AND we didn't hit the vacuum setpoint, break
      if ((millis() - t0 < cmd_time) && (prev_p1 < setpoint))
        break;
      // otherwise, stop the operation
      internal_state = INTERNAL_STATE_IDLE;
      // prevent fluid flow through valves
      digitalWrite(pin_valve_0, HIGH);
      digitalWrite(pin_valve_1, LOW);
      digitalWrite(pin_valve_2, LOW);
      digitalWrite(pin_valve_3, LOW);
      digitalWrite(pin_valve_4, HIGH);
      digitalWrite(pin_valve_5, LOW);
      // check if we timed out - report error
      if (millis() - t0 >= cmd_time) {
        command_execution_status = CMD_EXECUTION_ERROR;
      }
      // check if we hit pressure target - return no error
      else {
        command_execution_status = COMPLETED_WITHOUT_ERRORS;
      }
      break;
    // Calibrate the bubble sensors
    case INTERNAL_STATE_BUBBLE_START:
      // wait for the lines to clear
      if (millis() - t0 < cmd_time) {}
      //        break;
      // once sufficient time has elaped, turn off the disc pump and calibrate the sensors
      else {
        TTP_set_target(UART_TTP, 0);
        //        reset_valves();
        OPX350_init(OCB350_0_LOGIC, OCB350_0_CALIB);
        init_result =  OPX350_calib(OCB350_0_LOGIC, OCB350_0_CALIB);
        delay(10);
        OPX350_init(OCB350_1_LOGIC, OCB350_1_CALIB);
        init_result = init_result && OPX350_calib(OCB350_1_LOGIC, OCB350_1_CALIB);

        // If calibration failed, give up now
        if (!init_result) {
          command_execution_status = CMD_EXECUTION_ERROR;
          internal_state = INTERNAL_STATE_IDLE;
        }
        // Otherwise, set a new timer to let the calibration finish
        else {
          t0 = millis();
          internal_state = INTERNAL_STATE_BUBBLE_FINISH;
        }
      }
      break;
    case INTERNAL_STATE_BUBBLE_FINISH:
      // wait for the calibration to finish
      if (millis() - t0 < cmd_time)
        break;
      else {
        reset_valves();
        command_execution_status = COMPLETED_WITHOUT_ERRORS;
        internal_state = INTERNAL_STATE_IDLE;
      }
      break;
    case INTERNAL_STATE_LOAD_MEDIUM_START:
      // if we didn't hit the fluid or timeout, break
      if ((((SLF3X_0_readings[SLF3X_FLAG_IDX] & SLF3X_NO_FLUID) != 0) || OPX350_read(OCB350_0_LOGIC) ) && ((millis() - t0) < cmd_time)) {
        break;
      }
      // Otherwise, stop fluid flow
      TTP_set_target(UART_TTP, 0);
      // prevent fluid flow through valves
      digitalWrite(pin_valve_0, HIGH);
      digitalWrite(pin_valve_1, LOW);
      digitalWrite(pin_valve_2, HIGH);
      digitalWrite(pin_valve_3, LOW);
      digitalWrite(pin_valve_4, HIGH);
      internal_state = INTERNAL_STATE_IDLE;

      if ((millis() - t0) >= cmd_time)
        command_execution_status = CMD_EXECUTION_ERROR;
      else
        command_execution_status = COMPLETED_WITHOUT_ERRORS;

      break;
    case INTERNAL_STATE_LOAD_MEDIUM:
      // Check bubble sensor - stop if fluid hits the bubble sensor or if the correct volume was drawn or if there's timeout
      // note - this feature has been removed also stop if flowrate saturates
      //OPX350_read(OCB350_1_LOGIC) &&  - replace with better overfill detection
      // && !OPX350_read(OCB350_0_LOGIC) - replace with better bubble detection
      // && (abs(SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX])) < SLF3X_FS_VAL_uL_MIN) - flow sensor saturating - ok for two step fill
      if ((abs(SLF3X_0_volume_uL) < vol_load_uL) && ((millis() - t0) / 1000 < cmd_time)  )
        break;
      // !OPX350_read(OCB350_1_LOGIC) || || OPX350_read(OCB350_0_LOGIC)
      //  || (abs(SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX])) >= SLF3X_FS_VAL_uL_MIN)
      else if (((millis() - t0) / 1000 >= cmd_time))
        command_execution_status = CMD_EXECUTION_ERROR;
      else
        command_execution_status = COMPLETED_WITHOUT_ERRORS;

      TTP_set_target(UART_TTP, 0);
      bang_bang_mode = IDLE_LOOP;
      bang_bang_sign = 0;
      digitalWrite(pin_valve_0, HIGH);
      digitalWrite(pin_valve_1, LOW);
      digitalWrite(pin_valve_2, HIGH);
      digitalWrite(pin_valve_3, LOW);
      digitalWrite(pin_valve_4, HIGH);

      internal_state = INTERNAL_STATE_IDLE;

      break;
    default:
      break;
  }

  // Read the sensors
  if (flag_read_sensors) {
    // Indicate we have read the sensors
    flag_read_sensors = false;

    // Initialize shared variables
    float    disc_pump_power = 0;
    float    measurement = 0;    // Run bang-bang control with the new values:

    digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
    SLF3X_0_err = SLF3X_read(PERFORM_CRC, W_SLF3X, SLF3X_0_readings);

    OCB350_0_reading = OPX350_read(OCB350_0_LOGIC);
    OCB350_1_reading = OPX350_read(OCB350_1_LOGIC);
    digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
    SSCX_0_err = SSCX_read(W_SSCX, SSCX_0_readings);
    digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_1);
    SSCX_1_err = SSCX_read(W_SSCX, SSCX_1_readings);

    pr0 = SSCX_to_psi(SSCX_0_readings[SSCX_PRESS_IDX]);
    pr1 = SSCX_to_psi(SSCX_1_readings[SSCX_PRESS_IDX]);

    // Once all the sensors are read, perform the control loop to calculate the new pump power
    bang_bang_flowrate(UART_TTP, bang_bang_mode, SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]), bang_bang_sign, measurement, disc_pump_power);
    if (bang_bang_mode != BANG_BANG_FLOWRATE) {
      disc_pump_power = TTP_get_set_power();
    }

    // also calculate new filtered pressure
    prev_p0 = pressure_lpf(prev_p0, pr0);
    prev_p1 = pressure_lpf(prev_p1, pr1);

    // Send an update only if the reading has been done
    if (flag_send_update) {
      flag_send_update = false;
      // no fluid -> no flow
      if (SLF3X_0_readings[SLF3X_FLAG_IDX] & SLF3X_NO_FLUID)
        SLF3X_0_readings[SLF3X_FLOW_IDX] = 0;
      send_serial_data(TTP_MAX_PWR, VOL_uL_MAX, command_execution_status, internal_state, OCB350_0_reading, OCB350_1_reading, get_valve_state(), 0, SSCX_0_readings[SSCX_PRESS_IDX], SSCX_1_readings[SSCX_PRESS_IDX], -1 * SLF3X_0_readings[SLF3X_FLOW_IDX], (millis() - t0) / 1000.0,  -1 * SLF3X_0_volume_uL, set_position, disc_pump_power, psi_to_SSCX(prev_p0), psi_to_SSCX(prev_p1));
    }

    // integrate the flowrate:
    if (vol_integrate_flag) {
      SLF3X_0_volume_uL = vol_inc(SLF3X_0_volume_uL, SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]), (millis() - SLF3X_0_dt), (SLF3X_0_readings[SLF3X_FLAG_IDX] & SLF3X_NO_FLUID));
      SLF3X_0_dt  = millis();
    }

  }
}
void reset_valves() {
  // all low - save energy
  digitalWrite(pin_valve_0, LOW);
  digitalWrite(pin_valve_1, LOW);
  digitalWrite(pin_valve_2, LOW);
  digitalWrite(pin_valve_3, LOW);
  digitalWrite(pin_valve_4, LOW);
  return;
}
// get fluids from the reservoir to the open/closed chamber
void set_valves_pressure() {
  // air gets pushed into the second vacuum bottle from valve 4

  // valve 0 connects the disc pup pressure to the system
  digitalWrite(pin_valve_0, LOW);
  // connect air pressure to the reservoir
  digitalWrite(pin_valve_1, LOW);
  // connect reservoir to open/closed chamber
  digitalWrite(pin_valve_2, LOW);
  // disconnect vacuum bottle (and open/closed chamber) from anything
  digitalWrite(pin_valve_3, LOW);
  // connect to ambient air
  digitalWrite(pin_valve_4, LOW);
  return;
}
// get fluids from the samples into the reservoir
void set_valves_vacuum() {
  // air gets sucked out of the first vacuum bottle and ejected from valve 0

  // valve 0 connect to ambient air
  digitalWrite(pin_valve_0, HIGH);
  // connect reservoir to vacuum bottle
  digitalWrite(pin_valve_1, HIGH);
  // connect fluid samples to reservoir
  digitalWrite(pin_valve_2, HIGH);
  // connect vacuum bottle to next valve
  digitalWrite(pin_valve_3, LOW);
  // connect vacuum bottle to disc pump vacuum
  digitalWrite(pin_valve_4, HIGH);
  return;
}
void set_valves_to_vb1() {
  // air gets sucked out of the first vacuum bottle and pushed into the reservior

  // valve 0 connect to reservior
  digitalWrite(pin_valve_0, LOW);
  // connect reservoir to pump
  digitalWrite(pin_valve_1, LOW);
  // connect chamber to reservoir
  digitalWrite(pin_valve_2, LOW);
  // connect vacuum bottle to next valve
  digitalWrite(pin_valve_3, HIGH);
  // connect vacuum bottle to disc pump vacuum
  digitalWrite(pin_valve_4, HIGH);
  // Don't vent VB0
  digitalWrite(pin_valve_5, LOW);
  return;
}
void set_read_sensors_flag() {
  flag_read_sensors = true;
  return;
}
void set_send_update_flag() {
  flag_send_update = true;
  return;
}
float pressure_lpf(float prev, float current) {
    prev += DECAY * (current - prev);
    return prev;
}
float vol_inc(float prev_vol, float rate, uint16_t dt, bool air_flag) {
  if (air_flag) {
    return prev_vol;
  }
  else {
    return prev_vol + dt * rate / (60.0 * 1000.0);
  }
}
uint8_t get_valve_state() {
  uint8_t state = 0;
  state |= (digitalRead(pin_valve_0) & 0x01) << 0;
  state |= (digitalRead(pin_valve_1) & 0x01) << 1;
  state |= (digitalRead(pin_valve_2) & 0x01) << 2;
  state |= (digitalRead(pin_valve_3) & 0x01) << 3;
  state |= (digitalRead(pin_valve_4) & 0x01) << 4;
  state |= (digitalRead(pin_valve_5) & 0x01) << 5;

  return state;
}
