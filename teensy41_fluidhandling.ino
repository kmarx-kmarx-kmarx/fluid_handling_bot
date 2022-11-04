/*
    teensy41_fluidhandling.ino:
        This project will incrementally implement features necessary for the Prakash Lab's new fluid handling robot.

     DESCRIPTION: This file sets up and runs the main control loop.
        setup(): Initialize Serial and communications with the motor controller IC
        loop():  Read commands over Serial and perform them

        set_read_sensors_flag(): set a global bool indicating the sensors should be read
        set_send_update_flag():  set a global bool indicating the status should be sent over serial
        set_check_manual_input_flag(): set global bool indicating manual controls should be read

        stop_current_enable_manual(): stop the PID loop and enable manual control
        release_vacuum_vb0():         open the vacuum bottle 0 valve until the pressure is VB0_PRESSURE_THRESH PSI or more (closer to atmospheric pressure). This function is blocking.
        set_valves_suction():         set valves so the disc pump pulls fluid from the open chamber into vacuum bottle 1
        set_valves_pressure():        set valves so the pump pushes fluid from the reservoir into the chamber
        set_valves_vacuum():          set valves so the pump sucks fluid from the containers into the reservoir
        reset_valves():               de-energize all the valves
        float check_vacuum():         ensure vacuum bottle 0 is sealed properly by drawing out air until the pressure is CHECK_VACUUM_THRESH or less. Return the final pressure in PSI. Blocking, runs until timeout after CHECK_TIMEOUT_MS or the target pressure is reached.
        float check_pressure():       ensure fittings are connected properly by pumping air into the tubes until the pressure exceeds CHECK_PRESSURE_THRESH. Return the final pressure in PSI. Blocking, runs until tiemout after CHECK_TIMEOUT_MS or the target pressure is reached.
        float vol_inc():              integrates flow sensor to return current volume

    Shared Variables:
      Command Management:
        uint8_t  command_execution_status  - indicates the progress of a command. See SerialCommUtils.h for more information
        uint8_t  internal_program          - indicates state of internal state machine. See States.h
        float    vol_load_uL               - volume target
        uint16_t t0                        - used to track global elapsed time
        bool    manual_control_enabled_by_software - set true when manual control is available
        uint16_t cmd_time                  - used to track command elapsed time

      PID: PID currently is not implemented; the system uses a bang-bang controller to keep flowrates in check
        uint8_t pid_mode                  - Indicates whether the control variable is pressure or flowrate. See States.h
        int8_t  pid_sign                  - Indicates whether increasing motor power increases the control variable or decreases it (depending on valve settings)
        float pid_setpoint                - Target flowrate or pressure in uL/min or PSI

      Time Flags:
        volatile bool flag_read_sensors - indicates the sensors should be read during the next loop
        volatile bool flag_send_update  - indicates debug data should be sent during the next loop
        volatile bool flag_check_manual_inputs - indicates manual inputs should be check during the next loop

      Initialzation flags: Indicate a component was initialized properly
        bool    SLF3X_present          - set true when the flow sensor is initialized properly
        bool    OCB350_0_present       - set true when bubble sensor 0 is calibrated properly
        bool    OCB350_1_present
        bool    SSCX_0_present         - set true when pressure sensor 0 is iniialized properly
        bool    SSCX_1_present
        bool    TTP_present            - set true when disc pump initialized properly
        bool    TITAN_present          - set true when the rotary valve is initialized properly

      Measurement:
        float   SLF3X_0_volume_mL      - total volume that passed through the flow sensor
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

#include <Wire.h>
#include "SLF3X.h"
#include "SSCX.h"
#include "OPX350.h"
#include "TTP.h"
#include "NXP33996.h"
#include "TITAN.h"
#include "SerialCommUtils.h"
#include "IOdefs.h"
#include "States.h"

// SYSTEM PARAMETERS - change these depending on your system configuration
// operation parameters
#define DEBUG_WITH_SERIAL true // set true to send debug messages over ASCII instead of bits
#define DEMO_MODE         true // if set to true, constantly load and unload DEMO_VOL uL of fluid in a loop
#define DEMO_VOL           100
// system parameters
#define VOLUME_UL_MAX            400   // maximum volume that fits in the reservior in units uL
#define FLUID_LOAD_BUFFER_uL       5   // load an additional small volume of fluid to account for losses when moving fluids
#define VB0_PRESSURE_THRESH    -0.05   // pressure in PSI setting when vacuum bottle 0 is close enough to atmospheric pressure
#define LINE_CLEAR_TIME_MS      5000 // amount of time in ms it takes to 
// Calibration param
#define CHECK_PRESSURE_THRESH 5.50 // When checking pressure, ensure we meet or exceed this much PSI
#define CHECK_VACUUM_THRESH  -4.25 // When checking vacuum, ensure we meet or exceed this much PSI
#define CHECK_TIMEOUT_MS     70000 // Timeout - if we take longer than this to hit the thresholds, return an error
// manual control parameters
#define MANUAL_PWR_MAX  TTP_MAX_PWR        // Maximum pump power
#define MANUAL_MAX_FLOW_VAC   300.0        // In vacuum mode, the flowrate can be set between 0 and 300 uL/min
#define MANUAL_MIN_FLOW_PRES -200.0        // In pressure mode, the flowrate can be set between -200 and 0 uL/min
#define MANUAL_DEFAULT_MODE FLOWRATE_LOOP  // Manual mode only controlls flowrate

// Timer parameters
// Set flag to read the sensors every 5 ms
#define READ_SENSORS_INTERVAL_US 5000
volatile bool flag_read_sensors = false;
IntervalTimer Timer_read_sensors_input;
// Set flag to send updates every 20 ms
#define SEND_UPDATE_INTERVAL_US 20000
volatile bool flag_send_update = false;
IntervalTimer Timer_send_update_input;
// Set a flag to read manual inputs every 10 ms
#define CHECK_MANUAL_INTERVAL_US 10000
volatile bool flag_check_manual_inputs = false;
IntervalTimer Timer_check_manual_input;
volatile bool manual_control_enabled_by_software = false;

// todo - replace these with variables that can be set over serial
// pressure to initially get fluid at flow sensor
#define PSI_LOAD_FLUID -0.4
// flowrate for loading fluid
#define FLOW_LOAD_FLUID -100.0
// volume target
// flowrate for unloading fluid
#define FLOW_UNLOAD_FLUID 100.0

// command management
uint8_t  command_execution_status = 0;
uint8_t  internal_program = INTERNAL_PROGRAM_INITIAL;
float    vol_load_uL = 0;      // volume target
uint16_t cmd_time = 0;         // timing variable

uint16_t t0;

// PID control parameters and variables
// Mode and setpoint persist over loops
uint8_t pid_mode = IDLE_LOOP;
int8_t  pid_sign = 0;
float pid_setpoint = 0;


// SLF3X flow sensor parameters
#define W_SLF3X      Wire1
#define PERFORM_CRC  true
// SLF3X flow sensor variables
// flow sensor 0
bool    SLF3X_0_present = false;
float   SLF3X_0_volume_mL  = 0;
int32_t SLF3X_0_dt  = 0;

// OPX350 bubble sensor parameters
// bubble sensor 0
#define OCB350_0_CALIB  29
#define OCB350_0_LOGIC  30
bool    OCB350_0_present = false;
// bubble sensor 1
#define OCB350_1_CALIB  31
#define OCB350_1_LOGIC  32
bool    OCB350_1_present = false;

// SSCX pressure sensor parameters
#define W_SSCX           Wire1
// SSCX pressure sensor variables
// pressure sensor 0
bool    SSCX_0_present = false;
// pressure sensor 1
bool    SSCX_1_present = false;

// Disc pump parameters
#define UART_TTP             Serial8
#define TTP_SELECTED_MODE    TTP_MODE_MANUAL
#define TTP_SELECTED_STREAM  TTP_STREAM_DISABLE
#define TTP_SELECTED_SRC     TTP_SRC_SETVAL
#define TTP_PWR_LIM_mW       TTP_MAX_PWR

// pump variables
bool    TTP_present = false;

// Selector valve parameters
#define UART_TITAN          Serial5
bool    TITAN_present = false;

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

  analogWriteResolution(10);

  // Initialize I2C selector
  pinMode(PIN_SENSOR_SELECT, OUTPUT);
  digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);

  while (!Serial) {
    delay(1);
  }

  // Initialize flow sensor
  Serial.print("Initializing flow sensor 0... ");
  SLF3X_0_present = SLF3X_init(W_SLF3X, MEDIUM_WATER);
  if (SLF3X_0_present) {
    Serial.println("initialized");
  }
  else {
    Serial.println("not detected");
  }

  // Initialize pump
  TTP_present = TTP_init(UART_TTP, TTP_PWR_LIM_mW, TTP_SELECTED_SRC, TTP_SELECTED_MODE, TTP_SELECTED_STREAM);
  if (!TTP_present) {
    Serial.println("Pump initialization failed");
    while (true) {
      delay(50000);
    }
  }
  else {
    Serial.println("Pump initialized");
  }

  // Initialize and calibrate the bubble sensors
  // Clear the lines
  set_valves_pressure();
  TTP_set_target(UART_TTP, TTP_PWR_LIM_mW);
  Serial.println("Clearing fluid...");
  delay(LINE_CLEAR_TIME_MS);
  TTP_set_target(UART_TTP, 0);
  reset_valves();
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
  Serial.println("Finishing calibration...");
  delay(LINE_CLEAR_TIME_MS);

  // Initialize pressure sensor
  digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
  SSCX_init(W_SSCX);
  digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_1);
  SSCX_init(W_SSCX);

  // Initialize selector valve
  TITAN_present = TITAN_init(UART_TITAN);

  // Initialize the NXP33996
  NXP33996_init(pin_33996_CS_0, pin_33996_PWM, pin_33996_nRST);

  // Initialize timed interrupts
  // When they trigger, set a flag to indicate something should be done the next loop cycle
  Timer_read_sensors_input.begin(set_read_sensors_flag, READ_SENSORS_INTERVAL_US);
  //  Updates are initialized in the setup state
  //  Timer_send_update_input.begin(set_send_update_flag, SEND_UPDATE_INTERVAL_US);

  t0 = millis();
}

void loop() {
  // Init variables
  int16_t  SLF3X_0_readings[3];  // flowrate, temperature, and flags from the flow sensor
  uint8_t  SLF3X_0_err;          // indicates whether the read succeeded
  bool     OCB350_0_reading;     // readings from the bubble sensors
  bool     OCB350_1_reading;
  int16_t  SSCX_0_readings[2];   // raw pressure and temperature
  uint8_t  SSCX_0_err;           // indicates whether the reading succeeded
  int16_t  SSCX_1_readings[2];
  uint8_t  SSCX_1_err;
  uint16_t time_0 = millis();    // track current time
  float    pressure;             // pressure setpoint
  uint16_t op_time_ms = 0;       // serial command timestamp
  byte payloads[TO_MCU_CMD_LENGTH - 3]; // incoming serial payload
  uint32_t payload_data = 0;            // used for parsing the payload data
  uint8_t serial_command;               // which command to run

  // first, try reading a command. read_serial_command overwrites payloads[] and serial_command
  if (read_serial_command(payloads, serial_command)) {
    // We have a command in progress!
    command_execution_status = IN_PROGRESS;
    switch (serial_command) {
      // move fluid from the reservoir to vacuum botttle 1
      case REMOVE_MEDIUM:
        // convert payload into draining time (ms)
        payload_data = (uint32_t(payloads[0]) << 8) + uint32_t(payloads[1]);
        op_time_ms = payload_data;
        internal_program = INTERNAL_PROGRAM_CLEAR_LINES_START;
        break;
      // load a volume of fluid into the reservoir
      case LOAD_MEDIUM:
        // convert the payload into microliters
        payload_data = (uint32_t(payloads[0]) << 8) + uint32_t(payloads[1]);
        vol_load_uL = (float(payload_data) / UINT16_MAX) * 10000;
        internal_program = INTERNAL_PROGRAM_LOAD_RESERVOIR_START;
        break;
      // move fluid from reservior into chamber
      case UNLOAD_MEDIUM:
        payload_data = (uint32_t(payloads[0]) << 8) + uint32_t(payloads[1]);
        vol_load_uL = (float(payload_data) / UINT16_MAX) * 10000;
        internal_program = INTERNAL_PROGRAM_UNLOAD_START;
        break;
      // clear command is handled in SerialCommUtils
      case CLEAR:
        command_execution_status = COMPLETED_WITHOUT_ERRORS;
        break;
      // if the command was not one of the prior commands, it was invalid
      default:
        command_execution_status = CMD_INVALID;
        break;
    }
  }

  // Handle the timer flags next
  if (flag_read_sensors) {
    // Indicate we have read the sensors
    flag_read_sensors = false;
    // Only read the sensors that are present
    if (SLF3X_0_present) {
      digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
      SLF3X_0_err = SLF3X_read(PERFORM_CRC, W_SLF3X, SLF3X_0_readings);
    }
    if (OCB350_0_present) {
      OCB350_0_reading = OPX350_read(OCB350_0_LOGIC);
    }
    if (OCB350_1_present) {
      OCB350_1_reading = OPX350_read(OCB350_1_LOGIC);
    }
    digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
    SSCX_0_err = SSCX_read(W_SSCX, SSCX_0_readings);
    digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_1);
    SSCX_1_err = SSCX_read(W_SSCX, SSCX_1_readings);

    // Once all the sensors are read, perform the control loop to calculate the new pump power
    // Initialize shared variables
    // Note - PID isn't implemented, this just uses the measured flowrate for bang-bang control
    float    disc_pump_power = 0;
    float    measurement = 0;
    pid_loop(UART_TTP, pid_mode, SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]), SSCX_to_psi(SSCX_1_readings[SSCX_PRESS_IDX]), SSCX_to_psi(SSCX_0_readings[SSCX_PRESS_IDX]), pid_sign, pid_setpoint, measurement, disc_pump_power);

    // Send an update only if the reading has been done
    if (flag_send_update) {
      flag_send_update = false;
      if (DEBUG_WITH_SERIAL) {
        if (SLF3X_0_present) {
          Serial.print("SLF3X 0 Error:   ");
          Serial.println(SLF3X_0_err, BIN);
          Serial.print("Flow (uL/min): ");
          Serial.println(SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]));
          Serial.print("Temp (deg C):  ");
          Serial.println(SLF3X_to_celsius(SLF3X_0_readings[SLF3X_TEMP_IDX]));
          Serial.print("Flags:       ");
          Serial.println(SLF3X_0_readings[SLF3X_FLAG_IDX], BIN);
          Serial.print("Volume (mL): ");
          Serial.println(SLF3X_0_volume_mL);
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

        Serial.print("NXP33996:      ");
        Serial.println(NXP33996_get_state(pin_33996_CS_0), BIN);

        //     Print PID info
        Serial.print("PID: ");
        switch (pid_mode) {
          case VACUUM_LOOP:
            Serial.println("Pressure Mode");
            break;
          case FLOWRATE_LOOP:
            Serial.println("Flowrate Mode");
            break;
          default:
            Serial.println("disabled; neglect PID info");
        }
        Serial.print("Target: ");
        Serial.println(pid_setpoint);
        Serial.print("Measurement: ");
        Serial.println(measurement);
        Serial.print("Power (mW): ");
        Serial.println(disc_pump_power);
        Serial.println(millis());
      }
      else {
        // Send data back as bytes
        uint8_t time_elapsed_s = (t0 - time_0) / 1000;
        send_serial_data(TTP_MAX_PWR, VOLUME_UL_MAX, command_execution_status, internal_program, OCB350_0_reading, OCB350_1_reading, pin_valve_status(), NXP33996_get_state(pin_33996_CS_0), SSCX_0_readings[SSCX_PRESS_IDX], SSCX_1_readings[SSCX_PRESS_IDX], SLF3X_0_readings[SLF3X_FLOW_IDX], time_elapsed_s, SLF3X_0_volume_mL, 0, disc_pump_power); // TODO- add selector valve position
      }
    }
  }

  // Read the manual inputs
  // if we have the flag to set them AND we don't have manual control disabled AND if the manual control button is pressed (active low signal)
  if (flag_check_manual_inputs && manual_control_enabled_by_software) {
    // If the manual control button is not pressed, shut everything down
    // Get switch and potentiometer state
    bool mode_pressure_vacuum = digitalRead(pin_pressure_vacuum);
    bool manual_control_enable = !digitalRead(pin_manual_control_enable);
    int16_t setpoint = analogRead(pin_analog_in);
    // account for dead zone
    setpoint = max(0, setpoint - analog_deadzone);

    // if we don't have manual control enabled OR manual control is not enabled, set power to 0
    if (!manual_control_enable || !manual_control_enabled_by_software) {
      pid_mode = IDLE_LOOP;
      TTP_set_target(UART_TTP, 0);

      // reset the valves if manual ctrl not enabled
      if (!manual_control_enable) {
        reset_valves();
      }
    }
    // Do error handling - don't let liquid past the other bubble sensor
    else if (mode_pressure_vacuum && !OPX350_read(OCB350_1_LOGIC)) {
      pid_mode = IDLE_LOOP;
      TTP_set_target(UART_TTP, 0);
      reset_valves();
    }
    // Otherwise, set the valves and setpoint
    else {
      // enable PID
      pid_mode = MANUAL_DEFAULT_MODE;
      // true - set vacumm mode
      if (mode_pressure_vacuum) {
        set_valves_vacuum();
        // handle PID cases
        switch (pid_mode) {
          case VACUUM_LOOP:
            // TODO
            break;
          case FLOWRATE_LOOP:
            // convert potentiometer setpoint to a flow value
            pid_setpoint = -abs(setpoint * MANUAL_MAX_FLOW_VAC / (ANALOG_MAX - analog_deadzone));
            pid_sign = -1;
            break;
        }
      }
      else {
        set_valves_pressure();
        // handle PID cases
        switch (pid_mode) {
          case VACUUM_LOOP:
            // TODO
            break;
          case FLOWRATE_LOOP:
            pid_setpoint = abs(setpoint * MANUAL_MIN_FLOW_PRES / (ANALOG_MAX - analog_deadzone));
            pid_sign = 1;
            break;
        }
      }
    }
  }
  // Run the state machine every cycle
  switch (internal_program) {
    case INTERNAL_PROGRAM_IDLE:
      // Idle state - integrate the volume. Operation is handled by the manual controls
      SLF3X_0_volume_mL = vol_inc(SLF3X_0_volume_mL, SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]), (millis() - SLF3X_0_dt), (SLF3X_0_readings[SLF3X_FLAG_IDX] & SLF3X_NO_FLUID));
      SLF3X_0_dt  = millis();
      break;
    case INTERNAL_PROGRAM_INITIAL:
      if (DEBUG_WITH_SERIAL) {
        Serial.println("Starting demo program");
        Serial.println("Valve test (check LEDs)");
      }
      // Test the valves
      digitalWrite(pin_valve_0, HIGH);
      digitalWrite(pin_valve_1, HIGH);
      digitalWrite(pin_valve_2, HIGH);
      digitalWrite(pin_valve_3, HIGH);
      digitalWrite(pin_valve_4, HIGH);
      digitalWrite(pin_valve_5, HIGH);
      delay(500);
      digitalWrite(pin_valve_0, LOW);
      digitalWrite(pin_valve_1, LOW);
      digitalWrite(pin_valve_2, LOW);
      digitalWrite(pin_valve_3, LOW);
      digitalWrite(pin_valve_4, LOW);
      digitalWrite(pin_valve_5, LOW);

      // Test pressure
      digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
      if (DEBUG_WITH_SERIAL) {
        Serial.print("Pressure Test (fast): ");
      }
      time_0 = millis();
      pressure = check_pressure();
      if (DEBUG_WITH_SERIAL) {
        Serial.print(pressure > CHECK_PRESSURE_THRESH);
        Serial.print(" Pressure is ");
        Serial.print(pressure);
        Serial.print(" PSI achieved in ");
        Serial.print(millis() - time_0);
        Serial.println(" ms");
      }
      // Test vacumm
      digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_1);
      if (DEBUG_WITH_SERIAL) {
        Serial.print("Vacuum Test (slow!): ");
      }
      time_0 = millis();
      pressure = check_vacuum();
      if (DEBUG_WITH_SERIAL) {
        Serial.print( pressure < CHECK_VACUUM_THRESH);
        Serial.print(" Pressure is ");
        Serial.print(pressure);
        Serial.print(" PSI achieved in ");
        Serial.print(millis() - time_0);
        Serial.println(" ms");
      }

      // Clear the lines
      if (DEBUG_WITH_SERIAL) {
        Serial.println("Clearing lines");
      }
      set_valves_pressure();
      TTP_set_target(UART_TTP, TTP_PWR_LIM_mW);
      delay(LINE_CLEAR_TIME_MS);
      TTP_set_target(UART_TTP, 0);
      reset_valves();

      // Initialize timed interrupts
      // When they trigger, set a flag to indicate something should be done the next loop cycle
      Timer_send_update_input.begin(set_send_update_flag, SEND_UPDATE_INTERVAL_US);
      Timer_check_manual_input.begin(set_check_manual_input_flag, CHECK_MANUAL_INTERVAL_US);

      if (DEMO_MODE) {
        internal_program = INTERNAL_PROGRAM_LOAD_RESERVOIR_START;
      }
      else {
        internal_program = INTERNAL_PROGRAM_IDLE;
        manual_control_enabled_by_software = true;
      }
      command_execution_status = COMPLETED_WITHOUT_ERRORS;

      break;
    case INTERNAL_PROGRAM_LOAD_RESERVOIR_START:
      // Fill the reservoir with a set volume of fluid - initialization
      // Set the valves
      // valve 5: vent vacuum bottle 0 for a duration
      release_vacuum_vb0();
      set_valves_vacuum();

      // start pumping at low PSI until we hit the first bubble sensor
      pid_mode = VACUUM_LOOP;
      pid_setpoint = PSI_LOAD_FLUID;
      pid_sign = -1;
      internal_program = INTENRAL_PROGRAM_LOAD_FLOW;
      break;
    case INTENRAL_PROGRAM_LOAD_FLOW:
      // Wait until we hit the bubble sensor
      // Once we hit it, stop fluid flow and vent the vacuum bottle
      if (!OPX350_read(OCB350_0_LOGIC)) {
        // disable PID
        pid_mode = IDLE_LOOP;
        TTP_set_target(UART_TTP, 0);
        // prevent fluid flow through valves
        digitalWrite(pin_valve_0, HIGH);
        digitalWrite(pin_valve_1, LOW);
        digitalWrite(pin_valve_2, HIGH);
        // pins 3-5 set by venting function
        release_vacuum_vb0();
        // Return to vacuum state
        set_valves_vacuum();
        // Clear volume integration + timing variables
        SLF3X_0_volume_mL  = 0;
        SLF3X_0_dt  = millis();

        // Go to next state
        if (DEMO_MODE) {
          internal_program = INTERNAL_PROGRAM_LOAD_RESERVOIR;
          vol_load_uL = DEMO_VOL;
        }
        else {
          internal_program = INTERNAL_PROGRAM_IDLE;
          manual_control_enabled_by_software = true;
        }
        command_execution_status = COMPLETED_WITHOUT_ERRORS;
      }
      break;
    case INTERNAL_PROGRAM_LOAD_RESERVOIR:
      // Start flow control
      pid_mode = FLOWRATE_LOOP;
      pid_sign = -1;
      pid_setpoint = FLOW_LOAD_FLUID;
      // Stay in this state until we measure sufficient fluid or we hit the limit
      SLF3X_0_volume_mL = vol_inc(SLF3X_0_volume_mL, SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]), (millis() - SLF3X_0_dt), (SLF3X_0_readings[SLF3X_FLAG_IDX] & SLF3X_NO_FLUID));
      SLF3X_0_dt  = millis();

      // Check if we hit the bubble sensor - turn off pumps and set valves if we did
      if (!OPX350_read(OCB350_1_LOGIC)) {
        if (DEBUG_WITH_SERIAL) {
          Serial.println("FLAG - HIT BUBBLE SENSOR");
        }
        stop_current_enable_manual();
      }
      else if ( abs(SLF3X_0_volume_mL) > (vol_load_uL + FLUID_LOAD_BUFFER_uL) / 1000.0) {
        // If we have enough fluid, set valves and go to the next state - unload fluid into chamber.
        // Turn off pump
        TTP_set_target(UART_TTP, 0);
        set_valves_pressure();
        // reset the PID loop
        pid_reset();
        // positive flowrate loop params
        pid_sign = 1;
        pid_mode = FLOWRATE_LOOP;
        pid_setpoint = 0;

        // next state: unload fluid into chamber
        internal_program = INTERNAL_PROGRAM_UNLOAD_START;
        // reset fluid unloading to 0
        SLF3X_0_volume_mL = 0;
        //        internal_program = INTERNAL_PROGRAM_IDLE;
        //        manual_control_enabled_by_software = true;
        command_execution_status = COMPLETED_WITHOUT_ERRORS;
      }
      break;

    case INTERNAL_PROGRAM_CLEAR_LINES_START:
      set_valves_vacuum();
      TTP_set_target(UART_TTP, TTP_PWR_LIM_mW);
      cmd_time = millis() + op_time_ms;
      internal_program = INTERNAL_PROGRAM_CLEAR_LINES;
      break;
    case INTERNAL_PROGRAM_CLEAR_LINES:
      if (millis() > cmd_time) {
        TTP_set_target(UART_TTP, 0);
        reset_valves();
        internal_program = INTERNAL_PROGRAM_IDLE;
        manual_control_enabled_by_software = true;
        command_execution_status = COMPLETED_WITHOUT_ERRORS;
      }
      break;

    case INTERNAL_PROGRAM_UNLOAD_START:
      set_valves_pressure();
      // Start flow control
      pid_mode = FLOWRATE_LOOP;
      pid_sign = 1;
      pid_setpoint = FLOW_UNLOAD_FLUID;
      SLF3X_0_volume_mL = 0;
      SLF3X_0_dt  = millis();

      internal_program = INTERNAL_PROGRAM_UNLOAD;
      break;
    case INTERNAL_PROGRAM_UNLOAD:
      // Stay in this state until we measure sufficient fluid or we run out of fluid
      SLF3X_0_volume_mL = vol_inc(SLF3X_0_volume_mL, SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]), (millis() - SLF3X_0_dt), (SLF3X_0_readings[SLF3X_FLAG_IDX] & SLF3X_NO_FLUID));
      SLF3X_0_dt  = millis();

      // If there is fluid in the sensor and we hit our volume target, deposit everything and restart the loop
      if (abs(SLF3X_0_volume_mL) > vol_load_uL / 1000.0) {
        // suck in air to clear the reservoir - manually move the inlet
        set_valves_vacuum();
        TTP_set_target(UART_TTP, TTP_PWR_LIM_mW);
        if (DEBUG_WITH_SERIAL) {
          Serial.println("Move inlet to air - clear reservoir");
        }
        delay(10000);
        // vacuum away fluid remaining in the reservoir
        set_valves_pressure();
        TTP_set_target(UART_TTP, TTP_PWR_LIM_mW);
        if (DEBUG_WITH_SERIAL) {
          Serial.println("Move inlet back to fluid - depositing fluid");
        }
        delay(10000);
        internal_program = INTERNAL_PROGRAM_LOAD_RESERVOIR_START;
      }
      break;
    case INTERNAL_PROGRAM_PREUSE_CHECK_PRESSURE:
      pressure = check_pressure();
      break;
    case INTERNAL_PROGRAM_PREUSE_CHECK_VACUUM:
      pressure = check_vacuum();
      break;
    default:
      break;
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
void set_check_manual_input_flag() {
  flag_check_manual_inputs = true;
  return;
}

float check_pressure() {
  int16_t time_0;
  int16_t dt;
  int16_t SSCX_reading[2];
  float pressure;

  // Set valves
  // valve 0: connect to valve 1 (low)
  digitalWrite(pin_valve_0, LOW);
  // valve 1: disconnect valve 0 from anything
  digitalWrite(pin_valve_1, HIGH);
  // valve 2: don't care
  digitalWrite(pin_valve_2, LOW);
  // valve 3: don't care
  digitalWrite(pin_valve_3, LOW);
  // valve 4: seal valve 3, connect air to pump (low)
  digitalWrite(pin_valve_4, LOW);

  // get initial time
  time_0 = millis();
  // start pumping
  TTP_set_target(UART_TTP, TTP_PWR_LIM_mW);
  do {
    // Check the pressure
    dt = millis() - time_0;
    SSCX_read(W_SSCX, SSCX_reading);
    pressure = SSCX_to_psi(SSCX_reading[SSCX_PRESS_IDX]);
    // Keep checking until we time out or we exceed our pressure threshhold
  } while ((dt < CHECK_TIMEOUT_MS) && (pressure < CHECK_PRESSURE_THRESH));

  // Stop pumping air
  TTP_set_target(UART_TTP, 0);

  // reset valves
  reset_valves();

  return pressure;
}

float check_vacuum() {
  int16_t time_0;
  int16_t dt;
  int16_t SSCX_reading[2];
  float pressure;

  // Set valves
  // valve 0: connect to air
  digitalWrite(pin_valve_0, HIGH);
  // valve 1: cut off vacuum bottle
  digitalWrite(pin_valve_1, LOW);
  // valve 2: don't care
  digitalWrite(pin_valve_2, LOW);
  // valve 3: connect to vacuum bottle
  digitalWrite(pin_valve_3, LOW);
  // valve 4: connect to valve 3
  digitalWrite(pin_valve_4, HIGH);

  // get initial time
  time_0 = millis();
  // start pumping
  TTP_set_target(UART_TTP, TTP_PWR_LIM_mW);
  do {
    // Check the pressure
    dt = millis() - time_0;
    SSCX_read(W_SSCX, SSCX_reading);
    pressure = SSCX_to_psi(SSCX_reading[SSCX_PRESS_IDX]);
    // Keep checking until we time out or we exceed our pressure threshhold
  } while ((dt < CHECK_TIMEOUT_MS) && (pressure > CHECK_VACUUM_THRESH));

  // Stop pumping air
  TTP_set_target(UART_TTP, 0);
  // Reset valves
  reset_valves();

  return pressure;
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
// get fluids from the chamber to the second vacuum bottle using suction
void set_valves_suction() {
  // air gets pulled out of the second vacuum bottle and ejected from valve 0

  // valve 0 connects to ambient air
  digitalWrite(pin_valve_0, HIGH);
  // connect the reservoir to the first vacuum bottle
  digitalWrite(pin_valve_1, HIGH);
  // connect reservoir to open/closed chamber
  digitalWrite(pin_valve_2, LOW);
  // connect vacuum bottle (and open/closed chamber) from anything
  digitalWrite(pin_valve_3, HIGH);
  // connect to disc pump suction
  digitalWrite(pin_valve_4, HIGH);
  return;
}
// release vacuum in VB0
void release_vacuum_vb0() {
  int16_t  SSCX_readings[2];
  float psi;
  TTP_set_target(UART_TTP, 0);

  digitalWrite(pin_valve_4, HIGH);
  digitalWrite(pin_valve_3, LOW);
  delay(30);
  SSCX_read(W_SSCX, SSCX_readings);
  psi = SSCX_to_psi(SSCX_readings[SSCX_PRESS_IDX]);
  // open the valve
  digitalWrite(pin_valve_5, HIGH);
  // wait for pressure to equalize
  do {
    SSCX_read(W_SSCX, SSCX_readings);
    psi = SSCX_to_psi(SSCX_readings[SSCX_PRESS_IDX]);
    if (DEBUG_WITH_SERIAL) {
      Serial.println(psi);
    }
    delay(30);
  }  while (psi < VB0_PRESSURE_THRESH);
  // close the valve
  digitalWrite(pin_valve_5, LOW);
  digitalWrite(pin_valve_4, LOW);
  digitalWrite(pin_valve_3, LOW);

  return;
}

void stop_current_enable_manual() {
  // Set the valves
  // valve 0: connect to air - prevent backflow
  digitalWrite(pin_valve_0, HIGH);
  // valve 1: disconnect from vacuum bottle
  digitalWrite(pin_valve_1, LOW);
  // valve 2: stay connected to fluids
  digitalWrite(pin_valve_2, HIGH);
  // valve 3: connect to vacuum bottle
  digitalWrite(pin_valve_3, LOW);
  // valve 4: disconnect vacuum bottle from pump
  digitalWrite(pin_valve_4, LOW);

  // Turn off pump
  TTP_set_target(UART_TTP, 0);

  internal_program = INTERNAL_PROGRAM_IDLE;
  pid_reset();
  pid_mode = IDLE_LOOP;
  pid_setpoint = 0;
  TTP_set_target(UART_TTP, 0);
  manual_control_enabled_by_software = true;
  return;
}

byte pin_valve_status() {
  byte retval = 0;
  retval |= ((digitalRead(pin_valve_0) & 1) << 0);
  retval |= ((digitalRead(pin_valve_1) & 1) << 1);
  retval |= ((digitalRead(pin_valve_2) & 1) << 2);
  retval |= ((digitalRead(pin_valve_3) & 1) << 3);
  retval |= ((digitalRead(pin_valve_4) & 1) << 4);
  retval |= ((digitalRead(pin_valve_5) & 1) << 5);

  return retval;
}

float vol_inc(float prev_vol, float rate, uint16_t dt, bool air_flag) {
  if (air_flag) {
    return prev_vol;
  }
  else {
    return prev_vol + dt * rate / (60.0 * 1000.0 * 1000.0);
  }
}
