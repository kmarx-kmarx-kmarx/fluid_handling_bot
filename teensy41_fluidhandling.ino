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

        bool    OCB350_0_present       - set true when bubble sensor 0 is calibrated properly
        bool    OCB350_1_present
        bool    SSCX_0_present         - set true when pressure sensor 0 is iniialized properly

        bool    SSCX_1_present

        volatile bool flag_read_sensors - indicates the sensors should be read during the next loop
        volatile bool flag_send_update  - indicates debug data should be sent during the next loop

    Dependencies:
        SLF3X.h     : Functions for initializing and reading from the flow sensor
        SSCX.h      : Functions for initializing and reading from the pressure sensor
        OPX350.h    : Functions for initializing and reading from the bubble sensor
        TTP.h       : Functions for initializing, reading from, and writing to the disc pump
        TITAN.h     : Functions for initializing, reading from, and writing to the selector valve
        Wire.h      : For I2C communication

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

elapsedMillis elapsed_millis_since_the_start_of_the_internal_program = 0;
byte internal_program = INTERNAL_PROGRAM_INITIAL;

// PID control parameters and variables
#define IDLE_LOOP     0
#define PRESSURE_LOOP 1
#define FLOWRATE_LOOP 2
uint8_t pid_mode = IDLE_LOOP;
float pres_pid_p_coeff = 1;
float flow_pid_p_coeff = 1;
float pid_i_error = 0;
float pres_pid_i_coeff = 1;
float flow_pid_i_coeff = 1;
float pid_setpoint = 0;
#define MIN_PRES_POWER 0
#define MAX_PRES_POWER TTP_PWR_LIM_mW
#define MIN_FLOW_POWER 50
#define MAX_FLOW_POWER TTP_PWR_LIM_mW

// SLF3X flow sensor parameters
#define W_SLF3X      Wire1
#define PERFORM_CRC  true
// Calibration param
#define CHECK_PRESSURE_THRESH 5.50
#define CHECK_VACUUM_THRESH  -4.25
#define CHECK_TIMEOUT_MS     5000
// SLF3X flow sensor variables
// flow sensor 0
bool    SLF3X_0_present = false;
// flow sensor 1
bool    SLF3X_1_present = false;

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


// I2C bus select
#define PIN_SENSOR_SELECT 15
#define SELECT_SENSOR_0   LOW
#define SELECT_SENSOR_1   HIGH

// Disc pump parameters
#define UART_TTP             Serial8
#define TTP_SELECTED_MODE    TTP_MODE_MANUAL
#define TTP_SELECTED_STREAM  TTP_STREAM_DISABLE
#define TTP_SELECTED_SRC     TTP_SRC_SETVAL
#define TTP_PWR_LIM_mW       1000
// pump variables
bool    TTP_present = false;

// Selector valve parameters
#define UART_TITAN          Serial5
bool    TITAN_present = false;

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
  digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_1);
  Serial.print("Initializing flow sensor 1... ");
  SLF3X_1_present = SLF3X_init(W_SLF3X, MEDIUM_WATER);
  if (SLF3X_1_present) {
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
  digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
  SSCX_init(W_SSCX);
  digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_1);
  SSCX_init(W_SSCX);

  // Initialize pump
  TTP_present = TTP_init(UART_TTP, TTP_PWR_LIM_mW, TTP_SELECTED_SRC, TTP_SELECTED_MODE, TTP_SELECTED_STREAM);
  if (!TTP_present) {
    Serial.println("Pump initialization failed");
    while (true) {
      delay(50000);
    }
  }

  // Initialize selector valve
  TITAN_present = TITAN_init(UART_TITAN);

  // Initialize the NXP33996
  NXP33996_init(pin_33996_CS_0, pin_33996_PWM, pin_33996_nRST);

  // Initialize timed interrupts
  // When they trigger, set a flag to indicate something should be done the next loop cycle
  Timer_read_sensors_input.begin(set_read_sensors_flag, READ_SENSORS_INTERVAL_US);
  //  Updates are initialized in the setup state
  //  Timer_send_update_input.begin(set_send_update_flag, SEND_UPDATE_INTERVAL_US);
}

void loop() {
  // Init variables
  int16_t SLF3X_0_readings[3];
  uint8_t SLF3X_0_err;
  int16_t SLF3X_1_readings[3];
  uint8_t SLF3X_1_err;
  bool    OCB350_0_reading;
  bool    OCB350_1_reading;
  int16_t SSCX_0_readings[2];
  uint8_t SSCX_0_err;
  int16_t SSCX_1_readings[2];
  uint8_t SSCX_1_err;
  //  byte    byte_rx;
  uint16_t time_0 = millis();
  float disc_pump_power = 0;
  float pressure;

  // Handle the timer flags first
  if (flag_read_sensors) {
    flag_read_sensors = false;
    if (SLF3X_0_present) {
      digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
      SLF3X_0_err = SLF3X_read(PERFORM_CRC, W_SLF3X, SLF3X_0_readings);
    }
    if (SLF3X_1_present) {
      digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_1);
      SLF3X_1_err = SLF3X_read(PERFORM_CRC, W_SLF3X, SLF3X_1_readings);
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
    float error;
    float measurement;
    switch (pid_mode) {
      case PRESSURE_LOOP:
        measurement = SSCX_to_psi(SSCX_1_readings[SSCX_PRESS_IDX]);
        error = pid_setpoint - measurement;
        pid_i_error += error;
        pid_i_error = constrain(pid_i_error, 0, (1 / pres_pid_i_coeff));
        disc_pump_power = int((pid_i_error * pres_pid_i_coeff + error * pres_pid_p_coeff) * MAX_PRES_POWER);
        disc_pump_power = constrain(disc_pump_power, MIN_PRES_POWER, MAX_PRES_POWER);
        break;
      case FLOWRATE_LOOP:
        measurement = SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]);
        error = (pid_setpoint - measurement) / 1000;
        pid_i_error += error;
        pid_i_error = constrain(pid_i_error, 0, 1 / flow_pid_i_coeff);
        disc_pump_power = int((pid_i_error * flow_pid_i_coeff + error * flow_pid_p_coeff) * MAX_PRES_POWER);
        disc_pump_power = constrain(disc_pump_power, MIN_FLOW_POWER, MAX_FLOW_POWER);
        break;
      default:
        break;
    }
    // Set the pump power
    TTP_set_target(UART_TTP, disc_pump_power);

    // Send an update only if the reading has been done
    if (flag_send_update) {
      flag_send_update = false;
      if (SLF3X_0_present) {
        Serial.print("SLF3X 0 Error:   ");
        Serial.println(SLF3X_0_err, BIN);
        Serial.print("Flow (uL/min): ");
        Serial.println(SLF3X_to_uLmin(SLF3X_0_readings[SLF3X_FLOW_IDX]));
        Serial.print("Temp (deg C):  ");
        Serial.println(SLF3X_to_celsius(SLF3X_0_readings[SLF3X_TEMP_IDX]));
        Serial.print("Flags:       ");
        Serial.println(SLF3X_0_readings[SLF3X_FLAG_IDX], BIN);
      }
      if (SLF3X_1_present) {
        Serial.print("SLF3X 1 Error:   ");
        Serial.println(SLF3X_1_err, BIN);
        Serial.print("Flow (uL/min): ");
        Serial.println(SLF3X_to_uLmin(SLF3X_1_readings[SLF3X_FLOW_IDX]));
        Serial.print("Temp (deg C):  ");
        Serial.println(SLF3X_to_celsius(SLF3X_1_readings[SLF3X_TEMP_IDX]));
        Serial.print("Flags:       ");
        Serial.println(SLF3X_1_readings[SLF3X_FLAG_IDX], BIN);
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
      Serial.println(millis());
    }
  }
  //  // Handle Serial input
  //  if (Serial.available()) {
  //    byte_rx = Serial.read();
  //    switch (rx_byte) {
  //      case CMD_START_BYTE:
  //        // If we are reading a new command, reset the idx and array
  //        buffer_rx_ptr = 0;
  //        memset(buffer_rx, 0, TO_MCU_CMD_LENGTH);
  //        break;
  //      default:
  //        // Otherwise, read in the data. Throw out any bytes if they would cause buffer_rx to overflow
  //        if (buffer_rx_ptr < TO_MCU_CMD_LENGTH) {
  //          buffer_rx[buffer_rx_ptr++] = byte_rx;
  //        }
  //    }
  //  }
  //  // If our serial buffer is full, parse it.
  //  if (buffer_rx_ptr >= TO_MCU_CMD_LENGTH) {
  //    // Initialize local vars for parsing
  //    // uint16_t current_command_uid  = uint16_t(buffer_rx[0]) * 256 + uint16_t(buffer_rx[1]);
  //    uint8_t  current_command      = buffer_rx[2];
  //    uint8_t  payload1             = buffer_rx[3];
  //    uint8_t  payload2             = buffer_rx[4];
  //    uint16_t payload3             = (uint16_t(buffer_rx[5]) << 8) + uint16_t(buffer_rx[6]);
  //    uint32_t payload4             = (uint32_t(buffer_rx[7]) << 24) + (uint32_t(buffer_rx[8]) << 16) + (uint32_t(buffer_rx[9]) << 8) + (uint32_t(buffer_rx[10]));
  //
  //    switch(current_command){
  //      case CLEAR:
  //        // Clear the first two bytes of rx buffer - reset the command index
  //        buffer_rx[0] = 0;
  //        buffer_rx[1] = 0;
  //        // Indicate we successfully are doing nothing
  //        command_execution_status = COMPLETED_WITHOUT_ERRORS;
  //        break;
  //      case SET_MANUAL_CONTROL:
  //        if (payload1 != CMD_ENABLE){
  //          // enable manual control
  //          manual_control_enabled_by_software = true;
  //          // disable the pressure ctrl loop
  //          disable_pid();
  //        }
  //        else {
  //          // disable manual control
  //          manual_control_enabled_by_software = false;
  //        }
  //      case
  //    }
  //  }
  // Run the state machine
  switch (internal_program) {
    case INTERNAL_PROGRAM_INITIAL:
      Serial.println("Starting demo program");
      Serial.println("Valve test (check LEDs)");
      // Test the valves
      //      digitalWrite(pin_valve_0, HIGH);
      //      delay(500);
      //      digitalWrite(pin_valve_1, HIGH);
      //      delay(500);
      //      digitalWrite(pin_valve_2, HIGH);
      //      delay(500);
      //      digitalWrite(pin_valve_3, HIGH);
      //      delay(500);
      //      digitalWrite(pin_valve_4, HIGH);
      //      delay(500);
      //      digitalWrite(pin_valve_0, LOW);
      //      delay(500);
      //      digitalWrite(pin_valve_1, LOW);
      //      delay(500);
      //      digitalWrite(pin_valve_2, LOW);
      //      delay(500);
      //      digitalWrite(pin_valve_3, LOW);
      //      delay(500);
      //      digitalWrite(pin_valve_4, LOW);
      //      delay(500);

      // Test pressure
      digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_0);
      Serial.print("Pressure Test: ");
      time_0 = millis();
      pressure = check_pressure();
      Serial.print(pressure > CHECK_PRESSURE_THRESH);
      Serial.print(" Pressure is ");
      Serial.print(pressure);
      Serial.print(" PSI achieved in ");
      Serial.print(millis() - time_0);
      Serial.println(" ms");
      // Test vacumm
      digitalWrite(PIN_SENSOR_SELECT, SELECT_SENSOR_1);
      Serial.print("Vacuum Test: ");
      time_0 = millis();
      pressure = check_vacuum();
      Serial.print( pressure < CHECK_VACUUM_THRESH);
      Serial.print(" Pressure is ");
      Serial.print(pressure);
      Serial.print(" PSI achieved in ");
      Serial.print(millis() - time_0);
      Serial.println(" ms");

      // Initialize timed interrupts
      // When they trigger, set a flag to indicate something should be done the next loop cycle
      Timer_send_update_input.begin(set_send_update_flag, SEND_UPDATE_INTERVAL_US);

      internal_program = INTERNAL_PROGRAM_IDLE;
      break;
    case INTERNAL_PROGRAM_REMOVE_MEDIUM:
      //      aaa;
      break;
    case INTERNAL_PROGRAM_RAMP_UP_PRESSURE:
      //      asklfjsdkajlfsd;
      break;
    case INTERNAL_PROGRAM_PUMP_FLUID:
      //      kajsfdlka;
      break;
    case INTERNAL_PROGRAM_EMPTY_FLUIDIC_LINE:
      //      aklfsdjfsd;
      break;
    case INTERNAL_PROGRAM_PREUSE_CHECK_PRESSURE:
      //      klasdjf;
      break;
    case INTERNAL_PROGRAM_PREUSE_CHECK_VACUUM:
      //      ajfdld;
      break;
    default:
      break;
      //      asdfsadfsfasd;
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

void disable_pid() {
  // Clear integral error and disable the loop
  pid_mode = IDLE_LOOP;
  pid_i_error = 0;
  pid_setpoint = 0;
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
  digitalWrite(pin_valve_0, LOW);
  digitalWrite(pin_valve_1, LOW);
  digitalWrite(pin_valve_2, LOW);
  digitalWrite(pin_valve_3, LOW);
  digitalWrite(pin_valve_4, LOW);

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
  digitalWrite(pin_valve_0, LOW);
  digitalWrite(pin_valve_1, LOW);
  digitalWrite(pin_valve_2, LOW);
  digitalWrite(pin_valve_3, LOW);
  digitalWrite(pin_valve_4, LOW);

  return pressure;
}
