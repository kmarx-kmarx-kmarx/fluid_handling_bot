/*
    TTP.cpp
    This file contains functions for initializing and controlling the disc pump

    bool TTP_init: Initialize the disc pump's power limit, signal source, mode, and whether it should stream data. Returns true if successful.
      Arguments: HardwareSerial &S,  int16_t pwr_lim, uint8_t src, uint8_t mode, uint8_t stream
    bool TTP_enable: Enable/disable the disc pump. Returns true if successful.
      Arguments: HardwareSerial &S, bool en
    bool TTP_set_pwr_limit: change the power limit of the disc pump.
       Arguments: HardwareSerial &S, int16_t pwr_lim
    bool TTP_set_target:  Set the pump's setpoint. Returns true if successful.
      Arguments: HardwareSerial &S, float target
    TODO: Implement get_status
    bool TTP_get_status:  Get error codes, drive frequency, drive power, drive current, drive voltage, and the power limit and saves them to shared variables. Returns true if successful.
      Arguments: HardwareSerial &S, uint16_t &error_code, int16_t &drive_freq, float &dive_pwr, float &drive_current, float &drive_voltage, float &power_limit
    int16_t TTP_read_int: Read an integer from a register in the disc pump
      Arguments: HardwareSerial &S, uint8_t reg
    float TTP_read_float: Read a float from a register in the disc pump
      Arguments: HardwareSerial &S, uint8_t reg

    bool TTP_send_packet: Send the write buffer to the pump and populate the read the response into the read buffer. Also write the index on the response pointer showing when the pump's response begins
      Arguments: HardwareSerial &S, char *tx_buffer, char *rx_buffer, uint16_t &cmd_ptr
    bool TTP_write_register: Write a value to a register and return true if it was successfull. This function is overloaded
      Arguments: HardwareSerial &S, int16_t dat
      Arguments: HardwareSerial &S, float dat

*/
#include "TTP.h"

float pwr = 0;

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_send_packet() sends a byte stream to the disc pump and reads the response back into a shared array.

  OPERATION: We write the full contents of the tx_buffer to the pump over serial and wait for a response. We then store the response to the rx_buffer and make sure the response was echoed properly. If the response times out or is not echoed properly, we try sending the message again. We try up to TTP_N_TRIES times.

  ARGUMENTS:
      HardwareSerial &S: stream class to read from Serial, Serial1, etc.
      char *tx_buffer:   pointer to array containing characters to send. This buffer is not changed
      char *rx_buffer:   pointer to array containing recieved characters. This buffer is overwritten

  RETURNS:
      bool success:     Returns true if the read/write worked properly

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class
      char *tx_buffer:   array with characters to send, not modified
      char *rx_buffer:   array with received characters, overwritten

  GLOBAL VARIABLES: None

  DEPENDENCIES: Arduino.h
  -----------------------------------------------------------------------------
*/
bool TTP_send_packet(HardwareSerial &S, char *tx_buffer, char *rx_buffer) {
  uint16_t rx_ptr;
  uint16_t time_0;

  uint16_t cmd_ptr = strlen(tx_buffer);

  for (uint8_t i = 0; i < TTP_N_TRIES; i++) { // try writing multiple times
    rx_ptr = 0; // reset the rx pointer
    memset(rx_buffer, 0, strlen(rx_buffer)); // reset the read buffer
    S.clear(); // clear stream's RX buffer
    S.print(tx_buffer);
    S.flush(); // Wait for any transmitted data still in buffers to actually transmit

    // wait for a response
    time_0 = millis();
    while (!S.available() && (millis() - time_0) < TTP_READ_TIMEOUT_mS) {
      delayMicroseconds(TTP_IDLETIME_uS);
    }
    // wait for a response
    // if we have a response, process it. otherwise, fall through to the next for loop
    if (S.available()) {
      // read every byte into the buffer
      while (S.available()) {
        rx_buffer[rx_ptr++] = S.read();
        delayMicroseconds(TTP_IDLETIME_uS);
      }
      // We are expecting the response to echo the request and have additional data if we are reading from one of the pump's registers
      // If expectation met, return out. Otherwise, loop again
      if ((cmd_ptr <= rx_ptr) && (strncmp(tx_buffer, rx_buffer, cmd_ptr) == 0)) {
        return true;
      }
    }
  }
  return false; // not receiving the sent command within 1 ms for 3 attempts
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_read_int() requests data from a register and returns the data as an int

  OPERATION: We format the register number into a UART request to the disc pump. If we get a valid result, return it. Otherwise, return an error.

  ARGUMENTS:
      HardwareSerial &S:  stream class to read from Serial, Serial1, etc.
      uint8_t reg:        register to read from

  RETURNS:
      int16_t val:        Returns the value if the read worked properly and INT16_MIN otherwise

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: Arduino.h
  -----------------------------------------------------------------------------
*/
int16_t TTP_read_int(HardwareSerial &S, uint8_t reg) {
  char rx[TTP_BUFFER_SIZE];
  char tx[TTP_BUFFER_SIZE];
  bool is_valid;
  uint16_t cmd_len;

  // Format the UART command and get its length
  sprintf(tx, TTP_READ_REG_FORMAT, reg);
  cmd_len = strlen(tx);
  // Send the command
  is_valid = TTP_send_packet(S, tx, rx);

  // Read the response
  if (is_valid) {
    // rx has the command echoed and a comma before the returned int; index past those to get to the int
    return atoi(&rx[cmd_len + 1]);
  }
  else {
    return TTP_INT_READ_ERR;
  }
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_read_float() requests data from a register and returns the data as a float

  OPERATION: We format the register number into a UART request to the disc pump. If we get a valid result, return it. Otherwise, return an error.

  ARGUMENTS:
      HardwareSerial &S:  stream class to read from Serial, Serial1, etc.
      uint8_t reg:        register to read from

  RETURNS:
      float val:        Returns the value if the read worked properly and NAN otherwise

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: Arduino.h
  -----------------------------------------------------------------------------
*/
float TTP_read_float(HardwareSerial &S, uint8_t reg) {
  char rx[TTP_BUFFER_SIZE];
  char tx[TTP_BUFFER_SIZE];
  bool is_valid;
  uint16_t cmd_len;

  // Format the UART command and get its length
  sprintf(tx, TTP_READ_REG_FORMAT, reg);
  cmd_len = strlen(tx);
  // Send the command
  is_valid = TTP_send_packet(S, tx, rx);

  // Read the response
  if (is_valid) {
    // rx has the command echoed and a comma before the returned int; index past those to get to the int
    return atof(&rx[cmd_len + 1]);
  }
  else {
    return TTP_FLT_READ_ERR;
  }
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_write_register() writes either an int or a float to a register in the disc pump. This function is overloaded so it can take either an int or float as an argument

  OPERATION: We format the register number and int/float into a UART request to the disc pump. If we get a valid echo, return true.

  ARGUMENTS:
      HardwareSerial &S:  stream class to read from Serial, Serial1, etc.
      uint8_t reg:        register to read from
      int16_t/float dat:  data to write

  RETURNS:
      bool valid:         Returns true if the write succeeded

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: Arduino.h
  -----------------------------------------------------------------------------
*/
bool TTP_write_register(HardwareSerial &S, uint8_t reg, int16_t dat) {
  char rx[TTP_BUFFER_SIZE];
  char tx[TTP_BUFFER_SIZE];
  int16_t len;

  // Format the UART command
  len = sprintf(tx, TTP_WR_REG_INT_FORMAT, reg, dat);
  // return with error if sprintf failed
  if (len < 0) {
    return false;
  }
  // Send the command
  return TTP_send_packet(S, tx, rx);
}
bool TTP_write_register(HardwareSerial &S, uint8_t reg, float dat) {
  char rx[TTP_BUFFER_SIZE];
  char tx[TTP_BUFFER_SIZE];
  int16_t len;

  // Format the UART command
  len = sprintf(tx, TTP_WR_REG_FLT_FORMAT, reg, dat);
  // return with error if sprintf failed
  if (len < 0) {
    return false;
  }
  // Send the command
  return TTP_send_packet(S, tx, rx);
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_init() initializes the disc pump with the power limit, source, mode, and stream settings.

  OPERATION: We first constrain the power limit to be betweeen the min and max values. Then, we write the power limit, setpoint source, operation mode, and stream mode to the registers in this order. If all the writes are successful, return true.

  ARGUMENTS:
      HardwareSerial &S: stream class to read from Serial, Serial1, etc.
      int16_t pwr_lim:   Power limit in units milliwatts
      uint8_t src:       Flag indicating what the setpoint source should be, sent over serial, from a sensor, or from one of the analog pins
      uint8_t mode:      Flag indicating mode, manual, PID, or bang-bang
      uint8_t stream:    Flag indicating whether the pump controller should stream data

  RETURNS:
      bool pump_connected:     return true if initialization was successful

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: Arduino.h
  -----------------------------------------------------------------------------
*/

bool TTP_init(HardwareSerial &S, int16_t pwr_lim, uint8_t src, uint8_t mode, uint8_t stream) {
  int16_t time_0;
  bool success = true;
  S.begin(TTP_BAUDRATE);

  // wait for the serial port to open
  time_0 = millis();
  while (!S && (millis() - time_0) < TTP_READ_TIMEOUT_mS) {
    delayMicroseconds(TTP_IDLETIME_uS);
  }
  // time out
  if ((millis() - time_0) > TTP_READ_TIMEOUT_mS) {
    success = false;
  }

  pwr_lim = constrain(pwr_lim, TTP_MIN_PWR, TTP_MAX_PWR);

  // success is true only if all writes succeed
  success = TTP_write_register(S, TTP_PWR_LIMIT, (int16_t)pwr_lim)  && success;
  success = TTP_write_register(S, TTP_CTRL_MODE, (int16_t)mode)     && success;
  success = TTP_write_register(S, TTP_MANUAL_SRC, (int16_t)src)     && success;
  success = TTP_write_register(S, TTP_STREAM_MODE, (int16_t)stream) && success;

  return success;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_enable() enables/disables the disc pump

  OPERATION:  We write to the enable register and return true if the write was successful

  ARGUMENTS:
      HardwareSerial &S: stream class to read from Serial, Serial1, etc.
      int16_t pwr_lim:   Power limit in units milliwatts
      uint8_t src:       Flag indicating what the setpoint source should be, sent over serial, from a sensor, or from one of the analog pins
      uint8_t mode:      Flag indicating mode, manual, PID, or bang-bang
      uint8_t stream:    Flag indicating whether the pump controller should stream data

  RETURNS:
      bool success:     return true if write was successful

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/

bool TTP_enable(HardwareSerial &S, bool en) {
  return TTP_write_register(S, TTP_ENABLED, (int16_t)en);
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_set_pwr_limit() sets the pump's power limit.

  OPERATION:  We first constrian the power limit to be between the min and max value, then we write to the power limit register and return true if the write was successful

  ARGUMENTS:
      HardwareSerial &S: stream class to read from Serial, Serial1, etc.
      int16_t pwr_lim:   Power limit in units milliwatts

  RETURNS:
      bool success:     return true if write was successful

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
bool TTP_set_pwr_limit(HardwareSerial &S, int16_t pwr_lim) {
  pwr_lim = constrain(pwr_lim, TTP_MIN_PWR, TTP_MAX_PWR);
  return TTP_write_register(S, TTP_PWR_LIMIT, pwr_lim);
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_set_target() sets the pump's target(open loop) power.

  OPERATION:  We first constrian the power to be between the min and max value, then we write to the setpoint register and return true if the write was successful

  ARGUMENTS:
      HardwareSerial &S: stream class to read from Serial, Serial1, etc.
      float target:      Output power in units milliwatts

  RETURNS:
      bool success:     return true if write was successful

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
bool TTP_set_target(HardwareSerial &S, float target) {
  target = constrain(target, TTP_MIN_PWR, TTP_MAX_PWR);
  pwr = target;
  return TTP_write_register(S, TTP_SET_VALUE, target);
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_get_status() overwrites shared variables with the disc pump's error code register, drive frequency, drive power, drive current, drive voltage, and power limit. We return false of any readings failed.

  OPERATION:  We read the data from the registers into our shared variables. Then, if any of the shared variables have an error value, return false

  ARGUMENTS:
      HardwareSerial &S: stream class to read from Serial, Serial1, etc.
      uint16_t &error_code, int16_t &drive_freq, float &dive_pwr, float &drive_current, float &drive_voltage, float &power_limit

  RETURNS:
      bool success:     return true if write was successful

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
bool TTP_get_status(HardwareSerial &S, uint16_t &error_code, int16_t &drive_freq, float &drive_pwr, float &drive_current, float &drive_voltage, float &power_limit) {
  bool success = true;

  error_code     = TTP_read_int(S, TTP_ERR_CODE);
  success = success & (error_code != TTP_INT_READ_ERR); // set success false if we fail to read
  drive_freq     = TTP_read_int(S, TTP_DRV_FREQ);
  success = success & (error_code != TTP_INT_READ_ERR);
  drive_pwr      = TTP_read_float(S, TTP_DRV_PWR);
  success = success & (error_code != TTP_FLT_READ_ERR);
  drive_pwr      = TTP_read_float(S, TTP_DRV_PWR);
  success = success & (error_code != TTP_FLT_READ_ERR);
  drive_current  = TTP_read_float(S, TTP_DRV_CURRENT);
  success = success & (error_code != TTP_FLT_READ_ERR);
  drive_voltage  = TTP_read_float(S, TTP_DRV_VOLT);
  success = success & (error_code != TTP_FLT_READ_ERR);
  power_limit    = TTP_read_float(S, TTP_PWR_LIMIT);
  success = success & (error_code != TTP_FLT_READ_ERR);

  return success;
}
float   TTP_get_set_power(){
  return pwr;
}
