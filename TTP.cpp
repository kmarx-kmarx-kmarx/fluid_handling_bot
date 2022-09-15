/*
    TTP.cpp
    This file contains functions for initializing and controlling the disc pump

    uint8_t TTP_init: Initialize the disc pump's power limit, signal source, mode, and whether it should stream data. Returns flags indicating whether the initialization succeeded.
      Arguments: HardwareSerial &S,  int16_t pwr_lim, uint8_t src, uint8_t mode, uint8_t stream
    uint8_t TTP_enable: Enable/disable the disc pump. Returns flags indicating whether the command succeeded.
      Arguments: HardwareSerial &S, bool en
    uint8_t TTP_set_target:  Set the pump's setpoint. Returns flags indicating whether the command succeeded.
      Arguments: HardwareSerial &S, float target
    uint8_t TTP_get_status:  Get error codes, drive frequency, drive power, drive current, drive voltage, and the power limit and saves them to shared variables. Returns flags indicating whether the command succeeded.
      Arguments: HardwareSerial &S, uint16_t &error_code, int16_t &drive_freq, float &dive_pwr, float &drive_current, float &drive_voltage, float &power_limit

    bool TTP_send_packet: Send the write buffer to the pump and populate the read the response into the read buffer. Also write the index on the response pointer showing when the pump's response begins
      Arguments: HardwareSerial &S, char *tx_buffer, char *rx_buffer, uint16_t &cmd_ptr
    int32_t TTP_read_int: Read an integer from a register in the disc pump
      Arguments: HardwareSerial &S, uint8_t reg
    float TTP_read_float: Read a float from a register in the disc pump
      Arguments: HardwareSerial &S, uint8_t reg
    bool TTP_write_register: Write a value to a register and return true if it was successfull. This function is overloaded
      Arguments: HardwareSerial &S, int32_t dat
      Arguments: HardwareSerial &S, float dat

*/
#include "TTP.h"

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_send_packet() sends a byte stream to the disc pump and reads the response back into a shared array.

  OPERATION: We write the full contents of the tx_buffer to the pump over serial and wait for a response. We then store the response to the rx_buffer and make sure the response was echoed properly. If the response times out or is not echoed properly, we try sending the message again. We try up to TTP_N_TRIES times.

  ARGUMENTS:
      Stream &S:       stream class to read from Serial, Serial1, etc.
      char *tx_buffer: pointer to array containing characters to send. This buffer is not changed
      char *rx_buffer: pointer to array containing recieved characters. This buffer is overwritten

  RETURNS:
      bool success:     Returns true if the read/write worked properly

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      char *tx_buffer: array with characters to send, not modified
      char *rx_buffer: array with received characters, overwritten

  GLOBAL VARIABLES: None

  DEPENDENCIES: Arduino.h
  -----------------------------------------------------------------------------
*/
bool TTP_send_packet(HardwareSerial &S, char *tx_buffer, char *rx_buffer) {
  uint16_t rx_ptr;
  uint32_t time_0;

  uint16_t cmd_ptr = strlen(tx_buffer);

  for (uint8_t i = 0; i < TTP_N_TRIES; i++) { // try writing multiple times
    rx_ptr = 0; // reset the rx pointer
    memset(rx_buffer, 0, strlen(rx_buffer)); // reset the read buffer
    S.clear(); // clear stream's RX buffer
    S.print(tx_buffer);
    S.flush(); // Wait for any transmitted data still in buffers to actually transmit

    // wait for a response
    time_0 = micros();
    while (!S.available() && (micros() - time_0) < TTP_READ_TIMEOUT_uS) {
      delayMicroseconds(5);
    }
    // if we have a response, process it. otherwise, fall through to the next for loop
    if (S.available()) {
      // read every byte into the buffer
      while (S.available()) {
        rx_buffer[rx_ptr++] = S.read();
      }
      // We are expecting the response to echo the request and have additional data if we are reading from one of the pump's registers
      // If expectation met, return out. Otherwise, loop again
      if ((cmd_ptr <= rx_ptr) && (strncmp(tx_buffer, rx_buffer, cmd_ptr) == 0)){
        return true;
      } 
    }
  }
  return false; // not receiving the sent command within 1 ms for 3 attempts
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TTP_init() initializes the disc pump with the power limit, source, mode, and stream settings.

  OPERATION:

  ARGUMENTS:
      Stream &S:       stream class to read from Serial, Serial1, etc.
      int16_t pwr_lim: Power limit in units milliwatts
      uint8_t src:     Flag indicating what the setpoint source should be, sent over serial, from a sensor, or from one of the analog pins
      uint8_t mode:    Flag indicating mode, manual, PID, or bang-bang
      uint8_t stream:  Flag indicating whether the pump controller should stream data

  RETURNS:
      uint8_t err:     sets bits if any of the writes to the controller failed

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
