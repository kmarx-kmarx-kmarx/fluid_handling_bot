/*
    TITAN.cpp
    This file contains functions for initializing and controlling the selector valve

    bool TITAN_init: Initialize the serial communication with the selector valve
      Arguments: HardwareSerial &S

*/
#include "TITAN.h"

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: TITAN_init() initializes the selector valve serial communications.

  OPERATION: We initialize serial. If we time out, return false. Otherwise, return true.

  ARGUMENTS:
      HardwareSerial &S: stream class to read from Serial, Serial1, etc.

  RETURNS:
      bool success:      return true if initialization was successful

  INPUTS / OUTPUTS: The Serial lines are used as inputs and outputs to transmit data

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      HardwareSerial &S: Serial class

  GLOBAL VARIABLES: None

  DEPENDENCIES: Arduino.h
  -----------------------------------------------------------------------------
*/
bool TITAN_init(HardwareSerial &S) {
  int16_t time_0;
  bool success = true;
  S.begin(TITAN_BAUDRATE);

  // wait for the serial port to open
  time_0 = micros();
  while (!S && (micros() - time_0) < TITAN_READ_TIMEOUT_uS) {
    delayMicroseconds(10);
  }
  // time out
  if ((micros() - time_0) > TITAN_READ_TIMEOUT_uS) {
    success = false;
  }

  return success;
}


bool write_selector_valve_read_command(HardwareSerial &S, char* cmd_str)
{
  uint16_t idx;
  for (int i = 0; i < TITAN_N_TRIES; i++) // attempt 3 times
  {
    // empty the UART buffer
    while (S.available())
      S.read();

    S.print(cmd_str);
    S.flush(); // Wait for any transmitted data still in buffers to actually transmit
    uint16_t time_elapsed_us = micros();
    uart_titan_rx_ptr = 0;
    while ( micros()-time_elapsed_us < TITAN_READ_TIMEOUT_uS) // timeout 
    {
      while (S.available()){
        idx = min(TITAN_BUFFER_SIZE-1, uart_titan_rx_ptr++);
        uart_titan_rx_buffer[idx] = S.read();
      }
      if (uart_titan_rx_ptr > 0 && uart_titan_rx_buffer[uart_titan_rx_ptr - 1] == '\r')
        return true;
    }
  }

  return false;
}

bool write_selector_valve_move_command(HardwareSerial &S, char* cmd_str)
{
  for (int i = 0; i < TITAN_N_TRIES; i++) 
  {

    // empty the UART buffer
    while (S.available())
      S.read();

    S.print(cmd_str);
    S.flush(); // Wait for any transmitted data still in buffers to actually transmit
    elapsedMillis time_elapsed_ms;
    uart_titan_rx_ptr = 0;
    while ( time_elapsed_ms < 5000) // timeout after 5 second if the '/r' is not returned
    {
      while (S.available())
        uart_titan_rx_buffer[uart_titan_rx_ptr++] = S.read();
      if (uart_titan_rx_ptr > 0 && uart_titan_rx_buffer[uart_titan_rx_ptr - 1] == '\r') // can change to just read up to one byte
        return true;
    }
  }
  return false;
}

bool set_selector_valve_position(HardwareSerial &S, int pos)
{
  char cmd_str[TITAN_BUFFER_SIZE];
  sprintf(cmd_str, "P%02X\r", pos);
  return write_selector_valve_move_command(S, cmd_str);
}

bool check_selector_valve_position(HardwareSerial &S)
{
  // During the valve motion profile, driver board will not accept any commands
  // and will respond to any incoming data with ‘*’ [0x2A].
  char cmd_str[TITAN_BUFFER_SIZE];
  sprintf(cmd_str, "S\r");
  return write_selector_valve_read_command(S, cmd_str);
  // false will be returned during motion (the command will be sent three times and '*' will be returned)
  // when true is returned, the valve position is in the rx buffer
}

bool set_selector_valve_position_blocking(HardwareSerial &S, int pos)
{
  bool command_sent = set_selector_valve_position(pos);
  if (command_sent == false)
    return false; // in the future can return an error code

  while (check_selector_valve_position(S) == false)
  {
    delay(1);
  }
  return true;
}
