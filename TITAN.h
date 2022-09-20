/*
   TITAN.h
   Contains utilities for running the Titan selector valve.

    Created on: 9/20/2022
        Author: Kevin Marx
*/


#ifndef TITAN_H_
#define TITAN_H_

#include <stddef.h>
#include <stdint.h>
#include <Arduino.h>

// Constants for operation
// Constants for communication
#define TITAN_BAUDRATE       19200  // Selector valve baud rate
#define TITAN_BUFFER_SIZE       32  // 32 byte read/write buffer
#define TITAN_N_TRIES            3  // Try reading/writing 3 times
#define TITAN_READ_TIMEOUT_uS 10000 // wait this many microseconds before timing out
#define TITAN_MOVE_TIMEOUT_mS  5000 // wait this many milliseconds before timing out

// Function headers
bool    TITAN_init(HardwareSerial &S);

#endif /* TITAN_H_ */
