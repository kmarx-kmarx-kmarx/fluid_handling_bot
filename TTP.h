/*
   TTP.h
   Contains utilities for running the TTP disc pump.

    Created on: 9/15/2022
        Author: Kevin Marx
*/

#ifndef TTP_H_
#define TTP_H_

#include <stddef.h>
#include <stdint.h>
#include <Arduino.h>

// Constants for operation
// Register map
#define TTP_ERR_CODE        31
#define TTP_SET_VALUE       23
#define TTP_MANUAL_SRC      11
#define TTP_CTRL_MODE       10
#define TTP_DRV_FREQ         6
#define TTP_DRV_PWR          5
#define TTP_DRV_CURRENT      4
#define TTP_DRV_VOLT         3
#define TTP_STREAM_MODE      2
#define TTP_PWR_LIMIT        1
#define TTP_ENABLED          0
// Fields
#define TTP_SRC_SETVAL       0
#define TTP_MODE_MANUAL      0
#define TTP_STREAM_DISABLE   0
#define TTP_PUMP_ENABLE      1
#define TTP_PUMP_DISABLE     0

#define TTP_MIN_PWR          0
#define TTP_MAX_PWR       1400

#define TTP_BUFFER_SIZE       32 // 32 byte read/write buffer
#define TTP_N_TRIES            3 // Try reading/writing 3 times
#define TTP_READ_TIMEOUT_uS 1000 // wait this many microseconds before timing out

#define TTP_FLOAT_SIG          8 // number of digits past the decimal to send

// Function headers
uint8_t TTP_init(HardwareSerial &S, int16_t pwr_lim, uint8_t src, uint8_t mode, uint8_t stream);
uint8_t TTP_enable(HardwareSerial &S, bool en);
uint8_t TTP_set_pwr_limit(HardwareSerial &S, int16_t limit);
uint8_t TTP_set_target(HardwareSerial &S, float target);
uint8_t TTP_get_status(HardwareSerial &S, uint16_t &error_code, int16_t &drive_freq, float &dive_pwr, float &drive_current, float &drive_voltage, float &power_limit);

#endif /* TTP_H_ */
