#include "SerialCommUtils.h"

uint16_t current_command_uid = 0;
uint8_t current_command = 0;

void send_serial_data(float ttp_max_pwr, float VOLUME_UL_MAX, uint8_t command_execution_status, uint8_t internal_program, uint8_t fluids, uint8_t valveset, uint16_t NXP33996_state, int16_t pressure_0_raw, int16_t pressure_1_raw, int16_t flowrate_0_raw, uint8_t time_elapsed_s, float volume_ul, uint8_t selectorset, float disc_pump_power, int16_t iir0, int16_t iir1) {
  /*
    byte 0-1  : computer -> MCU CMD counter (UID)
    byte 2    : cmd from host computer (error checking through check sum => no need to transmit back the parameters associated with the command)
    byte 3    : status of the command
    byte 4    : MCU internal program being executed
    byte 5    : state of valve x,x,x,x,bubble_sensor_1,bubble_sensor_2,x,x
    byte 6    : state of valve C1-C7, manual input bit
    byte 7-8  : state of valve D1-D16
    byte 9    : state of selector valve
    byte 10-11  : pump power
    byte 12-13  : pressure sensor 1 reading (vacuum)
    byte 14-15  : pressure sensor 2 reading (pressure)
    byte 16-17  : flow sensor 1 reading (downstram)
    byte 18-19  : reserved
    byte 20     : elapsed time since the start of the last internal program (in seconds)
    byte 21-22  : volume (ul), range: 0 - 5000
    byte 23-24  : reserved
  */
  byte buffer_tx[FROM_MCU_MSG_LENGTH];
  buffer_tx[0] = byte(current_command_uid >> 8);
  buffer_tx[1] = byte(current_command_uid & 0xFF);

  buffer_tx[2] = current_command;
  buffer_tx[3] = command_execution_status;
  buffer_tx[4] = internal_program;

  buffer_tx[5] = fluids;

  buffer_tx[6] =  byte(valveset);
  buffer_tx[7] =  byte(NXP33996_state >> 8);
  buffer_tx[8] =  byte(NXP33996_state & 0xFF);
  buffer_tx[9] =  byte(selectorset);
  buffer_tx[10] = byte(int(disc_pump_power / ttp_max_pwr * UINT16_MAX) >> 8);
  buffer_tx[11] = byte(int(disc_pump_power / ttp_max_pwr * UINT16_MAX) & 0xFF);
  buffer_tx[12] = byte(pressure_0_raw >> 8); // vacuum
  buffer_tx[13] = byte(pressure_0_raw & 0xFF); // vacuum
  buffer_tx[14] = byte(pressure_1_raw >> 8); // pressure
  buffer_tx[15] = byte(pressure_1_raw & 0xFF ); // pressure
  buffer_tx[16] = byte(flowrate_0_raw >> 8);
  buffer_tx[17] = byte(flowrate_0_raw & 0xFF);
  buffer_tx[18] = 0; // We don't have a second flow meter
  buffer_tx[19] = 0;
  buffer_tx[20] = byte(time_elapsed_s);
  int16_t volume_ul_int16 = (volume_ul / VOLUME_UL_MAX) * INT16_MAX;
  buffer_tx[21] = byte(volume_ul_int16 >> 8);
  buffer_tx[22] = byte(volume_ul_int16 & 0xFF);

  // IIR Filtered Pressure
  buffer_tx[18] = byte(iir0 >> 8); // vac
  buffer_tx[19] = byte(iir0 & 0xFF ); // vac
  buffer_tx[23] = byte(iir1 >> 8);
  buffer_tx[24] = byte(iir1 & 0xFF);
  SerialUSB.write(buffer_tx, FROM_MCU_MSG_LENGTH);
  
  return;
}

bool read_serial_command(byte payloads[TO_MCU_CMD_LENGTH - 3], uint8_t &cmd) {
  byte buffer_rx[3];

  uint16_t idx = 0;
  if (Serial.available()) {
    // Start reading in the rest of the command
    while (Serial.available() > 0 && idx < TO_MCU_CMD_LENGTH) {
      if (idx < 3) {
        buffer_rx[idx++] = Serial.read();
      }
      else {
        payloads[(idx++ - 3)] = Serial.read();
      }
    }

    current_command_uid = uint16_t(buffer_rx[0]) << 8 + uint16_t(buffer_rx[1]);
    current_command = buffer_rx[2];
    cmd = current_command; // copy data into shared variable

    // Handle the CLEAR command
    if (current_command == CLEAR) {
      current_command_uid = 0;
      return true;
    }

    // Verify we got enough bytes to process the other commands - return if we didn't fill the buffer
    if (idx < TO_MCU_CMD_LENGTH) {
      return false;
    }
    // payloads are loaded into shared array
    return true;
  }
  else {
    for (idx = 0; idx < TO_MCU_CMD_LENGTH; idx++) {
      if (idx < 3) {
        buffer_rx[idx++] = 0;
      }
      else {
        payloads[idx++] = 0;
      }
    }
    return false;
  }
}
