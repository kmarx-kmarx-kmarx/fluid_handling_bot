#include "NXP33996.h"

// shared variable
uint16_t local_state = 0;

bool NXP33996_init(uint8_t CS_pin, uint8_t PWM_pin, uint8_t rRST_pin) {
  pinMode(CS_pin, OUTPUT);
  pinMode(PWM_pin, OUTPUT);
  pinMode(rRST_pin, OUTPUT);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);
  digitalWrite(rRST_pin, HIGH);

  return true;
}

uint16_t NXP33996_transfer(uint8_t CS_pin, uint16_t state) {
  uint16_t result;
  digitalWrite(CS_pin, LOW);
  SPI.transfer(0x00);
  result = SPI.transfer16(state); //16 output bits
  digitalWrite(CS_pin, HIGH);

  return result;
}

void NXP33996_clear_all(uint8_t CS_pin) {
  local_state = 0;
  NXP33996_transfer(CS_pin, local_state);
}

void NXP33996_turn_on(uint8_t CS_pin, uint16_t id) {
  local_state |= (uint16_t)0x0001 << id;
  local_state = NXP33996_transfer(CS_pin, local_state);
  local_state |= (uint16_t)0x0001 << id;
  return;
}

void NXP33996_turn_off(uint8_t CS_pin, uint16_t id) {
  local_state &= ~((uint16_t)0x0001 << id);
  local_state = NXP33996_transfer(CS_pin, local_state);
  local_state &= ~((uint16_t)0x0001 << id);
  return;

}

uint16_t NXP33996_get_state(uint8_t CS_pin) {
  local_state = NXP33996_transfer(CS_pin, local_state);

  NXP33996_transfer(CS_pin, local_state);

  return local_state;
}
