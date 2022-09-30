
#ifndef NXP33996_H_
#define NXP33996_H_

#include "Arduino.h"
#include <SPI.h>

bool NXP33996_init(uint8_t CS_pin, uint8_t PWM_pin, uint8_t rRST_pin);
void NXP33996_clear_all(uint8_t CS_pin);
void NXP33996_turn_on(uint8_t CS_pin, uint16_t id);
void NXP33996_turn_off(uint8_t CS_pin, uint16_t id);
uint16_t NXP33996_get_state(uint8_t CS_pin);

#endif /* NXP33996_H_ */
