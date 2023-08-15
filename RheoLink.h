/* RheoLink.h
   Define RheoLink Class
   https://www.idex-hs.com/docs/default-source/product-manuals/rheolink-i2c-communication-protocol-for-titanex.pdf.zip
   Control and IDEX selector valve using I2C

   This class implements the folowing features:
        - Initialization
        - Set new I2C address
        - Set position (with direction of rotation)
        - Get current position
        - Report status

   Class Members:
    Variables:
        uint8_t address_ - I2C address of the device
        TwoWire *w_ - Pointer to Wire object
    Functions:
        RheoLink: set the private variables
        bool begin: Initialize the device. Return false if failed
        uint8_t send_command: Send a command to the selector valve and return an error
        uint8_t read_register: Read a register in the selector valve and return its value or an error

       Created on: 8/10/2023
         Author: Kevin Marx
*/

#include <Wire.h>
#include <Arduino.h>

// Commands
#define RheoLink_POS 'P'
#define RheoLink_CW  '-'
#define RheoLink_CCW '+'
#define RheoLink_NEW_ADDR 'N'
#define RheoLink_STATUS 'S'

#define DUMMY_DATA 'x'

class RheoLink {
  public:
    RheoLink(uint8_t address);
    uint8_t begin(TwoWire &w);
    uint8_t send_command(uint8_t cmd, uint8_t data = DUMMY_DATA);
    uint8_t read_register(uint8_t target);

  private:
    uint8_t address_;
    TwoWire *w_;
};
