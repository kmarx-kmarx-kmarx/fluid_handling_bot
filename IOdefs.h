#ifndef _IODEFS_H_
#define _IODEFS_H_

// pin defs
#define pin_manual_control_enable 24 // manual ctrl enable button
#define pin_pressure_vacuum 25       // pressure/vacuum selector switch
#define pin_analog_in A12            // potentiometer setpoint
// analog control defs
#define analog_deadzone 23           // ignore analog values below the dead zone
#define ANALOG_MAX      1023

// indicator lights
#define pin_LED_error 23
#define pin_LED_1 22

// Select which I2C bus to connect to
#define pin_sensor_select 15

// Valve pins
//   Valves are 0-indexed starting at the disc pump output and indexing clockwise through the fluid loop  
#define pin_valve_0 4 // Pneumatic valve. Right after the disc pump. High: Vent air out. Low: push air to valve 1
#define pin_valve_1 5 // Media isolation valve. High: connect pump to reservior (push air into reservior) Low: connect reservior to valve 5
#define pin_valve_2 6 // Media isolation valve. High: Connect reservior to flow cell Low: connect reservior to rotary valve
#define pin_valve_3 7 // Pneumatic valve. High: Connect flow cell's vacuum bottle (VB 1) to the next valve Low: connect reservior's vacuum bottle (VB 0) to the next valve
#define pin_valve_4 8 // Pneumatic valve. High: Connect previous valve to the disc pump Low: connect ambient air to disc pump
#define pin_valve_5 9 // Media isolation valve. High: vent air into Vacuum Bottle 0 Low: Connect VB1 to the fluid reservior


// I2C bus select
#define PIN_SENSOR_SELECT 15
#define SELECT_SENSOR_0   LOW
#define SELECT_SENSOR_1   HIGH


#endif /* _IODEFS_H_ */
