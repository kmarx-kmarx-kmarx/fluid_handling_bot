// pin defs
#define pin_manual_control_enable 24 // manual ctrl enable button
#define pin_pressure_vacuum 25       // pressure/vacuum selector switch
#define pin_analog_in A12            // potentiometer setpoint

// indicator lights
#define pin_LED_error 23
#define pin_LED_1 22

// Select which I2C bus to connect to
#define pin_sensor_select 15

// Valve pins
//   Valves are 0-indexed starting at the disc pump output and indexing clockwise through the fluid loop  
#define pin_valve_0 4 // Pneumatic valve. Right after the disc pump. High: Vent air out. Low: push air to valve 1
#define pin_valve_1 5 // Media isolation valve. High: connect pump to reservior (push air into reservior) Low: connect reservior to vacuum bottle
#define pin_valve_2 6 // Media isolation valve. High: Connect reservior to flow cell Low: connect reservior to rotary valve
#define pin_valve_3 7 // Pneumatic valve. High: Connect flow cell's vacuum bottle to the next valve Low: connect reservior's vacuum bottle to the next valve
#define pin_valve_4 8 // Pneumatic valve. High: Connect previous valve to the disc pump Low: connect ambient air to disc pump

// comms to the 33996
#define pin_33996_CS_0 10
#define pin_33996_PWM 41
#define pin_33996_nRST 40
