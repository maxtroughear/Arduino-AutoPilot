#pragma once

// Configurable pins (not I2C, SPI and HWSerial)

// IO

#define IN_ROLL			A8
#define IN_PITCH		A9
#define IN_THROTTLE		A10
#define IN_YAW			A11
#define IN_AUX1			A12
#define IN_AUX2			A13
#define IN_AUX3			A14
#define IN_AUX4			A15

#define OUT_ELEVATOR	23
#define OUT_AILERON1	25
#define OUT_AILERON2	27
#define OUT_RUDDER		29
#define OUT_MOTOR1		31
#define OUT_MOTOR2		33

// LCD

#define TFT_INTERRUPT	48
#define TFT_RESET		49
#define TFT_SELECT		53