#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "../Config.h"

#define IO_IN_ROLL 0
#define IO_IN_PITCH 1
#define IO_IN_THROTTLE 2
#define IO_IN_YAW 3
#define IO_IN_AUX1 4
#define IO_IN_AUX2 5
#define IO_IN_AUX3 6
#define IO_IN_AUX4 7

#define IO_OUT_ELEVATOR 0
#define IO_OUT_AILERON1 1
#define IO_OUT_AILERON2 2
#define IO_OUT_RUDDER 3
#define IO_OUT_MOTOR1 4
#define IO_OUT_MOTOR2 5

class IO
{
public:
	static unsigned short input_channels[8];
	static unsigned short final_channels[6];

	static void Initialise();
	static void LoopInput();
	static void LoopOutput();

private:
	static void calcValues(unsigned int channel, unsigned int pin);

	static void calcValues_pitch();
	static void calcValues_roll();
	static void calcValues_throttle();
	static void calcValues_yaw();
	static void calcValues_aux1();
	static void calcValues_aux2();
	static void calcValues_aux3();
	static void calcValues_aux4();

	static unsigned short input_buffer[8];
	static unsigned short input_channels_filter[8][IO_INPUT_FILTER_AMOUNT];
	static unsigned short filterIndex;
	static unsigned int filterTotals[8];
	
	static unsigned int input_start[8];
	static volatile unsigned short input_shared[8];
	static volatile unsigned long input_last;

	static int input[8];
	static int outputPin[6];
	static Servo output[6];
};
	