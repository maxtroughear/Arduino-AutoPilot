#include "IO.h"

#include "../PinConfig.h"

#include <NeoHWSerial.h>

#define NO_PORTB_PINCHANGES
#define NO_PORTJ_PINCHANGES

#include <PinChangeInt/PinChangeInt.h>

// Define all variables

unsigned short IO::input_channels[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned short IO::final_channels[6] = { 0, 0, 0, 0, 0, 0 };

unsigned short IO::input_buffer[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned int IO::input_start[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
volatile unsigned short IO::input_shared[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
volatile unsigned long IO::input_last = 0;

unsigned short IO::input_channels_filter[8][IO_INPUT_FILTER_AMOUNT];
unsigned short IO::filterIndex = 0;
unsigned int IO::filterTotals[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int IO::input[8] = { IN_ROLL, IN_PITCH, IN_THROTTLE, IN_YAW, IN_AUX1, IN_AUX2, IN_AUX3, IN_AUX4 };
int IO::outputPin[6] = { OUT_ELEVATOR, OUT_AILERON1, OUT_AILERON2, OUT_RUDDER, OUT_MOTOR1, OUT_MOTOR2 };
Servo IO::output[6];

// Initialise the input/output pins and interrupts 

void IO::Initialise()
{
	for (int i = 0; i < 8; i++)
		pinMode(input[i], INPUT);

	for (int i = 0; i < 6; i++)
		output[i].attach(outputPin[i]);

	PCintPort::attachInterrupt(IN_ROLL, calcValues_roll, CHANGE);
	PCintPort::attachInterrupt(IN_PITCH, calcValues_pitch, CHANGE);
	PCintPort::attachInterrupt(IN_THROTTLE, calcValues_throttle, CHANGE);
	PCintPort::attachInterrupt(IN_YAW, calcValues_yaw, CHANGE);
	PCintPort::attachInterrupt(IN_AUX1, calcValues_aux1, CHANGE);
	PCintPort::attachInterrupt(IN_AUX2, calcValues_aux2, CHANGE);
	PCintPort::attachInterrupt(IN_AUX3, calcValues_aux3, CHANGE);
	PCintPort::attachInterrupt(IN_AUX4, calcValues_aux4, CHANGE);

	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < IO_INPUT_FILTER_AMOUNT; j++)
			input_channels_filter[i][j] = 0;
	}

	//Serial.println("IO Initialisation Done");
}

// Called at the start of loop, to retrieve input values and smooth the inputs
// Also monitors the inputs to detect loss of input to trigger the failsafe

void IO::LoopInput()
{
	noInterrupts();		// disables interrupts, so memory can safely be copied to non-volatile memory
	memcpy(input_buffer, (const void *)input_shared, sizeof(input_shared));
	interrupts();

	if ((millis() - input_last) > 2000)
	{
		// Failsafe code
		//Serial.println("Failsafe");
		input_channels[0] = 1500;
		input_channels[1] = 1500;
		input_channels[2] = 1500;
		input_channels[3] = 1500;
		// Eventually will change to circle over point or RTH then circle
	}
	else
	{
		// Smoothing/filtering code
		for (short i = 0; i < 8; i++)
		{
			filterTotals[i] -= input_channels_filter[i][filterIndex];

			unsigned short in = input_buffer[i];

			if (in != 0)
				input_channels_filter[i][filterIndex] = in;

			filterTotals[i] += input_channels_filter[i][filterIndex];

			input_channels[i] = filterTotals[i] / IO_INPUT_FILTER_AMOUNT;
		}
		filterIndex++;
		if (filterIndex >= IO_INPUT_FILTER_AMOUNT)
			filterIndex = 0;
	}
}

// Called at the end of loop, generates the outputs for the motor ESC and servo for steering

void IO::LoopOutput()
{
	for (short i = 0; i < 6; i++)
		output[i].writeMicroseconds(final_channels[i]);
}

// Used as part of the interrupt routine, to calculate the pulse length of individual signals

void IO::calcValues(unsigned int channel, unsigned int pin)
{
	input_last = millis();
	if (digitalRead(pin) == HIGH)
		input_start[channel] = micros();
	else
	{
		uint16_t rc_compare = (uint16_t)(micros() - input_start[channel]);
		input_shared[channel] = rc_compare;
	}
}

// Wrappers for the calcValues function
// The interrupts cannot be given parameters so wrappers must be used

void IO::calcValues_roll()
{
	calcValues(0, IN_ROLL);
}

void IO::calcValues_pitch()
{
	calcValues(1, IN_PITCH);
}

void IO::calcValues_throttle()
{
	calcValues(2, IN_THROTTLE);
}

void IO::calcValues_yaw()
{
	calcValues(3, IN_YAW);
}

void IO::calcValues_aux1()
{
	calcValues(4, IN_AUX1);
}

void IO::calcValues_aux2()
{
	calcValues(5, IN_AUX2);
}

void IO::calcValues_aux3()
{
	calcValues(6, IN_AUX3);
}

void IO::calcValues_aux4()
{
	calcValues(7, IN_AUX4);
}