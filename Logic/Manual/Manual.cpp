#include "Manual.h"

#include <NeoHWSerial.h>

#include "../../IO/IO.h"

using namespace FlightModes;

void Manual::Start()
{
	//
}

unsigned long deltaManualT = 0;

void Manual::Loop()
{
	IO::final_channels[IO_OUT_AILERON1] = map(IO::input_channels[IO_IN_ROLL], FLIGHT_STICK_MIN, FLIGHT_STICK_MAX, IO_MIN_AILERON, IO_MAX_AILERON);
	IO::final_channels[IO_OUT_AILERON2] = map(IO::input_channels[IO_IN_ROLL], FLIGHT_STICK_MIN, FLIGHT_STICK_MAX, IO_MIN_AILERON, IO_MAX_AILERON);
	IO::final_channels[IO_OUT_ELEVATOR] = map(IO::input_channels[IO_IN_PITCH], FLIGHT_STICK_MIN, FLIGHT_STICK_MAX, IO_MIN_ELEVATOR, IO_MAX_ELEVATOR);
	IO::final_channels[IO_OUT_RUDDER] = map(IO::input_channels[IO_IN_YAW], FLIGHT_STICK_MIN, FLIGHT_STICK_MAX, IO_MIN_RUDDER, IO_MAX_RUDDER);

	// make option for differential thrust using yaw
	IO::final_channels[IO_OUT_MOTOR1] = IO::input_channels[IO_IN_THROTTLE];
	IO::final_channels[IO_OUT_MOTOR2] = IO::input_channels[IO_IN_THROTTLE];

	/*if (millis() - deltaManualT >= 100)
	{

		for (int i = 0; i < 8; i++)
		{
			NeoSerial.print(IO::input_channels[i]);
			NeoSerial.print("\t");
		}

		NeoSerial.println();

		for (int i = 0; i < 6; i++)
		{
			NeoSerial.print(IO::final_channels[i]);
			NeoSerial.print("\t");
		}

		NeoSerial.println();
		NeoSerial.println();

		deltaManualT = millis();
	}*/
}