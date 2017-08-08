#include "Stabilise.h"

#include <NeoHWSerial.h>

#include "../../Config.h"
#include "../../Sensors/Sensors.h"
#include "../../IO/IO.h"

using namespace FlightModes;

float Stabilise::targetRoll = 0;
float Stabilise::targetPitch = 0;
float Stabilise::targetYaw = 0;

int Stabilise::timeGainRoll = 0;
int Stabilise::timeGainPitch = 0;
int Stabilise::timeGainYaw = 0;

bool Stabilise::stabilisingRoll = false;
bool Stabilise::stabilisingPitch = false;
bool Stabilise::stabilisingYaw = false;

unsigned long deltaStableT = 0;

void Stabilise::Start()
{
	//
}

void Stabilise::Loop()
{
	// check if sticks are in a neutral position
	
	// pitch and roll
	
	if (IO::input_channels[IO_IN_ROLL] >= IO_CENTRE_AILERON - FLIGHT_STABILISE_DEADZONE && IO::input_channels[IO_IN_ROLL] <= IO_CENTRE_AILERON + FLIGHT_STABILISE_DEADZONE)
	{
		if (!stabilisingRoll)
		{
			targetRoll = Sensors::MotionData.AHRS.roll;
			timeGainRoll = 0;
			NeoSerial.print("Stabilising Roll "); NeoSerial.println(targetRoll);
		}
		stabilisingRoll = true;
	}
	else
		stabilisingRoll = false;

	if (IO::input_channels[IO_IN_PITCH] >= IO_CENTRE_ELEVATOR - FLIGHT_STABILISE_DEADZONE && IO::input_channels[IO_IN_PITCH] <= IO_CENTRE_ELEVATOR + FLIGHT_STABILISE_DEADZONE)
	{
		if (!stabilisingPitch)
		{
			targetPitch = Sensors::MotionData.AHRS.pitch;
			timeGainPitch = 0;
			NeoSerial.print("Stabilising Pitch "); NeoSerial.println(targetPitch);
		}
		stabilisingPitch = true;
	}
	else
		stabilisingPitch = false;

	if (IO::input_channels[IO_IN_YAW] >= IO_CENTRE_RUDDER - FLIGHT_STABILISE_DEADZONE && IO::input_channels[IO_IN_YAW] <= IO_CENTRE_RUDDER + FLIGHT_STABILISE_DEADZONE)
	{
		if (!stabilisingYaw)
		{
			targetYaw = Sensors::MotionData.AHRS.yaw;
			timeGainYaw = 0;
			NeoSerial.print("Stabilising Yaw "); NeoSerial.println(targetYaw);
		}
		stabilisingYaw = true;
	}
	else
		stabilisingYaw = false;

	if (stabilisingRoll)
	{
		float deltaRoll = targetRoll - Sensors::MotionData.AHRS.roll;

		timeGainRoll += abs(deltaRoll);
		timeGainRoll /= FLIGHT_STABILISE_TIME;

		deltaRoll *= FLIGHT_STABILISE_GAIN;

		/*if (deltaRoll > 0)
			deltaRoll += timeGainRoll;
		else
			deltaRoll -= timeGainRoll;*/

		unsigned short channelOutput = constrain(IO_CENTRE_AILERON + deltaRoll, IO_MIN_AILERON, IO_MAX_AILERON);

		IO::final_channels[IO_OUT_AILERON1] = channelOutput;
		IO::final_channels[IO_OUT_AILERON2] = channelOutput;
	}
	else
	{
		IO::final_channels[IO_OUT_AILERON1] = map(IO::input_channels[IO_IN_ROLL], FLIGHT_STICK_MIN, FLIGHT_STICK_MAX, IO_MIN_AILERON, IO_MAX_AILERON);
		IO::final_channels[IO_OUT_AILERON2] = map(IO::input_channels[IO_IN_ROLL], FLIGHT_STICK_MIN, FLIGHT_STICK_MAX, IO_MIN_AILERON, IO_MAX_AILERON);
	}
	if (stabilisingPitch)
	{
		float deltaPitch = targetPitch - Sensors::MotionData.AHRS.pitch;

		timeGainPitch += abs(deltaPitch);
		timeGainPitch /= FLIGHT_STABILISE_TIME;

		deltaPitch *= FLIGHT_STABILISE_GAIN;

		/*if (deltaPitch > 0)
			deltaPitch += timeGainPitch;
		else
			deltaPitch -= timeGainPitch;*/

		unsigned short channelOutput = constrain(IO_CENTRE_ELEVATOR - deltaPitch, IO_MIN_ELEVATOR, IO_MAX_ELEVATOR);

		IO::final_channels[IO_OUT_ELEVATOR] = channelOutput;
	}
	else
	{
		IO::final_channels[IO_OUT_ELEVATOR] = map(IO::input_channels[IO_IN_PITCH], FLIGHT_STICK_MIN, FLIGHT_STICK_MAX, IO_MIN_ELEVATOR, IO_MAX_ELEVATOR);
	}
	if (stabilisingYaw)
	{
		float deltaYaw = targetYaw - Sensors::MotionData.AHRS.yaw;

		if (deltaYaw > 180)
			deltaYaw -= 360;
		else if (deltaYaw < -180)
			deltaYaw += 360;

		timeGainYaw += abs(deltaYaw);
		timeGainYaw /= FLIGHT_STABILISE_TIME_YAW;

		deltaYaw *= FLIGHT_STABILISE_GAIN_YAW;

		if (deltaYaw > 0)
			deltaYaw += timeGainYaw;
		else
			deltaYaw -= timeGainYaw;

		unsigned short channelOutput = constrain(IO_CENTRE_RUDDER + deltaYaw, IO_MIN_RUDDER, IO_MAX_RUDDER);

		IO::final_channels[IO_OUT_RUDDER] = channelOutput;
	}
	else
	{
		IO::final_channels[IO_OUT_RUDDER] = map(IO::input_channels[IO_IN_YAW], FLIGHT_STICK_MIN, FLIGHT_STICK_MAX, IO_MIN_RUDDER, IO_MAX_RUDDER);
	}

	IO::final_channels[IO_OUT_MOTOR1] = IO::input_channels[IO_IN_THROTTLE];
	IO::final_channels[IO_OUT_MOTOR2] = IO::input_channels[IO_IN_THROTTLE];

	if (millis() - deltaStableT >= 100)
	{
		/*for (int i = 0; i < 6; i++)
		{
			NeoSerial.print(IO::final_channels[i]);
			NeoSerial.print("\t");
		}*/

		NeoSerial.print(Sensors::MotionData.AHRS.pitch);
		NeoSerial.print(",");
		NeoSerial.print(Sensors::MotionData.AHRS.roll);
		NeoSerial.print(",");
		NeoSerial.print(Sensors::MotionData.AHRS.yaw);

		NeoSerial.println();

		deltaStableT = millis();
	}
}