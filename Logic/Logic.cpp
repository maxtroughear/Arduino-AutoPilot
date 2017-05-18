#include "Logic.h"

#include "Manual/Manual.h"
#include "Stabilise/Stabilise.h"
#include "Navigate/Navigate.h"

#include "../GPS/GPS.h"
#include "../Sensors/Sensors.h"
#include "../IO/IO.h"

Logic::FlightMode Logic::Mode = Logic::MANUAL;
bool Logic::failsafe = false;
bool Logic::armed = false;

NeoGPS::Location_t schoolClass(-36.89801493, 174.9029734);

void Logic::Initialise()
{
	Mode = MANUAL;

	// Wait for fix
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	bool ledOn = false;
	while (GPS::GPSData.quality == 0)
	{
		digitalWrite(LED_BUILTIN, ledOn);
		ledOn = !ledOn;
		GPS::Loop();
		delay(500);
	}
	digitalWrite(LED_BUILTIN, LOW);
	delay(5000);
	digitalWrite(LED_BUILTIN, HIGH);
	// Set home location
	FlightModes::Navigate::SetHome(NeoGPS::Location_t(GPS::GPSData.location.lat, GPS::GPSData.location.lon));

	// Set ground altitude
	FlightModes::Navigate::SetGroundAltitude(Sensors::Altitude);

	FlightModes::Navigate::AddWaypoint(schoolClass);
}

void Logic::Loop()
{
	if (IO::input_channels[IO_IN_THROTTLE] < FLIGHT_STICK_ARM_MIN && IO::input_channels[IO_IN_YAW] < FLIGHT_STICK_ARM_MIN)
	{
		if (abs(GPS::GPSData.speed) < 3)	// prevent disarming while flying
		{
			armed = false;
			NeoSerial.println("Disarmed");
		}
	}

	if (!armed)
	{
		if (IO::input_channels[IO_IN_THROTTLE] < FLIGHT_STICK_ARM_MIN && IO::input_channels[IO_IN_YAW] > FLIGHT_STICK_ARM_MAX)
		{
			if (abs(Sensors::MotionData.AHRS.pitch) < 30 && abs(Sensors::MotionData.AHRS.roll) < 30 && abs(GPS::GPSData.speed) < 3)
			{
				armed = true;
				NeoSerial.println("Armed");
			}
		}
		else
		{
			// update altitude ground to account for drift and update home position
			FlightModes::Navigate::SetHome(NeoGPS::Location_t(GPS::GPSData.location.lat, GPS::GPSData.location.lon));
			FlightModes::Navigate::SetGroundAltitude(Sensors::Altitude);
		}
		return;
	}

	if (failsafe)
	{
		// Circle
		//FlightModes::Hold::Loop();
		return;
	}

	// check for mode change using IO
	if (IO::input_channels[IO_IN_AUX1] < 1500)
		SetMode(MANUAL);
	else
	{
		if (IO::input_channels[IO_IN_AUX2] < 1200)
			SetMode(STABILISE);
		else if (IO::input_channels[IO_IN_AUX2] >= 1250 && IO::input_channels[IO_IN_AUX2] < 1750)
			SetMode(NAVIGATE);
		else
			SetMode(HOLD);
	}

	if (Mode == MANUAL)
		FlightModes::Manual::Loop();
	else if (Mode == STABILISE)
		FlightModes::Stabilise::Loop();
	else if (Mode == NAVIGATE)
		FlightModes::Navigate::Loop();
}

Logic::FlightMode Logic::GetMode()
{
	return Mode;
}

void Logic::SetMode(FlightMode newMode)
{
	if (Mode == newMode)
		return;

	Mode = newMode;

	if (Mode == MANUAL)
		FlightModes::Manual::Start();
	else if (Mode == STABILISE)
		FlightModes::Stabilise::Start();
	else if (Mode == NAVIGATE)
		FlightModes::Navigate::Start();

	NeoSerial.print("Mode changed: "); NeoSerial.println(newMode);
}

void Logic::SetFailsafeEnabled(bool enabled)
{

}

bool Logic::GetFailsafeEnabled()
{
	
}