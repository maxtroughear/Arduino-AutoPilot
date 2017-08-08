#include "Logic.h"

#include "Manual/Manual.h"
#include "Stabilise/Stabilise.h"
#include "Navigate/Navigate.h"
#include "Hold/Hold.h"

#include "../GPS/GPS.h"
#include "../Sensors/Sensors.h"
#include "../IO/IO.h"

Logic::FlightMode Logic::Mode = Logic::MANUAL;
bool Logic::failsafe = false;
bool Logic::armed = false;

unsigned long Logic::timeSinceCal = 0;

NeoGPS::Location_t schoolClass(-36.89801493, 174.9029734);

void Logic::Initialise()
{
	Mode = MANUAL;

	// Wait for fix
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	bool ledOn = false;
	NeoSerial.println("Wait for GPS fix");
	while (GPS::GPSData.quality == 0)
	{
		digitalWrite(LED_BUILTIN, ledOn);
		ledOn = !ledOn;
		GPS::Loop();
		delay(500);
	}
	NeoSerial.println("GPS fix obtained");
	digitalWrite(LED_BUILTIN, LOW);
	delay(5000);
	digitalWrite(LED_BUILTIN, HIGH);
	// Set home location
	FlightModes::Navigate::SetHome(NeoGPS::Location_t(GPS::GPSData.location.lat, GPS::GPSData.location.lon));

	// Set ground altitude
	FlightModes::Navigate::SetGroundAltitude(Sensors::Altitude);

	FlightModes::Navigate::AddWaypoint(schoolClass);


	NeoSerial.println("Ready!");
}

void Logic::Loop()
{
	if (IO::input_channels[IO_IN_THROTTLE] < FLIGHT_STICK_ARM_MIN && IO::input_channels[IO_IN_YAW] < FLIGHT_STICK_ARM_MIN && armed)
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
		else if (IO::input_channels[IO_IN_THROTTLE] > FLIGHT_STICK_ARM_MAX && IO::input_channels[IO_IN_YAW] > FLIGHT_STICK_ARM_MAX)
		{
			if (millis() - timeSinceCal >= 5000)
			{
				// calibrate magnetometer
				NeoSerial.println("Move in figure 8 motion for 15 seconds");
				NeoSerial.println("Calibrate Compass in 4 seconds...");
				delay(4000);
				NeoSerial.println("Calibrating now!");
				Sensors::CalibrateMagnetometer();
				NeoSerial.println("Done");

				timeSinceCal = millis();
			}
		}
		else if (IO::input_channels[IO_IN_THROTTLE] > FLIGHT_STICK_ARM_MAX && IO::input_channels[IO_IN_YAW] < FLIGHT_STICK_ARM_MIN)
		{
			if (millis() - timeSinceCal >= 5000)
			{
				// calibrate accel/gyro
				NeoSerial.println("Calibrate Accelerometer and Gyroscope in 4 seconds");
				delay(4000);
				NeoSerial.println("Calibrating now!");
				Sensors::CalibrateAccelerometer();
				NeoSerial.println("Done");

				timeSinceCal = millis();
			}
		}
		else
		{
			// update altitude ground to account for drift and update home position
			FlightModes::Navigate::SetHome(NeoGPS::Location_t(GPS::GPSData.location.lat, GPS::GPSData.location.lon));
			FlightModes::Navigate::SetGroundAltitude(Sensors::Altitude);
			IO::final_channels[IO_OUT_MOTOR1] = 900;
			IO::final_channels[IO_OUT_MOTOR2] = 900;
			IO::final_channels[IO_OUT_AILERON1] = IO_CENTRE_AILERON;
			IO::final_channels[IO_OUT_AILERON2] = IO_CENTRE_AILERON;
			IO::final_channels[IO_OUT_ELEVATOR] = IO_CENTRE_ELEVATOR;
			IO::final_channels[IO_OUT_RUDDER] = IO_CENTRE_RUDDER;
		}
		return;
	}

	if (IO::Failsafe())
	{
		// Circle
		//FlightModes::Hold::Loop();
		FlightModes::Navigate::ReturnToHome();
		NeoSerial.println("Failsafe");
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
	else if (Mode == RTH)
		FlightModes::Navigate::ReturnToHome();
	else if (Mode = HOLD)
		FlightModes::Hold::Loop();
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
	else if (Mode == HOLD)
		FlightModes::Hold::Start();

	NeoSerial.print("Mode changed: "); NeoSerial.println(newMode);
}

void Logic::SetFailsafeEnabled(bool enabled)
{

}

bool Logic::GetFailsafeEnabled()
{
	
}