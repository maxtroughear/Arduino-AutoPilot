#include "Navigate.h"

#include "../../Sensors/Sensors.h"
#include "../../IO/IO.h"

using namespace FlightModes;

NeoGPS::Location_t Navigate::home;
NeoGPS::Location_t Navigate::waypoints[FLIGHT_NAVIGATE_MAX_WAYPOINTS];
unsigned int Navigate::waypointCount = 0;
unsigned int Navigate::targetWaypoint = 0;

unsigned short Navigate::throttle = 1000;

float Navigate::groundAltitude = -1;

float Navigate::targetRoll, Navigate::targetPitch, Navigate::targetYaw;

unsigned long deltaNavT = 0;

void Navigate::Start()
{
	if (waypointCount == 0)
		NeoSerial.println("No waypoints for navigation flight mode! Will RTH when mode is selected");
}

void Navigate::Loop()
{
	if (waypointCount == 0)
		navigateTo(home);
	else
		navigateTo(waypoints[targetWaypoint]);
}

void Navigate::ReturnToHome()
{
	navigateTo(home);
}

void Navigate::SetHome(NeoGPS::Location_t home)
{
	Navigate::home = home;
}

void Navigate::SetGroundAltitude(float altitude)
{
	groundAltitude = altitude;
}

void Navigate::AddWaypoint(NeoGPS::Location_t point)
{
	if (waypointCount < FLIGHT_NAVIGATE_MAX_WAYPOINTS)
		waypoints[waypointCount++] = point;
	else
		NeoSerial.println("Unable to add waypoint to navigation flight mode, FLIGHT_NAVIGATE_MAX_WAYPOINTS reached");
}

void Navigate::navigateTo(NeoGPS::Location_t &point)
{
	float targetDistance = GPS::DistanceTo(point);

	// altitude checks
	if (Sensors::Altitude < groundAltitude + FLIGHT_NAVIGATE_ALTITUDE_MIN)
	{
		// too low
		// pitch should be increased dramatically. Should be rather extreme (also consider forcing thrust increase)
		targetPitch = FLIGHT_NAVIGATE_PITCH_MAX;
	}
	else if (Sensors::Altitude > groundAltitude + FLIGHT_NAVIGATE_ALTITUDE_MAX)
	{
		// too high
		// pitch should be decreased somewhat. Not extreme
		targetPitch = -FLIGHT_NAVIGATE_PITCH_TARGET;
	}
	else
	{
		float deltaAltitude = (groundAltitude + FLIGHT_NAVIGATE_ALTITUDE_TARGET) - Sensors::Altitude;
		// altitude okay, aim for target altitude
		targetPitch = atan(deltaAltitude / targetDistance) * 180 / PI;
	}

	// calculate target AHRS

	// calculate yaw
	
	float targetBearing = GPS::BearingTo(point) - 180.f;

	float gpsBearing = GPS::GPSData.heading;
	//if (gpsBearing < 0) // convert to 0 - 360 format
	//	gpsBearing += 360;

	float ahrsBearing = Sensors::MotionData.AHRS.yaw;
	//if (ahrsBearing < 0) // convert to 0 - 360 format
	//	ahrsBearing += 360;

	// calculate potential heading error
	float errorBearing = (ahrsBearing - gpsBearing) / gpsBearing;	// decimal
	
	float currentBearing;

	if (errorBearing >= 0.2 || GPS::GPSData.speed < 3)
	{
		// more than 20% error
		// warning
		currentBearing = ahrsBearing;
	}
	else
	{
		// merge GPS and sensor bearings
		currentBearing = (gpsBearing + ahrsBearing) / 2.f;
	}

	targetYaw = targetBearing - currentBearing;

	

	targetRoll = targetYaw * FLIGHT_NAVIGATE_ROLL_GAIN;
	targetRoll = constrain(targetRoll, -FLIGHT_NAVIGATE_ROLL_MAX, FLIGHT_NAVIGATE_ROLL_MAX);

	float rollAmount = targetRoll - Sensors::MotionData.AHRS.roll;

	if (rollAmount > 180)
		rollAmount -= 360;
	else if (rollAmount < -180)
		rollAmount += 360;

	targetPitch = targetPitch * FLIGHT_NAVIGATE_PITCH_GAIN;
	targetPitch = constrain(targetPitch, -FLIGHT_NAVIGATE_PITCH_MAX, FLIGHT_NAVIGATE_PITCH_MAX);

	float pitchAmount = targetPitch - Sensors::MotionData.AHRS.pitch;

	if (pitchAmount > 180)
		pitchAmount -= 360;
	else if (pitchAmount < -180)
		pitchAmount += 360;

	float yawAmount = 0;	// temporary

	if (yawAmount > 180)
		yawAmount -= 360;
	else if (yawAmount < -180)
		yawAmount += 360;

	unsigned short aileronChannel = FLIGHT_STICK_CENTRE;
	unsigned short elevatorChannel = FLIGHT_STICK_CENTRE;
	unsigned short rudderChannel = FLIGHT_STICK_CENTRE;

	aileronChannel += rollAmount * FLIGHT_NAVIGATE_GAIN;
	elevatorChannel += pitchAmount * FLIGHT_NAVIGATE_GAIN;
	rudderChannel += yawAmount;

	if (GPS::GPSData.speed >= FLIGHT_NAVIGATE_SPEED_TARGET - 1 && GPS::GPSData.speed <= FLIGHT_NAVIGATE_SPEED_TARGET + 1)
	{
		// no change to throttle
	}
	else if (GPS::GPSData.speed > FLIGHT_NAVIGATE_SPEED_TARGET)
	{
		throttle -= 1;
	}
	else
	{
		throttle += 1;
	}

	IO::final_channels[IO_OUT_AILERON1] = aileronChannel;
	IO::final_channels[IO_OUT_AILERON2] = aileronChannel;
	IO::final_channels[IO_OUT_ELEVATOR] = elevatorChannel;
	IO::final_channels[IO_OUT_RUDDER] = rudderChannel;

	IO::final_channels[IO_OUT_MOTOR1] = constrain(throttle, 1200, 1800);
	IO::final_channels[IO_OUT_MOTOR2] = constrain(throttle, 1200, 1800);

	// channel output
	if (millis() - deltaNavT >= 100)
	{
		/*for (int i = 0; i < 6; i++)
		{
			NeoSerial.print(IO::final_channels[i]);
			NeoSerial.print("\t");
		}*/
		NeoSerial.print(Sensors::MotionData.AHRS.pitch);
		NeoSerial.print("\t");
		NeoSerial.print(Sensors::MotionData.AHRS.roll);
		NeoSerial.print("\t");
		NeoSerial.print(currentBearing);
		NeoSerial.print("\t");

		//NeoSerial.println();

		NeoSerial.print(pitchAmount);
		NeoSerial.print("\t");
		NeoSerial.print(rollAmount);
		NeoSerial.print("\t");
		NeoSerial.print(targetPitch);
		NeoSerial.print("\t");
		NeoSerial.print(targetRoll);
		NeoSerial.print("\t");
		NeoSerial.print(targetBearing);
		NeoSerial.print("\t");

		NeoSerial.println();

		deltaNavT = millis();
	}
}