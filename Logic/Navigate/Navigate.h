#pragma once

#include "../../Config.h"

#include "../../GPS/GPS.h"

namespace FlightModes
{
	class Navigate
	{
	public:
		static void Start();

		static void Loop();

		static void SetHome(NeoGPS::Location_t home);
		static void SetGroundAltitude(float altitude);

		static void AddWaypoint(NeoGPS::Location_t point);

		static void ReturnToHome();

	private:
		static NeoGPS::Location_t home;
		static NeoGPS::Location_t waypoints[];
		static unsigned int waypointCount;
		static unsigned int targetWaypoint;

		static unsigned short throttle;

		static float groundAltitude;

		static float targetRoll, targetPitch, targetYaw;

		static void navigateTo(NeoGPS::Location_t &home);
	};
}