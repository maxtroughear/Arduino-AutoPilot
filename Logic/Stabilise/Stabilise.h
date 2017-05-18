#pragma once

namespace FlightModes
{
	class Stabilise
	{
	public:
		static void Start();

		static void Loop();

	private:
		static float targetRoll;
		static float targetPitch;
		static float targetYaw;

		static int timeGainRoll;
		static int timeGainPitch;
		static int timeGainYaw;
		
		static bool stabilisingRoll;
		static bool stabilisingPitch;
		static bool stabilisingYaw;
	};
}