#pragma once

class Logic
{
public:
	enum FlightMode
	{
		MANUAL = 0,		// Manual flight, no assistance or anything
		STABILISE,		// Stabilise the manual flight using gyroscope
		HOLD,			// Circle around where mode was switched
		NAVIGATE,		// Follow waypoints and circle at last
		RTH				// Return to where flight started and circle
	}
	;
	static void Initialise();

	static void Loop();

	static FlightMode GetMode();

	static void SetFailsafeEnabled(bool enabled);
	static bool GetFailsafeEnabled();

private:
	static FlightMode Mode;

	static bool failsafe;
	static bool armed;

	static void SetMode(FlightMode newMode);
};