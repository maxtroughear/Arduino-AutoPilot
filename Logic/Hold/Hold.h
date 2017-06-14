#pragma once

#include "../../Config.h"

#include "../../GPS/GPS.h"

namespace FlightModes
{
	class Hold
	{
	public:
		static void Start();

		static void Loop();

	private:
		static NeoGPS::Location_t target;
	};
}