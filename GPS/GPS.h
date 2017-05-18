#pragma once

#include <NeoHWSerial.h>
#include "../NeoGPS/NMEAGPS.h"
#include "../NeoGPS/ublox/ubxGPS.h"
#include "../NeoGPS/ublox/ubxmsg.h"

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

struct GPSData
{
	struct Location
	{
		long lat, lon;
	} location;

	enum FixQuality
	{
		INVALID = 0,
		GPS,
		DGPS
	} quality;

	float altitude;
	float speed;
	float heading;
	unsigned short satellites;
	String timeString;
	uint16_t time;


};

class GPS
{
public:
	static void Initialise();

	static void Loop();

	static float DistanceTo(const NeoGPS::Location_t &point);
	static float BearingTo(const NeoGPS::Location_t &point);

	static void PassThrough();

	static GPSData GPSData;

private:
	//static NMEAGPS gps;
	static ubloxGPS gps;
	static gps_fix fix;

	static void updateData();

	static void sendUBX(const unsigned char *progmemBytes, size_t len);
};