#include "GPS.h"
#include "ubxMessages.h"
#include "../Config.h"

//NMEAGPS GPS::gps;
ubloxGPS GPS::gps = ubloxGPS(&NeoSerial1);
gps_fix GPS::fix;
uint8_t LastSentenceInInterval = 0xFF;
GPSData GPS::GPSData;

void GPS::Initialise()
{
	NeoSerial.println("GPS initialising");

	// Find last message in interval
	LastSentenceInInterval = UBX_LAST_MSG_ID_IN_INTERVAL;
	//LastSentenceInInterval = ubloxGPS::UBX_LAST_MSG;

	GPS_PORT.begin(9600);

	delay(2000);

	// disable all NMEA messages
	sendUBX(ubxDisableGLL, sizeof(ubxDisableGLL));
	sendUBX(ubxDisableGSA, sizeof(ubxDisableGSA));
	sendUBX(ubxDisableGSV, sizeof(ubxDisableGSV));
	sendUBX(ubxDisableVTG, sizeof(ubxDisableVTG));
	sendUBX(ubxDisableZDA, sizeof(ubxDisableZDA));
	sendUBX(ubxDisableRMC, sizeof(ubxDisableZDA));
	sendUBX(ubxDisableGGA, sizeof(ubxDisableGGA));

	// disable UBX messages
	sendUBX(ubxDisableNavStatus, sizeof(ubxDisableNavStatus));
	sendUBX(ubxDisablePOSLLH, sizeof(ubxDisablePOSLLH));
	sendUBX(ubxDisableVELNED, sizeof(ubxDisableVELNED));
	sendUBX(ubxDisableTIMEUTC, sizeof(ubxDisableTIMEUTC));

	// set GPS rate
	sendUBX(GPS_RATE, sizeof(GPS_RATE));

	// enable UBX messages
	sendUBX(ubxEnableNavStatus, sizeof(ubxEnableNavStatus));
	sendUBX(ubxEnablePOSLLH, sizeof(ubxEnablePOSLLH));
	sendUBX(ubxEnableVELNED, sizeof(ubxEnableVELNED));
	sendUBX(ubxEnableTIMEUTC, sizeof(ubxEnableTIMEUTC));
	
	NeoSerial.println("GPS initialised");
}

void GPS::Loop()
{
#if GPS_DEBUG_ECHO
	// Echo to the Serial output all data from the GPS Serial port
	while (GPS_PORT.available())
	{
		char c = GPS_PORT.read();
		NeoSerial.write(c);
		gps.handle(c);
		while (gps.available())
		{
			fix = gps.read();
			updateData();
		}
	}
	NeoSerial.println();
#else
	// Process data from the GPS Serial port
	while (gps.available(GPS_PORT))
	{
		fix = gps.read();
		updateData();
	}
#endif

	// handle
}

// Calculate the distance in metres from the current position to the location parameter
float GPS::DistanceTo(const NeoGPS::Location_t &point)
{
	return fix.location.DistanceKm(point) * 1000;
}

// Calculate the bearing from the current position to the location parameter
float GPS::BearingTo(const NeoGPS::Location_t &point)
{
	return fix.location.BearingToDegrees(point);
}

// Endless function to switch the program to a simple pass through for the GPS Serial port
// Used for debugging in the 'uBlox u-center' application
void GPS::PassThrough()
{
	GPS_PORT.begin(9600);

	while (true)
	{
		if (GPS_PORT.available())
			NeoSerial.write(GPS_PORT.read());
		if (NeoSerial.available())
			GPS_PORT.write(NeoSerial.read());
	}
}

// Update the public GPSData object with only valid information from the GPS fix data
void GPS::updateData()
{
	if (fix.valid.status)
	{
		if (fix.status == fix.STATUS_NONE)
			GPSData.quality = GPSData.INVALID;
		else if (fix.status == fix.STATUS_STD)
			GPSData.quality = GPSData.GPS;
		else if (fix.status == fix.STATUS_DGPS)
			GPSData.quality = GPSData.DGPS;
		else
			GPSData.quality = GPSData.INVALID;
	}
	if (fix.valid.location)
	{
		GPSData.location.lat = fix.latitudeL();
		GPSData.location.lon = fix.longitudeL();
	}
	if (fix.valid.altitude)
	{
		GPSData.altitude = fix.altitude();
	}
	if (fix.valid.time)
	{
		GPSData.time = fix.dateTime_ms();
		GPSData.timeString = fix.dateTime.hours;
		GPSData.timeString.concat(":");
		GPSData.timeString.concat(fix.dateTime.minutes);
		GPSData.timeString.concat(":");
		GPSData.timeString.concat(fix.dateTime.seconds);
	}
	if (fix.valid.heading)
	{
		GPSData.heading = fix.heading();
	}
	if (fix.valid.speed)
	{
		GPSData.speed = fix.speed_metersph() / 60.f;
	}
}

// send a ubx message prefixed with required bytes and suffixed checksum bytes
void GPS::sendUBX(const unsigned char *progmemBytes, size_t len)
{
	GPS_PORT.write(0xB5); // SYNC1
	GPS_PORT.write(0x62); // SYNC2

	uint8_t a = 0, b = 0;
	while (len-- > 0) {
		uint8_t c = pgm_read_byte(progmemBytes++);
		a += c;
		b += a;
		GPS_PORT.write(c);
	}

	GPS_PORT.write(a); // CHECKSUM A
	GPS_PORT.write(b); // CHECKSUM B
	delay(10);
}