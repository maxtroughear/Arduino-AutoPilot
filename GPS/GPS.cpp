#include "GPS.h"

#include "../Config.h"

// U-blox rate commands
const unsigned char ubxRate1Hz[] PROGMEM =
{ 0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00 };
const unsigned char ubxRate5Hz[] PROGMEM =
{ 0x06,0x08,0x06,0x00,200,0x00,0x01,0x00,0x01,0x00 };
const unsigned char ubxRate10Hz[] PROGMEM =
{ 0x06,0x08,0x06,0x00,100,0x00,0x01,0x00,0x01,0x00 };
const unsigned char ubxRate16Hz[] PROGMEM =
{ 0x06,0x08,0x06,0x00,50,0x00,0x01,0x00,0x01,0x00 };

const unsigned char ubxBaud38400[] PROGMEM =
{ 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Disable specific NMEA sentences
const unsigned char ubxDisableGGA[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisableGLL[] PROGMEM =				 
{ 0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisableGSA[] PROGMEM =				 
{ 0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisableGSV[] PROGMEM =				 
{ 0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisableRMC[] PROGMEM =				 
{ 0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisableVTG[] PROGMEM =				 
{ 0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisableZDA[] PROGMEM =				 
{ 0x06,0x01,0x08,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x00 };

const unsigned char ubxEnableNavStatus[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00 };
const unsigned char ubxEnablePOSLLH[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00 };
const unsigned char ubxEnableVELNED[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00 };
const unsigned char ubxEnableTIMEUTC[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0x01,0x21,0x00,0x01,0x00,0x00,0x00,0x00 };

const unsigned char ubxDisableNavStatus[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisablePOSLLH[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisableVELNED[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x00,0x00,0x00,0x00,0x00 };
const unsigned char ubxDisableTIMEUTC[] PROGMEM =
{ 0x06,0x01,0x08,0x00,0x01,0x21,0x00,0x00,0x00,0x00,0x00,0x00 };

//const char disableRMC[] PROGMEM = "PUBX,40,RMC,0,0,0,0,0,0";
//const char disableGLL[] PROGMEM = "PUBX,40,GLL,0,0,0,0,0,0";
//const char disableGSV[] PROGMEM = "PUBX,40,GSV,0,0,0,0,0,0";
//const char disableGSA[] PROGMEM = "PUBX,40,GSA,0,0,0,0,0,0";
//const char disableGGA[] PROGMEM = "PUBX,40,GGA,0,0,0,0,0,0";
//const char disableVTG[] PROGMEM = "PUBX,40,VTG,0,0,0,0,0,0";
//const char disableZDA[] PROGMEM = "PUBX,40,ZDA,0,0,0,0,0,0";
//
//const char enableRMC[] PROGMEM = "PUBX,40,RMC,0,1,0,0,0,0";
//const char enableGLL[] PROGMEM = "PUBX,40,GLL,0,1,0,0,0,0";
//const char enableGSV[] PROGMEM = "PUBX,40,GSV,0,1,0,0,0,0";
//const char enableGSA[] PROGMEM = "PUBX,40,GSA,0,1,0,0,0,0";
//const char enableGGA[] PROGMEM = "PUBX,40,GGA,0,1,0,0,0,0";
//const char enableVTG[] PROGMEM = "PUBX,40,VTG,0,1,0,0,0,0";
//const char enableZDA[] PROGMEM = "PUBX,40,ZDA,0,1,0,0,0,0";

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
	// set baud rate
	/*sendUBX(ubxBaud38400, sizeof(ubxBaud38400));

	GPS_PORT.flush();
	delay(100);
	GPS_PORT.end();
	delay(100);
	GPS_PORT.begin(38400);*/

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
	while (gps.available(GPS_PORT))
	{
		fix = gps.read();
		updateData();
	}
#endif

	// handle
}

float GPS::DistanceTo(const NeoGPS::Location_t &point)
{
	return fix.location.DistanceKm(point) * 1000;
}

float GPS::BearingTo(const NeoGPS::Location_t &point)
{
	return fix.location.BearingToDegrees(point);
}

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