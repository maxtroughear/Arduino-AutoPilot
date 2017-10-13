#include "Configurator.h"

#include "../EEPROMConfig.h"
#include "../Config.h"

#include "../GPS/GPS.h"
#include "../Sensors/Sensors.h"
#include "../Logic/Logic.h"
#include "../Logic/Navigate/Navigate.h"

#include <NeoHWSerial.h>

bool Configurator::connected = false;

unsigned long Configurator::lastpoll = 0;
unsigned long Configurator::lastGPS = 0;

void Configurator::Init()
{
	CONFIGURE_PORT.begin(9600);
	delay(50);
	CONFIGURE_PORT.setTimeout(10);
}

void Configurator::Loop()
{	
	while (CONFIGURE_PORT.available())
	{
		byte buffer[20];

		short size = CONFIGURE_PORT.readBytes(buffer, 20);	// max of 20 bytes per packet

		decode(buffer, size);
	}

	if (millis() - 1000 > lastpoll)
	{
		connected = false;
	}

	if (connected)
	{
		if (millis() - CONFIGURE_SEND_GPS_TIME > lastGPS)
			sendGPS();
	}
}

void Configurator::resetEEPROM()
{
	// reset config values in eeprom to defaults
}

void Configurator::decode(byte buffer[], short size)
{
	for (int i = 0; i < size; i++)
	{
		CONFIGURE_PORT.write(buffer[i]);
	}

	//if (size < 5)	// min 5 bytes
	//	return;
	if (buffer[0] != 'A' && buffer[1] != 'P')
		return;

	if (buffer[2] == 'S' && buffer[3] == 'C')
	{
		//CONFIGURE_PORT.println("Connect Poll!");
		connected = true;
		lastpoll = millis();
	}

	else if (buffer[2] == 'C')
	{
		if (buffer[3] == 'N')
		{
			// other variables
			if (buffer[4] == 'O')
			{
				if (buffer[5] == 'A') // altitude
				{

				}
			}
				
			// waypoints
			else if (buffer[4] == 'A')
			{
				// add
				long lat = 0, lon = 0;

				lat += buffer[5] << 24;
				lat += buffer[6] << 16;
				lat += buffer[7] << 8;
				lat += buffer[8];

				lon += buffer[9] << 24;
				lon += buffer[10] << 16;
				lon += buffer[11] << 8;
				lon += buffer[12];

				FlightModes::Navigate::AddWaypoint(NeoGPS::Location_t(lat, lon));

				// no return/ack (relies on remote end querying the waypoints again)
			}
			else if (buffer[4] == 'R')
			{
				// remove
				char *numBuffer = new char[size - 5];

				for (int i = 5, j = 0; i < size; i++, j++)
					numBuffer[j] = buffer[i];

				FlightModes::Navigate::RemoveWaypoint(atoi(numBuffer));

				// no return/ack (relies on remote end querying the waypoints again)

				delete numBuffer;
			}
			else if (buffer[4] == 'G')
			{
				if (buffer[5] == 'C')
				{
					CONFIGURE_PORT.write('A');
					CONFIGURE_PORT.write('P');	// header
					CONFIGURE_PORT.write('C');	// config
					CONFIGURE_PORT.write('N');	// nav
					CONFIGURE_PORT.write('G');	// get
					CONFIGURE_PORT.write('C');	// count
					CONFIGURE_PORT.write(FlightModes::Navigate::GetWaypointCount());
				}
				else if (buffer[5] == 'A')
				{
					for (int i = 0; i < FlightModes::Navigate::GetWaypointCount(); i++)
					{
						NeoGPS::Location_t point = FlightModes::Navigate::GetWaypoint(i);

						byte latB[4];
						byte lonB[4];

						longToBytes(point.lat(), latB);
						longToBytes(point.lon(), lonB);

						CONFIGURE_PORT.write('A');
						CONFIGURE_PORT.write('P');	// header
						CONFIGURE_PORT.write('C');	// config
						CONFIGURE_PORT.write('N');	// nav
						CONFIGURE_PORT.write('G');	// get
						CONFIGURE_PORT.write('W');
						CONFIGURE_PORT.write(i);

						for (int j = 0; j < 4; j++)
							CONFIGURE_PORT.write(latB[j]);
						for (int j = 0; j < 4; j++)
							CONFIGURE_PORT.write(lonB[j]);
					}
				}
				else
				{
					char *numBuffer = new char[size - 5]; // create a char array with a size of the remaining bytes from the buffer

					for (int i = 5, j = 0; i < size; i++, j++) // take the number characters and add them to the numBuffer
						numBuffer[j] = buffer[i];

					NeoGPS::Location_t point = FlightModes::Navigate::GetWaypoint(atoi(numBuffer));
					
					byte latB[4];
					byte lonB[4];

					longToBytes(point.lat(), latB);
					longToBytes(point.lon(), lonB);

					CONFIGURE_PORT.write('A');
					CONFIGURE_PORT.write('P');	// header
					CONFIGURE_PORT.write('C');	// config
					CONFIGURE_PORT.write('N');	// nav
					CONFIGURE_PORT.write('G');	// get
					CONFIGURE_PORT.write(atoi(numBuffer));

					for (int i = 0; i < 4; i++)
						CONFIGURE_PORT.write(latB[i]);
					for (int i = 0; i < 4; i++)
						CONFIGURE_PORT.write(lonB[i]);

					// 13 byte packet

					delete numBuffer;
				}
			}
		}
	}
}

void Configurator::longToBytes(long val, byte b[4])
{
	b[0] = (byte)((val >> 24) & 0xff);
	b[1] = (byte)((val >> 16) & 0xff);
	b[2] = (byte)((val >> 8) & 0xff);
	b[3] = (byte)((val) & 0xff);
}

void Configurator::sendGPS()
{
	uint8_t *altitudeBytes;
	altitudeBytes = (uint8_t*)(&GPS::GPSData.altitude);

	uint8_t *bearingBytes;
	bearingBytes = (uint8_t*)(&Sensors::MotionData.AHRS.yaw);

	byte latB[4];
	byte lonB[4];

	longToBytes(GPS::GPSData.location.lat, latB);
	longToBytes(GPS::GPSData.location.lon, lonB);

	CONFIGURE_PORT.write('A');
	CONFIGURE_PORT.write('P');
	CONFIGURE_PORT.write('G');
	CONFIGURE_PORT.write('P');
	for (int i = 0; i < 4; i++)
		CONFIGURE_PORT.write(latB[i]);
	for (int i = 0; i < 4; i++)
		CONFIGURE_PORT.write(lonB[i]);
	for (short i = 0; i < 4; i++)
		CONFIGURE_PORT.write(altitudeBytes[i]);
	for (short i = 0; i < 4; i++)
		CONFIGURE_PORT.write(bearingBytes[i]);

	// 20 byte packet
	
	lastGPS = millis();
}