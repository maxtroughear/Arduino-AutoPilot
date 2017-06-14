#include "Configurator.h"

#include "../EEPROMConfig.h"
#include "../Config.h"

#include "../GPS/GPS.h"

#include <NeoHWSerial.h>

bool Configurator::connected = false;

unsigned long Configurator::lastpoll = 0;
unsigned long Configurator::lastGPS = 0;

void Configurator::Init()
{
	NeoSerial.setTimeout(10);
}

void Configurator::Loop()
{	
	while (NeoSerial.available())
	{
		byte buffer[20];

		short size = NeoSerial.readBytes(buffer, 20);

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
	if (size < 5)	// min 5 bytes
		return;
	if (buffer[0] != 'A' && buffer[1] != 'P')
		return;

	if (buffer[2] == 'S' && buffer[3] == 'C')
	{
		connected = true;
		lastpoll = millis();
	}

	else if (buffer[2] == 'C' && buffer[3] == 'F')
	{
		
	}
}

void Configurator::sendGPS()
{
	uint8_t *altitudeBytes;
	altitudeBytes = (uint8_t*)(&GPS::GPSData.altitude);

	uint8_t *speedBytes;
	speedBytes = (uint8_t*)(&GPS::GPSData.speed);

	NeoSerial.write('A');
	NeoSerial.write('P');
	NeoSerial.write('G');
	NeoSerial.write('P');
	NeoSerial.write(GPS::GPSData.location.lat);
	NeoSerial.write(GPS::GPSData.location.lon);
	for (short i = 0; i < 4; i++)
		NeoSerial.write(altitudeBytes[i]);
	for (short i = 0; i < 4; i++)
		NeoSerial.write(speedBytes[i]);

	lastGPS = millis();
}