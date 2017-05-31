#include "Configurator.h"

#include "../EEPROMConfig.h"

#include <NeoHWSerial.h>

bool Configurator::connected = false;
char *Configurator::words[30];

void Configurator::Init()
{

}

void Configurator::Loop()
{	
	state = IDLE;
	while (NeoSerial.available())
	{
		char *buffer = new char[20];

		short size = NeoSerial.readBytesUntil(' ', buffer, 20);
		buffer[size] = '\0';

		decode(buffer, size);

		// finished with buffer
		delete buffer;
		buffer = NULL;
	}
}

void Configurator::resetEEPROM()
{
	// reset config values in eeprom to defaults
}

void Configurator::decode(char *word, short size)
{
	if (strcasecmp_P(word, "APC") == 0)
	{
		state = HAS_APC;
	}
	else if (strcasecmp_P(word, "") == 0)
	{

	}
}