#pragma once

#include <arduino.h>

class Configurator
{
public:
	static void Init();

	static void Loop();

private:
	static void resetEEPROM();

	static void decode(byte buffer[], short size);

	static void sendGPS();

	static void longToBytes(long val, byte b[4]);

	static bool connected;

	static unsigned long lastpoll;

	static unsigned long lastGPS;
};