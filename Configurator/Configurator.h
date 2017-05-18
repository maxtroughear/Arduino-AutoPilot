#pragma once

class Configurator
{
public:
	static void Init();

	static void Loop();

private:

	static enum STATE
	{
		IDLE = 0,
		HAS_APC
	} state;

	static void resetEEPROM();

	static void decode(char *word, short size);

	static bool connected;

	static char *words[];
};