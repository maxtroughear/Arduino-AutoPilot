#pragma once

#include "../LCD.h"

#if LCD_ENABLED

// Scene class, used by the LCD class to display data for particular things

class Scenes
{
public:
	enum SCENES
	{
		NONE = 0,
		SENSOR_DEBUG
	};

	static void InitialiseLCD(RA8875 &tft);

	static void LoopLCD();

private:
	static RA8875 *tft;

	static SCENES activeScene;
	static SCENES lastScene;

	static void SensorDebug(bool init = false);
};

#endif