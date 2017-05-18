#include "Scene.h"

RA8875 *Scenes::tft;

Scenes::SCENES Scenes::activeScene = SENSOR_DEBUG;
Scenes::SCENES Scenes::lastScene = NONE;

void Scenes::InitialiseLCD(RA8875 &tft)
{
	Scenes::tft = &tft;
}

void Scenes::LoopLCD()
{
	if (lastScene != activeScene)
	{
		// initialise scene

		if (activeScene == SENSOR_DEBUG)
			SensorDebug(true);

		else
		{
			tft->setActiveWindow(tft->width() / 4, (tft->width() / 4) * 3, tft->height() / 3, (tft->height() / 3) * 2);
			tft->fillWindow(tft->Color565(128, 128, 128));
			tft->setCursor(CENTER, CENTER);
			tft->setTextColor(tft->Color565(255, 255, 255), tft->Color565(128, 128, 128));
			tft->print("No Active Scene!");
		}

		lastScene = activeScene;
	}
	else
	{

		if (activeScene == SENSOR_DEBUG)
			SensorDebug();


	}
}