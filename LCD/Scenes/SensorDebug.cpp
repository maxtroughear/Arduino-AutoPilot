#include "Scene.h"

#include <NeoHWSerial.h>

#include "../../Sensors/Sensors.h"

void Scenes::SensorDebug(bool init)
{
	RA8875 &tft = *Scenes::tft;
	int16_t x, y;
	
	if (init)
	{
		tft.clearScreen(RA8875_WHITE);
		
		// init bottom bar
		tft.setActiveWindow(0, tft.width(), tft.height() - 80, tft.height());
		tft.fillWindow(tft.Color565(220, 220, 220));
		
		tft.setTextColor(tft.Color565(0, 0, 0), tft.Color565(220, 220, 220));

		tft.drawRect(tft.width() - 200, tft.height() - 80, 200, 80, RA8875_BLACK);
		tft.drawRect(tft.width() - 400, tft.height() - 80, 200, 80, RA8875_BLACK);
		tft.drawRect(tft.width() - 600, tft.height() - 80, 200, 80, RA8875_BLACK);
		tft.drawRect(tft.width() - 800, tft.height() - 80, 200, 80, RA8875_BLACK);
		
		tft.setCursor(tft.width() - 375, tft.height() - 70);
		tft.println("Calibrate");
		tft.setCursor(tft.width() - 375, tft.getCursorY());
		tft.println("Accel/Gyro");
		
		tft.setCursor(tft.width() - 175, tft.height() - 70);
		tft.println("Calibrate");
		tft.setCursor(tft.width() - 175, tft.getCursorY());
		tft.println("Compass");

		// init main view
		tft.setActiveWindow(0, tft.width(), 0, tft.height() - 81);
		tft.setTextColor(tft.Color565(0, 0, 0), tft.Color565(255, 255, 255));

		NeoSerial.print("Colour Mode: "); NeoSerial.println(tft.getColorBpp());

	}

	tft.fillWindow(RA8875_WHITE);

	tft.setCursor(0, 0, false);
	tft.print("Alt:  "); tft.print(Sensors::Altitude); tft.println("m");
	tft.print("Temp: "); tft.print(Sensors::Temperature); tft.println(" c");

	tft.println("Motion Data");
	tft.print("id");

	tft.getCursorFast(x, y);
	tft.setCursor(170, y);

	tft.print("x");

	tft.getCursorFast(x, y);
	tft.setCursor(320, y);

	tft.print("y");

	tft.getCursorFast(x, y);
	tft.setCursor(470, y);

	tft.println("z");

	tft.print("ac");

	tft.getCursorFast(x, y);
	tft.setCursor(150, y);

	tft.print(Sensors::MotionData.Accel.x);

	tft.getCursorFast(x, y);
	tft.setCursor(300, y);

	tft.print(Sensors::MotionData.Accel.y);

	tft.getCursorFast(x, y);
	tft.setCursor(450, y);

	tft.println(Sensors::MotionData.Accel.z);

	tft.print("gy");

	tft.getCursorFast(x, y);
	tft.setCursor(150, y);

	tft.print(Sensors::MotionData.Gyro.x);

	tft.getCursorFast(x, y);
	tft.setCursor(300, y);

	tft.print(Sensors::MotionData.Gyro.y);

	tft.getCursorFast(x, y);
	tft.setCursor(450, y);

	tft.println(Sensors::MotionData.Gyro.y);

	tft.print("AHRS");

	tft.getCursorFast(x, y);
	tft.setCursor(150, y);

	tft.print(Sensors::MotionData.AHRS.pitch);

	tft.getCursorFast(x, y);
	tft.setCursor(300, y);

	tft.print(Sensors::MotionData.AHRS.roll);

	tft.getCursorFast(x, y);
	tft.setCursor(450, y);

	tft.println(Sensors::MotionData.AHRS.yaw);

	tft.println();

	tft.print("Rate: "); tft.print(Sensors::MotionData.rate, 2); tft.println("Hz");

	// touch events

	tft.touchEnable(true);

	if (LCD::Touched())
	{
		tsPoint_t point;
		LCD::TouchRead(&point);
		NeoSerial.print("Touch @ "); NeoSerial.print(point.x); NeoSerial.print(", "); NeoSerial.println(point.y);
		if (point.x >= tft.width() - 200 && point.y >= tft.height() - 60)
		{
			NeoSerial.println("Calibration of compass will begin in 4 seconds. Move in a figure 8 motion facing the compass in all directions for 15 seconds.");

			tft.setActiveWindow(tft.width() / 10, (tft.width() / 10) * 9, tft.height() / 4, (tft.height() / 4) * 3);
			tft.fillWindow(tft.Color565(128, 128, 128));
			tft.setTextColor(tft.Color565(255, 255, 255), tft.Color565(128, 128, 128));
			tft.setCursor(CENTER, 200);
			tft.println("Compass Calibration begins in 4 seconds");
			tft.setCursor(CENTER, tft.getCursorY());
			tft.println("Move in a figure 8 motion facing");
			tft.setCursor(CENTER, tft.getCursorY());
			tft.println("in all directions for 15 seconds.");

			delay(4000);

			NeoSerial.print("Calibration starting...");

			tft.fillWindow(tft.Color565(128, 128, 128));
			tft.setTextColor(tft.Color565(255, 255, 255), tft.Color565(128, 128, 128));
			tft.setCursor(CENTER, 220);
			tft.println("Calibration starting...");
			tft.setCursor(CENTER, tft.getCursorY());
			tft.println("Go!");

			Sensors::CalibrateMagnetometer();
			NeoSerial.println("Done!");

			tft.fillWindow(tft.Color565(128, 128, 128));
			tft.setTextColor(tft.Color565(255, 255, 255), tft.Color565(128, 128, 128));
			tft.setCursor(CENTER, CENTER);
			tft.println("Compass Calibration Completed");

			delay(2000);

			tft.setActiveWindow(0, tft.width(), 0, tft.height() - 80);
			tft.setTextColor(tft.Color565(0, 0, 0), tft.Color565(255, 255, 255));
		}
		else if (point.x < tft.width() - 200 && point.x >= tft.width() - 400 && point.y >= tft.height() - 60)
		{
			NeoSerial.println("Calibration of accelerometer will begin in 4 seconds. Move in a figure 8 motion facing the compass in all directions for 15 seconds.");

			tft.setActiveWindow(tft.width() / 10, (tft.width() / 10) * 9, tft.height() / 4, (tft.height() / 4) * 3);
			tft.fillWindow(tft.Color565(128, 128, 128));
			tft.setTextColor(tft.Color565(255, 255, 255), tft.Color565(128, 128, 128));
			tft.setCursor(CENTER, 200);
			tft.println("Accelerometer & Compass Calibration");
			tft.setCursor(CENTER, tft.getCursorY());
			tft.println("begins in 4 seconds. Place on a flat");
			tft.setCursor(CENTER, tft.getCursorY());
			tft.println("level surface.");

			delay(4000);

			NeoSerial.print("Calibration starting...");

			tft.fillWindow(tft.Color565(128, 128, 128));
			tft.setTextColor(tft.Color565(255, 255, 255), tft.Color565(128, 128, 128));
			tft.setCursor(CENTER, 220);
			tft.println("Calibration starting...");
			tft.setCursor(CENTER, tft.getCursorY());
			tft.println("Go!");

			Sensors::CalibrateAccelerometer();
			NeoSerial.println("Done!");

			tft.fillWindow(tft.Color565(128, 128, 128));
			tft.setTextColor(tft.Color565(255, 255, 255), tft.Color565(128, 128, 128));
			tft.setCursor(CENTER, CENTER);
			tft.println("Accelerometer & Compass Calibration");
			tft.setCursor(CENTER, tft.getCursorY());
			tft.println("Completed");

			delay(2000);

			tft.setActiveWindow(0, tft.width(), 0, tft.height() - 80);
			tft.setTextColor(tft.Color565(0, 0, 0), tft.Color565(255, 255, 255));
		}

		if (LCD::Touched())
			LCD::TouchRead(&point);
	}
}