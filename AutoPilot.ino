#include <NeoHWSerial.h>

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>

#include <Filter.h>
#include <quaternionFilters.h>
#include <MPU9250.h>
#include <SparkFunMPL3115A2.h>
#include <RA8875.h>

#include "LCD/LCD.h"
#include "Sensors/Sensors.h"
#include "GPS/GPS.h"
#include "IO/IO.h"
#include "Logic/Logic.h"
#include "Configurator/Configurator.h"

void setup()
{
	NeoSerial.begin(115200);
	//GPS::PassThrough();
	
	//LCD::Initialise();

	IO::final_channels[2];

	IO::Initialise();

	Sensors::Initialise();

	GPS::Initialise();

	Logic::Initialise();
}

unsigned long deltaTime = 0;

void loop()
{
	IO::LoopInput();

	Sensors::Loop();
	GPS::Loop();

	Logic::Loop();

	if (millis() - deltaTime >= 200)
	{
		/*NeoSerial.print("Alt: "); NeoSerial.print(Sensors::Altitude); NeoSerial.print("m"); NeoSerial.print("\t");
		NeoSerial.print("Temp: "); NeoSerial.print(Sensors::Temperature); NeoSerial.print(" C"); NeoSerial.println();
		NeoSerial.print(" - "); NeoSerial.print("\t"); NeoSerial.print("x"); NeoSerial.print("\t"); NeoSerial.print("y"); NeoSerial.print("\t"); NeoSerial.println("z");
		NeoSerial.print("Accel"); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.Accel.x); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.Accel.y); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.Accel.z); NeoSerial.println();
		NeoSerial.print("Gyro"); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.Gyro.x); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.Gyro.y); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.Gyro.z); NeoSerial.println();
		NeoSerial.print("AHRS"); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.AHRS.pitch); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.AHRS.roll); NeoSerial.print("\t"); NeoSerial.print(Sensors::MotionData.AHRS.yaw); NeoSerial.println();
		*/

		// GPS
		/*NeoSerial.print("Status: "); NeoSerial.println(GPS::GPSData.quality);
		NeoSerial.print("Time: "); NeoSerial.println(GPS::GPSData.timeString);
		NeoSerial.print("Lat: "); NeoSerial.println(GPS::GPSData.location.lat, 10);
		NeoSerial.print("Lon: "); NeoSerial.println(GPS::GPSData.location.lon, 10);
		NeoSerial.print("Alt: "); NeoSerial.print(GPS::GPSData.altitude); NeoSerial.println("m");
		NeoSerial.print("Spd: "); NeoSerial.print(GPS::GPSData.speed); NeoSerial.println("m/s");*/

		//NeoSerial.print(GPS::DistanceTo(schoolClass)); NeoSerial.print("m \t"); NeoSerial.println(GPS::BearingTo(schoolClass));

		deltaTime = millis();
	}

	//LCD::Loop();

	IO::LoopOutput();
}
