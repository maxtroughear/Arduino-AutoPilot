#pragma once

#include <Wire.h>
#include <EEPROM.h>

#include <Filter.h>
#include <quaternionFilters.h>
#include <SparkFunMPL3115A2.h>
#include <MPU9250.h>

#include "../Config.h"

struct MotionData
{
	struct Gyro
	{
		float x, y, z;
	} Gyro;
	struct Accel
	{
		float x, y, z;
	} Accel;
	struct AHRS
	{
		float pitch, roll, yaw;
	} AHRS;

	float rate;
};

class Sensors
{
public:
	static void Initialise();

	static void Loop();

	static void CalibrateAccelerometer();

	// Move the sensor in a figure-8 motion for 15 seconds
	static void CalibrateMagnetometer();

	static void LoadMagnetometerCalibration();

	static bool IsAccelWorking();
	static bool IsMagWorking();
	static bool IsAltWorking();

	static float Altitude;
	static float Temperature;

	static MotionData MotionData;

private:
	static bool accelWorking;
	static bool magWorking;
	static bool altWorking;

	static MPL3115A2 pressureSensor;
	static MPU9250 motionSensor;

	static ExponentialFilter<float> motionFilterPitch;
	static ExponentialFilter<float> motionFilterRoll;
	static ExponentialFilter<float> motionFilterYaw;
};