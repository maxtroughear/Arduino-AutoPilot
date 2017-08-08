#include "Sensors.h"

#include "../EEPROMConfig.h"

#include <NeoHWSerial.h>

#define SENSORS_PRESSURE_INT 47

float Sensors::Altitude = 0;
float Sensors::Temperature = 0;

MotionData Sensors::MotionData;

MPL3115A2 Sensors::pressureSensor;
MPU9250 Sensors::motionSensor;

bool Sensors::accelWorking = false;
bool Sensors::magWorking = false;
bool Sensors::altWorking = false;

unsigned long Sensors::timeSinceCal = 0;

#if SENSORS_MOTION_FILTER
ExponentialFilter<float> Sensors::motionFilterPitch = ExponentialFilter<float>(SENSORS_MOTION_FILTER_PERCENT, 0);
ExponentialFilter<float> Sensors::motionFilterRoll = ExponentialFilter<float>(SENSORS_MOTION_FILTER_PERCENT, 0);
ExponentialFilter<float> Sensors::motionFilterYaw = ExponentialFilter<float>(SENSORS_MOTION_FILTER_PERCENT, 0);
#endif

void Sensors::Initialise()
{
	NeoSerial.println("Sensors initialising");
	
	NeoSerial.print("Motion sensor filtering: "); NeoSerial.println(SENSORS_MOTION_FILTER);

	if (!SENSORS_MOTION_FUSION)
		NeoSerial.println("Warning! Motion sensor fusion is disabled, system may not function as intended!");

	Wire.begin();

	delay(100);

	// Pressure sensor initialisation
	
	pressureSensor.begin();

	pressureSensor.setModeAltimeter();
	pressureSensor.setOversampleRate(7);
	pressureSensor.enableEventFlags();

	NeoSerial.println("MPL3115A2 initialised");

	altWorking = true;

	// Motion sensor initialisation

	byte c = motionSensor.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

	if (c == 0x71)
	{
		// correct
		accelWorking = true;
	}
	else
	{
		NeoSerial.print("MPU9250 WHO_AM_I failed! Reported as 0x"); NeoSerial.print(c, HEX); NeoSerial.println(" Should be 0x71");
		accelWorking = false;
	}

	motionSensor.MPU9250SelfTest(motionSensor.selfTest);
	// Debug printing
	//NeoSerial.print(F("x-axis self test: acceleration trim within : "));
	//NeoSerial.print(motionSensor.selfTest[0], 1); NeoSerial.println("% of factory value");
	//NeoSerial.print(F("y-axis self test: acceleration trim within : "));
	//NeoSerial.print(motionSensor.selfTest[1], 1); NeoSerial.println("% of factory value");
	//NeoSerial.print(F("z-axis self test: acceleration trim within : "));
	//NeoSerial.print(motionSensor.selfTest[2], 1); NeoSerial.println("% of factory value");
	//NeoSerial.print(F("x-axis self test: gyration trim within : "));
	//NeoSerial.print(motionSensor.selfTest[3], 1); NeoSerial.println("% of factory value");
	//NeoSerial.print(F("y-axis self test: gyration trim within : "));
	//NeoSerial.print(motionSensor.selfTest[4], 1); NeoSerial.println("% of factory value");
	//NeoSerial.print(F("z-axis self test: gyration trim within : "));
	//NeoSerial.print(motionSensor.selfTest[5], 1); NeoSerial.println("% of factory value");

	motionSensor.calibrateMPU9250(motionSensor.gyroBias, motionSensor.accelBias);

	motionSensor.initMPU9250();

	NeoSerial.println("MPU-9250 initialised");

	delay(100);

	byte d = motionSensor.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
	
	if (d == 0xff)
	{
		// correct
		magWorking = true; // ?
	}
	else
	{
		NeoSerial.print("AK8963 WHO_AM_I failed! Reported as 0x"); NeoSerial.print(d, HEX); NeoSerial.println(" Should be 0x48 or 0xff?");
		magWorking = false;
	}

	motionSensor.initAK8963(motionSensor.factoryMagCalibration);

	// Debug printing
	//NeoSerial.println("Calibration values: ");
	//NeoSerial.print("X-Axis factory sensitivity adjustment value ");
	//NeoSerial.println(motionSensor.factoryMagCalibration[0], 2);
	//NeoSerial.print("Y-Axis factory sensitivity adjustment value ");
	//NeoSerial.println(motionSensor.factoryMagCalibration[1], 2);
	//NeoSerial.print("Z-Axis factory sensitivity adjustment value ");
	//NeoSerial.println(motionSensor.factoryMagCalibration[2], 2);

	NeoSerial.println("AK8963 initialised");

	motionSensor.getAres();
	motionSensor.getGres();
	motionSensor.getMres();

	if (false)
	{
		NeoSerial.print("Calibration starting. Move in a figure 8 motion for 15 seconds...");
		CalibrateMagnetometer();
		NeoSerial.println("Done!");

	}
	LoadMagnetometerCalibration();

	NeoSerial.println("Sensors initialised");
}

// **********************************************
// TODO: Use ExponentialFilter to smooth data
// **********************************************

void Sensors::Loop()
{
	Altitude = pressureSensor.readAltitude() + 100.f;	// offset?
	Temperature = pressureSensor.readTemp();

	if (motionSensor.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	{
		motionSensor.readAccelData(motionSensor.accelCount);
		motionSensor.ax = (float)motionSensor.accelCount[0] * motionSensor.aRes;
		motionSensor.ay = (float)motionSensor.accelCount[1] * motionSensor.aRes;
		motionSensor.az = (float)motionSensor.accelCount[2] * motionSensor.aRes;

		motionSensor.readGyroData(motionSensor.gyroCount);
		motionSensor.gx = (float)motionSensor.gyroCount[0] * motionSensor.gRes;
		motionSensor.gy = (float)motionSensor.gyroCount[1] * motionSensor.gRes;
		motionSensor.gz = (float)motionSensor.gyroCount[2] * motionSensor.gRes;

		motionSensor.readMagData(motionSensor.magCount);
		motionSensor.mx = (float)motionSensor.magCount[0] * motionSensor.mRes * motionSensor.factoryMagCalibration[0] - motionSensor.magBias[0];
		motionSensor.my = (float)motionSensor.magCount[1] * motionSensor.mRes * motionSensor.factoryMagCalibration[1] - motionSensor.magBias[1];
		motionSensor.mz = (float)motionSensor.magCount[2] * motionSensor.mRes * motionSensor.factoryMagCalibration[2] - motionSensor.magBias[2];

		motionSensor.updateTime();

		motionSensor.delt_t = millis() - motionSensor.count;

		MadgwickQuaternionUpdate(motionSensor.ax, motionSensor.ay, motionSensor.az,
			motionSensor.gx * DEG_TO_RAD, motionSensor.gy * DEG_TO_RAD, motionSensor.gz * DEG_TO_RAD,
			motionSensor.my, motionSensor.mx, motionSensor.mz,
			motionSensor.deltat);

		motionSensor.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
			* *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1)
			* *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3)
			* *(getQ() + 3));
		motionSensor.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
			* *(getQ() + 2)));
		motionSensor.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2)
			* *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1)
			* *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3)
			* *(getQ() + 3));
		motionSensor.pitch *= RAD_TO_DEG;
		motionSensor.pitch *= -1;
		motionSensor.yaw *= RAD_TO_DEG;
		motionSensor.yaw *= -1;
		//motionSensor.yaw -= 19.71;
		motionSensor.yaw += 90;
		motionSensor.roll *= RAD_TO_DEG;
		motionSensor.roll *= -1;

		MotionData.Accel.x = motionSensor.ay;
		MotionData.Accel.y = motionSensor.ax;
		MotionData.Accel.z = motionSensor.az;

		MotionData.Gyro.x = motionSensor.gy;
		MotionData.Gyro.y = motionSensor.gx;
		MotionData.Gyro.z = motionSensor.gz;

#if SENSORS_MOTION_FILTER == true
		motionFilterPitch.Filter(motionSensor.roll);
		motionFilterRoll.Filter(motionSensor.pitch);
		motionFilterYaw.Filter(motionSensor.yaw);

		MotionData.AHRS.pitch = motionFilterPitch.Current();
		MotionData.AHRS.roll = motionFilterRoll.Current();
		MotionData.AHRS.yaw = motionFilterYaw.Current();
#else
		MotionData.AHRS.pitch = motionSensor.roll;
		MotionData.AHRS.roll = motionSensor.pitch;
		MotionData.AHRS.yaw = motionSensor.yaw;
#endif

#if SENSORS_DEBUG_LOG_AHRS == true
		NeoSerial.print(MotionData.AHRS.pitch);
		NeoSerial.print("\t");
		NeoSerial.print(MotionData.AHRS.roll);
		NeoSerial.print("\t");
		NeoSerial.print(MotionData.AHRS.yaw);
		NeoSerial.println();
#endif

		MotionData.rate = (float)motionSensor.sumCount / motionSensor.sum;

		motionSensor.count = millis();
		motionSensor.sumCount = 0;
		motionSensor.sum = 0;
	}
}

void Sensors::CalibrateAccelerometer()
{
	motionSensor.calibrateMPU9250(motionSensor.gyroBias, motionSensor.accelBias);
}

void Sensors::CalibrateMagnetometer()
{
	motionSensor.magCalMPU9250(motionSensor.magBias, motionSensor.magScale);	// store calibration to EEPROM and retrieve it when starting
	EEPROM.put(SENSORS_MOTION_EEPROM_MAGBIAS, motionSensor.magBias);
	EEPROM.put(SENSORS_MOTION_EEPROM_MAGSCALE, motionSensor.magScale);
	EEPROM.update(SENSORS_MOTION_EEPROM_MAGDONE, 1);
}

void Sensors::LoadMagnetometerCalibration()
{
	if (EEPROM.read(SENSORS_MOTION_EEPROM_MAGDONE) == 1)
	{
		EEPROM.get(SENSORS_MOTION_EEPROM_MAGBIAS, motionSensor.magBias);
		EEPROM.get(SENSORS_MOTION_EEPROM_MAGSCALE, motionSensor.magScale);
	}
	else
	{
		NeoSerial.println("Magnetometer not calibrated!");
		CalibrateMagnetometer();
	}
}

bool Sensors::IsAccelWorking()
{
	return accelWorking;
}

bool Sensors::IsMagWorking()
{
	return magWorking;
}

bool Sensors::IsAltWorking()
{
	return altWorking;
}