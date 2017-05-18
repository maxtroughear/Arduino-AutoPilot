#pragma once

// EEPROM Locations



// Sensors

#define SENSORS_MOTION_EEPROM_MAGBIAS 4070													// 2048 to 2048 + sizeof(float[3])
#define SENSORS_MOTION_EEPROM_MAGSCALE SENSORS_MOTION_EEPROM_MAGBIAS + sizeof(float[3])		// 2048 + sizeof(float[3]) to 2048 + 2 * sizeof(float[3])
#define SENSORS_MOTION_EEPROM_MAGDONE SENSORS_MOTION_EEPROM_MAGSCALE + sizeof(float[3])