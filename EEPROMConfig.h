#pragma once

#include "Config.h"

// EEPROM Locations



// Sensors

#define SENSORS_MOTION_EEPROM_MAGBIAS 4070													// 4070 to 4070 + sizeof(float[3])
#define SENSORS_MOTION_EEPROM_MAGSCALE SENSORS_MOTION_EEPROM_MAGBIAS + sizeof(float[3])		// 4070 + sizeof(float[3]) to 4070 + 2 * sizeof(float[3])
#define SENSORS_MOTION_EEPROM_MAGDONE SENSORS_MOTION_EEPROM_MAGSCALE + sizeof(float[3])		// 4082 to 4093

// Nav storage

#define FLIGHT_STABILISE_EEPROM_DEADZONE 2860												// uint8
#define FLIGHT_STABILISE_EEPROM_GAIN 2861													// uint8
#define FLIGHT_STABILISE_EEPROM_GAIN_YAW 2862												// uint8
#define FLIGHT_STABILISE_EEPROM_TIME 2863													// uint8
#define FLIGHT_STABILISE_EEPROM_TIME_YAW 2864												// uint8

#define FLIGHT_NAVIGATE_EEPROM_GAIN 2985													// int8
#define FLIGHT_NAVIGATE_EEPROM_GAIN_YAW 2986												// int8
#define FLIGHT_NAVIGATE_EEPROM_TIME 2987													// int8
#define FLIGHT_NAVIGATE_EEPROM_TIME_YAW 2988												// int8
#define FLIGHT_NAVIGATE_EEPROM_WAYPOINT_MAX 2989											// uint8
#define FLIGHT_NAVIGATE_EEPROM_SPEED_TARGET 2990											// uint8
#define FLIGHT_NAVIGATE_EERPOM_ROLL_MAX 2991												// uint8 (0 -> 180)
#define FLIGHT_NAVIGATE_EEPROM_ROLL_TARGET 2992												// uint8 (0 -> 180)
#define FLIGHT_NAVIGATE_EEPROM_PITCH_MAX 2993												// uint8 (0 -> 180)
#define FLIGHT_NAVIGATE_EEPROM_PITCH_TARGET 2994											// uint8 (0 -> 180)
#define FLIGHT_NAVIGATE_EEPROM_ROLL_GAIN 2995												// int8
#define FLIGHT_NAVIGATE_EEPROM_PITCH_GAIN 2996												// int8
																							//ends at 2912

#define FLIGHT_NAVIGATE_EEPROM_ALTITUDE_MAXIMUM 2997										// uint8
#define FLIGHT_NAVIGATE_EEPROM_ALTITUDE_MINIMUM 2998										// uint8
#define FLIGHT_NAVIGATE_EEPROM_ALTITUDE_TARGET 2999											// uint8

#define FLIGHT_NAVIGATE_EEPROM_WAYPOINT_SIZE 3000											// allows for a max of 255 waypoints to be held (a maximum of 2040 bytes of EEPROM used, be sensible about this).
#define FLIGHT_NAVIGATE_EEPROM_WAYPOINT_START 3001											// marks the starting byte for the waypoints storage. each waypoint is 2x 4 byte integers (long)
