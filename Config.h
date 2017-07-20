#pragma once

// LCD

#define LCD_DIM_ENABLED						true
#define LCD_DIM_TIMEOUT						1000 * 30

#define LCD_UPDATE_DELTA					50

#define LCD_TOUCH_DELTATIME					100
#define LCD_TOUCH_ENABLED					true

// Sensors

#define SENSORS_MOTION_FUSION				true
#define SENSORS_MOTION_FILTER				false
#define SENSORS_MOTION_FILTER_PERCENT		60
#define SENSORS_DEBUG_LOG_AHRS				true

// GPS

#define GPS_PORT							NeoSerial1
#define GPS_DEBUG_ECHO						false
#define GPS_RATE							ubxRate10Hz

// Configurator

#define CONFIGURE_SEND_GPS_TIME				200

// IO

#define IO_INPUT_FILTER_AMOUNT				10
#define IO_DEBUG_LOG_IN						false
#define IO_DEBUG_LOG_OUT					false

// Input

#define FLIGHT_STICK_CENTRE					1500
#define FLIGHT_STICK_MAX					1950
#define FLIGHT_STICK_MIN					1050

#define FLIGHT_STICK_ARM_MIN				1150
#define FLIGHT_STICK_ARM_MAX				1850

// Flight Modes

// - Manual

// - Stabilise

#define FLIGHT_STABILISE_DEADZONE			20
#define FLIGHT_STABILISE_GAIN				10
#define FLIGHT_STABILISE_GAIN_YAW			4
#define FLIGHT_STABILISE_TIME				0							// amount to divide the time gain difference
#define FLIGHT_STABILISE_TIME_YAW			10

// - Navigation

#define FLIGHT_NAVIGATE_GAIN				6
#define FLIGHT_NAVIGATE_GAIN_YAW			4
#define FLIGHT_NAVIGATE_TIME				1
#define FLIGHT_NAVIGATE_TIME_YAW			10
#define FLIGHT_NAVIGATE_MAX_WAYPOINTS		10
#define FLIGHT_NAVIGATE_ALTITUDE_MIN		20
#define FLIGHT_NAVIGATE_ALTITUDE_MAX		60		
#define FLIGHT_NAVIGATE_ALTITUDE_TARGET		30
#define FLIGHT_NAVIGATE_SPEED_TARGET		10
#define FLIGHT_NAVIGATE_ROLL_MAX			45
#define FLIGHT_NAVIGATE_ROLL_TARGET			40
#define FLIGHT_NAVIGATE_PITCH_MAX			30
#define FLIGHT_NAVIGATE_PITCH_TARGET		20
#define FLIGHT_NAVIGATE_ROLL_GAIN			2
#define FLIGHT_NAVIGATE_PITCH_GAIN			1