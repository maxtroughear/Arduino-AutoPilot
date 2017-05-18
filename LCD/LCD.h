#pragma once

#include <EEPROM.h>

#include <RA8875.h>
#include <Filter.h>

#include "../Config.h"
#include "../PinConfig.h"

struct tsPoint_t
{
	uint32_t x, y;
};

struct tsMatrix_t
{
	int32_t An, Bn, Cn, Dn, En, Fn, Divider;
};

class LCD
{
public:
	static void Initialise();

	static void Loop();

	static RA8875 *GetTFT();

	static void WaitForTouch(tsPoint_t *point);

	static bool Touched();
	static void TouchRead(tsPoint_t *point);

private:
	/*static void eepromWriteLong(int address, long value);
	static void eepromUpdateLong(int address, long value);
	static long eepromReadLong(int address);*/

	/*static int calTouchPoint(tsPoint_t *displayPtr, tsPoint_t *screenPtr, tsMatrix_t *matrixPtr);

	static tsPoint_t renderCalScreen(uint16_t x, uint16_t y, uint16_t radius);*/

	/*static bool loadCalMatrix();
	static int setCalMatrix(tsPoint_t *displayPtr, tsPoint_t *screenPtr, tsMatrix_t *matrixPtr);*/

	static uint16_t RGB24toRGB565(uint8_t r, uint8_t g, uint8_t b);

	static RA8875 tft;

	static ExponentialFilter<long> touchFilterX;
	static ExponentialFilter<long> touchFilterY;

	static unsigned long deltaTouchTime;
	static unsigned long deltaUpdateTime;

	static bool dimmed;
	static bool shouldIgnore;

	// calibration
	//static tsPoint_t cal_tsLCDPoints[3];
	//static tsPoint_t cal_tsTSPoints[3];
	//static tsMatrix_t cal_tsMatrix;
};