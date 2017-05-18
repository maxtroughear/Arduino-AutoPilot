#include "LCD.h"

#include <NeoHWSerial.h>

#include "Scenes/Scene.h"

#include "../Sensors/Sensors.h"

#define LCD_EEPROM_TOUCH_CAL_AN 1024
#define LCD_EEPROM_TOUCH_CAL_BN LCD_EEPROM_TOUCH_CAL_AN+4
#define LCD_EEPROM_TOUCH_CAL_CN LCD_EEPROM_TOUCH_CAL_BN+4
#define LCD_EEPROM_TOUCH_CAL_DN LCD_EEPROM_TOUCH_CAL_CN+4
#define LCD_EEPROM_TOUCH_CAL_EN LCD_EEPROM_TOUCH_CAL_DN+4
#define LCD_EEPROM_TOUCH_CAL_FN LCD_EEPROM_TOUCH_CAL_EN+4
#define LCD_EEPROM_TOUCH_CAL_DIVIDER LCD_EEPROM_TOUCH_CAL_FN+4
#define LCD_EEPROM_TOUCH_CAL_DONE LCD_EEPROM_TOUCH_CAL_DIVIDER+4 // single byte (boolean)

// tft interface pins (SPI)

RA8875 LCD::tft = RA8875(TFT_SELECT, TFT_RESET);

ExponentialFilter<long> LCD::touchFilterX = ExponentialFilter<long>(60, 0);
ExponentialFilter<long> LCD::touchFilterY = ExponentialFilter<long>(60, 0);

unsigned long LCD::deltaTouchTime = 0;
unsigned long LCD::deltaUpdateTime = 0;

bool LCD::dimmed = false;
bool LCD::shouldIgnore = false;

void LCD::Initialise()
{
	tft.begin(Adafruit_800x480);

	/* Enables the display and sets up the backlight */
	NeoSerial.println("TFT Found");
	tft.displayOn(true);
	tft.GPIOX(true); // Enable TFT - display enable tied to GPIOX
	tft.backlight(true);
	tft.PWMout(true, RA8875_PWM_CLK_DIV64); // PWM output for backlight
	tft.brightness(0);

	tft.setRotation(0);

	Scenes::InitialiseLCD(tft);

	tft.clearScreen();

	//tft.layerEffect(RA8875boolean::OR);

	for (int i = 0; i <= 255; i += 5)
	{
		tft.brightness(i);
		//tft.writeTo(L2);
		tft.fillWindow(tft.Color565(i, i, i));
		//delay(10);
		//tft.writeTo(L1);
		delay(20);
	}

	NeoSerial.print("Colour mode: "); NeoSerial.println(tft.getColorBpp());

#if LCD_TOUCH_ENABLED
	/* Enable the touch screen */
	NeoSerial.println("Enabled the touch screen");
	pinMode(TFT_INTERRUPT, INPUT);
	digitalWrite(TFT_INTERRUPT, HIGH);
	tft.useINT(TFT_INTERRUPT);
	tft.touchBegin();
#endif

	//tft.fillScreen(RA8875_WHITE);
	delay(100);

	//tft.setFont();
	tft.setFontScale(1);
	tft.setCursor(0, 0, false);
	tft.setTextColor(RA8875_BLACK, RA8875_WHITE);
	tft.println("Welcome");
}

void LCD::Loop()
{
	if (millis() - deltaTouchTime >= LCD_TOUCH_DELTATIME)
	{
		shouldIgnore = false;
	}

#if LCD_DIM_ENABLED
	if (millis() - deltaTouchTime >= LCD_DIM_TIMEOUT && LCD_TOUCH_ENABLED)
	{
		// only dim if touch is enabled

		if (!dimmed)
		{
			for (int i = 255; i >= 0; i--)
			{
				tft.brightness(i);
				delay(5);
			}
		}
		shouldIgnore = true;
		dimmed = true;
	}
#endif

	if (millis() - deltaUpdateTime >= LCD_UPDATE_DELTA)
	{
		// update the display
		// render a "scene"?
		// TODO: decide on a rendering convention

		Scenes::LoopLCD();

#if LCD_TOUCH_ENABLED
		tft.touchEnable(true);
#endif

		deltaUpdateTime = millis();
	}
}

RA8875 *LCD::GetTFT()
{
	return &tft;
}

void LCD::WaitForTouch(tsPoint_t *point)
{
#if LCD_TOUCH_ENABLED
	/* Clear the touch data object and placeholder variables */
	memset(point, 0, sizeof(tsPoint_t));

	/* Clear any previous interrupts to avoid false buffered reads */
	uint16_t x, y;
	tft.touchReadPixel(&x, &y);
	delay(1);

	/* Wait around for a new touch event (INT pin goes low) */
	while (digitalRead(TFT_INTERRUPT))
	{
	}

	/* Make sure this is really a touch event */
	if (tft.touched())
	{
		tft.touchReadPixel(&x, &y);
		point->x = tft.width() - x;
		point->y = tft.height() - y;
	}
	else
	{
		point->x = 0;
		point->y = 0;
	}
#else
	point->x = 0;
	point->y = 0;
#endif
}

bool LCD::Touched()
{
#if LCD_TOUCH_ENABLED
	if (tft.touched())
	{
#if LCD_DIM_ENABLED
		if (millis() - deltaTouchTime >= LCD_DIM_TIMEOUT)
		{
			tft.brightness(255);
			deltaTouchTime = millis();
			dimmed = false;
			return false;
		}
#endif
		if (shouldIgnore)
		{
			deltaTouchTime = millis();
			return false;
		}
		shouldIgnore = true;
		return true;
	}
	else
		return false;
#else
	return false;
#endif
}

void LCD::TouchRead(tsPoint_t *point)
{
#if LCD_TOUCH_ENABLED
	memset(point, 0, sizeof(tsPoint_t));

	uint16_t x, y;
	tft.touchReadPixel(&x, &y);

	if (millis() - deltaTouchTime >= LCD_TOUCH_DELTATIME)
	{
		touchFilterX.SetCurrent(tft.width() - x);
		touchFilterY.SetCurrent(tft.height() - y);
	}

	touchFilterX.Filter(tft.width() - x);
	touchFilterY.Filter(tft.height() - y);

	point->x = touchFilterX.Current();
	point->y = touchFilterY.Current();

	deltaTouchTime = millis();
#else
	point->x = 0;
	point->y = 0;
#endif
}

uint16_t LCD::RGB24toRGB565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r / 8) << 11) | ((g / 4) << 5) | (b / 8);
}