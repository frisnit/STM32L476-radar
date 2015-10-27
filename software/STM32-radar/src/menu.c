/*
 * menu.c
 *
 *  Created on: 31 Aug 2015
 *      Author: mark
 */
#include "menu.h"

static uint8_t *mainMenu[] =
{
		"SPEED ",
		"PEAK  ",
		"WINDOW",
		"FFTLEN",
		0
};

static uint8_t *fftLengthMenu[] =
{
//		" 4k PT",
//		" 2k PT",
		" 1k PT",
		"512 PT",
		"256 PT",
		"128 PT",
		" 64 PT",
		" 32 PT",
		0
};

uint8_t menuMode;
uint8_t fftLengthIndex;
uint8_t displayMode;
uint8_t menuState;

// initialise menu variables
void menuInit(void)
{
	menuMode		= MODE_SPEED;// top level menu item
	fftLengthIndex	= FFT_LENGTH_1024;
	displayMode		= DISPLAY_MPH;
	menuState		= MENU_STATE_MAIN;// top or second level item
}

// display current menu
// or do nothing if the screen is in use by the radar
void menuDisplayMenu(void)
{
	uint8_t *string;

	if(menuState==MENU_STATE_MAIN)
	{
		string=mainMenu[menuMode];
		BSP_LCD_GLASS_DisplayString(string);
	}
	else if(menuState==MENU_STATE_FFT_LENGTH)
	{
		string=fftLengthMenu[fftLengthIndex];
		BSP_LCD_GLASS_DisplayString(string);
	}

}

void menuUp(void)
{
	// top level
	if(menuState==MENU_STATE_MAIN)
	{
		menuMode--;
		menuMode%=MODE_LAST;
	}
	// speed or peak modes
	else if(menuState==MENU_STATE_RUN)
	{
		displayMode--;
		displayMode%=DISPLAY_LAST;
	}
	// fft length mode
	else if(menuState==MENU_STATE_FFT_LENGTH)
	{
		fftLengthIndex--;
		fftLengthIndex%=FFT_LENGTH_LAST;
	}
}

void menuDown(void)
{
	if(menuState==MENU_STATE_MAIN)
	{
		menuMode++;
		menuMode%=MODE_LAST;
	}
	else if(menuState==MENU_STATE_RUN)
	{
		displayMode++;
		displayMode%=DISPLAY_LAST;
	}
	else if(menuState==MENU_STATE_FFT_LENGTH)
	{
		fftLengthIndex++;
		fftLengthIndex%=FFT_LENGTH_LAST;
	}
}

void menuLeft(void)
{
	if(menuState==MENU_STATE_RUN || menuState==MENU_STATE_FFT_LENGTH)
	{
		menuState=MENU_STATE_MAIN;
	}
}

void menuRight(void)
{
	if(menuMode==MODE_FFT_LENGTH)
		menuState=MENU_STATE_FFT_LENGTH;
	else
		menuState=MENU_STATE_RUN;
}

uint8_t menuGetMenuState(void)
{
	return menuState;
}

uint8_t menuGetMenuMode(void)
{
	return menuMode;
}

uint8_t menuGetFftLengthIndex(void)
{
	return fftLengthIndex;
}

uint8_t menuGetDisplayMode(void)
{
	return displayMode;
}

