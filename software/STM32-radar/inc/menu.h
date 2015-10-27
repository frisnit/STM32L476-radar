/*
 * menu.h
 *
 *  Created on: 31 Aug 2015
 *      Author: mark
 */

#ifndef MENU_H_
#define MENU_H_

#include "stm32l476g_discovery.h"

enum
{
//	FFT_LENGTH_4096=0,
//	FFT_LENGTH_2048,
	FFT_LENGTH_1024=0,
	FFT_LENGTH_512,
	FFT_LENGTH_256,
	FFT_LENGTH_128,
	FFT_LENGTH_64,
	FFT_LENGTH_32,
	FFT_LENGTH_LAST
};

// main menu options
enum
{
	MODE_SPEED=0,
	MODE_PEAK,
	MODE_WINDOW,
	MODE_FFT_LENGTH,
	MODE_LAST
};

// display menu options
enum
{
	DISPLAY_MPH=0,
	DISPLAY_KMH,
	DISPLAY_MS,
	DISPLAY_HZ,
	DISPLAY_LAST
};

// menu states
enum
{
	MENU_STATE_MAIN=0,// top level menu
		MENU_STATE_FFT_LENGTH,// fft length select
		MENU_STATE_RUN // hand over display to radar
};


extern void menuInit(void);
extern void menuDisplayMenu(void);
extern void menuUp(void);
extern void menuDown(void);
extern void menuLeft(void);
extern void menuRight(void);
//extern void menuSelect(void);
extern uint8_t menuGetMenuState(void);
extern uint8_t menuGetDisplayMode(void);
extern uint8_t menuGetMenuMode(void);
extern uint8_t menuGetFftLengthIndex(void);

#endif /* MENU_H_ */
