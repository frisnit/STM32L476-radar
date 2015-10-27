/*
 * fft.h
 *
 *  Created on: 30 Aug 2015
 *      Author: mark
 */

#ifndef FFT_FFT_H_
#define FFT_FFT_H_

#include "arm_math.h"
#include "arm_const_structs.h"

#define FFT_MAX_SIZE	4096//1024
#define SAMPLE_RATE		8000

extern void doFFT(uint16_t *inputBuffer, float32_t *hertz, float32_t* average, float32_t output[], uint32_t *maxIndex, uint16_t fftLengthIndex);
extern uint16_t getFftLength(uint16_t index);

#endif /* FFT_FFT_H_ */
