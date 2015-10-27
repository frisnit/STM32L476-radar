/*
 * audio_out.h
 *
 *  Created on: 7 Sep 2015
 *      Author: mark
 */

#ifndef AUDIO_OUT_H_
#define AUDIO_OUT_H_

#include "stm32l4xx_hal.h"
#include "stm32l476g_discovery.h"
#include "audio.h"
#include "../Components/cs43l22/cs43l22.h"


// 0.5 second buffer
#define AUDIO_BUFFER_LENGTH	1024

void appendAudioSample(uint16_t sample);

#endif /* AUDIO_OUT_H_ */
