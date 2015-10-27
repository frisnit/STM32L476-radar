/*
 * audio_out.c
 *
 *  Created on: 7 Sep 2015
 *      Author: mark
 */


#include "audio_out.h"
#include "arm_math.h"
#include "stm32l4xx.h"
#include "stm32l476g_discovery_audio.h"


#define SAMPLE_FREQUENCY	SAI_AUDIO_FREQUENCY_8K

SAI_HandleTypeDef            SaiHandle;
//DMA_HandleTypeDef            hSaiDma;

int16_t                      playBuffer[AUDIO_BUFFER_LENGTH];
int16_t                      *incomingBuffer=&playBuffer[0];//[AUDIO_BUFFER_LENGTH];

static int16_t incomingBufferPointer=0;

void AudioPlayer_Error_CallBack(void);
void AudioPlayer_TransferComplete_CallBack(void);
void AudioPlayer_HalfTransfer_CallBack(void);

//void copyFromIncomingBuffer(uint16_t *destination, uint16_t length);


static void Playback_Init(void);


// create a 1kHz tone in the playback buffer
void createAudioData(void)
{
	static float32_t twoPi = 2.0f*M_PI;
	static float32_t accumulator1=0.0f;
	static float32_t accumulator2=0.0f;
	uint16_t i;

	float32_t radiansPerSample1=(1000*twoPi)/(float32_t)SAMPLE_FREQUENCY;
	float32_t radiansPerSample2=(500*twoPi)/(float32_t)SAMPLE_FREQUENCY;

	for(i=0;i<AUDIO_BUFFER_LENGTH;i++)
	{
		/*
		if(i<AUDIO_BUFFER_LENGTH/2)
			playBuffer[i]=(int16_t)(sin(accumulator1)*32768.0f)+32768;
		else
			playBuffer[i]=(int16_t)(sin(accumulator2)*32768.0f)+32768;
*/


		playBuffer[i]=(int16_t)0;
		/*
		accumulator1+=radiansPerSample1;
		if(accumulator1>twoPi)
		{
			accumulator1-=twoPi;
		}

		accumulator2+=radiansPerSample2;
		if(accumulator2>twoPi)
		{
			accumulator2-=twoPi;
		}
*/
	}
}

void appendAudioSample(uint16_t sample)
{
//	playBuffer[incomingBufferPointer]=sample;


	// amplify
	sample<<=3;
/*
	if(incomingBufferPointer<AUDIO_BUFFER_LENGTH/4)
	{
		incomingBuffer[incomingBufferPointer*2]=sample;
		incomingBuffer[incomingBufferPointer*2+1]=0;
		incomingBufferPointer+=2;
	}
*/

	if(incomingBufferPointer<AUDIO_BUFFER_LENGTH/2)
	{
		incomingBuffer[incomingBufferPointer]=sample;
		incomingBufferPointer++;
	}

	/*
	// interpolate from 4kHz to 8kHz
//	playBuffer[incomingBufferPointer]=0;
//	playBuffer[incomingBufferPointer]=sample;
	incomingBuffer[incomingBufferPointer]=sample;

	incomingBufferPointer++;
	incomingBufferPointer%=AUDIO_BUFFER_LENGTH;
*/
}


void initAudio(void)
{
//	createAudioData();// for testing

	//Playback_Init();


	if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 75, SAMPLE_FREQUENCY) == AUDIO_OK)
	{
		BSP_AUDIO_OUT_ChangeAudioConfig(BSP_AUDIO_OUT_MONOMODE|BSP_AUDIO_OUT_CIRCULARMODE);
		BSP_AUDIO_OUT_RegisterCallbacks(AudioPlayer_Error_CallBack,
							   AudioPlayer_HalfTransfer_CallBack,
							   AudioPlayer_TransferComplete_CallBack);
	}
}

void startPlayback(void)
{
	if(cs43l22_drv.Play(AUDIO_I2C_ADDRESS, NULL, 0)!=0)
	{
		BSP_LCD_GLASS_DisplayString("PLAY X");

		Error_Handler();
	}


	BSP_AUDIO_OUT_Play((uint16_t*)playBuffer, AUDIO_BUFFER_LENGTH);
	/*
	if(HAL_SAI_Transmit_DMA(&SaiHandle, (uint8_t *)playBuffer, AUDIO_BUFFER_LENGTH) != HAL_OK)
	{
		BSP_LCD_GLASS_DisplayString("DMASRT");

		Error_Handler();
	}
	*/
}

/**
  * @brief  Playback initialization
  * @param  None
  * @retval None
  */
static void Playback_Init(void)
{
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

  /* Configure and enable PLLSAI1 clock to generate 11.294MHz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
  RCC_PeriphCLKInitStruct.PLLSAI1.PLLSAI1N        = 24;
  RCC_PeriphCLKInitStruct.PLLSAI1.PLLSAI1P        = 17;
  RCC_PeriphCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  RCC_PeriphCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;


  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
  {

		BSP_LCD_GLASS_DisplayString("CFGERR");


    Error_Handler();
  }

  /* Initialize SAI */
  __HAL_SAI_RESET_HANDLE_STATE(&SaiHandle);

  SaiHandle.Instance = SAI1_Block_A;

  SaiHandle.Init.AudioMode      = SAI_MODEMASTER_TX;
  SaiHandle.Init.Synchro        = SAI_ASYNCHRONOUS;
  SaiHandle.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
  SaiHandle.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  SaiHandle.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  SaiHandle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  SaiHandle.Init.AudioFrequency = SAMPLE_FREQUENCY;
  SaiHandle.Init.Mckdiv         = 0; /* N.U */
  SaiHandle.Init.MonoStereoMode = SAI_MONOMODE;//SAI_STEREOMODE;
  SaiHandle.Init.CompandingMode = SAI_NOCOMPANDING;
  SaiHandle.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
  SaiHandle.Init.Protocol       = SAI_FREE_PROTOCOL;
  SaiHandle.Init.DataSize       = SAI_DATASIZE_16;
  SaiHandle.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  SaiHandle.Init.ClockStrobing  = SAI_CLOCKSTROBING_RISINGEDGE;

  SaiHandle.FrameInit.FrameLength       = 32;
  SaiHandle.FrameInit.ActiveFrameLength = 16;
  SaiHandle.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  SaiHandle.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  SaiHandle.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

  SaiHandle.SlotInit.FirstBitOffset = 0;
  SaiHandle.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
  SaiHandle.SlotInit.SlotNumber     = 2;
  SaiHandle.SlotInit.SlotActive     = (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1);

  if(HAL_OK != HAL_SAI_Init(&SaiHandle))
  {
		BSP_LCD_GLASS_DisplayString("SAIERR");

    Error_Handler();
  }

  /* Enable SAI to generate clock used by audio driver */
  __HAL_SAI_ENABLE(&SaiHandle);

  /* Initialize audio driver */
  if (cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS) != CS43L22_ID)
  {

		BSP_LCD_GLASS_DisplayString("ID ERR");


	  Error_Handler();
  }

  /* Reset the audio codec Registers */
  /* Initialize the audio driver structure */
  cs43l22_drv.Reset(AUDIO_I2C_ADDRESS);

  /* Initialize the audio codec internal registers */
  if(cs43l22_drv.Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 75, SAMPLE_FREQUENCY) != 0)
  {

		BSP_LCD_GLASS_DisplayString("INTERR");

    Error_Handler();
  }

}

/**
  * @brief  SAI MSP Init.
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
/*
void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  GPIO_Init;

  __HAL_RCC_SAI1_CLK_ENABLE();

  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_Init.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init.Pull      = GPIO_NOPULL;
  GPIO_Init.Speed     = GPIO_SPEED_HIGH;
  GPIO_Init.Alternate = GPIO_AF13_SAI1;
  GPIO_Init.Pin       = (GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

  HAL_GPIO_Init(GPIOE, &GPIO_Init);

  __HAL_RCC_DMA2_CLK_ENABLE();
  hSaiDma.Init.Request             = DMA_REQUEST_1;
  hSaiDma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hSaiDma.Init.PeriphInc           = DMA_PINC_DISABLE;
  hSaiDma.Init.MemInc              = DMA_MINC_ENABLE;
  hSaiDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hSaiDma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hSaiDma.Init.Mode                = DMA_CIRCULAR;
  hSaiDma.Init.Priority            = DMA_PRIORITY_HIGH;
  hSaiDma.Instance                 = DMA2_Channel1;
  __HAL_LINKDMA(hsai, hdmatx, hSaiDma);
  if (HAL_OK != HAL_DMA_Init(&hSaiDma))
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0x01, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}
*/
void AudioPlayer_Error_CallBack(void)
{
	BSP_LCD_GLASS_DisplayString("ERRCBK");
//	BSP_LED_On(LED_RED);
}

void AudioPlayer_TransferComplete_CallBack(void)
{

	incomingBuffer=&playBuffer[AUDIO_BUFFER_LENGTH/2];
	incomingBufferPointer=0;

	/*
	incomingBufferPointer=AUDIO_BUFFER_LENGTH/2;
	BSP_LED_Toggle(LED_RED);


	if(HAL_SAI_Transmit_DMA(&SaiHandle, (uint8_t *)&playBuffer[0], AUDIO_BUFFER_LENGTH/2) != HAL_OK)
	{
		BSP_LCD_GLASS_DisplayString("CPTTFR");
		Error_Handler();
	}


	// start accumulating data in last half of buffer
	incomingBuffer=&playBuffer[AUDIO_BUFFER_LENGTH/2];
	incomingBufferPointer=0;
	*/

}

void AudioPlayer_HalfTransfer_CallBack(void)
{
	//BSP_LED_Toggle(LED_GREEN);

	incomingBuffer=&playBuffer[0];
	incomingBufferPointer=0;

	/*
	HAL_StatusTypeDef result;
	result = HAL_SAI_Transmit_DMA(&SaiHandle, (uint8_t *)&playBuffer[AUDIO_BUFFER_LENGTH/2], AUDIO_BUFFER_LENGTH/2);

	if(result != HAL_OK)
	{
		uint8_t string[16];
		sprintf(string,"HF %03d", (uint16_t)result);
		BSP_LCD_GLASS_DisplayString(string);
		//Error_Handler();
	}

	// start accumulating data in first half of buffer
	incomingBuffer=&playBuffer[0];
	incomingBufferPointer=0;
	*/
}
//
/*
void copyFromIncomingBuffer(uint16_t *destination, uint16_t length)
{
	uint16_t n;
	uint16_t ptr=(incomingBufferPointer-length)%AUDIO_BUFFER_LENGTH;

	for(n=0;n<AUDIO_BUFFER_LENGTH;n++)
	{
		destination[n]=incomingBuffer[ptr];

		ptr++;

	}

}
*/

// TODO - remove this
void Error_Handler(void)
{
  /* LED1 On in error case */
  BSP_LED_On(LED_RED);
  //BSP_LCD_GLASS_DisplayString("ERROR ");

  while (1)
  {
  }
}
