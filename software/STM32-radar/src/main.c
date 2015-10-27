#include <stdio.h>
#include <stdarg.h>

#include "main.h"
#include "menu.h"
#include "adc.h"
#include "fft.h"
#include "uart.h"
#include "audio_out.h"

#include "stm32l476g_discovery_audio.h"

USBD_HandleTypeDef USBD_Device;

static uint16_t fftLength;// current FFT length

static uint16_t sampleNumber=0;
static uint16_t	sampleReady=0;// flag to set when background sampling complete

// double buffered ADC
static uint16_t adcBuffer1[FFT_MAX_SIZE];// longest FFT
static uint16_t adcBuffer2[FFT_MAX_SIZE];// longest FFT

#define MEASUREMENT_WINDOW	20
static uint8_t currentMeasurement=0;
static float32_t measurementsBuffer[MEASUREMENT_WINDOW];

uint16_t *currentAdcBuffer = adcBuffer2;

void setLCDBars(float32_t value);


// generate test audio data
uint16_t getTestSample(void)
{
	static float32_t accumulator=0.0f;
	static float32_t radiansPerSample=(1000*2.0*M_PI)/(float32_t)SAMPLE_RATE;

	accumulator+=radiansPerSample;

	if(accumulator>2.0*M_PI)
	{
		accumulator-=2.0*M_PI;
	}

	return (uint16_t)((sin(accumulator)*1000.0f)+2000.0f);
}

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void SystemClock_Config_Usb(void);
static void Error_Handler(void);



TIM_HandleTypeDef    AdcTimHandle;

float32_t signalLevel=0.0f;

// display peripherals
enum
{
	PERIPHERAL_LCD,// the built in LCD screen
	PERIPHERAL_LED,// the external LED screen (over UART)
	PERIPHERAL_USB // the USB serial device
};

// send a formatted string to LCD, LED or USB
void peripheralPrintf(uint16_t peripheral, char *format, ...)
{
	char buffer[64];

	va_list args;
	va_start(args, format);
	vsprintf(buffer, format, args);
	va_end(args);

	switch(peripheral)
	{
	case PERIPHERAL_LCD:
		BSP_LCD_GLASS_DisplayString(buffer);
		setLCDBars(signalLevel);
		break;
	case PERIPHERAL_LED:
		UartTx(buffer, strlen(buffer));
		break;
	case PERIPHERAL_USB:
		USBD_CDC_SetTxBuffer(&USBD_Device, buffer, strlen(buffer));
		USBD_CDC_TransmitPacket(&USBD_Device);
		break;
	default:
		break;
	}
}


// set timer interrupt for 3kHz sample rate
void initSampleClock(void)
{
	uint32_t uwPrescalerValue = 0;


	// SystemCoreClock = 80MHz
	/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
	  uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000000) - 1;

	  /* Set TIMx instance */
	  AdcTimHandle.Instance = TIM4;

	  /* Initialize TIMx peripheral as follows:
	       + Period = 3000 - 1
	       + Prescaler = (SystemCoreClock/10000) - 1
	       + ClockDivision = 0
	       + Counter direction = Up
	  */

	  AdcTimHandle.Init.Period            = (1000000.0f/(float)SAMPLE_RATE) - 1;
	  AdcTimHandle.Init.Prescaler         = uwPrescalerValue;
	  AdcTimHandle.Init.ClockDivision     = 0;
	  AdcTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	//  AdcTimHandle.Init.RepetitionCounter = 0;

	  __HAL_RCC_TIM4_CLK_ENABLE();

	  if(HAL_TIM_Base_Init(&AdcTimHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

	  if (HAL_TIM_Base_Start_IT(&AdcTimHandle) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /*##-1- Enable peripherals and GPIO Clocks #################################*/
	  /* TIMx Peripheral clock enable */

	  /*##-2- Configure the NVIC for TIMx ########################################*/
	  /* Set the TIMx priority */
	  HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);

	  /* Enable the TIMx global Interrupt */
	  HAL_NVIC_EnableIRQ(TIM4_IRQn);

}


// sampling clock interrupt
void TIM4_IRQHandler(void)
{
	static uint16_t max = 0;
	static uint16_t min = 0xfff;

	    uint16_t value = sampleInput();

	    if(value>max)
	    {
	    	max = value;
	    }

	    if(value<min)
	    {
	    	min = value;
	    }

	    // put samples in the buffer that's NOT being used by the foreground
	    if(currentAdcBuffer==&adcBuffer1)
	    {
	    	adcBuffer2[sampleNumber] = value;
	    }
	    else
	    {
	    	adcBuffer1[sampleNumber] = value;
	    }

	    // send sample to audio output
	    appendAudioSample(value);

		sampleNumber++;

		// filled this buffer so inform foreground and switch to next one
		if(sampleNumber>=fftLength)
		{
			// if the foreground has finished with the last buffer...
			if(sampleReady==0)
			{
				if(currentAdcBuffer==&adcBuffer1)
				{
					currentAdcBuffer=adcBuffer2;
				}
				else
				{
					currentAdcBuffer=adcBuffer1;
				}

				sampleReady=1;
			}

			sampleNumber=0;
		}


	    __HAL_TIM_CLEAR_FLAG(&AdcTimHandle, TIM_FLAG_UPDATE);
}


void displayData(float32_t hertz)
{
	// calculate the number of Doppler Hertz per m/s for this radar frequency;
	static float32_t hertzPerMs = 2.0f*RADAR_FREQUENCY/SPEED_OF_LIGHT;

	float32_t metersPerSecond;
	float32_t mph;
	float32_t kmh;

	metersPerSecond	= hertz/hertzPerMs;
	mph				= metersPerSecond*MPH_FACTOR;
	kmh				= metersPerSecond*KMH_FACTOR;

	switch(menuGetDisplayMode())
	{
	case DISPLAY_MPH:
		peripheralPrintf(PERIPHERAL_LCD,"%2d MPH",(uint16_t)mph);
		peripheralPrintf(PERIPHERAL_LED,"% 9.1f\r",mph);
		break;
	case DISPLAY_KMH:
		peripheralPrintf(PERIPHERAL_LCD,"%2d KMH",(uint16_t)kmh);
		peripheralPrintf(PERIPHERAL_LED,"% 9.1f\r",kmh);
		break;
	case DISPLAY_MS:
		peripheralPrintf(PERIPHERAL_LCD,"%3d ms",(uint16_t)metersPerSecond);
		peripheralPrintf(PERIPHERAL_LED,"% 9.1f\r",metersPerSecond);
		break;
	case DISPLAY_HZ:
		peripheralPrintf(PERIPHERAL_LCD,"%4dHz",(uint16_t)hertz);
		peripheralPrintf(PERIPHERAL_LED,"% 9.1f\r",hertz);
		break;
	default:
		break;
	}
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int main(void)
{
	HAL_Init();

	SystemClock_Config_Usb();

	__HAL_RCC_PWR_CLK_ENABLE();

    /* enable USB power on Pwrctrl CR2 register */
    HAL_PWREx_EnableVddUSB();

	BSP_LCD_GLASS_Init();
	BSP_LED_Init(LED_RED);
	BSP_LED_Init(LED_GREEN);

	BSP_LCD_GLASS_Clear();

	BSP_LCD_GLASS_DisplayString("INIT  ");

	fftLength = getFftLength(menuGetFftLengthIndex());


	if(initADC()!=HAL_OK)
	{
		BSP_LCD_GLASS_DisplayString("ADCERR");
		while(1);
	}

	if(UartConfig() != HAL_OK)
	{
		BSP_LED_On(LED_RED);
		BSP_LCD_GLASS_DisplayString("UARTXX");
		while(1);
	}

	// display a single dp to show we're on
	UartTx("        .\r", 10);

	// start USB CDC device

  if(USBD_Init(&USBD_Device, &VCP_Desc, 0) == USBD_FAIL)
  {
    BSP_LED_On(LED_RED);
    while(1);
  }


  if(USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS) == USBD_FAIL)
      {
        BSP_LED_On(LED_RED);
        while(1);
      }

 //  Add CDC Interface Class
  if(USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops) == USBD_FAIL)
    {
      BSP_LED_On(LED_RED);
      while(1);
    }

  if(USBD_Start(&USBD_Device) == USBD_FAIL)
    {
      BSP_LED_On(LED_RED);
      while(1);
    }

   BSP_LCD_GLASS_DisplayString("READY ");


   // set up SAI interface and DMA
   initAudio();

   // start first DMA transfer
   // subsequent transfers will be started in DMA progress callbacks
   startPlayback();

	// init sampling clock
	initSampleClock();


	assert_param(1==0);

	int n=0;

	menuInit();
	menuDisplayMenu();

	BSP_JOY_Init(JOY_MODE_GPIO);

	float32_t hertz=0.0f;
	float32_t maxHertz=0.0f;

	float32_t average=0.0f;// data for 'signal strength' meter bars
	float32_t output[1024];
	uint32_t maxIndex=0;// index of largest bin

	JOYState_TypeDef joyState=JOY_NONE;

	while(1)
	{
		JOYState_TypeDef newState = BSP_JOY_GetState();

		if(joyState!=newState)
		{
			joyState=newState;

			switch(joyState)
			{
			case JOY_UP:
				menuUp();
				break;

			case JOY_DOWN:
				menuDown();
				break;

			case JOY_SEL:
			//	menuSelect();
				break;

			case JOY_LEFT:
				menuLeft();
				break;

			case JOY_RIGHT:
				menuRight();
				maxHertz=0.0f;// reset peak reading
				break;

			case JOY_NONE:
			default:
				break;
			}

			// update things that might have changed

			// update display on LCD and external LED
			if(menuGetMenuState()==MENU_STATE_RUN)
			{
				displayData(hertz);
			}

			fftLength = getFftLength(menuGetFftLengthIndex());
			//peripheralPrintf(PERIPHERAL_LED,"% 8d\r",fftLength);

		}

		// 6.67Hz resolution = 0.20m/s @ 10GHz = 0.72km/h = 0.45mph
		// so bins must be 6Hz
		// FFT must have at least 250 real bins
		// FFT size must be 250*2 = 500 points, make it 512
		// each bin will be 1500/256 Hz = 5.86Hz ~= 0.1758m/s
		// so sample length must be 512 points, at 3kHz gives a measurement time of ~150ms

		// sample the output from the radar, process it and display the result
		if(menuGetMenuState()==MENU_STATE_RUN)
		{
			// the input buffer has been filled in the background by the timer IRQ
			if(sampleReady)
			{

				// find the peak frequency in the sampled data
				doFFT(currentAdcBuffer, &hertz, &average, output, &maxIndex,menuGetFftLengthIndex());


				// is it above the detection threshold?
				if(output[maxIndex]<3.0f)
				{
					hertz=0.0f;
				}

				if(maxHertz<hertz)
					maxHertz=hertz;

				// show the max speed since last reset
				if(menuGetMenuMode()==MODE_PEAK)
					hertz=maxHertz;


				// maintain rolling window of measurements
				measurementsBuffer[currentMeasurement]=hertz;
				currentMeasurement++;

				if(currentMeasurement==MEASUREMENT_WINDOW)
				{
					currentMeasurement=0;
				}

				// show the max speed over last few measurements
				if(menuGetMenuMode()==MODE_WINDOW)
				{
					float32_t result;
					uint32_t index;
					arm_max_f32(measurementsBuffer, MEASUREMENT_WINDOW, &result, &index);
					hertz=result;
				}

				// display on LCD and external LED
				displayData(hertz);

				// display 'signal level' bars
				signalLevel = output[maxIndex]/40.0f;



				//HAL_Delay(500);


				// send debug info over USB
				peripheralPrintf(PERIPHERAL_USB,"F, ");// frequency domain data
				for(n=0;n<(fftLength/2);n++)
				{
					peripheralPrintf(PERIPHERAL_USB,"%.3f%s",output[n],n<(fftLength/2-1)?", ":"\n");
				}

				// send debug info over USB
				peripheralPrintf(PERIPHERAL_USB,"T, ");// time domain data
				for(n=0;n<fftLength;n++)
				{
					peripheralPrintf(PERIPHERAL_USB,"%.3f%s",(float32_t)currentAdcBuffer[n]/4095.0f,n<fftLength-1?", ":"\n");// time domain data
				}

				// clear the ready flag
				sampleReady=0;

				//BSP_LED_Off(LED_RED);

			}
			//
		}
		else
		{
			menuDisplayMenu();
			HAL_Delay(10);
		}
	}
}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *
  *         If define USB_USE_LSE_MSI_CLOCK enabled:
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 2
  *            MSI Frequency(Hz)              = 4800000
  *            LSE Frequency(Hz)              = 32768
  *            PLL_M                          = 6
  *            PLL_N                          = 40
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            PLL_R                          = 4
  *            Flash Latency(WS)              = 4
  * 
  *         If define USB_USE_HSE_CLOCK enabled:
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 1
  *            PLL_N                          = 24
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            PLL_R                          = 4
  *            Flash Latency(WS)              = 4
  * 
  * @param  None
  * @retval None
  */
static void SystemClock_Config_Usb(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	#if defined (USB_USE_LSE_MSI_CLOCK)

	  /* Enable the LSE Oscilator */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
	  HAL_RCCEx_DisableLSECSS();

	  /* Enable MSI Oscillator and activate PLL with MSI as source */
	  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
	  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
	  RCC_OscInitStruct.PLL.PLLM            = 6;
	  RCC_OscInitStruct.PLL.PLLN            = 40;
	  RCC_OscInitStruct.PLL.PLLP            = 7;
	  RCC_OscInitStruct.PLL.PLLQ            = 4;
	  RCC_OscInitStruct.PLL.PLLR            = 4;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Enable MSI Auto-calibration through LSE */
	  HAL_RCCEx_EnableMSIPLLMode();

	  /* Select MSI output as USB clock source */
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
	  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	  clocks dividers */
	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	  {
	    Error_Handler();
	  }

	#elif defined (USB_USE_HSE_CLOCK)

	  /* Enable HSE Oscillator and activate PLL with HSE as source */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 1;
	  RCC_OscInitStruct.PLL.PLLN = 24;
	  RCC_OscInitStruct.PLL.PLLR = 4;
	  RCC_OscInitStruct.PLL.PLLP = 7;
	  RCC_OscInitStruct.PLL.PLLQ = 4;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /*Select Main PLL output as USB clock source */
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
	  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);


	  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	  clocks dividers */
	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	  {
	    Error_Handler();
	  }

	#endif /* USB_USE_LSE_MSI_CLOCK */
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	BSP_LED_On(LED_RED);
	while (1);
}

// set the LCD bars in the range 0.0 <= value <= 1.0
void setLCDBars(float32_t value)
{
	if(value>=0.2f)
		BSP_LCD_GLASS_DisplayBar(LCD_BAR_0);
	else
		BSP_LCD_GLASS_ClearBar(LCD_BAR_0);

	if(value>=0.4f)
		BSP_LCD_GLASS_DisplayBar(LCD_BAR_1);
	else
		BSP_LCD_GLASS_ClearBar(LCD_BAR_1);

	if(value>=0.60f)
		BSP_LCD_GLASS_DisplayBar(LCD_BAR_2);
	else
		BSP_LCD_GLASS_ClearBar(LCD_BAR_2);

	if(value>=0.8f)
		BSP_LCD_GLASS_DisplayBar(LCD_BAR_3);
	else
		BSP_LCD_GLASS_ClearBar(LCD_BAR_3);
}

/**
  * @brief  Toggle LEDs to shows user input state.   
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  static uint32_t ticks;
  
  if(ticks++ == 100)
  {
    BSP_LED_Toggle(LED_GREEN);
    
    ticks = 0;
  }  
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	peripheralPrintf (PERIPHERAL_LCD,"ASSERT");
	peripheralPrintf (PERIPHERAL_USB,"Wrong parameters value: file %s on line %d\r\n", file, line);


  /* Infinite loop */
  while (1)
  {
  }
}
#endif

// From Joseph Yiu, minor edits by FVH
// hard fault handler in C,
// with stack frame location as input parameter
// called from HardFault_Handler in file xxx.s

void hard_fault_handler_c (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  peripheralPrintf (PERIPHERAL_LCD,"FAULT ");

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  peripheralPrintf (PERIPHERAL_USB,"\n\n[Hard fault handler - all numbers in hex]\n");
  peripheralPrintf (PERIPHERAL_USB,"R0 = %x\n", stacked_r0);
  peripheralPrintf (PERIPHERAL_USB,"R1 = %x\n", stacked_r1);
  peripheralPrintf (PERIPHERAL_USB,"R2 = %x\n", stacked_r2);
  peripheralPrintf (PERIPHERAL_USB,"R3 = %x\n", stacked_r3);
  peripheralPrintf (PERIPHERAL_USB,"R12 = %x\n", stacked_r12);
  peripheralPrintf (PERIPHERAL_USB,"LR [R14] = %x  subroutine call return address\n", stacked_lr);
  peripheralPrintf (PERIPHERAL_USB,"PC [R15] = %x  program counter\n", stacked_pc);
  peripheralPrintf (PERIPHERAL_USB,"PSR = %x\n", stacked_psr);
  peripheralPrintf (PERIPHERAL_USB,"BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
  peripheralPrintf (PERIPHERAL_USB,"CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
  peripheralPrintf (PERIPHERAL_USB,"HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
  peripheralPrintf (PERIPHERAL_USB,"DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
  peripheralPrintf (PERIPHERAL_USB,"AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
  peripheralPrintf (PERIPHERAL_USB,"SCB_SHCSR = %x\n", SCB->SHCSR);

  while (1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
