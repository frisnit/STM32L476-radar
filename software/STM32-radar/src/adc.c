
#include "stm32l476g_discovery.h"
#include "adc.h"

static ADC_HandleTypeDef		hAdc;
static ADC_ChannelConfTypeDef	hAdc_AdcChannelConfig;

/**
  * @brief  Initializes ADC HAL.
  * @retval None
  */
static HAL_StatusTypeDef ADCx_Init(void)
{
	hAdc.Instance					= ADCx;

	if (HAL_ADC_DeInit(&hAdc) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (HAL_ADC_GetState(&hAdc) == HAL_ADC_STATE_RESET)
	{
		hAdc.Init.ClockPrescaler		= ADC_CLOCK_ASYNC_DIV1;
		hAdc.Init.Resolution			= ADC_RESOLUTION_12B;
		hAdc.Init.DataAlign				= ADC_DATAALIGN_RIGHT;
		hAdc.Init.ScanConvMode			= DISABLE;
		hAdc.Init.EOCSelection			= ADC_EOC_SINGLE_CONV;
		hAdc.Init.LowPowerAutoWait		= DISABLE;
		hAdc.Init.ContinuousConvMode	= DISABLE;
		hAdc.Init.NbrOfConversion		= 1;
		hAdc.Init.DiscontinuousConvMode	= DISABLE;
		hAdc.Init.NbrOfDiscConversion	= 1;
		hAdc.Init.ExternalTrigConv		= ADC_SOFTWARE_START;
		hAdc.Init.ExternalTrigConvEdge	= ADC_EXTERNALTRIGCONVEDGE_NONE;
		hAdc.Init.DMAContinuousRequests	= ENABLE;
		hAdc.Init.Overrun				= ADC_OVR_DATA_OVERWRITTEN;
		hAdc.Init.OversamplingMode		= DISABLE;

		// set up the clocks
		__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
		ADCx_CLK_ENABLE();

		if (HAL_ADC_Init(&hAdc) != HAL_OK)
		{
			return HAL_ERROR;
		}

		hAdc_AdcChannelConfig.Channel 		= ADCx_CHANNEL;
		hAdc_AdcChannelConfig.SamplingTime	= SAMPLINGTIME;
		hAdc_AdcChannelConfig.Rank			= ADC_REGULAR_RANK_1;
		hAdc_AdcChannelConfig.SingleDiff	= ADC_SINGLE_ENDED;
		hAdc_AdcChannelConfig.OffsetNumber	= ADC_OFFSET_NONE;
		hAdc_AdcChannelConfig.Offset		= 0;

		if (HAL_ADC_ConfigChannel(&hAdc, &hAdc_AdcChannelConfig) != HAL_OK)
		{
			return HAL_ERROR;
		}

	    HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE1);
	    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);
	    HAL_SYSCFG_EnableVREFBUF();

		if (HAL_ADCEx_Calibration_Start(&hAdc, ADC_SINGLE_ENDED) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}


// main function to initialise the ADC and input pins
uint8_t initADC(void)
{
	if (ADCx_Init() != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

// read a sample value from the ADC
uint16_t sampleInput(void)
{
	uint16_t value = 0;

	if (HAL_ADC_Start(&hAdc) != HAL_OK)
	{
	   return HAL_ERROR;
	}

	if (HAL_ADC_PollForConversion(&hAdc, 10) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if((HAL_ADC_GetState(&hAdc) & HAL_ADC_STATE_REG_EOC) !=  HAL_ADC_STATE_REG_EOC)
	{
		return HAL_ERROR;
	}

	value = HAL_ADC_GetValue(&hAdc);

	return value&0x0fff;// 12 bit result

}





