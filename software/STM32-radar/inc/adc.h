
#ifndef ADC_H_
#define ADC_H_

#include "stm32l476xx.h"

#define ADCx						ADC2

/* Definition of ADCx clock resources */
#define ADCx_CLK_ENABLE()			__HAL_RCC_ADC_CLK_ENABLE()
#define ADCx_CLK_DISABLE()			__HAL_RCC_ADC_CLK_DISABLE()

#define ADCx_FORCE_RESET()			__HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()		__HAL_RCC_ADC_RELEASE_RESET()

/* Definition of ADCx channels  PA0 */
#define SAMPLINGTIME				ADC_SAMPLETIME_24CYCLES_5
#define ADCx_GPIO_PORT				GPIOA
#define ADCx_GPIO_PIN				GPIO_PIN_0
#define ADCx_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_GPIO_CLK_DISABLE()		__HAL_RCC_GPIOA_CLK_DISABLE()

#define ADCx_CHANNEL				ADC_CHANNEL_13


/* Definition of ADCx DMA resources */
/*
#define ADCx_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define ADCx_DMA                        DMA1_Channel2

#define ADCx_DMA_IRQn                   DMA1_Channel2_IRQn//DMA1_Channel1_IRQn
#define ADCx_DMA_IRQHandler             DMA1_Channel2_IRQHandler
*/
/* Definition of ADCx NVIC resources */
#define ADCx_IRQn                       ADC1_2_IRQn
#define ADCx_IRQHandler                 ADC1_2_IRQHandler







extern uint16_t sampleInput(void);
extern uint8_t initADC(void);

extern uint16_t startAdcDma(uint32_t *buffer, uint16_t length);


#endif /* ADC_H_ */
