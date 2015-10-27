/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/stm32l4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-June-2015
  * @brief   HAL MSP module.    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uart.h"
#include "adc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
	  static DMA_HandleTypeDef         DmaHandle;

  GPIO_InitTypeDef          GPIO_InitStruct;


  /*##-1- Enable peripherals and GPIO Clocks #################################*/

  /* ADC Periph clock enable */
  ADCx_CLK_ENABLE();

  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);

  /* Enable clock of DMA associated to the peripheral */
  //ADCx_DMA_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Enable GPIO clock ****************************************/
  ADCx_GPIO_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = ADCx_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_GPIO_PORT, &GPIO_InitStruct);

  /*********************** Configure DMA parameters ***************************/
    DmaHandle.Instance                 = DMA1_Channel2;
    DmaHandle.Init.Request             = DMA_REQUEST_0;
    DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
    DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    DmaHandle.Init.Mode                = DMA_CIRCULAR;
    DmaHandle.Init.Priority            = DMA_PRIORITY_MEDIUM;

    /* Deinitialize  & Initialize the DMA for new transfer */
    HAL_DMA_DeInit(&DmaHandle);
    HAL_DMA_Init(&DmaHandle);

    /* Associate the DMA handle */
    __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

    /* NVIC configuration for DMA Input data interrupt */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{

  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();
  /* ADC Periph clock disable
   (automatically reset all ADC's) */
  __HAL_RCC_ADC_CLK_DISABLE();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */
  HAL_GPIO_DeInit(ADCx_GPIO_PORT, ADCx_GPIO_PIN);

}




/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  *           - NVIC configuration for DMA interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
//  static DMA_HandleTypeDef hdma_tx;
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();

  /* Enable USARTx clock */
  USARTx_CLK_ENABLE();

  /* Enable DMA clock */
 // DMAx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for UART ########################################*/   
//  HAL_NVIC_SetPriority(USARTx_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(USARTx_IRQn);
  
  /*##-4- Configure the DMA ##################################################*/
  /* Configure the DMA handler for Transmission process */
  /*
  hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.Request             = USARTx_TX_DMA_REQUEST;

  HAL_DMA_Init(&hdma_tx);
*/
  /* Associate the initialized DMA handle to the UART handle */
//  __HAL_LINKDMA(huart, hdmatx, hdma_tx);

  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
/*
  HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
  */
    /*##-6- Enable TIM peripherals Clock #######################################*/
 // TIMx_CLK_ENABLE();
  
  /*##-7- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */ 
//  HAL_NVIC_SetPriority(TIMx_IRQn, 2, 0);
  
  /* Enable the TIMx global Interrupt */
 // HAL_NVIC_EnableIRQ(TIMx_IRQn);

}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  TIMx_CLK_ENABLE();

  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
  HAL_NVIC_SetPriority(TIMx_IRQn, 3, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
}


/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{

  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

  /*##-3- Disable the DMA Channels ###########################################*/
  /* De-Initialize the DMA Channel associated to transmission process */
  HAL_DMA_DeInit(&hdma_tx);
  /* De-Initialize the DMA Channel associated to reception process */
  HAL_DMA_DeInit(&hdma_rx);

  /*##-4- Disable the NVIC for DMA ###########################################*/
 // HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn);
 // HAL_NVIC_DisableIRQ(USARTx_DMA_RX_IRQn);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
