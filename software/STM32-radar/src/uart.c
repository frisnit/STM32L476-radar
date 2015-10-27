/*
 * uart.c
 *
 *  Created on: 31 Aug 2015
 *      Author: mark
 */

#include "uart.h"

UART_HandleTypeDef UartHandle;


uint16_t UartTx(uint8_t *data, uint16_t length)
{
/*
	while (HAL_USART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	  {
	  }
*/
	return HAL_UART_Transmit(&UartHandle, data, length, 500);
}

uint16_t UartConfig(void)
{

	if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
	{
		return HAL_ERROR;
	}

	UartHandle.Instance				= USARTx;
	UartHandle.Init.StopBits		= UART_STOPBITS_1;
	UartHandle.Init.Parity			= UART_PARITY_NONE;
	UartHandle.Init.WordLength		= UART_WORDLENGTH_8B;
	UartHandle.Init.BaudRate		= 9600;
	UartHandle.Init.HwFlowCtl		= UART_HWCONTROL_NONE;
	UartHandle.Init.Mode			= UART_MODE_TX_RX;
	UartHandle.Init.OverSampling	= UART_OVERSAMPLING_8;//UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		/* Initialization Error */
		return HAL_ERROR;
	}

	return HAL_OK;

}

