/**
  ******************************************************************************
  * @file    stm32l476g_discovery_compass.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-June-2015
  * @brief   This file contains definitions for stm32l476g_discovery_compass.c 
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L476G_DISCOVERY_COMPASS_H
#define __STM32L476G_DISCOVERY_COMPASS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l476g_discovery.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L476G_DISCOVERY_COMPASS
  * @{
  */

/** @defgroup STM32L476G_DISCOVERY_COMPASS_Exported_Types Exported Types
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_COMPASS_Exported_Constants Exported Constants
  * @{
  */
typedef enum 
{
  COMPASS_OK = 0,
  COMPASS_ERROR = 1,
  COMPASS_TIMEOUT = 2
} 
COMPASS_StatusTypeDef;

/**
  * @}
  */

/** @defgroup STM32L476G_DISCOVERY_COMPASS_Exported_Macros Exported Macros
  * @{
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32L476G_DISCOVERY_COMPASS_Exported_Functions Exported Functions
  * @{
  */
/* COMPASS functions */
COMPASS_StatusTypeDef   BSP_COMPASS_Init(void);
void                    BSP_COMPASS_DeInit(void);
void                    BSP_COMPASS_LowPower(void);
void                    BSP_COMPASS_MagGetXYZ(int16_t *pDataXYZ);
void                    BSP_COMPASS_AccGetXYZ(int16_t *pDataXYZ);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L476G_DISCOVERY_COMPASS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
