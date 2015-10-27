/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-June-2015
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_desc.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//#define HZ_PER_MS	(float32_t)66.67f	// Hertz per m/s

#define	RADAR_FREQUENCY	10525000000.0f	// 10.525GHz
#define	SPEED_OF_LIGHT	299792458.0f	//
#define KMH_FACTOR	(float32_t)3.6f		// 1 m/s in kmh
#define MPH_FACTOR	(float32_t)2.237f	// 1 m/s in mph

/* Enable one of the following defines to select which clock will be used for USB */
#define USB_USE_LSE_MSI_CLOCK   /* Use MSI clock automatically trimmed by LSE as USB clock */
//#define USB_USE_HSE_CLOCK /* Use HSE as clock source for USB */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Toggle_Leds(void);
void setLCDBars(float32_t value);


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
