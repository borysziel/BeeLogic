/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_CARD_SCK_Pin GPIO_PIN_2
#define SD_CARD_SCK_GPIO_Port GPIOE
#define SD_CARD_DETECT_Pin GPIO_PIN_3
#define SD_CARD_DETECT_GPIO_Port GPIOE
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOE
#define SD_CARD_MISO_Pin GPIO_PIN_5
#define SD_CARD_MISO_GPIO_Port GPIOE
#define SD_CARD_MOSI_Pin GPIO_PIN_6
#define SD_CARD_MOSI_GPIO_Port GPIOE
#define UVC_ADC_5_Pin GPIO_PIN_0
#define UVC_ADC_5_GPIO_Port GPIOC
#define UVC_ADC_4_Pin GPIO_PIN_1
#define UVC_ADC_4_GPIO_Port GPIOC
#define UVC_ADC_3_Pin GPIO_PIN_2
#define UVC_ADC_3_GPIO_Port GPIOC
#define UVC_ADC_2_Pin GPIO_PIN_3
#define UVC_ADC_2_GPIO_Port GPIOC
#define COM_BLUE_Pin GPIO_PIN_0
#define COM_BLUE_GPIO_Port GPIOA
#define COM_RED_Pin GPIO_PIN_1
#define COM_RED_GPIO_Port GPIOA
#define COM_GREEN_Pin GPIO_PIN_2
#define COM_GREEN_GPIO_Port GPIOA
#define UVC_ADC_1_Pin GPIO_PIN_3
#define UVC_ADC_1_GPIO_Port GPIOA
#define MPU_CS_Pin GPIO_PIN_4
#define MPU_CS_GPIO_Port GPIOA
#define MPU_SCK_Pin GPIO_PIN_5
#define MPU_SCK_GPIO_Port GPIOA
#define MPU_MISO_Pin GPIO_PIN_6
#define MPU_MISO_GPIO_Port GPIOA
#define MPU_MOSI_Pin GPIO_PIN_7
#define MPU_MOSI_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_4
#define MPU_INT_GPIO_Port GPIOC
#define BATT_VOLTAGE_Pin GPIO_PIN_5
#define BATT_VOLTAGE_GPIO_Port GPIOC
#define UVC_ADC_7_Pin GPIO_PIN_0
#define UVC_ADC_7_GPIO_Port GPIOB
#define UVC_ADC_6_Pin GPIO_PIN_1
#define UVC_ADC_6_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
#define CONTROL_SENSOR_B_Pin GPIO_PIN_13
#define CONTROL_SENSOR_B_GPIO_Port GPIOE
#define CONTROL_SENSOR_A_Pin GPIO_PIN_14
#define CONTROL_SENSOR_A_GPIO_Port GPIOE
#define SENSOR_AB_SCL_Pin GPIO_PIN_10
#define SENSOR_AB_SCL_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB
#define FLASH_SCK_Pin GPIO_PIN_13
#define FLASH_SCK_GPIO_Port GPIOB
#define FLASH_MISO_Pin GPIO_PIN_14
#define FLASH_MISO_GPIO_Port GPIOB
#define FLASH_MOSI_Pin GPIO_PIN_15
#define FLASH_MOSI_GPIO_Port GPIOB
#define FLASH_RST_Pin GPIO_PIN_8
#define FLASH_RST_GPIO_Port GPIOD
#define SENSOR_D_DRDY_Pin GPIO_PIN_11
#define SENSOR_D_DRDY_GPIO_Port GPIOD
#define SENSOR_C_DRDY_Pin GPIO_PIN_12
#define SENSOR_C_DRDY_GPIO_Port GPIOD
#define SENSOR_B_DRDY_Pin GPIO_PIN_13
#define SENSOR_B_DRDY_GPIO_Port GPIOD
#define SENSOR_A_DRDY_Pin GPIO_PIN_14
#define SENSOR_A_DRDY_GPIO_Port GPIOD
#define SENSOR_C_SDA_Pin GPIO_PIN_9
#define SENSOR_C_SDA_GPIO_Port GPIOC
#define SENSOR_C_SCL_Pin GPIO_PIN_8
#define SENSOR_C_SCL_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
#define SENSOR_AB_SDA_Pin GPIO_PIN_12
#define SENSOR_AB_SDA_GPIO_Port GPIOC
#define STLINK_TX_Pin GPIO_PIN_5
#define STLINK_TX_GPIO_Port GPIOD
#define STLINK_RX_Pin GPIO_PIN_6
#define STLINK_RX_GPIO_Port GPIOD
#define MS56_CS_Pin GPIO_PIN_7
#define MS56_CS_GPIO_Port GPIOD
#define MS56_SCK_Pin GPIO_PIN_3
#define MS56_SCK_GPIO_Port GPIOB
#define MS56_MISO_Pin GPIO_PIN_4
#define MS56_MISO_GPIO_Port GPIOB
#define MS56_MOSI_Pin GPIO_PIN_5
#define MS56_MOSI_GPIO_Port GPIOB
#define SENSOR_D_SCL_Pin GPIO_PIN_6
#define SENSOR_D_SCL_GPIO_Port GPIOB
#define SENSOR_D_SDA_Pin GPIO_PIN_7
#define SENSOR_D_SDA_GPIO_Port GPIOB
#define LOOP_LED_Pin GPIO_PIN_0
#define LOOP_LED_GPIO_Port GPIOE
#define IND_LED_Pin GPIO_PIN_1
#define IND_LED_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
