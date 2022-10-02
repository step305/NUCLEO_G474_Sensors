/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	float rate[3];
	float acc[3];
	float adc[3];
	float temper;
	uint32_t cntr;
} ADIS_DATA_Type;


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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define ADIS_D1_Pin GPIO_PIN_0
#define ADIS_D1_GPIO_Port GPIOC
#define ADIS_D2_DRDY_Pin GPIO_PIN_1
#define ADIS_D2_DRDY_GPIO_Port GPIOC
#define ADIS_D2_DRDY_EXTI_IRQn EXTI1_IRQn
#define ADIS_CS_Pin GPIO_PIN_2
#define ADIS_CS_GPIO_Port GPIOC
#define ADIS_RST_Pin GPIO_PIN_3
#define ADIS_RST_GPIO_Port GPIOC
#define ADIS_D4_Pin GPIO_PIN_0
#define ADIS_D4_GPIO_Port GPIOA
#define ADIS_D3_Pin GPIO_PIN_1
#define ADIS_D3_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ADS1263_SCK_Pin GPIO_PIN_13
#define ADS1263_SCK_GPIO_Port GPIOB
#define ADS1263_DOUT_Pin GPIO_PIN_14
#define ADS1263_DOUT_GPIO_Port GPIOB
#define ADS1263_DIN_Pin GPIO_PIN_15
#define ADS1263_DIN_GPIO_Port GPIOB
#define ADS1263_RESET_Pin GPIO_PIN_9
#define ADS1263_RESET_GPIO_Port GPIOA
#define ADS1263_CS_Pin GPIO_PIN_10
#define ADS1263_CS_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define ADIS_SCLK_Pin GPIO_PIN_10
#define ADIS_SCLK_GPIO_Port GPIOC
#define ADIS_DOUT_Pin GPIO_PIN_11
#define ADIS_DOUT_GPIO_Port GPIOC
#define ADIS_DIN_Pin GPIO_PIN_12
#define ADIS_DIN_GPIO_Port GPIOC
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define ADS1263_DRDY_Pin GPIO_PIN_6
#define ADS1263_DRDY_GPIO_Port GPIOB
#define ADS1263_DRDY_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
