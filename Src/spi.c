/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = ADS1263_SCK_Pin|ADS1263_DOUT_Pin|ADS1263_DIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = ADIS_SCLK_Pin|ADIS_DOUT_Pin|ADIS_DIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, ADS1263_SCK_Pin|ADS1263_DOUT_Pin|ADS1263_DIN_Pin);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, ADIS_SCLK_Pin|ADIS_DOUT_Pin|ADIS_DIN_Pin);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
