/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "packet.h"
#include "string.h"
#include "ADIS16485.h"
#include "ADS126x.h"
#include "ADIS16505.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADIS_DATA_Type imu_data;

uint8_t UART_buffer[128] = {0,};
uint8_t UART_buffer_shadow[128] = {0,};
uint8_t UART_empty = 1;
uint16_t UART_buffer_len = 0;

__IO uint8_t adis_ready = 0;
uint32_t t0, t1;

ADS126xData adc_result;
uint8_t ADS126x_Init_Done = 0;

float ADS126xVolt[20] = {0.0f,};
uint8_t ADS126xCnt = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	UART_empty = 1;
}

void Transmit_UART(uint8_t* data, uint16_t size) {
	if (UART_empty ==1) {
		memcpy(UART_buffer_shadow, (uint8_t*)data, size);
		HAL_UART_Transmit_DMA(&hlpuart1, UART_buffer_shadow, size);
		UART_empty = 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ADIS_D2_DRDY_Pin) {
        adis_ready = 1;
    } else if (GPIO_Pin == ADS1263_DRDY_Pin && ADS126x_Init_Done == 1) {
    	ADS126x_Ready_flag = 1;
    }
}

float mean_val(float* arr, uint8_t len) {
	float sum = 0.0f;
	for (uint8_t i = 0; i < len; i++) {
		sum += arr[i];
	}
	return sum/(float)len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t test;
	uint16_t t00, t11, t000, t111;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim16);
//  config_adis();
//  change_page(0);
//  test = read_adis_reg(PROD_ID);
//  test = read_adis_reg(PROD_ID);
//  test = read_adis_reg(PROD_ID);
//  test = read_adis_reg(PROD_ID);
  	  	  HAL_GPIO_WritePin(ADIS_RST_GPIO_Port, ADIS_RST_Pin, GPIO_PIN_RESET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(ADIS_RST_GPIO_Port, ADIS_RST_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		write_ADIS(0x60, 0x03D1);
		write_ADIS(0x64, 0x13); // say ADIS to tent data with 100 Hz. Result: 2000Hz/(19+1)
  ADS126xInitADC1();
  ADS126x_Init_Done = 1;
  imu_data.adc[1] = 0.0f;
  imu_data.adc[2] = 0.0f;
  start_single_convert(0);
  ADS126xCnt = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (check_conv_complete(&adc_result) == 0){
		  ADS126xVolt[ADS126xCnt] = (float)adc_result.Value / 2147483648.0f * 2.5f;
		  ADS126xCnt++;
		  start_single_convert(0);
	  }
      if (adis_ready == 1) {
    	  t000 = get_tick_us(&htim16);
    	  if (ADS126xCnt == 0) {
    		  start_single_convert(0);
    	  }
    	  imu_data.adc[0] = mean_val(ADS126xVolt, ADS126xCnt);
    	  ADS126xCnt = 0;
          adis_ready = 0;
//          read_adis_data(&imu_data);
          burst_read_ADIS(&imu_data);
          t00 = get_tick_us(&htim16);
          t11 = get_tick_us(&htim16) - t00;
          //imu_data.rate[0] = 0.0f;
          //imu_data.rate[1] = 100.0f / 3600.0f * 3.141926f / 180.0f * 0.01f;
          //imu_data.rate[2] = 0.0f;
          UART_buffer_len = pack_ADIS_data(&imu_data, UART_buffer);
          Transmit_UART(UART_buffer, UART_buffer_len);
          t1 = t0;
          t0 = HAL_GetTick();
          t111 = get_tick_us(&htim16) - t000;
       //   HAL_Delay(10);
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
