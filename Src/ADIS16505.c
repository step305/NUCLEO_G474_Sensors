/*
 * ADIS16505.c
 *
 *  Created on: 6 нояб. 2021 г.
 *      Author: driver
 */
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "ADIS16505.h"
#include "utils.h"

uint16_t send_ADIS(uint8_t adr, uint8_t dat)
{
	uint16_t tx_data[16] = {0,};
	uint16_t rx_data[16] = {0,};

	tx_data[0] = ((uint16_t)adr << 8)|(uint16_t)dat;

	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_RESET);
	delay_us(&htim16, 30);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)tx_data, (uint8_t*)rx_data, 1, 1000);
	delay_us(&htim16, 30);
	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_SET);
	return rx_data[0];
}

void write_ADIS(uint8_t adr, uint16_t dat)
{
	uint16_t tx_data[16] = {0,};
	uint16_t rx_data[16];

	tx_data[0]= (uint16_t)0x8000 | ((uint16_t)(adr+1)<<8) | ((dat>>8)&0x00FF);
	tx_data[1]= (uint16_t)0x8000 | ((uint16_t)(adr)<<8) | (dat&0x00FF);

	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_RESET);
	delay_us(&htim16, 30);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&tx_data[1], (uint8_t*)rx_data, 1, 1000);
	delay_us(&htim16, 30);
	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_SET);
	delay_us(&htim16, 30);
	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_RESET);
	delay_us(&htim16, 30);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&tx_data[0], (uint8_t*)rx_data, 1, 1000);
	delay_us(&htim16, 30);
	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_SET);
}

void burst_read_ADIS(ADIS_DATA_Type* dat)
{
	uint16_t tx_data[17] = {0,};
	uint16_t rx_data[17] = {0,};
	uint16_t byteLow;
	uint16_t byteHigh;

	tx_data[0] = 0x6800;
	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_RESET);
	delay_us(&htim16, 30);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)tx_data, (uint8_t*)rx_data, 17, 1000000);
	delay_us(&htim16, 30);
	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_SET);

	//dat->diag_status = rx_data[1];

	byteLow = rx_data[2];
	byteHigh = rx_data[3];
	dat->rate[0] = (int32_t)((uint32_t)(byteHigh<<16)|(uint32_t)byteLow);

	byteLow = rx_data[4];
	byteHigh = rx_data[5];
	dat->rate[1] = (int32_t)((uint32_t)(byteHigh<<16)|(uint32_t)byteLow);

	byteLow = rx_data[6];
	byteHigh = rx_data[7];
	dat->rate[2] = (int32_t)((uint32_t)(byteHigh<<16)|(uint32_t)byteLow);


	byteLow = rx_data[8];
	byteHigh = rx_data[9];
	dat->acc[0] = (int32_t)((uint32_t)(byteHigh<<16)|(uint32_t)byteLow);

	byteLow = rx_data[10];
	byteHigh = rx_data[11];
	dat->acc[1] = (int32_t)((uint32_t)(byteHigh<<16)|(uint32_t)byteLow);

	byteLow = rx_data[12];
	byteHigh = rx_data[13];
	dat->acc[2] = (int32_t)((uint32_t)(byteHigh<<16)|(uint32_t)byteLow);

	dat->temper = rx_data[14];
	dat->cntr = rx_data[15];

	//dat->crc = rx_data[16];

}
