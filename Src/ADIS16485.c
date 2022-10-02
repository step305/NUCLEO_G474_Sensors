//
// Created by driver on 20.07.2021.
//
#include "ADIS16485.h"
#define CODE2DEG    (720.0f * 0.00000000023283064365386962890625f)  // /2^32
#define CODE2VEL    (50.0f * 0.0000152587890625f)                   // /2^16

//void delay_us(uint32_t val) {
//    for (__IO uint32_t i = 0; i < (val*170); i++) {}
//}

uint16_t write_adis_reg(uint8_t addr, uint8_t data) {
    uint16_t result = 0;
    uint16_t write_data = 0x8000 | (((uint16_t)addr<<8)&0x7F00) | (((uint16_t)data)&0x00FF);

    HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&write_data, (uint8_t*)&result, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_SET);
    delay_us(&htim16, 2);
    return result;
}

void change_page(uint8_t page) {
    write_adis_reg(PAGE_ID, page);
}

uint16_t read_adis_reg(uint8_t addr) {
    uint16_t result = 0;
    uint16_t write_data = ((uint16_t)addr<<8)&0x7F00;

    HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&write_data, (uint8_t*)&result, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_SET);

    return result;
}

void read_adis_data(ADIS_DATA_Type* pack) {
    uint16_t result = 0;
    uint32_t result32 = 0;

    change_page(0);
    read_adis_reg(X_DELTANG_OUT);
    result = read_adis_reg(X_DELTANG_LOW);
    result32 = (uint32_t)result;
    result32 = result32 << 16;
    result = read_adis_reg(Y_DELTANG_OUT);
    result32 |= (uint32_t)result;
    pack->rate[0] = (float)((int32_t)result32) * CODE2DEG * 3.141926f / 180.0f; //*720/ 4294967296.0f;

    result = read_adis_reg(Y_DELTANG_LOW);
    result32 = (uint32_t)result;
    result32 = result32 << 16;
    result = read_adis_reg(Z_DELTANG_OUT);
    result32 |= (uint32_t)result;
    pack->rate[1] = (float)((int32_t)result32) * CODE2DEG * 3.141926f / 180.0f; //*720/ 4294967296.0f;

    result = read_adis_reg(Z_DELTANG_LOW);
    result32 = (uint32_t)result;
    result32 = result32 << 16;
    result = read_adis_reg(X_DELTAVEL_OUT);
    result32 |= (uint32_t)result;
    pack->rate[2] = (float)((int32_t)result32) * CODE2DEG * 3.141926f / 180.0f; //*720/ 4294967296.0f;

    result = read_adis_reg(Y_DELTAVEL_OUT);
    pack->acc[0] = (float)((int16_t)result) * CODE2VEL;
    result = read_adis_reg(Z_DELTAVEL_OUT);
    pack->acc[1] = (float)((int16_t)result) * CODE2DEG;
    result = read_adis_reg(TEMP_OUT);
    pack->acc[2] = (float)((int16_t)result) * CODE2DEG;

    result = read_adis_reg(PROD_ID);
    pack->temper = (float)((int16_t)result) * 0.00565f;

    pack->cntr++;
}

void config_adis(void) {
    change_page(3);
    write_adis_reg(GLOB_CMD, 0x80);
    write_adis_reg(GLOB_CMD+1, 0x00);
    HAL_Delay(300);
    change_page(3);
    write_adis_reg(DEC_RATE, 0x18);
    write_adis_reg(DEC_RATE+1, 0x00);
}
