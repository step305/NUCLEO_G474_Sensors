//
// Created by step on 20.07.2021.
//
#include "main.h"
#include "spi.h"
#include "utils.h"
#include "tim.h"

#ifndef NUCLEO_G474_SENSORS_ADIS16485_H
#define NUCLEO_G474_SENSORS_ADIS16485_H

// PAGE0 Registers
#define PAGE_ID         0x00
#define SYS_E_FLAG      0x08
#define DIAG_STS        0x0A
#define ALM_STS         0x0C
#define TEMP_OUT        0x0E
#define X_GYRO_LOW      0x10
#define X_GYRO_OUT      0x12
#define Y_GYRO_LOW      0x14
#define Y_GYRO_OUT      0x16
#define Z_GYRO_LOW      0x18
#define Z_GYRO_OUT      0x1A
#define X_ACCL_LOW      0x1C
#define X_ACCL_OUT      0x1E
#define Y_ACCL_LOW      0x20
#define Y_ACCL_OUT      0x22
#define Z_ACCL_LOW      0x24
#define Z_ACCL_OUT      0x26
#define X_DELTANG_LOW   0x40
#define X_DELTANG_OUT   0x42
#define Y_DELTANG_LOW   0x44
#define Y_DELTANG_OUT   0x46
#define Z_DELTANG_LOW   0x48
#define Z_DELTANG_OUT   0x4A
#define X_DELTAVEL_LOW  0x4C
#define X_DELTAVEL_OUT  0x4E
#define Y_DELTAVEL_LOW  0x50
#define Y_DELTAVEL_OUT  0x52
#define Z_DELTAVEL_LOW  0x54
#define Z_DELTAVEL_OUT  0x56
#define TIME_MS_OUT     0x78
#define TIME_DH_OUT     0x7A
#define TIME_YM_OUT     0x7C
#define PROD_ID         0x7E

#define PROD_ID_ADIS16485   0x4065

// PAGE2 Registers

// PAGE3 Registers
#define GLOB_CMD        0x02
#define FNCTIO_CTRL     0x06
#define GPIO_CTRL       0x08
#define CONFIG          0x0A
#define DEC_RATE        0x0C
#define NULL_CNFG       0x0E
#define SLP_CNT         0x10
#define FILTR_BNK_0     0x16
#define FILTR_BNK_1     0x18
#define ALM_CNFG_0      0x20
#define ALM_CNFG_1      0x22

#define FIRM_REV        0x78
#define FIRM_DM         0x7A
#define FIRM_Y          0x7C

// PAGE4 Registers
#define SERIAL_NUM      0x20

void change_page(uint8_t page);
uint16_t write_adis_reg(uint8_t addr, uint8_t data);
uint16_t read_adis_reg(uint8_t addr);
void read_adis_data(ADIS_DATA_Type* pack);
void config_adis(void);

#endif //NUCLEO_G474_SENSORS_ADIS16485_H
