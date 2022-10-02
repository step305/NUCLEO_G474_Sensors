#ifndef __UTILS_H__
#define __UTILS_H__

#include "main.h"

void delay_us(TIM_HandleTypeDef* timHandle, uint16_t period_us);
uint16_t get_tick_us(TIM_HandleTypeDef* timHandle);

#endif
