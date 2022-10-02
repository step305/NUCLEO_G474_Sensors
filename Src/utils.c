#include "utils.h"

void delay_us(TIM_HandleTypeDef* timHandle, uint16_t period_us) {
	uint16_t tim_v = __HAL_TIM_GET_COUNTER(timHandle);
	while ((__HAL_TIM_GET_COUNTER(timHandle) - tim_v) < period_us);
	//__HAL_TIM_SET_COUNTER(timHandle, 0);
	//while (__HAL_TIM_GET_COUNTER(timHandle) < period_us);
}

uint16_t get_tick_us(TIM_HandleTypeDef* timHandle) {
	return __HAL_TIM_GET_COUNTER(timHandle);
}
