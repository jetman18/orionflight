#ifndef __TIMECLOCK_H__
#define __TIMECLOCK_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_hal.h"
#include "tim.h"

void initTimeloop(TIM_HandleTypeDef*);
void delay_us(uint32_t val);
void delay_ms(uint32_t val);
uint32_t millis();
uint64_t micros();

#ifdef __cplusplus
}
#endif

#endif



