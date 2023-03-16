#ifndef __TIMECLOCK_H__
#define __TIMECLOCK_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_hal.h"
#include "tim.h"
typedef struct time{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}bootTime_t;

bootTime_t getBootTime();
void initTimeloop(TIM_HandleTypeDef*);
void delay_us(uint32_t);
void delay_ms(uint32_t);
uint32_t millis();
uint64_t micros();

#ifdef __cplusplus
}
#endif

#endif



