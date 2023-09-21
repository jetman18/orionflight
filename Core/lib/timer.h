
#ifndef LIB_TIMER_H_
#define LIB_TIMER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stdio.h"
typedef struct{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}bootTime_t;

extern TIM_HandleTypeDef *htimmz;
static uint32_t micross;
#define HZ_TO_MICRO(hz)  (uint32_t)(((1.0f)/(hz))*1000000)
#define TIMER_CALLBACK() (micross += 65535UL)
#define micros() (uint32_t)((micross) + (__HAL_TIM_GET_COUNTER(htimmz)))
#define millis()  (uint32_t)(micross/1000UL)
void timer_start(TIM_HandleTypeDef *htimz);

#ifdef __cplusplus
}
#endif
#endif
