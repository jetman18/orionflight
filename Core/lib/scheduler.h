#ifndef _SCHEDULER_
#define _SCHEDULER_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "tim.h"

#define FEQUENCY_DIV(div,z) if(fequency_division(div,z))
#define HZ_TO_MICRO(hz)  (uint32_t)(((1.0f)/(hz))*1000000)
typedef struct time{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}bootTime_t;

void loop_run(uint32_t us);
int fequency_division(uint16_t division,int k);
bootTime_t getBootTime();
void initTimeloop(TIM_HandleTypeDef*);
void delay_us(uint32_t);
void delay_ms(uint32_t);
uint32_t millis();
uint64_t micros();
void timeCallback();
#ifdef __cplusplus
}
#endif
#endif
