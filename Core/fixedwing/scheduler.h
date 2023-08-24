#ifndef _SCHEDULER_
#define _SCHEDULER_

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

typedef struct {
    void (*exec)();
    uint32_t execution_time_us;
    uint32_t execution_cycle_us;
    uint32_t last_exec_time_us;
    uint32_t period;
} task_t;


extern TIM_HandleTypeDef *htimmz;
extern uint32_t micross;
extern bootTime_t boottime;

#define HZ_TO_MICRO(hz)  (uint32_t)(((1.0f)/(hz))*1000000)
#define TIME_CALLBACK() (micross += 65535UL)
#define micros() (uint32_t)((micross) + (__HAL_TIM_GET_COUNTER(htimmz)))
#define millis()  (uint32_t)(micross/1000UL)

void start_scheduler();
void init_sche(TIM_HandleTypeDef *htimz);
void delay_us(uint32_t);
void delay_ms(uint32_t);
#ifdef __cplusplus
}
#endif
#endif
