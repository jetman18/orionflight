#ifndef __TIMECLOCK_H__
#define __TIMECLOCK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"
#include "stm32f1xx_hal.h"


#define micros() (micross + __HAL_TIM_GET_COUNTER(&htim4))
#define millis() (micross/1000UL)
#define reset()   (micross=0)
#define callBack() (micross += 0xffff)
#define HZ_TO_MICRO(hz)  (uint32_t)(((1.0f)/(hz))*1000000)

static uint64_t micross;
static uint64_t time1,time2;
/*
 * tao vong lap voi chu ki hz
 */
void looptime(uint16_t hz)
{
    do{
   	 time2=micros();
     }while((time2-time1)<hz);
    time1=time2;
}

void delay_us(uint32_t val)
{
	static uint32_t time_1;
   	time_1=micros();
    while((micros() - time_1)<val);

}

void delay_ms(uint32_t val)
{
	static uint32_t time_2;
   	time_2=millis();
    while((millis() - time_2)<val);

}

#ifdef __cplusplus
}
#endif

#endif
/**  ex
 *
 * HAL_TIM_Base_Start_IT(&htim4);  thêm vào main()
 *
 *
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	callBack();
}
 */


