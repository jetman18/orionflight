#ifndef _SCHEDULER_
#define _SCHEDULER_

#ifdef __cplusplus
extern "C" {
#endif

#define HZ_TO_MICRO(hz)  (uint32_t)(((1.0f)/(hz))*1000000)


#include "stm32f1xx_hal.h"
#include "timeclock.h"

/**
 *@brief  lap      lich nhiem vu
 *@param  hz       tan so lap
 *@param  timeout  thoi gian cho
 *@param  task     ham nhiem vu
 */
static uint64_t pTime;
void schedule_Task(uint16_t hz,void task())
{
	if((micros()-pTime)>HZ_TO_MICRO(hz)){
		task();
	    pTime = micros();
	}
}
#ifdef __cplusplus
}
#endif
#endif
