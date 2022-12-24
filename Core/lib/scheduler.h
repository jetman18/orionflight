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
/*
 * tao vong lap voi chu ki hz
 */
#define feqRun(hz) if(isxxHZ(hz))

static uint16_t count=0;
static uint16_t feq=0;
uint64_t time1,time2;
void looptime(uint16_t us)
{
	feq = 1/(us*0.000001f);
	count ++;
	if(count > feq)count=0;
    do{
   	 time2=micros();
     }while((time2-time1)<us);
    time1=time2;
}
int isxxHZ(uint16_t k){
	if(feq==0)return 0;
    float dT= 1.0f/k;
    uint16_t cou = dT*feq;
    if(count%cou==0)return 1;

    return 0;
}

#ifdef __cplusplus
}
#endif
#endif
