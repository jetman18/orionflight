#ifndef _PPMRECEIVE_
#define _PPMRECEIVE_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "timeclock.h"
#include "gpio.h"

#define GET_PIN_STATE (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14))
#define TIME_MARK_START 7000  //m
#define NUM_OF_CHANNEL 8
typedef struct t{
	uint16_t ch[8];
}rcChannel_t;

/*brief rc channel reveive
 *Param  chh rcChannel
 */


void callBackFuncition(rcChannel_t *chh){
	static uint8_t start = 0;
	static uint8_t count = 0;
	static uint32_t timeVal,currentTime,prerviTime;
	static uint16_t time;
    /********************/
	if(!start){
		if(GET_PIN_STATE){
			timeVal=micros();
		  }
		else if(!GET_PIN_STATE){
		   time=micros()-timeVal;
		   if(time>TIME_MARK_START)start=1;
		}
	}

	if(start && !GET_PIN_STATE){
		currentTime=micros();
		if(count>0)chh->ch[count-1]=currentTime-prerviTime;
		prerviTime=currentTime;
		count++;
		if(count > (NUM_OF_CHANNEL + 1) ){
			start =0;
			count=0;
		}
	}

}
#ifdef __cplusplus
}
#endif
#endif
