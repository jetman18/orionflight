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

static uint32_t timeVal=0,currentTime=0,prerviTime=0;
static uint16_t time=0;
static uint8_t start = 0;

void callBackFuncition(rcChannel_t *chh){
	static uint8_t count = 0;
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

float ChannelTodec(uint16_t ch,uint16_t ch_cl,float gain){
	if(ch==0)return 0.0f;
	else {
	  return (float)(ch - ch_cl)*gain;
	}
}

int isRxupdate(){
   return !start;
}

void rxCalibrate(rcChannel_t t,uint16_t *chh1,uint16_t *chh2,uint16_t *chh4){
	uint32_t ch_1=0,ch_2=0,ch_4=0;
	for(int i=0;i<100;i++){
        ch_1 +=t.ch[0];
        ch_2 +=t.ch[1];
        ch_4 +=t.ch[3];
        HAL_Delay(33);
	}
	*chh1 = ch_1/100;
	*chh2 = ch_2/100;
	*chh4 = ch_4/100;
}

#ifdef __cplusplus
}
#endif
#endif
