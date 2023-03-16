#include "timeclock.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

static TIM_HandleTypeDef *htimmz;
static uint64_t micross;


static uint16_t setoverFlow(int val,int flow_val){
    uint8_t k,l;
    l =flow_val + 1;
    k = val/l;
    k = val - (l*k);
    return k;
}
static bootTime_t t;

bootTime_t getBootTime(){
	static uint16_t sec_L  =0;

    sec_L = millis()/1000;
	t.sec   = setoverFlow(sec_L,59); 
	t.min   = setoverFlow((sec_L/60),59); 
	t.hour  = setoverFlow((sec_L/3600),23);
	return t;
};
void initTimeloop(TIM_HandleTypeDef *htimz){
	htimmz = htimz;
	micross=0;
	HAL_TIM_Base_Start_IT(htimmz);
}

uint64_t micros(){
	return (uint64_t)(micross + __HAL_TIM_GET_COUNTER(htimmz));
}

uint32_t millis(){
	return (micross/1000UL);
}

void delay_ms(uint32_t val){
	delay_us(val*1000);
}

void delay_us(uint32_t val){
	static uint32_t time_1;
   	time_1=micros();
    while((micros() - time_1)<val);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == htimmz)
	{
		micross += (0xffff-1);
	}
}
