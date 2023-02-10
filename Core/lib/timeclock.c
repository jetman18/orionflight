#include "timeclock.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

TIM_HandleTypeDef *htimmz;
static uint64_t micross=0;

void initTimeloop(TIM_HandleTypeDef *htimz){
	htimmz = htimz;
	HAL_TIM_Base_Start_IT(htimmz);
}

uint64_t micros(){
	return (micross + __HAL_TIM_GET_COUNTER(htimmz));
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
