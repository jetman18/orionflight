#include "timer.h"
#include "stm32f1xx_hal.h"


TIM_HandleTypeDef *htimmz;
bootTime_t boottime;
static uint16_t setoverFlow(int val,int flow_val);

void delay_ms(uint32_t val)
{
	delay_us(val*1000);
}

void delay_us(uint32_t val){
	static uint32_t time_us;
  time_us = micros();
  while((micros() - time_us)<val);
}

static uint16_t setoverFlow(int val,int flow_val){
    uint8_t k,l;
    l =flow_val + 1;
    k = val/l;
    k = val - (l*k);
    return k;
}

static void time_inf(){
  static uint16_t sec_L  =0;
  sec_L = millis()/1000;
  boottime.sec   = setoverFlow(sec_L,59);
  boottime.min   = setoverFlow((sec_L/60),59);
  boottime.hour  = setoverFlow((sec_L/3600),23);
}
void timer_start(TIM_HandleTypeDef *htimz){
	htimmz = htimz;
	HAL_TIM_Base_Start_IT(htimmz);
}
