#include "scheduler.h"

static uint16_t count_=1;
static uint16_t feq=0;
static uint64_t time1 =0;
static bootTime_t t;
static TIM_HandleTypeDef *htimmz;
static uint64_t micross;

void loop_run(uint32_t us)
{
	feq = 1/(us*0.000001);
	if(count_ >= feq)count_=1;
	count_ ++;
    while((micros()-time1)<us);
    time1=micros();
}
int fequency_division(uint16_t division,int k){
	if(!k)return 0;
    if(count_%division==0)return 1;
    else if(feq==0)return 0;
    return 0;
}

static uint16_t setoverFlow(int val,int flow_val){
    uint8_t k,l;
    l =flow_val + 1;
    k = val/l;
    k = val - (l*k);
    return k;
}
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
void timeCallback(){
	micross = micross +(0xffff-1);
}