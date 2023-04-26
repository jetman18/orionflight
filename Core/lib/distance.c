/*
 * distance.cpp
 *
 *  Created on: Apr 16, 2023
 *      Author: sudo
 */

#include "distance.h"
#include "timeclock.h"
static int pin;
static uint16_t pulse;
static uint32_t time_;

void hc_sr04_callback(){
	 if( pin==0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)){
	       time_=micros();
	       pin=1;
	   }
	 else if( pin==1 && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)){
	       pulse=micros()-time_;
	       pin=0;
	   }
}
int isHcNewdata(){
	return (!pin);
}
int hc_sr04_get_dis(){
	return (pulse*(float)(1e-06f)*170000); //mm
}
void hc_sr04_send_trige(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
    delay_us(11);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
}
/*
 * FEQUENCY_DIV(12,1){
	hc_sr04_send_trige();
	static float last_dis = BIAS_ALTITUDE;
	float ddis = hc_sr04_get_dis();
	ddis = ef_constrainf(last_dis,ddis,-10,10);
	last_dis = ddis;
	dis = dis + 0.9f*(ddis - dis);
	dis = dis*cos_approx(DEGREES_TO_RADIANS(quad_.pitch))*cos_approx(DEGREES_TO_RADIANS(quad_.roll));
	//if((quad_.pitch < fabs(10)) && (quad_.roll < fabs(10))){
		if(ch5>1600){
			init_th +=100*0.024f; //0.024 (s)
			init_th = constrainf(init_th,0,1200);
			pidCalculate(&Pid_altitude_t,dis,ch3_ + BIAS_ALTITUDE,24000);
			Pid_altitude_t.PID *=-1;
			t1 =init_th + Pid_altitude_t.PID;
		}else{
			t1 = throttle;
			init_th=1000;
			Pid_altitude_t.I=0;
			Pid_altitude_t.PID=0;
		}
}
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_8)
    {
     hc_sr04_callback();
    }
}
