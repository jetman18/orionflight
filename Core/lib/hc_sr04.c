/*
 * distance.cpp
 *
 *  Created on: Apr 16, 2023
 *      Author: sudo
 */

#include "hc_sr04.h"
#include "scheduler.h"
#include "imu.h"
#include "maths.h"
#include "ibus.h"
#include "pid.h"
extern attitude_t quad_;
static int pin=1;
static uint16_t pulse;
static uint32_t time_;
extern uint16_t ch5;
int hc04_Throttle;
extern uint16_t ch3_;
pid__t Pid_altitude_t;

int isHcNewdata(){
	return (!pin);
}
int hc_sr04_get_dis(){
	return (pulse*(float)(1e-06f)*170000); //mm
}
void hc_sr04_send_trige(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,1);
    delay_us(11);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,0);
}
void hc_sr04_start(){
	//------------altitude------------
	hc04_Throttle=0;
	Pid_altitude_t.kp =1.0;
	Pid_altitude_t.ki =0.12;
	Pid_altitude_t.kd =0.3; 
	Pid_altitude_t.max_i = 70.0;   
	Pid_altitude_t.max_pid =300.0; 
	Pid_altitude_t.f_cut_D =50.0;
	hc_sr04_send_trige();
}
void hc_sr04_run(){
	static float last_dis = 0;
	int dis = hc_sr04_get_dis();
	dis = val_change_limiter(last_dis,dis,-15,15);
	last_dis = dis;
	//dis = dis*cos_approx(DEGREES_TO_RADIANS(quad_.pitch))*cos_approx(DEGREES_TO_RADIANS(quad_.roll));
	//if((quad_.pitch < fabs(10)) && (quad_.roll < fabs(10))){
		if(ch5>1600){
			hc04_Throttle = hc04_Throttle + 100*0.025f;
			hc04_Throttle = constrain(hc04_Throttle,0,200);
			pidCalculate(&Pid_altitude_t,dis,ch3_,24000);
			Pid_altitude_t.PID *=-1;
			hc04_Throttle = hc04_Throttle + Pid_altitude_t.PID;
		}else{
			resetPID(&Pid_altitude_t);
			hc04_Throttle=0;
		}
		hc_sr04_send_trige();
}
void hc_sr04_callback(){
	 if( pin==0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)){
	       time_=micros();
	       pin=1;
	   }
	 else if( pin==1 && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)){
	       pulse=micros()-time_;
	       pin=0;
	   }
}
