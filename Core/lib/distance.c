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
	 if( pin==0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)){
	       time_=micros();
	       pin=1;
	   }
	 else if( pin==1 && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)){
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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,1);
    delay_us(11);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,0);
}
