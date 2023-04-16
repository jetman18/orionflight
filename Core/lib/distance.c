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
void rs04Callback(){
	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	 if( pin==0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)){
	       time_=micros();
	       pin=1;
	   }
	 else if( pin==1 && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)){
	       pulse=micros()-time_;
	       pin=0;
	   }
}
uint16_t getDistance(){
	if(!pin)return pulse;
	return 0;
}
