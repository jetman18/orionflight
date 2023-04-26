#include "pwmwrite.h"
#include "tim.h"
#include "maths.h"
#include "pid.h"

#define PWM_RELOAD_AFFTER 2200     // 2200 US
#define ONESHOT_RELOAD_AFFTER 400  //400 US  2500HZ


extern uint16_t moto1,moto2,moto3,moto4;

/**timer pwm
 *
 */
TIM_HandleTypeDef *htimm;
void initPWM(TIM_HandleTypeDef *htim){
	htimm = htim;
	HAL_TIM_PWM_Start(htim,ch1);
	HAL_TIM_PWM_Start(htim,ch2);
	HAL_TIM_PWM_Start(htim,ch3);
	HAL_TIM_PWM_Start(htim,ch4);
	__HAL_TIM_SetAutoreload(htimm,PWM_RELOAD_AFFTER);
}

void initOneshot125(TIM_HandleTypeDef *htim){
	htimm = htim;
	HAL_TIM_PWM_Start(htim,ch1);
	HAL_TIM_PWM_Start(htim,ch2);
	HAL_TIM_PWM_Start(htim,ch3);
	HAL_TIM_PWM_Start(htim,ch4);
	__HAL_TIM_SetAutoreload(htimm,ONESHOT_RELOAD_AFFTER);
}

void writePwm(uint32_t Channel,int16_t dulty)
{
	  dulty = constrain(dulty,1000,2000);
	__HAL_TIM_SetCompare (htimm,Channel,dulty);
}
void writeOneshot125(uint32_t Channel,int16_t dulty)
{
	  dulty = constrain(dulty,120,250);
	__HAL_TIM_SetCompare (htimm,Channel,dulty);
}
void motoIdle(){
	__HAL_TIM_SetCompare (htimm,ch1,1000);
	__HAL_TIM_SetCompare (htimm,ch2,1000);
	__HAL_TIM_SetCompare (htimm,ch3,1000);
	__HAL_TIM_SetCompare (htimm,ch4,1000);
}
void pwm2esc(){
	moto1 = constrain(moto1,1000,2000);
	__HAL_TIM_SetCompare (htimm,ch1,moto1);

	moto2 = constrain(moto2,1000,2000);
	__HAL_TIM_SetCompare (htimm,ch2,moto2);

	moto3 = constrain(moto3,1000,2000);
	__HAL_TIM_SetCompare (htimm,ch3,moto3);

	moto4 = constrain(moto4,1000,2000);
	__HAL_TIM_SetCompare (htimm,ch4,moto4);
};


