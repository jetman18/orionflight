#include "mainloop.h"
#include "sensor.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "timeclock.h"
#include "scheduler.h"
#include "pwmwrite.h"
#include "ppmreceive.h"
#include "pid.h"


#define LOOPTIME  1000

uint16_t moto[6];
euler_angle_t mpu;
rcChannel_t rx;

float pid_pitch,pid_roll,pid_yaw;
pid_gain_t pitch_t,roll_t,yaw_t;
void main_loop(){
	// int main
	HAL_TIM_Base_Start_IT(&htim4);
	MPU_init();

	/* set all rx channels to 1000 */
	for(int i=0;i<6;i++){
		rx.ch[i]=1000;
	}

	//qmc5883_init();


	/*init pid gain*/
	pitch_t.kp = 1;
	pitch_t.ki = 1;
	pitch_t.kd = 1;

	roll_t.kp = 0;
	roll_t.ki = 0;
	roll_t.kd = 0;

	yaw_t.kp = 0;
	yaw_t.ki = 0;
	yaw_t.kd = 0;



	while(1){

		MPU_update(&mpu,LOOPTIME);  //4000 DELTA T


		/* update PID 400hz*/
		feqRun(500){
			pid_pitch = pidCalcutate(pitch_t,mpu.pitch,0);
			pid_roll  = pidCalcutate(roll_t,mpu.roll,0);
			pid_yaw   = pidCalcutate(yaw_t,mpu.yaw,0);

			/*
			moto[0] = rx.ch[0] +
			moto[1] = rx.ch[0] +
			moto[2] = rx.ch[0] +
			moto[3] = rx.ch[0] +
            */
		}

		/* update throtle to esc 300hz */
		feqRun(400)writePWM(moto);

		/*loop feq 1000hz*/
		looptime(LOOPTIME);
		}
}













/*-------------ISR HANDLER----------------------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim4)
	{
		callBack();
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14) // If The INT Source Is EXTI 14
    {
    	callBackFuncition(&rx);
    }

}
