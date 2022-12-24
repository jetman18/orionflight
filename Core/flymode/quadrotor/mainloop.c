#include "mainloop.h"
#include "sensor.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "timeclock.h"
#include "pwmwrite.h"
#include "ppmreceive.h"


uint16_t moto[6];
euler_angle_t m;
MAG_t t;
rcChannel_t rx;
void main_loop(){
	// int main
	HAL_TIM_Base_Start_IT(&htim4);
	MPU_init();

	for(int i=0;i<6;i++){
		rx.ch[i]=1000;
	}

	//qmc5883_init();

while(1){

	MPU_update(&m,4000);  //4000 DELTA T



    //writePWM(moto);
    looptime(4000);
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
