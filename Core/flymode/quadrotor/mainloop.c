#include "mainloop.h"
#include "sensor.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "timeclock.h"

uint8_t k;
MAG_t t;
IMU_raw_t data;
void main_loop(){
	// int main
	HAL_TIM_Base_Start_IT(&htim4);
	MPU_init();
	qmc5883_init();

	    for(int i=1; i<128; i++)
	    {
	        int ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5);

	        if(ret == HAL_OK)
	        {
               k=i;
               break;
	        }
	    }
while(1){

    MPU_get_acc(&data);
    MPU_get_gyro(&data);
    qmc_get_values(&t,0,0);
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    loopFequency(100);

	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim4)
	{
		callBack();
	}
}
