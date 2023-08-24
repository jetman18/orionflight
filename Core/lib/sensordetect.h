#ifndef _SENSORDETECT_
#define _SENSORDETECT_

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

#define MAXSENSOR 5
#define mpu6500 0x68
#define SSD1306 0x78
#define qmc5883 0x0d
#define bmp280  0x76

static uint8_t addr[MAXSENSOR];
void i2cDectect(I2C_HandleTypeDef *i2c){
	int sensor_count=0;
	int temp;
	for(int i=0;i<128;i++){
		temp = HAL_I2C_IsDeviceReady(i2c,(uint16_t)(i<<1), 3, 5);
		if(temp == HAL_OK)
			addr[sensor_count++] = i;
	}
	if(sensor_count == 0){
		while(1){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_Delay(100);
		}
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(1000);
		for(int j = 0;j <= sensor_count*2;j++){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_Delay(300);
		}
	}


}

#ifdef __cplusplus
}
#endif
#endif
