#ifndef _SENSORDETECT_H_
#define _SENSORDETECT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "i2c.h"
#define MAXSENSOR 5
/*i2c sensor detect*/
#define mpu6500 0x68
#define SSD1306 0x78
#define qmc5883 0x0d
#define bmp280  0x76

static uint8_t addr[MAXSENSOR];
int ki;
void i2cDectect(I2C_HandleTypeDef *i2c){
	int t=0;
	for(int i=0;i<128;i++){
		ki = HAL_I2C_IsDeviceReady(i2c,(uint16_t)(i<<1), 3, 5);
		if(ki == HAL_OK)
			{
			addr[t++] = i;
			}
	}
	if(t == 0){
		while(1);
		//error
	}
}
#ifdef __cplusplus
}
#endif

#endif
