#ifndef _SENSORDETECT_H_
#define _SENSORDETECT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "i2c.h"

int i2cDectect(I2C_HandleTypeDef *i2c,uint8_t address){
	return HAL_I2C_IsDeviceReady(i2c,(uint16_t)(address<<1), 3, 5);
}
#ifdef __cplusplus
}
#endif

#endif
