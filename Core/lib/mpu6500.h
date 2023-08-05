#ifndef _SENSORDETECT_H_
#define _SENSORDETECT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "axis.h"
typedef enum{
    GYRO_250 = 0,
    GYRO_500,
    GYRO_1000,
    GYRO_2000
}GYRO_SCALE_;

typedef enum{
    ACC_2G = 0,
    ACC_4G,
    ACC_8G,
    ACC_16G
}ACC_SCALE_;

int gyro_read_raw(axis3_t *k);
int acc_read_raw(axis3_t *k);
void MPU_i2c_init(I2C_HandleTypeDef *i2cport);

#ifdef __cplusplus
}
#endif

#endif