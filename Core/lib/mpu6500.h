#ifndef _MPU6500_H_
#define _MPU6500_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"


typedef struct{
    float pitch; /*1800  -1800*/
	float roll;
    float yaw;
}euler_angle_t;

typedef struct{
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
}IMU_raw_t;

//void MPU_spi_init(SPI_HandleTypeDef *spiportt,GPIO_TypeDef  *gpio_port,uint16_t pin);
void imu_update(euler_angle_t *m,uint16_t dt);
void get_AccAngle(euler_angle_t *m);
void MPU_i2c_init(I2C_HandleTypeDef *i2cport);
int16_t get_gyro(int axis);
void resetVector();
#ifdef __cplusplus
}
#endif

#endif
