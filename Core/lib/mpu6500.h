#ifndef _MPU6500_H_
#define _MPU6500_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
}axis3_t;

typedef struct{
    float x;
    float y;
    float z;
}faxis3_t;


typedef struct{
    float pitch;
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

typedef struct{
   float pitch;
   float roll;
   float yaw;

   float xx;
   float yy;
   float zz;

}IMU_t;

void MPU_i2c_init(I2C_HandleTypeDef *i2c);
void MPU_spi_init(SPI_HandleTypeDef *spiportt,GPIO_TypeDef  *gpio_port,uint16_t pin);
void mpu_update(euler_angle_t *m,uint16_t dt);
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void computeAnglesFromQuaternion(euler_angle_t *m);
void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#ifdef __cplusplus
}
#endif

#endif
