#ifndef _SENSOR_H_
#define _SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"



/**
 *
 *
 *
 * 
 */
typedef struct
{
    float pitch;
	float roll;
    float yaw;
}euler_angle_t;

typedef struct
{
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

void MPU_init();
void mpu_get_acc(IMU_raw_t*);
void mpu_get_gyro(IMU_raw_t*);
void MPU_update(euler_angle_t *m,int delta_t);
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void computeAnglesFromQuaternion(euler_angle_t *m);
void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
//////////////////////QMC5883L////////////////////////////////////////

typedef struct{
    int16_t mx;
    int16_t my;
    int16_t mz;
   float compas;
}MAG_t;
void qmc5883_init();
void qmc_get_values(MAG_t *t,float pitch,float roll);
void magnet_sensor_calibrate();
#ifdef __cplusplus
}
#endif

#endif
