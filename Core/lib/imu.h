#ifndef _IMU_H_
#define _IMU_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "axis.h"

typedef struct {
    float w;
    float x;
    float y;
    float z;
}quaternion_t;


typedef struct{
    float pitch;
	float roll;
    float yaw;

    float pitch_velocity;
    float roll_velocity;
    float yaw_velocity;

    float acc_x;
    float acc_y;
    float acc_z;
}attitude_t;

typedef struct{
    int16_t accx;
    int16_t accy;
    int16_t accz;

    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
}IMU_raw_t;


typedef struct imu_config{
    float gyro_f_cut;
    float acc_f_cut;
    float cpl_gain;
    float gyro_slew_threshold;
    float acc_slew_threshold;
    uint32_t dt;
    float gyr_lsb;
}imu_config_t;

extern attitude_t quad_;
extern imu_config_t config;
void mpu6050_init();
void imu_update();
void gyro_read(faxis3_t *angle);
#ifdef __cplusplus
}
#endif

#endif
