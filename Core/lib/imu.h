#ifndef _IMU_H_
#define _IMU_H_

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

typedef struct{
    float pitch;
	float roll;
    float yaw;

    float pitch_velocity;
    float roll_velocity;
    float yaw_velocity;

    int16_t raw_acc_x;
    int16_t raw_acc_y;
    int16_t raw_acc_z;
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
    float imu_gyro_Sensitivity_Scale_Factor;
    uint8_t imu_adrr;
    uint16_t offset_cycle;
    uint8_t imu_acc_res_cgf_val;
    uint8_t imu_acc_data_res;
    uint8_t imu_gyro_res_cgf_val;
    uint8_t imu_gyro_data_res;
    uint8_t imu_acc_regsiter_config;
    uint8_t imu_gyro_regsiter_config;
}imu_config_t;

extern attitude_t quad_;
extern imu_config_t config;

void get_Acc_Angle(attitude_t *m);
void imu_update();
void MPU_i2c_init(I2C_HandleTypeDef *i2cport);
#ifdef __cplusplus
}
#endif

#endif
