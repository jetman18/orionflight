#ifndef _QMC5883_H_
#define _QMC5883_H_
#include "axis.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

//////////////////////QMC5883L////////////////////////////////////////

typedef struct{
    int16_t mx;
    int16_t my;
    int16_t mz;
   float compas;
}MAG_t;

extern float heading;
extern float gyro_yaw;

void qmc5883_init(I2C_HandleTypeDef *i2cport);
void magnet_sensor_calibrate();
void qmc_get_raw();
void compass_get_heading_();
#ifdef __cplusplus
}
#endif

#endif

