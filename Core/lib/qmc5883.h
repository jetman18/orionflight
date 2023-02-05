#ifndef _QMC5883_H_
#define _QMC5883_H_

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
void qmc5883_init(I2C_HandleTypeDef *i2cport);
void qmc_get_raw(MAG_t *t);
void qmc_get_values(MAG_t *t,float pitch,float roll);
void magnet_sensor_calibrate();
void qmc_get_3axil_values(MAG_t *t);
#ifdef __cplusplus
}
#endif

#endif

