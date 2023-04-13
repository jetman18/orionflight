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
void qmc5883_init(I2C_HandleTypeDef *i2cport);
void magnet_sensor_calibrate();
int qmc_get_Heading(float *heading,float pitch,float roll);  //degre*10
void qmc_get(axis3_t *t,float pitch,float roll);
#ifdef __cplusplus
}
#endif

#endif

