#ifndef _SENSORDETECT_H_
#define _SENSORDETECT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "axis.h"

int gyro_read_raw(axis3_t *k);
int acc_read_raw(axis3_t *k);
void mpu_read_all(axis3_t *acc, axis3_t *gyr);
void mpu6500_init();
int16_t get_axis_register(uint8_t addr);
void write_axis_register(uint8_t addr,uint8_t val);
#ifdef __cplusplus
}
#endif

#endif
