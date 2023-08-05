#ifndef __AHRS_H__
#define __AHRS_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "imu.h"
extern float dcm[3][3];
extern attitude_t AHRS;
void ahrs_update();

#ifdef __cplusplus
}
#endif

#endif
