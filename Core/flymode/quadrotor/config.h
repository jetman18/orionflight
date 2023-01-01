/*
 * config.h
 *
 *  Created on: Dec 9, 2022
 *      Author: sudo
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif



#include "gpio.h"
#include "spi.h"
#include "i2c.h"


#define ENABLE  1
#define DISABLE 0

/* mpu configuration*/
#define MPU_VIA_SPI
//#define MPU_VIA_I2C



/* magneto sensor*/


#ifdef __cplusplus
}
#endif


#endif /* FLYMODE_QUADROTOR_CONFIG_H_ */
