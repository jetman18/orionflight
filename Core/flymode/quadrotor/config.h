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




/* mpu configuration*/
//#define MPU_VIA_SPI
//#define SPI_PORT    &hspi1
//#define GPIO_CS_PIN   GPIO_PIN_4
//#define GPIO_PORT     GPIOA


/* mpu configuration */
#define MPU_VIA_I2C

/* magneto sensor*/
#define I2C_PORT &hi2c2


#ifdef __cplusplus
}
#endif


#endif /* FLYMODE_QUADROTOR_CONFIG_H_ */
