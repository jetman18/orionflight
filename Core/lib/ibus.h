/*
 * ibus.h
 *
 *  Created on: 10 thg 2, 2023
 *      Author: sudo
 */

#ifndef LIB_IBUS_H_
#define LIB_IBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

enum index{
	CH1 = 0,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6,
	CH7,
	CH8
};

int ibusFrameComplete(void);
void ibusDataReceive(uint16_t c);
uint16_t ibusReadRawRC(uint8_t chan);
float ibusReadf(uint8_t chan,float gain);
void ibusInit(UART_HandleTypeDef *uartt,uint32_t baudrate);
void ibusCallback();
#ifdef __cplusplus
}
#endif

#endif /* LIB_IBUS_H_ */
