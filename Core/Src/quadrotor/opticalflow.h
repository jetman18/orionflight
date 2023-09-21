/*
 * opticalflow.h
 *
 *  Created on: Apr 3, 2023
 *      Author: sudo
 */

#ifndef LIB_OPTICALFLOW_H_
#define LIB_OPTICALFLOW_H_
#ifdef __cplusplus
extern "C" {
#endif

#include"stdio.h"
#include "usart.h"
#include "../lib/pid.h"

extern pid__t x_flow_t;
extern pid__t y_flow_t;
void flowInit(UART_HandleTypeDef *uartt,uint32_t baudrate);
void flowCallback();
void optical_flow_run();
#ifdef __cplusplus
}
#endif
#endif /* LIB_OPTICALFLOW_H_ */
