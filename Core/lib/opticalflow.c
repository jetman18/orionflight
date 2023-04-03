/*
 * opticalflow.c
 *
 *  Created on: Apr 3, 2023
 *      Author: sudo
 */

#include <opticalflow.h>
#include "usart.h"
static UART_HandleTypeDef *uart;

static char buffer;
void flowInit(UART_HandleTypeDef *uartt,uint32_t baudrate){
	uart = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt); //reInit
	HAL_UART_Receive_IT(uart, &buffer,1);
}
void pasrseMsg(char k){

};
void flowCallback()
{
	pasrseMsg(buffer);
    HAL_UART_Receive_IT(uart, &buffer,1);
}

