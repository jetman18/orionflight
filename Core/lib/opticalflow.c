/*
 * opticalflow.c
 *
 *  Created on: Apr 3, 2023
 *      Author: sudo
 */

#include <opticalflow.h>
#include "usart.h"
static UART_HandleTypeDef *uart_;
static uint8_t bytee;
static uint8_t buffer[5];
static int isReady;
static int flow_x,flow_y,quality;
void flowInit(UART_HandleTypeDef *uartt,uint32_t baudrate){
	uart_ = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt); //reInit
	HAL_UART_Receive_IT(uart_, &bytee,1);
}
void pasrseMsg(uint8_t k){
	static uint8_t index=0;
    static uint8_t step =0;
	switch(k){
		case '0xb5':
			if(step==0)step++;
			break;
		case '0x62':
			if(step==1)step++;
			break;
		default:
			if(step == 2){
				buffer[index++] = k;
				if(index == 5){
					int16_t sum = (int8_t)buffer[0] + (int8_t)buffer[2] + (int8_t)buffer[3];
					char *ck =  *(char*)&sum;
                    char ckA = ck[0];
                    char ckB = ck[1];

					if((ckA == buffer[3]) && (ckB == buffer[4])){

					}
					flow_x = (int)buffer[0];
					flow_x = (int)buffer[0];
					step=0;
				}
			}
			break;
	}

};
void flowCallback()
{
	pasrseMsg(bytee);
    HAL_UART_Receive_IT(uart_, &bytee,1);
}

