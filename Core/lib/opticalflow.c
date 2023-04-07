/*
 * opticalflow.c
 *
 *  Created on: Apr 3, 2023
 *      Author: sudo
 */

#include <opticalflow.h>
static UART_HandleTypeDef *uart_;
static uint8_t bytee;

static int flow_x,flow_y,quality;
static uint8_t step =0;
static uint8_t index=0;
void flowInit(UART_HandleTypeDef *uartt,uint32_t baudrate){
	uart_ = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt); //reInit
	HAL_UART_Receive_IT(uart_, &bytee,1);
}
static void pasrseMsg(uint8_t k){

	switch(step){
		case 0:
			if(k == 181)//0xb5
				step++;
			break;
		case 1:
			if(k == 98)
				step++; //0x62
			break;
		case 2:
			flow_x  = (int8_t)k;
            step++;
            break;
		case 3:
			flow_y  = (int8_t)k;
            step++;
            break;
		case 4:
			quality = k;
            step =0;
            break;
	}

};
void flowCallback()
{
	pasrseMsg(bytee);
    HAL_UART_Receive_IT(uart_, &bytee,1);
}
int get_flow(int x){
	if(x==0)
		return flow_x;
	if(x==1)
		return flow_y;
	if(x==2)
		return quality;
	return 0;
}

