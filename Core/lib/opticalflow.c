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
/*
FEQUENCY_DIV(20,ENABLE){
		static float gyro_x,sub_x,gyro_y,sub_y;
		static float flowx,flowy;
		quality_=get_flow(2);
		flowx=0.22*(get_flow(X) - flowx); //0.22
		gyro_x =get_gyro(X)*0.0022;
		gyro_x = constrainf(gyro_x,-23,23);
		gyro_x = storeGyro1(gyro_x) + flowx;
		sub_x +=0.05*(gyro_x - sub_x);

		flowy=0.22*(get_flow(Y) - flowy);
		gyro_y =get_gyro(Y)*0.0033;
		gyro_y = constrainf(gyro_y,-23,23);
		gyro_y =storeGyro2(gyro_y) - flowy ;
		sub_y +=0.05*(gyro_y - sub_y);

		write_int(&huart1,sub_x*50);
		write_char(&huart1," ");
		write_int(&huart1,sub_y*50);
	    write_char(&huart1,"\n");

       if(ch5>1500 && fabs(rx_ch1)<1 && fabs(rx_ch2)<1){
			pidCalculate(&flow_x_axis,sub_x*1.2,0,25000);
			pidCalculate(&flow_y_axis,sub_y,0,25000);
       }
       else{
       	flow_x_axis.I =0;
       	flow_y_axis.I =0;
       }
   }


float storeGyro1(float data){
	buffe1[GYRO_DELAY-1] = data;
	for(int i=0;i<(GYRO_DELAY-1);i++){
		 buffe1[i] =  buffe1[i+1];
	}
	return  buffe1[0];
}
float storeGyro2(float data){
	buffe2[GYRO_DELAY-1] = data;
	for(int i=0;i<(GYRO_DELAY-1);i++){
		 buffe2[i] =  buffe2[i+1];
	}
	return  buffe2[0];
}
   */
