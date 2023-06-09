#include <opticalflow.h>
#include "imu.h"
#include "maths.h"
#include "hc_sr04.h"
static UART_HandleTypeDef *uart_;
static uint8_t bytee;
static uint8_t buf[5];
static int flow_x,flow_y;
static uint8_t quality;

void flowInit(UART_HandleTypeDef *uartt,uint32_t baudrate){
	uart_ = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt); //
	HAL_UART_Receive_IT(uart_, &bytee,1);
}
static void pasrseMsg(uint8_t c){
	    static unsigned char last_char;
	    static int start = 0;
	    static int index=0;
	    if(c == 98 && last_char == 181 ){
	         start = 1;
	         return;
	    }
	    last_char = c;
	    if(start){
	    	buf[index++] = c;
	        if(index>4){
	        	flow_x = (int8_t)buf[0];
	        	flow_y = (int8_t)buf[1];
	        	quality = buf[2];
	            index =0;
	            start =0;
	        }
	    }
	}
/* Callback function*/
void flowCallback()
{
	pasrseMsg(bytee);
    HAL_UART_Receive_IT(uart_, &bytee,1);
}


#define GYRO_DELAY 4
#define FLOW_LPF_GAIN 0.3f
#define GYRO_LPF_GAIN 0.4f
#define GYRO_SCALE_FACTO 1.3f
static float buffe1[GYRO_DELAY+1],buffe2[GYRO_DELAY+1];
float gyro_get_old_val(float *buf,float data){
	buf[GYRO_DELAY-1] = data;
	for(int i=0;i<(GYRO_DELAY-1);i++){
		 buf[i] =  buf[i+1];
	}
	return  buf[0];
}

float flow_facto_x,flow_facto_y;
static float flowy,gyro_yy,flowx,gyro_xx; //global for debugging
void optical_flow_run(){
	    // Y -> PITCH axis
		flowy = flowy*(1 - FLOW_LPF_GAIN) + FLOW_LPF_GAIN*flow_y;
		float gyroyy =gyro_get_old_val(buffe1,quad_.pitch_velocity);
		gyroyy  = constrainf(gyroyy, -60,60);
		gyro_yy = gyro_yy*(1 - GYRO_LPF_GAIN) + GYRO_LPF_GAIN*gyroyy;
		gyro_yy *= GYRO_SCALE_FACTO;
		flow_facto_y = flowy - gyro_yy;
		// X -> ROLL axis
		flowx = flowx*(1 - FLOW_LPF_GAIN) + FLOW_LPF_GAIN*flow_x;
		float gyroxx =gyro_get_old_val(buffe2,-quad_.roll_velocity);
		gyroxx  = constrainf(gyroxx, -60,60);
		gyro_xx = gyro_xx*(1 - GYRO_LPF_GAIN) + GYRO_LPF_GAIN*gyroxx;
		gyro_xx *= GYRO_SCALE_FACTO;
		flow_facto_x = flowx - gyro_xx;
   }

