#include <opticalflow.h>
#include "imu.h"
#include "maths.h"
#include "hc_sr04.h"
#include "math.h"
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
#define GYRO_DELAY_CYCLE 8
static float buffe1[GYRO_DELAY_CYCLE+1],buffe2[GYRO_DELAY_CYCLE+1];
float gyro_get_old_val(float *buf,float data){
	buf[GYRO_DELAY_CYCLE-1] = data;
	for(int i=0;i<(GYRO_DELAY_CYCLE-1);i++){
		 buf[i] =  buf[i+1];
	}
	return  buf[0];
}

#define FLOW_LPF_GAIN 0.23f
#define GYRO_LPF_GAIN 0.4f   //0.23
#define GYRO_SCALE_FACTO 0.53f
#define GYRO_LIMITER 20.0f
float flow_factor_x,flow_factor_y;
float flow_y1pt,gyro_y1pt;
float flow_x1pt,gyro_x1pt;
pid__t x_flow_t ={
	 .f_cut_D =0,
	 .kp = 1,
	 .ki = 0,
	 .kd = 0,
	 .max_P = 300,
	 .max_I = 0,
	 .max_D = 0,
	 .max_pid = 400
};
pid__t y_flow_t ={
	 .f_cut_D =0,
	 .kp = 1,
	 .ki = 0,
	 .kd = 0,
	 .max_P = 300,
	 .max_I = 0,
	 .max_D = 0,
	 .max_pid = 10   // 10 degree
};
	 
void optical_flow_run(){
	    // Y -> PITCH axis
	    static float last_flow_value;
		flow_y1pt = flow_y1pt*(1 - FLOW_LPF_GAIN) + FLOW_LPF_GAIN*flow_y;
		float gyroy =gyro_get_old_val(buffe1,quad_.pitch_velocity*GYRO_SCALE_FACTO);
		gyro_y1pt = gyro_y1pt*(1 - GYRO_LPF_GAIN) + GYRO_LPF_GAIN*gyroy;
		if(fabs(gyro_y1pt)>fabs(flow_y1pt)){
		 	gyro_y1pt = flow_y1pt;
		}
		flow_factor_y = flow_y1pt - gyro_y1pt;
		if(fabs(gyro_y1pt)>GYRO_LIMITER){
			flow_factor_y = 0.0f;
		}
		// X -> ROLL axis
		flow_x1pt = flow_x1pt*(1 - FLOW_LPF_GAIN) + FLOW_LPF_GAIN*flow_x;
		float gyrox =gyro_get_old_val(buffe2,-quad_.roll_velocity*GYRO_SCALE_FACTO);
		gyro_x1pt = gyro_x1pt*(1 - GYRO_LPF_GAIN) + GYRO_LPF_GAIN*gyrox;
		if(fabs(gyro_x1pt)>fabs(flow_x1pt)){
		 	gyro_x1pt = flow_x1pt;
		}
		flow_factor_x = flow_x1pt - gyro_x1pt;
		if(fabs(gyro_x1pt)>GYRO_LIMITER){
			flow_factor_x = 0.0f;
		}
		pidCalculate(&x_flow_t,flow_factor_x,0,26000);
		pidCalculate(&x_flow_t,flow_factor_x,0,26000);

   }

