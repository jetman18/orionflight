#include <log.h>
#include "mainloop.h"
#include "mpu6500.h"
#include "qmc5883.h"
#include "i2c.h"
//#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "timeclock.h"
#include "scheduler.h"
#include "pwmwrite.h"
#include "ppmreceiver.h"
#include "pid.h"
#include "maths.h"
#include "lpf.h"
#include "bmp280.h"
#include "ibus.h"
#include "config.h"
#include <string.h>
#include "gps.h"
#include "./ssd1306/ssd1306.h"
#include "opticalflow.h"
#define GYRO_DELAY 4

static float  rx_ch1,rx_ch2,rx_ch4;
static uint16_t throttle,ch5;
static pid__t pitch_t,roll_t,yaw_t,flow_x_axis,flow_y_axis;
static uint16_t moto1,moto2,moto3,moto4;
static euler_angle_t mpu;
float buffe1[GYRO_DELAY];
float buffe2[GYRO_DELAY];
float storeGyro1(float data);
float storeGyro2(float data);

//BMP280_HandleTypedef bmp280;
//static float temperature,pressure,altitude;
void main_loop(){
/*----------PARAMETER-init----------------------*/
	pitch_t.kp = 4.1f;
	pitch_t.ki =6.9f;
	pitch_t.kd =0.1f;
	pitch_t.max_i = 200;
	pitch_t.max_pid = 400;
	pitch_t.f_cut_D =300;

	yaw_t.kp =200;
	yaw_t.ki =1000;
	yaw_t.kd = 0;
	yaw_t.max_i = 300;
	yaw_t.max_pid = 400;
	yaw_t.f_cut_D =0;

	flow_x_axis.kp =7.0f;
	flow_x_axis.ki =0;
	flow_x_axis.kd =0;
	flow_x_axis.max_i = 3.0f;
	flow_x_axis.max_pid =40.0f;
	flow_x_axis.f_cut_D = 100;

	const float ch1_gain = 0.12f;
	const float ch2_gain = 0.12f;
	const float ch4_gain =0.012f;

	const uint16_t dttime = 2000;
	flow_y_axis.kp=flow_x_axis.kp;
	flow_y_axis.ki=flow_x_axis.ki;
	flow_y_axis.kd=flow_x_axis.kd;
	flow_y_axis.max_i = flow_x_axis.max_i;
	flow_y_axis.max_pid = flow_x_axis.max_pid;
	flow_y_axis.f_cut_D =flow_x_axis.f_cut_D;

	roll_t.kp = pitch_t.kp;
	roll_t.ki = pitch_t.ki;
	roll_t.kd = pitch_t.kd;
	roll_t.max_i = pitch_t.max_i;
	roll_t.max_pid = pitch_t.max_pid;
	roll_t.f_cut_D =pitch_t.f_cut_D;

/*-----------------INIT MODULES----------------------*/
    /*--pwm-out--*/
	initPWM(&htim4);
	motoIdle();
	/*------------------*/
	initTimeloop(&htim3);
	/*--ibus--*/
	ibusInit(&huart1,115200);
	/*--MPU--*/
	//MPU_spi_init(&hspi1,GPIOA,GPIO_PIN_4);
	MPU_i2c_init(&hi2c2);
	/*--GPS--*/
	//gpsInit(&huart3,57600);

	//bmp280 sensor
	//bmp280_init_default_params(&bmp280.params);
    //bmp280.addr = BMP280_I2C_ADDRESS_0;
    //bmp280.i2c = &hi2c1;
	//bmp280_init(&bmp280,&bmp280.params);

	//qmc5883_init(&hi2c1);

	/*flow*/
	flowInit(&huart2,115200);
/*--------------------------------------------*/
while(1){
	static uint8_t quality_;
    static int start_ = 0;
    if(ibusFrameComplete()){
		rx_ch1=ibusReadf(CH1,ch1_gain);
		rx_ch2=ibusReadf(CH2,ch2_gain);
		throttle=ibusReadRawRC(CH3);
		ch5=ibusReadRawRC(CH5);
		rx_ch4=ibusReadf(CH4,ch4_gain);
		if(!start_){
			if(ibusReadRawRC(CH3)<1100 &&  ibusReadRawRC(CH4)<1100 &&
			   ibusReadRawRC(CH2)>1900 &&  ibusReadRawRC(CH1)<1100){
               start_ =1;
		    }
        }
    }
	imu_update(&mpu,dttime);
	/*optical flow  25hz*/
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
/*
			write_int(&huart1,sub_x*50);
			write_char(&huart1," ");
			write_int(&huart1,sub_y*50);
		    write_char(&huart1,"\n");
*/
            if(ch5>1500 && fabs(rx_ch1)<1 && fabs(rx_ch2)<1){
				pidCalculate(&flow_x_axis,sub_x*1.2,0,25000);
				pidCalculate(&flow_y_axis,sub_y,0,25000);
            }
            else{
            	flow_x_axis.I =0;
            	flow_y_axis.I =0;
            }
	    }

	   /*Pid rate 500hz*/
		if( start_ && throttle>1030){
			pidCalculate(&pitch_t,mpu.pitch,rx_ch2 + flow_y_axis.PID,2000);//rx_ch2
			pidCalculate(&roll_t ,mpu.roll,rx_ch1  - flow_x_axis.PID,2000);//rx_ch1
			pidCalculate(&yaw_t  ,mpu.yaw,rx_ch4,2000);//rx_ch4

			moto1 = throttle - pitch_t.PID - roll_t.PID - yaw_t.PID;
			moto2 = throttle - pitch_t.PID + roll_t.PID + yaw_t.PID;
			moto3 = throttle + pitch_t.PID + roll_t.PID - yaw_t.PID;
			moto4 = throttle + pitch_t.PID - roll_t.PID + yaw_t.PID;

			writePwm(ch1,moto1);
			writePwm(ch2,moto2);
			writePwm(ch3,moto3);
			writePwm(ch4,moto4);
		}
		else{
			pitch_t.I=0.0f;
			roll_t.I =0.0f;
			yaw_t.I  =0.0f;
			resetVector();
			motoIdle();
		   }
     /*blink led pb3-pb4 */
	 FEQUENCY_DIV(500,ENABLE){
     	 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
     }
	 /*quality check*/
	FEQUENCY_DIV(100,ENABLE){
		 if(quality_<100)
			  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		 else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,0);
    }
    looptime(dttime);
   }
}


// IQR function
/*----------------------------------IQR--Handle-----------------------------*/
                                                                            //
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)                     //
{                                                                           //
	/* ibus */                                                              //
    if(huart == &huart1){                                                   //
    	ibusCallback();                                                     //
    }                                                                       //
    /* optical flow */                                                      //
    else if(huart == &huart2){                                              //
    	flowCallback();
     }                                                                      //
}                                                                           //
/*----------------------------------IQR--Handle-----------------------------*/

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


