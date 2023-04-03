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
#include "debug.h"
#include "maths.h"
#include "lpf.h"
#include "bmp280.h"
#include "ibus.h"
#include "config.h"
#include <string.h>
#include "gps.h"
#include "./ssd1306/ssd1306.h"
static float  rx_ch1,rx_ch2,rx_ch4;
static uint16_t throttle;
static pid__t pitch_t,roll_t,yaw_t,flow_x,flow_y;
static uint16_t moto1,moto2,moto3,moto4;
static euler_angle_t mpu;
//BMP280_HandleTypedef bmp280;
//static float temperature,pressure,altitude;
void main_loop()
{
/*----------INIT-PARAMETER-----------------------*/
	pitch_t.kp =4.5f;
	pitch_t.ki =7.9f;
	pitch_t.kd =0.12f;
	pitch_t.max_i = 200;
	pitch_t.max_pid = 400;
	pitch_t.f_cut_D =300;

	yaw_t.kp =200;
	yaw_t.ki =1000;
	yaw_t.kd = 0;
	yaw_t.max_i = 300;
	yaw_t.max_pid = 400;
	yaw_t.f_cut_D =0;

	flow_x.kp =0;
	flow_x.ki =0;
	flow_x.kd =0;
	flow_x.max_i = 5.0f;
	flow_x.max_pid =7.0f;
	flow_x.f_cut_D = 100;

	const float ch1_gain = 0.2f;
	const float ch2_gain = 0.2f;
	const float ch4_gain =0.001f;

	static const uint16_t dttime = 1000;
/////////////////////////////////////////////////////////
	flow_y.kp=flow_x.kp;
	flow_y.ki=flow_x.ki;
	flow_y.kd=flow_x.kd;
	flow_y.max_i = flow_x.max_i;
	flow_y.max_pid = flow_x.max_pid;
	flow_y.f_cut_D =flow_x.f_cut_D;

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
	//debugInit(&huart1,&hi2c2);
	/*--ibus--*/
	ibusInit(&huart2,115200);
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


while(1){
    static int start_ = 0;
    if(ibusFrameComplete()){
		rx_ch1=ibusReadf(CH1,ch1_gain);
		rx_ch2=ibusReadf(CH2,ch2_gain);
		throttle=ibusReadRawRC(CH3);
		rx_ch4=ibusReadf(CH4,ch4_gain);
		if(!start_){
			if(ibusReadRawRC(CH3)<1100 &&  ibusReadRawRC(CH4)<1100 &&
			   ibusReadRawRC(CH2)>1900 &&  ibusReadRawRC(CH1)<1100){
			  //countt++;
               start_ =1;
		    }
        }
    }
	imu_update(&mpu,dttime);
	    /*optical flow processing 33hz*/
		 FEQUENCY_DIV(30,ENABLE){
			  //float intput_rate_control_x;
			  //float intput_rate_control_y;
			 // pidCalculate(&flow_x,0,intput_rate_control_x,2000);
			 // pidCalculate(&flow_y,0,intput_rate_control_y,2000);
	     }
	   /*Pid rate 500hz*/
	   FEQUENCY_DIV(2,ENABLE){
		if( start_ && throttle>1030){
			// pid calculate
			pidCalculate(&pitch_t,mpu.pitch,rx_ch2,2000);//rx_ch2
			pidCalculate(&roll_t ,mpu.roll,rx_ch1,2000);//rx_ch1
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
			//reset I
			pitch_t.I=0.0f;
			roll_t.I =0.0f;
			yaw_t.I  =0.0f;
			motoIdle();
		   }
		}
     /*blink led pb3-pb4 */
	 FEQUENCY_DIV(500,ENABLE){
     	 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
    }

    looptime(dttime);
   }
}


//reDefine IQR function
/*----------------------------------IQR--Handle-----------------------------*/
                                                                            //
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)                     //
{                                                                           //
	/* ibus */                                                              //
    if(huart == &huart2){                                                   //
    	ibusCallback(&huart2);                                              //
    }                                                                       //
    /* optical flow */                                                      //
    else if(huart == &huart1){                                              //
    	//gpsCallback();                                                    //
     }                                                                      //
}                                                                           //
/*----------------------------------IQR--Handle-----------------------------*/

