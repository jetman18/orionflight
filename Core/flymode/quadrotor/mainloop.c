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
/******************************************/
const float ch1_gain = 0.2f;
const float ch2_gain = 0.2f;
const float ch4_gain =0.001f;
static const uint16_t dttime = 1000;
//faxis3_t t;
static float  rx_ch1,rx_ch2,rx_ch4;
static uint16_t throttle;
pid__t pitch_t,roll_t,yaw_t;
static uint16_t moto1,moto2,moto3,moto4;
static euler_angle_t mpu;
//BMP280_HandleTypedef bmp280;
//static float temperature,pressure,altitude;
void main_loop(){
	initTimeloop(&htim3);
	//debugInit(&huart1,&hi2c2);
	/*--ibus--*/
	ibusInit(&huart2,115200);
	/*--MPU--*/
	//MPU_spi_init(&hspi1,GPIOA,GPIO_PIN_4);
	MPU_i2c_init(&hi2c2);
	/*--GPS--*/
	//gpsInit(&huart3,57600);
    /*--pwm-out--*/
	initPWM(&htim4);
	motoIdle();

	//bmp280 sensor
	//bipbip(4);
	//bmp280_init_default_params(&bmp280.params);
    //bmp280.addr = BMP280_I2C_ADDRESS_0;
    //bmp280.i2c = &hi2c1;
	//bmp280_init(&bmp280,&bmp280.params);

	//qmc5883_init(&hi2c1);
	//screen_dpl();

	/*init pid gain*/
	pitch_t.kp =4.5f;
	pitch_t.ki =7.9f;
	pitch_t.kd =0.12f;

	roll_t.kp = pitch_t.kp;
	roll_t.ki = pitch_t.ki;
	roll_t.kd = pitch_t.kd;

	yaw_t.kp =200;
	yaw_t.ki =1000;
	yaw_t.kd = 0;

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
    start_=1;
	    imu_update(&mpu,dttime);
	   /*pid rate 500hz*/
	   FEQUENCY_DIV(2,ENABLE){
		if( start_ && throttle>1030){
			// pid calculate
			pidCalculate(&pitch_t,mpu.pitch,rx_ch2,2000,300);//rx_ch2
			pidCalculate(&roll_t ,mpu.roll,rx_ch1,2000,300);//rx_ch1
			pidCalculate(&yaw_t  ,mpu.yaw,rx_ch4,2000,0);//rx_ch4

			moto1 = throttle - pitch_t.PID - roll_t.PID - yaw_t.PID;
			moto2 = throttle - pitch_t.PID + roll_t.PID + yaw_t.PID;
			moto3 = throttle + pitch_t.PID + roll_t.PID - yaw_t.PID;
			moto4 = throttle + pitch_t.PID - roll_t.PID + yaw_t.PID;

		}
		else{
			//reset I
			pitch_t.I=0.0f;
			roll_t.I =0.0f;
			yaw_t.I  =0.0f;

			moto1 = 1000;
			moto2 = 1000;
			moto3 = 1000;
			moto4 = 1000;
		   }
			//update throttle to esc
			writePwm(ch1,moto1);
			writePwm(ch2,moto2);
			writePwm(ch3,moto3);
			writePwm(ch4,moto4);
		}
     /*blink led */

	 FEQUENCY_DIV(500,ENABLE){
     	 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
    }

    looptime(dttime);
   }
}


//reDefine iqr function
/*----------------------------------IQR--Handle-----------------------------*/
                                                                            //
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)                     //
{                                                                           //
	/* ibus */                                                              //
    if(huart == &huart2){                                                   //
    	ibusCallback(&huart2);
    }                                                                       //
    /* optical flow */                                                               //
    else if(huart == &huart1){                                              //
    	//gpsCallback();                                                      //
     }                                                                       //
}                                                                           //
/*----------------------------------IQR--Handle-----------------------------*/

