#include "mainloop.h"
#include "mpu6500.h"
#include "qmc5883.h"
#include "i2c.h"
#include "spi.h"
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
/******************************************/
float 	Ch1,Ch2,Ch4,Ch5,Ch6;
uint16_t throttle;

pid__t pitch_t,roll_t,yaw_t;
uint16_t moto1,moto2,moto3,moto4;

euler_angle_t mpu;
MAG_t t;

//bmp280
BMP280_HandleTypedef bmp280;
float temperature,pressure,altitude;
int k,k1;
void main_loop(){
	initTimeloop(&htim4);
	ibusInit(&huart1);
	MPU_spi_init(&hspi1,GPIOA,GPIO_PIN_4);
	//qmc5883_init(&hi2c2);
	//magnet_sensor_calibrate();

	/*
	//bmp280 sensor
	bmp280_init_default_params(&bmp280.params);
    bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280_init(&bmp280,&bmp280.params);
	*/

	memset(&pitch_t, 0, sizeof(pid__t));
    memset(&roll_t, 0, sizeof(pid__t));
    memset(&yaw_t, 0, sizeof(pid__t));

    const float ch1_gain = 0.11f;
    const float ch2_gain = 0.11f;
    const float ch4_gain =0.000005;
#ifdef BALAN
	initPWM(&htim2);
	const uint16_t dttime = 1000;
	/*  PID 250HZ
	 *  KP = 3.2
	 *  KI = 0.6
	 *  KD = 1.0
	 */

	/*init pid gain*/
	pitch_t.kp =3.0f;   //3.3f
	pitch_t.ki =0.00002f;
	pitch_t.kd =0.7f;  //1.0f

	roll_t.kp = pitch_t.kp;
	roll_t.ki = pitch_t.ki;
	roll_t.kd = pitch_t.kd;

	yaw_t.kp = 120000;//1000;
	yaw_t.ki = 20;//400;
	yaw_t.kd = 0;

#else
	initOneshot125(&htim2);
	const uint16_t dttime = 500;
	/*init pid gain*/
	pitch_t.kp =250;//400;   //3.3f
	pitch_t.ki =1100;//30.0f;  //0.6
	pitch_t.kd =0;  //1.0f

	roll_t.kp = pitch_t.kp;
	roll_t.ki = pitch_t.ki;
	roll_t.kd = pitch_t.kd;

	yaw_t.kp = 230;//1000;
	yaw_t.ki = 1050;//500;
	yaw_t.kd = 0; //0

	ch1_gain =0.003f;
	ch2_gain =0.003f;
	ch4_gain =0.003f;
#endif
while(1){
    if(ibusFrameComplete()){
		Ch1=ibusReadf(CH1,ch1_gain);
		Ch2=ibusReadf(CH2,ch2_gain);
		throttle=ibusReadRawRC(CH3);
		Ch4=ibusReadf(CH4,ch4_gain);
    }

#ifdef BALAN
	       mpu_update(&mpu,dttime);
	       //bmp280_read_fixed(&bmp280, &temperature, &pressure,&altitude);
	       //qmc_get_values(&t,0,0);
#else
    	   MPU_update_acro(&mpu,dttime);
#endif

    	   //print_float(Ch2);
    	   //print_int(mpu.roll);
    	   //print_char("\n");

#ifdef BALAN
    	  FEQUENCY_DIV(2,ENABLE){
#else
          if(1){
#endif

        	if(throttle>MINIMUM_THROTLE){
                // pid calculate
        		pidCalculate(&pitch_t,mpu.pitch,Ch2,100);//Ch2
    			pidCalculate(&roll_t ,mpu.roll ,Ch1,100);//Ch1
    			pidCalculate(&yaw_t  ,mpu.yaw  ,Ch4,100);//Ch4

    			moto1 = throttle + (-pitch_t.PID - roll_t.PID - yaw_t.PID);
    			moto2 = throttle + (-pitch_t.PID + roll_t.PID + yaw_t.PID);
    			moto3 = throttle + ( pitch_t.PID + roll_t.PID - yaw_t.PID);
    			moto4 = throttle + ( pitch_t.PID - roll_t.PID + yaw_t.PID);

        	   }
        	else {
        		   pitch_t.I=0.0f;
        		   roll_t.I =0.0f;
        		   yaw_t.I  =0.0f;

        		   moto1 = 1000;
        		   moto2 = 1000;
        		   moto3 = 1000;
        		   moto4 = 1000;

        	   }

            	// update throttle to esc
    #ifdef BALAN
        		writePwm(ch1,moto1);
        		writePwm(ch2,moto2);
        		writePwm(ch3,moto3);
        		writePwm(ch4,moto4);
    #else
        		moto1 = (240.0f/2000)*moto1;
        		moto2 = (240.0f/2000)*moto2;
        		moto3 = (240.0f/2000)*moto3;
        		moto4 = (240.0f/2000)*moto4;

        		writeOneshot125(ch1,moto1);
        		writeOneshot125(ch2,moto2);
        		writeOneshot125(ch3,moto3);
        		writeOneshot125(ch4,moto4);
    #endif
    		}

        FEQUENCY_DIV(500,ENABLE){
    	     HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
         }

     /*loop feq */
    looptime(dttime);
   }
}




