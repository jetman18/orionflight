#include "mainloop.h"
#include "mpu6500.h"
#include "qmc5883.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
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
#include <string.h>
#define MINIMUM_THROTLE  1050
#define BALAN


euler_angle_t mpu;
rcChannel_t rx;
MAG_t t;

uint16_t dttime;
float 	Ch1,Ch2,Ch4;
float ch1_gain;
float ch2_gain;
float ch4_gain;
uint16_t cl_ch1,cl_ch2,cl_ch4,throtle;

float pid_pitch,pid_roll,pid_yaw;
pid__t pitch_t,roll_t,yaw_t;
float acc[3],gyr[3];
uint16_t moto1,moto2,moto3,moto4;

uint16_t mytime;

//bmp280
BMP280_HandleTypedef bmp280;
float temperature,pressure,altitude;
faxis3_t angle;
int k;
void main_loop(){
	//---system--init------------------------------
	HAL_TIM_Base_Start_IT(&htim4);
	//---external------------------------------
	MPU_spi_init(&hspi1,GPIOA,GPIO_PIN_4);
	//qmc5883_init(&hi2c2);
	//magnet_sensor_calibrate();

	//bmp280 sensor
	//bmp280_init_default_params(&bmp280.params);
    //bmp280.i2c = &hi2c2;
	//bmp280.addr = BMP280_I2C_ADDRESS_0;
	//bmp280_init(&bmp280,&bmp280.params);

	memset(&pitch_t, 0, sizeof(pid__t));
    memset(&roll_t, 0, sizeof(pid__t));
    memset(&yaw_t, 0, sizeof(pid__t));
	memset(&rx, 0, sizeof(rcChannel_t));
#ifdef BALAN

	initPWM(&htim2);
	dttime =1000;
	/*  PID 250HZ
	 *  KP = 3.2
	 *  KI = 0.6
	 *  KD = 1.0
	 */

	/*init pid gain*/
	pitch_t.kp =3.0f;   //3.3f
	pitch_t.ki =0.6f;  //0.6
	pitch_t.kd =0.7f;  //1.0f

	roll_t.kp = pitch_t.kp;
	roll_t.ki = pitch_t.ki;
	roll_t.kd = pitch_t.kd;

	yaw_t.kp = 1000;//1000;
	yaw_t.ki = 500;//400;
	yaw_t.kd = 0;

	ch1_gain =0.11f;
	ch2_gain =0.11f;
	ch4_gain =0.001f;
#else
	initOneshot125(&htim2);
	dttime =500;
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
/*
	while((rx.ch[2] > MINIMUM_THROTLE) || (rx.ch[2]<1000) ){
           HAL_Delay(100);
           HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	rxCalibrate(rx,&cl_ch1,&cl_ch2,&cl_ch4);
*/
while(1){

    #ifdef BALAN
	 	   time_start_measure();
	       mpu_update(&mpu,dttime);
	 	   //gyro_read(&angle);
	       //bmp280_read_fixed(&bmp280, &temperature, &pressure,&altitude);
	       //qmc_get_values(&t,0,0);
	       k = get_measure_time();

    #else
    	   MPU_update_acro(&mpu,dttime);
    #endif

    	   //print_float(mpu.roll);
    	  // print_float(mpu.roll);
    	   //print_char("\n");
/*
    #ifdef BALAN
    	   FEQUENCY_DIV(2,ENABLE){
    #else
    		if(1){
    #endif
        	if(rx.ch[2]>MINIMUM_THROTLE){
                // pid calculate
        		pidCalculate(&pitch_t,mpu.pitch,Ch2);
    			pidCalculate(&roll_t ,mpu.roll ,Ch1);
    			pidCalculate(&yaw_t  ,mpu.yaw  ,Ch4);

    			moto1 = rx.ch[2] + (- pitch_t.PID - roll_t.PID - yaw_t.PID);
    			moto2 = rx.ch[2] + (- pitch_t.PID + roll_t.PID + yaw_t.PID);
    			moto3 = rx.ch[2] + (  pitch_t.PID + roll_t.PID - yaw_t.PID);
    			moto4 = rx.ch[2] + (  pitch_t.PID - roll_t.PID + yaw_t.PID);

        	   }
        	else {
        		   pitch_t.I=0.0f;
        		   roll_t.I =0.0f;
        		   yaw_t.I  =0.0f;

        		   moto1 = 1000;
        		   moto2 = 1000;
        		   moto3 = 1000;
        		   moto4 = 1000;

        		   mpu.pitch= 0.0f;
        		   mpu.roll = 0.0f;
        		   mpu.yaw  = 0.0f;
        	   }

            	// update throtle to esc
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

        FEQUENCY_DIV(250,ENABLE){
    	     HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
         }
         */
    	/*loop feq */
    	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        looptime(dttime);
      }
}
/*-------------ISR HANDLER----------------------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim4)
	{
		callBack();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14) // If The INT Source Is EXTI 14
    {
    	callBackFuncition(&rx);
    	if(isRxupdate()){
    		Ch1 = ChannelTodec(rx.ch[0],cl_ch1,ch1_gain);
    		Ch2 = ChannelTodec(rx.ch[1],cl_ch2,ch2_gain);
    		Ch4 = ChannelTodec(rx.ch[3],cl_ch4,ch4_gain);
    	}
    }

}
