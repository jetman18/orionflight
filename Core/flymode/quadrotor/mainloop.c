#include "mainloop.h"
#include "sensor.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "timeclock.h"
#include "scheduler.h"
#include "pwmwrite.h"
#include "ppmreceive.h"
#include "pid.h"
#include "debug.h"
#include "maths.h"
#include "lpf.h"

#define LOOPTIME  1000
#define MINIMUMTHROTLE  1050

euler_angle_t mpu;
rcChannel_t rx;
MAG_t t;

//IMU_raw_t k;

float 	Ch1,Ch2,Ch4;
float ch1_gain =0.1f;
float ch2_gain =0.1f;
float ch4_gain =0.0007f;
uint16_t cl_ch1,cl_ch2,cl_ch4;

float pid_pitch=0,pid_roll=0,pid_yaw=0;
pid_gain_t pitch_t,roll_t,yaw_t;
float acc[3],gyr[3];
uint16_t moto1,moto2,moto3,moto4;

void main_loop(){
	//---system--init------------------------------
	HAL_TIM_Base_Start_IT(&htim4);

	//---external------------------------------

	MPU_spi_init(&hspi1,GPIOA,GPIO_PIN_4);
	initPWM(&htim2);
	qmc5883_init(&hi2c2);

	//magnet_sensor_calibrate();

	/*init pid gain*/
	pitch_t.kp =3.3f;
	pitch_t.ki =0.5f;
	pitch_t.kd =1.0f;
	pitch_t.I =0;

	roll_t.kp = pitch_t.kp;
	roll_t.ki = pitch_t.ki;
	roll_t.kd = pitch_t.kd;
	roll_t.I =0;

	yaw_t.kp = 800;
	yaw_t.ki = 0;
	yaw_t.kd = 0;
    yaw_t.I  = 0;

    /*
	while((rx.ch[2] > MINIMUMTHROTLE) || (rx.ch[2]<1000) ){
           HAL_Delay(300);
           HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	rxCalibrate(rx,&cl_ch1,&cl_ch2,&cl_ch4);
   */

	while(1){


       MPU_update(&mpu,2000);
       qmc_get_values(&t,0,0);
        //mahony filter
		//mpu_get_gyro_calib(&k,1000);
        //updateIMU(k.gyrox*0.01745f,k.gyroy*0.01745f,k.gyroz*0.01745f,k.accx,k.accy,k.accz);
        //computeAnglesFromQuaternion(&mpu);

	   /* update PID 400hz*/

       //FEQUENCY_DIV(2,DISABLE)
      //{

    	if(rx.ch[2]>MINIMUMTHROTLE){
            // pid calculate
    		pidCalculate(&pitch_t,mpu.pitch,Ch2,2000);
			pidCalculate(&roll_t ,mpu.roll ,Ch1,2000);
			pidCalculate(&yaw_t  ,mpu.yaw  ,Ch4,2000);

			moto1 = rx.ch[2] + (- pitch_t.PID - roll_t.PID - yaw_t.PID);
			moto2 = rx.ch[2] + (- pitch_t.PID + roll_t.PID + yaw_t.PID);
			moto3 = rx.ch[2] + ( pitch_t.PID + roll_t.PID - yaw_t.PID);
			moto4 = rx.ch[2] + ( pitch_t.PID - roll_t.PID + yaw_t.PID);

			constrain(moto1,1000,2000);
			constrain(moto2,1000,2000);
			constrain(moto3,1000,2000);
			constrain(moto4,1000,2000);
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
    		writePwm(ch1,moto1);
    		writePwm(ch2,moto2);
    		writePwm(ch3,moto3);
    		writePwm(ch4,moto4);

    // FEQUENCY_DIV(100,ENABLE){
	   //  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	 //}
	/*loop feq */
    looptime(2000);
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
