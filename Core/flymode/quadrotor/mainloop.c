#include "MAVLink/common/mavlink.h"
#include <log.h>
#include "mainloop.h"
#include "mpu6500.h"
#include "qmc5883.h"
#include "i2c.h"
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
#define MINIMUN_THROTTLE 1030
uint32_t m1,m2;
static float  rx_ch1,rx_ch2,rx_ch4;
static uint16_t throttle,ch5;
pid__t Pid_angle_pitch_t,Pid_angle_roll_t,Pid_angle_yaw_t,flow_x_axis,flow_y_axis;
pid__t Pid_velocity_pitch_t,Pid_velocity_roll_t, Pid_velocity_yaw_t;
static uint16_t moto1,moto2,moto3,moto4;
static attitude_t mpu;
float buffe1[GYRO_DELAY];
float buffe2[GYRO_DELAY];
float storeGyro1(float data);
float storeGyro2(float data);
static float gyro_yaw,qmc_yaw;
//BMP280_HandleTypedef bmp280;
//static float temperature,pressure,altitude;
void main_loop(){
/*----------PARAMETER-init----------------------*/

    /*-----------angle PID------------*/
	Pid_angle_pitch_t.kp =1.2f;
	Pid_angle_pitch_t.ki =0;
	Pid_angle_pitch_t.kd =0;
	Pid_angle_pitch_t.max_i = 0;   //
	Pid_angle_pitch_t.max_pid =120;  //
	Pid_angle_pitch_t.f_cut_D =200;

	Pid_angle_yaw_t.kp =2;
	Pid_angle_yaw_t.ki =0;//1000
	Pid_angle_yaw_t.kd = 0;
	Pid_angle_yaw_t.max_i = 0;
	Pid_angle_yaw_t.max_pid = 120;
	Pid_angle_yaw_t.f_cut_D =0;

    /*-------angle velocity PID--*/
   	Pid_velocity_pitch_t.kp = 2.5;
	Pid_velocity_pitch_t.ki = 1.6;
	Pid_velocity_pitch_t.kd = 0;
	Pid_velocity_pitch_t.max_i = 200;
	Pid_velocity_pitch_t.max_pid = 400;
	Pid_velocity_pitch_t.f_cut_D =300;

	Pid_velocity_yaw_t.kp =3;
	Pid_velocity_yaw_t.ki =2;
	Pid_velocity_yaw_t.kd = 0;
	Pid_velocity_yaw_t.max_i = 100;
	Pid_velocity_yaw_t.max_pid = 150;
	Pid_velocity_yaw_t.f_cut_D =0;

	const float ch1_scale = 0.3f;
	const float ch2_scale = 0.3f;
	const float ch4_scale = 10;
    
	const uint16_t dttime = 2000;
#if 0
	flow_x_axis.kp =7.0f;
	flow_x_axis.ki =0;
	flow_x_axis.kd =0;
	flow_x_axis.max_i = 3.0f;
	flow_x_axis.max_pid =40.0f;
	flow_x_axis.f_cut_D = 100;

	flow_y_axis.kp=flow_x_axis.kp;
	flow_y_axis.ki=flow_x_axis.ki;
	flow_y_axis.kd=flow_x_axis.kd;
	flow_y_axis.max_i = flow_x_axis.max_i;
	flow_y_axis.max_pid = flow_x_axis.max_pid;
	flow_y_axis.f_cut_D =flow_x_axis.f_cut_D;
#endif
   	Pid_velocity_roll_t.kp = Pid_velocity_pitch_t.kp;
	Pid_velocity_roll_t.ki = Pid_velocity_pitch_t.ki;
	Pid_velocity_roll_t.kd = Pid_velocity_pitch_t.kd;
	Pid_velocity_roll_t.max_i = Pid_velocity_pitch_t.max_i;
	Pid_velocity_roll_t.max_pid = Pid_velocity_pitch_t.max_pid;
	Pid_velocity_roll_t.f_cut_D =Pid_velocity_pitch_t.f_cut_D;

	Pid_angle_roll_t.kp = Pid_angle_pitch_t.kp;
	Pid_angle_roll_t.ki = Pid_angle_pitch_t.ki;
	Pid_angle_roll_t.kd = Pid_angle_pitch_t.kd;
	Pid_angle_roll_t.max_i = Pid_angle_pitch_t.max_i;
	Pid_angle_roll_t.max_pid = Pid_angle_pitch_t.max_pid;
	Pid_angle_roll_t.f_cut_D =Pid_angle_pitch_t.f_cut_D;

/*-----------------INIT MODULES----------------------*/
	initPWM(&htim4);
	motoIdle();
	initTimeloop(&htim3);
	ibusInit(&huart1,115200);
	MPU_i2c_init(&hi2c2);
	qmc5883_init(&hi2c2);

	/*--GPS--*/
	//gpsInit(&huart3,57600);

	//bmp280 sensor
	//bmp280_init_default_params(&bmp280.params);
    //bmp280.addr = BMP280_I2C_ADDRESS_0;
    //bmp280.i2c = &hi2c1;
	//bmp280_init(&bmp280,&bmp280.params);

	/*flow*/
	//flowInit(&huart2,115200);
/*--------------------------------------------*/
while(1){
	//static uint8_t quality_;
	static float k_ =1.0f;
    static int start_ = 0;
    static float rx_yaw;

    start_ = 1;
    if(ibusFrameComplete()){
		rx_ch1=ibusReadf(CH1,ch1_scale);
		rx_ch2=ibusReadf(CH2,ch2_scale);
		throttle=ibusReadRawRC(CH3);
		ch5=ibusReadRawRC(CH5);
		rx_ch4=ibusReadf(CH4,ch4_scale);
		/*
		if(!start_){
			if(ibusReadRawRC(CH3)<1100 &&  ibusReadRawRC(CH4)<1100 &&
			   ibusReadRawRC(CH2)>1900 &&  ibusReadRawRC(CH1)<1100){
               start_ =1;
		    }
        }
        */
    }
    rx_yaw += rx_ch4*(float)(1e-06f)*50;
	imu_update(&mpu,dttime);
	gyro_yaw = gyro_yaw - mpu.yaw_velocity*dttime*(float)(1e-06f);
	if(qmc_get_Heading(&qmc_yaw,mpu.pitch,mpu.roll)){
		gyro_yaw = gyro_yaw + k_*(qmc_yaw - gyro_yaw);
		if(k_>0.02f){
			k_ = k_ - 0.01f;
			rx_yaw = gyro_yaw;
		}
	}
	/*optical flow  25hz*/
#if 0
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
#endif  
	   /*Pid */
		if(start_ && throttle>MINIMUN_THROTTLE){
			//angle
			pidCalculate(&Pid_angle_pitch_t,mpu.pitch,rx_ch2,2000);//rx_ch2
			pidCalculate(&Pid_angle_roll_t ,mpu.roll,rx_ch1,2000);//rx_ch1
			pidCalculate(&Pid_angle_yaw_t,gyro_yaw,rx_yaw,2000);
			//velocity
			pidCalculate(&Pid_velocity_pitch_t,mpu.pitch_velocity,Pid_angle_pitch_t.PID,2000);
			pidCalculate(&Pid_velocity_roll_t,mpu.roll_velocity,-Pid_angle_roll_t.PID,2000);
			pidCalculate(&Pid_velocity_yaw_t ,mpu.yaw_velocity,Pid_angle_yaw_t.PID,2000);

			moto1 = throttle + Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
			moto2 = throttle + Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;
			moto3 = throttle - Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
			moto4 = throttle - Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;

			writePwm(ch1,moto1);
			writePwm(ch2,moto2);
			writePwm(ch3,moto3);
			writePwm(ch4,moto4);
		}
		else{
			Pid_angle_pitch_t.I=0.0f;
			Pid_angle_roll_t.I =0.0f;
			Pid_angle_yaw_t.I  =0.0f;

			Pid_velocity_pitch_t.I=0.0f;
			Pid_velocity_roll_t.I =0.0f;
			Pid_velocity_yaw_t.I  =0.0f;

			rx_yaw = gyro_yaw;
			IMUresetVector();
			motoIdle();
		   }

/*-------------------------------------------------------------------------------------*/
	 /*send attitude to pc*/
	 static mavlink_message_t msg;
	 FEQUENCY_DIV(25,ENABLE){
		 //60 us
		 static uint8_t buffer[50];
		 uint16_t mil = millis();
         mavlink_msg_attitude_pack(0,0,&msg,mil,mpu.roll,mpu.pitch,mpu.yaw,
        		          mpu.roll_velocity,mpu.pitch_velocity,mpu.yaw_velocity);
         int len = mavlink_msg_to_send_buffer(buffer,&msg);
         HAL_UART_Transmit_DMA(&huart2,buffer,len);
    }
     /*blink led pb3-pb4 */
	 FEQUENCY_DIV(250,ENABLE){
     	 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
     }
	 /*quality check*/
	 /*
	FEQUENCY_DIV(100,ENABLE){
		 if(quality_<100)
			  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		 else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,0);
    }*/
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


