#include "MAVLink/common/mavlink.h"
#include <log.h>
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
#include <string.h>
#include "gps.h"
#include "distance.h"
#include "./ssd1306/ssd1306.h"
#include "opticalflow.h"
#include "../quadrotor/config.h"
#include "mavlink_handler.h"
#define BIAS_ALTITUDE 80
#define GYRO_DELAY 4
#define MINIMUN_THROTTLE 1030
#define SYS_ID 1
static float  rx_ch1,rx_ch2,rx_ch4;
static uint16_t throttle,ch5,ch3_;
pid__t Pid_angle_pitch_t,Pid_angle_roll_t,Pid_angle_yaw_t,flow_x_axis,flow_y_axis;
pid__t Pid_velocity_pitch_t,Pid_velocity_roll_t, Pid_velocity_yaw_t;
pid__t Pid_altitude_t;
static uint16_t moto1,moto2,moto3,moto4;
static attitude_t quad_;
float buffe1[GYRO_DELAY];
float buffe2[GYRO_DELAY];
float storeGyro1(float data);
float storeGyro2(float data);
static float gyro_yaw,qmc_yaw;
static void rotate(faxis3_t *vector,faxis3_t delta);
//BMP280_HandleTypedef bmp280;
//static float temperature,pressure,altitude;
/************************************test var***************/
int accx,accy,accz;
uint32_t mil;
axis3_t t;
float dis;
//static int count;
static float init_th=1000;
float altitude_rx;
int set_altitude;
uint16_t t1;
void main_loop(){
/*----------PARAMETER-init----------------------*/
	/*------------altitude PID --------*/
	Pid_altitude_t.kp =0.2f;   //0.5
	Pid_altitude_t.ki =0.12f;   //0.2
	Pid_altitude_t.kd =0.2f;  //0.55
	Pid_altitude_t.max_i = 300.0f;   //
	Pid_altitude_t.max_pid =300.0f;  //
	Pid_altitude_t.f_cut_D =30.0f;

    /*-----------angle PID------------*/
	Pid_angle_pitch_t.kp =1.2f;
	Pid_angle_pitch_t.ki =0;
	Pid_angle_pitch_t.kd =0.01;
	Pid_angle_pitch_t.max_i = 0;   //
	Pid_angle_pitch_t.max_pid =50;  //
	Pid_angle_pitch_t.f_cut_D =80;

	Pid_angle_yaw_t.kp =2;
	Pid_angle_yaw_t.ki =0;//1000
	Pid_angle_yaw_t.kd = 0;
	Pid_angle_yaw_t.max_i = 0;
	Pid_angle_yaw_t.max_pid = 120;
	Pid_angle_yaw_t.f_cut_D =0;

    /*--angle velocity PID--*/
   	Pid_velocity_pitch_t.kp =2.5;
	Pid_velocity_pitch_t.ki =1.6;
	Pid_velocity_pitch_t.kd = 0;
	Pid_velocity_pitch_t.max_i = 200;
	Pid_velocity_pitch_t.max_pid = 400;
	Pid_velocity_pitch_t.f_cut_D =100;

	Pid_velocity_yaw_t.kp =3;
	Pid_velocity_yaw_t.ki =2;
	Pid_velocity_yaw_t.kd =0;
	Pid_velocity_yaw_t.max_i = 100;
	Pid_velocity_yaw_t.max_pid = 150;
	Pid_velocity_yaw_t.f_cut_D =0;

	const float ch1_scale = 0.3f;
	const float ch2_scale = 0.3f;
	//const float ch3_scale = 0.23f;  // -> 200mm/s
	const float ch4_scale = 0.6f;
    
	const uint16_t dt = 2000;
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
	mavlinkInit(SYS_ID,0,&huart2,115200);
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
    static float rx_yaw;
/*RX data receive-----*/
    if(ibusFrameComplete()){
		rx_ch1=ibusReadf(CH1,ch1_scale);
		rx_ch2=ibusReadf(CH2,ch2_scale);
		//rx_ch3=ibusReadf(CH3,ch3_scale);
		throttle=ibusReadRawRC(CH3);
		ch3_ = throttle;
		ch3_ -=1000;// 1000-200 -> 0-1000
		ch3_ *=1.4;   // altitude  1 - 2 meter
		ch5=ibusReadRawRC(CH5);
		rx_ch4=ibusReadf(CH4,ch4_scale);
     }
/*IMU calc--*/
	imu_update(&quad_,dt);
/*HEADING ---*/
	float temp = (float)(1e-06f)*dt;
	rx_yaw = rx_yaw + rx_ch4*temp;
	gyro_yaw = gyro_yaw - quad_.yaw_velocity*temp;
	if(qmc_get_Heading(&qmc_yaw,quad_.pitch,quad_.roll)){
	 	gyro_yaw = gyro_yaw + k_*(qmc_yaw - gyro_yaw);
		if(k_>0.1)k_ = k_ - 0.01f;
	}
/*HC-SR04  altitude hoding*/
FEQUENCY_DIV(12,1){
	hc_sr04_send_trige();
	static float last_dis = BIAS_ALTITUDE;
	float ddis = hc_sr04_get_dis();
	ddis = p_constrainf(last_dis,ddis,-10,10);
	last_dis = ddis;
	dis = dis + 0.9f*(ddis - dis);
	dis = dis*cos_approx(DEGREES_TO_RADIANS(quad_.pitch))*cos_approx(DEGREES_TO_RADIANS(quad_.roll));
	//if((quad_.pitch < fabs(10)) && (quad_.roll < fabs(10))){
		if(ch5>1600){
			init_th +=100*0.024f; //0.024 (s)
			init_th = constrainf(init_th,0,1200);
			pidCalculate(&Pid_altitude_t,dis,ch3_ + BIAS_ALTITUDE,24000);
			Pid_altitude_t.PID *=-1;
			t1 =init_th + Pid_altitude_t.PID;
		}else{
			t1 = throttle;
			init_th=1000;
			Pid_altitude_t.I=0;
			Pid_altitude_t.PID=0;
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
if(throttle>MINIMUN_THROTTLE){
	//angle pid
	pidCalculate(&Pid_angle_pitch_t,quad_.pitch,rx_ch2,dt);//rx_ch2
	pidCalculate(&Pid_angle_roll_t ,quad_.roll,rx_ch1,dt);//rx_ch1
	pidCalculate(&Pid_angle_yaw_t,gyro_yaw,rx_yaw,dt);
	//velocity pid
	pidCalculate(&Pid_velocity_pitch_t,quad_.pitch_velocity,Pid_angle_pitch_t.PID,dt);
	pidCalculate(&Pid_velocity_roll_t,quad_.roll_velocity,-Pid_angle_roll_t.PID,dt);
	pidCalculate(&Pid_velocity_yaw_t ,quad_.yaw_velocity,Pid_angle_yaw_t.PID,dt);

	moto1 = t1 + Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
	moto2 = t1 + Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;
	moto3 = t1 - Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
	moto4 = t1 - Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;

	writePwm(ch1,moto1);
	writePwm(ch2,moto2);
	writePwm(ch3,moto3);
	writePwm(ch4,moto4);
}else{
	Pid_angle_pitch_t.I=0.0f;
	Pid_angle_roll_t.I =0.0f;
	Pid_angle_yaw_t.I  =0.0f;

	Pid_velocity_pitch_t.I=0.0f;
	Pid_velocity_roll_t.I =0.0f;
	Pid_velocity_yaw_t.I  =0.0f;

	rx_yaw = gyro_yaw;
	writePwm(ch1,init_th);
	writePwm(ch2,init_th);
	writePwm(ch3,init_th);
	writePwm(ch4,init_th);
	//motoIdle();
	}
  // motoIdle();

/*send attitude to pc*/
FEQUENCY_DIV(100,ENABLE)
{
		  set_altitude = named_value_int_().value;
		  mav_pack_attitude(quad_.roll,quad_.pitch,gyro_yaw,0,0,0);
          mav_pack_named_value_int("throttle",throttle);
          mavlink_send();
    }
    /*blink led pb3-pb4 */
	FEQUENCY_DIV(250,ENABLE){
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
}
	 /*quality check*/
//FEQUENCY_DIV(100,ENABLE){
		 //if(quality_<100)
		//	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		// else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,0);
//}
    looptime(dt);
   }
}
// IQR function
/*----------------------------------IQR--Handle-----------------------------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* ibus */
    if(huart == &huart1)
	{
        ibusCallback();
    }                                                                                                                           //
    else if(huart == &huart2){
      	mavlinkCallback();
     }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// ibus
    if(huart == &huart2)
	{
    	mav_tx_cpl_callback();
    }                                                                                                                          //
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_8)
    {
     hc_sr04_callback();
    }
}

//
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


void rotate(faxis3_t *vector,faxis3_t delta)
{
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;
	faxis3_t vec;

	float toRaddt = 0.01745f;
	float angleX = delta.x*toRaddt;
	float angleY = delta.y*toRaddt;
	float angleZ = delta.z*toRaddt;

    cosx = cos_approx(angleX);
    sinx = sin_approx(angleX);
    cosy = cos_approx(angleY);
    siny = sin_approx(angleY);
    cosz = cos_approx(angleZ);
    sinz = sin_approx(angleZ);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    vec.x = vector->x * mat[0][0] + vector->y * mat[1][0] + vector->z * mat[2][0];
    vec.y = vector->x * mat[0][1] + vector->y * mat[1][1] + vector->z * mat[2][1];
    vec.z = vector->x * mat[0][2] + vector->y * mat[1][2] + vector->z * mat[2][2];

    vector->x =  vec.x;
	vector->y =  vec.y;
	vector->z =  vec.z;

}

