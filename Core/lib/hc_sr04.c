#include "hc_sr04.h"
#include "scheduler.h"
#include "imu.h"
#include "maths.h"
#include "ibus.h"
#include "pid.h"
#include "math.h"
#define THOROTTLE_TAKE_OFF 1500
#define MINIMUN_ANGLE 10

static int pin=1;
static uint16_t pulse;
static uint32_t time_;
int hc04_Throttle;
const uint16_t ofset_alti = 30;  //mm
static int dis;
pid__t Pid_altitude_t;
int isHcNewdata(){
	return (!pin);
}

int hc_sr04_get_dis(){
	return (pulse*(float)(1e-06f)*170000); //mm
}

void hc_sr04_send_trige(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,1);
    delay_us(11);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,0);
}
void hc_sr04_start(){
	//-
	hc04_Throttle=1000;
	Pid_altitude_t.kp =0.5;
	Pid_altitude_t.ki =0.07;
	Pid_altitude_t.kd =0.3;   //0.3
	Pid_altitude_t.max_P = 400;
	Pid_altitude_t.max_I = 400;
	Pid_altitude_t.max_D = 300;
	Pid_altitude_t.max_pid =500;
	Pid_altitude_t.f_cut_D =10;
};
static float dis_lpf;
#define MOTO_CUTOFF_FEQ 5  //hz
void hc_sr04_run(){
	static float last_dis = 0;
	dis = hc_sr04_get_dis();
	dis = val_change_limiter(last_dis,dis,-20,20);

    float RC = 1.0f/(2*M_PIf*MOTO_CUTOFF_FEQ);
	float gain_lpf = 0.04f/(RC + 0.04f);

	dis_lpf = dis_lpf + gain_lpf*(dis - dis_lpf);
	last_dis = dis;
	//Pid_altitude_t.kp =vr_2; //0-1
	//Pid_altitude_t.ki =vr_1;  //0-1

	//dis = dis*cos_approx(DEGREES_TO_RADIANS(quad_.pitch))*cos_approx(DEGREES_TO_RADIANS(quad_.roll));
	if((fabs(quad_.pitch) < MINIMUN_ANGLE ) && (fabs(quad_.roll) < MINIMUN_ANGLE )){
		if(altitude_stick>1700 && throttle >1020){
			hc04_Throttle = hc04_Throttle + (THOROTTLE_TAKE_OFF-1000)*0.04f;
			hc04_Throttle = constrain(hc04_Throttle,1000,THOROTTLE_TAKE_OFF);
			pidCalculate(&Pid_altitude_t,-dis + ofset_alti,-ch3_,40000);
		}
	    if(throttle<1020){
			resetPID(&Pid_altitude_t);
			hc04_Throttle = hc04_Throttle - (THOROTTLE_TAKE_OFF-1000)*0.04f;
			hc04_Throttle = constrain(hc04_Throttle,1000,THOROTTLE_TAKE_OFF);
			resetPID(&Pid_altitude_t);
		}
    }
	hc_sr04_send_trige();
}

// callback
void hc_sr04_callback(){
	 if( pin==0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)){
	       time_=micros();
	       pin=1;
	   }
	 else if( pin==1 && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)){
	       pulse=micros()-time_;
	       pin=0;
	   }
}
