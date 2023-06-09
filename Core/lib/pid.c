#include "pid.h"
#include "filter.h"
#include "maths.h"
#include "scheduler.h"
#include "imu.h"
#include "ibus.h"
#include "qmc5883.h"
#include "hc_sr04.h"
#include "pwmwrite.h"
#define US2SEC(x)  (float)((x)*(1e-06f))
#define MINIMUN_THROTTLE 1000U
#define MINIMUN_THROTTLE_SET 1100U
#define MOTO_CUTOFF_FEQ 40.0f
//#define ACCRO

pid__t Pid_angle_pitch_t,Pid_angle_roll_t,Pid_angle_yaw_t;
pid__t Pid_velocity_pitch_t,Pid_velocity_roll_t, Pid_velocity_yaw_t;

uint16_t moto1,moto2,moto3,moto4;
void PID_init_param(){
	//------angle------------
	Pid_angle_pitch_t.kp =7;
	Pid_angle_pitch_t.ki =0;
	Pid_angle_pitch_t.kd =0;
	Pid_angle_pitch_t.max_P = 300;
	Pid_angle_pitch_t.max_I = 0;  
	Pid_angle_pitch_t.max_D = 0;    
	Pid_angle_pitch_t.max_pid =300;// = setRate_limit 
	Pid_angle_pitch_t.f_cut_D =50;

	Pid_angle_yaw_t.kp =7;
	Pid_angle_yaw_t.ki =0;
	Pid_angle_yaw_t.kd = 0;
	Pid_angle_yaw_t.max_P = 300;
	Pid_angle_yaw_t.max_I = 0;  
	Pid_angle_yaw_t.max_D = 0; 
	Pid_angle_yaw_t.max_pid =200;
	Pid_angle_yaw_t.f_cut_D =0;

	//------velocity----
	Pid_velocity_pitch_t.kp =1.6;
	Pid_velocity_pitch_t.ki =1.6;
	Pid_velocity_pitch_t.kd =0.04;//0.12
	Pid_velocity_pitch_t.max_P = 400;
	Pid_velocity_pitch_t.max_I = 400;
	Pid_velocity_pitch_t.max_D = 200;
	Pid_velocity_pitch_t.max_pid = 300;
	Pid_velocity_pitch_t.f_cut_D =30;

	Pid_velocity_yaw_t.kp =20;
	Pid_velocity_yaw_t.ki =0;
	Pid_velocity_yaw_t.kd =0;
	Pid_velocity_yaw_t.max_P = 400;
	Pid_velocity_yaw_t.max_I = 400;
	Pid_velocity_yaw_t.max_D = 200;
	Pid_velocity_yaw_t.max_pid = 300;
	Pid_velocity_yaw_t.f_cut_D =0;

	Pid_velocity_roll_t.kp = Pid_velocity_pitch_t.kp;
	Pid_velocity_roll_t.ki = Pid_velocity_pitch_t.ki;
	Pid_velocity_roll_t.kd = Pid_velocity_pitch_t.kd;
	Pid_velocity_roll_t.max_pid = Pid_velocity_pitch_t.max_pid;
	Pid_velocity_roll_t.f_cut_D =Pid_velocity_pitch_t.f_cut_D;
    Pid_velocity_roll_t.max_P = Pid_velocity_pitch_t.max_P;
	Pid_velocity_roll_t.max_I = Pid_velocity_pitch_t.max_I;
	Pid_velocity_roll_t.max_D = Pid_velocity_pitch_t.max_D; 

	Pid_angle_roll_t.kp =    Pid_angle_pitch_t.kp;
	Pid_angle_roll_t.ki =    Pid_angle_pitch_t.ki;
	Pid_angle_roll_t.kd =    Pid_angle_pitch_t.kd;
	Pid_angle_roll_t.max_P = Pid_angle_pitch_t.max_P;
	Pid_angle_roll_t.max_I = Pid_angle_pitch_t.max_I;
	Pid_angle_roll_t.max_D = Pid_angle_pitch_t.max_D;
	Pid_angle_roll_t.max_pid = Pid_angle_pitch_t.max_pid;
	Pid_angle_roll_t.f_cut_D =Pid_angle_pitch_t.f_cut_D;
}

void pidCalculate(pid__t *pid_temp,float sensor,float control,uint32_t Dt_time)
{
	float error,P,D;
    const float RC = 1.0f / (2 *M_PIf *pid_temp->f_cut_D);
	float gain_lpf = Dt_time*(1e-06f)/(RC + Dt_time*(1e-06f));
	                              
    error =  sensor - control;
    pid_temp->e = error;
    P  =  error*pid_temp->kp;
	P =  constrainf(P,-pid_temp->max_P,pid_temp->max_P);
    
    pid_temp->I   += error*pid_temp->ki*US2SEC(Dt_time);
    pid_temp->I   = constrainf(pid_temp->I,-pid_temp->max_I,pid_temp->max_I);

    D  = (sensor - pid_temp->pre_value)*pid_temp->kd/US2SEC(Dt_time);
	D =  constrainf(D,-pid_temp->max_D,pid_temp->max_D);

	//Noise filtering for D-term -> low pass filter
    pid_temp->D_smooth = pid_temp->D_smooth* (1-gain_lpf) + gain_lpf*D;

    pid_temp->PID = (P + pid_temp->I + pid_temp->D_smooth);
    pid_temp->PID = constrainf(pid_temp->PID,-pid_temp->max_pid,pid_temp->max_pid);
    pid_temp->pre_value = sensor;
}
void resetPID(pid__t *t)
{
	t->I =0.0f;
	t->pre_value=0.0f;
}
void pidUpdate(){
	if(throttle>1020){
	//if(altitude_stick>1700){
		static uint16_t m1,m2,m3,m4;

#ifdef ACCRO
		pidCalculate(&Pid_velocity_pitch_t,quad_.pitch_velocity,-rx_ch2,config.dt);//Pid_angle_pitch_t.PID
		pidCalculate(&Pid_velocity_roll_t,quad_.roll_velocity,rx_ch1,config.dt);//-Pid_angle_roll_t.PID
		pidCalculate(&Pid_velocity_yaw_t ,-quad_.yaw_velocity,-rx_ch4,config.dt);//Pid_angle_yaw_t.PID
#else
		//angles
		pidCalculate(&Pid_angle_pitch_t,quad_.pitch,rx_ch2,config.dt);//rx_ch2
		pidCalculate(&Pid_angle_roll_t ,quad_.roll + 1.0f,rx_ch1,config.dt);//rx_ch1
		//pidCalculate(&Pid_angle_yaw_t,gyro_yaw,rx_yaw,config.dt);
		
		Pid_angle_pitch_t.PID = constrainf(Pid_angle_pitch_t.PID,-300.0f,300.0f);
		Pid_angle_roll_t.PID  = constrainf(Pid_angle_roll_t.PID,-300.0f,300.0f);
		Pid_angle_yaw_t.PID   = constrainf(Pid_angle_yaw_t.PID,-300.0f,300.0f);
		 //velocity
		pidCalculate(&Pid_velocity_pitch_t,quad_.pitch_velocity,Pid_angle_pitch_t.PID,config.dt);//Pid_angle_pitch_t.PID
		pidCalculate(&Pid_velocity_roll_t,quad_.roll_velocity,-Pid_angle_roll_t.PID,config.dt);//-Pid_angle_roll_t.PID
		pidCalculate(&Pid_velocity_yaw_t ,-quad_.yaw_velocity,-rx_ch4,config.dt);//Pid_angle_yaw_t.PID
#endif

		//m1 =throttle  + Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
		//m2 =throttle  + Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;
		//m3 =throttle  - Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
		//m4 =throttle  - Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;

		m1 =hc04_Throttle + Pid_altitude_t.PID + Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
		m2 =hc04_Throttle + Pid_altitude_t.PID + Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;
		m3 =hc04_Throttle + Pid_altitude_t.PID - Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
		m4 =hc04_Throttle + Pid_altitude_t.PID - Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;

		//moto smooth
	    float RC = 1.0f/(2*M_PIf*MOTO_CUTOFF_FEQ);
		float gain_lpf = config.dt*(1e-06f)/(RC + config.dt*(1e-06f));
        moto1 = moto1 + gain_lpf*(m1 - moto1);
        moto2 = moto2 + gain_lpf*(m2 - moto2);
        moto3 = moto3 + gain_lpf*(m3 - moto3);
        moto4 = moto4 + gain_lpf*(m4 - moto4);

		if(moto1<MINIMUN_THROTTLE_SET)moto1=throttle;
		if(moto2<MINIMUN_THROTTLE_SET)moto2=throttle;
		if(moto3<MINIMUN_THROTTLE_SET)moto3=throttle;
		if(moto4<MINIMUN_THROTTLE_SET)moto4=throttle;
	}
	else{
		// reset PID
		resetPID(&Pid_angle_pitch_t);
        resetPID(&Pid_angle_roll_t);
		resetPID(&Pid_angle_yaw_t);

	    resetPID(&Pid_velocity_pitch_t);
		resetPID(&Pid_velocity_roll_t);
		resetPID(&Pid_velocity_yaw_t);

		moto1 = MINIMUN_THROTTLE;
		moto2 = MINIMUN_THROTTLE;
		moto3 = MINIMUN_THROTTLE;
		moto4 = MINIMUN_THROTTLE;

		gyro_yaw = heading;
		//rx_yaw = gyro_yaw;
	   }
	}


