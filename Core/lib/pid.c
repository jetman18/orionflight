#include "pid.h"
#include "lpf.h"
#include "maths.h"
#include "timeclock.h"
#include "imu.h"
#include "ibus.h"
#include "qmc5883.h"
#define US2SEC(x)  (float)((x)*(1e-06f))
#define MINIMUN_THROTTLE 1000U
#define MINIMUN_THROTTLE_SET 1050U
extern float  rx_ch1,rx_ch2,rx_ch4;
extern uint16_t throttle,ch5,ch3_;
extern float heading;
extern imu_config_t config;
extern attitude_t quad_;
extern float rx_yaw;
extern float gyro_yaw;
pid__t Pid_angle_pitch_t,Pid_angle_roll_t,Pid_angle_yaw_t,flow_x_axis,flow_y_axis;
pid__t Pid_velocity_pitch_t,Pid_velocity_roll_t, Pid_velocity_yaw_t;
pid__t Pid_altitude_t;
uint16_t moto1,moto2,moto3,moto4;
void PID_init_param(){
	//------------altitude------------
	Pid_altitude_t.kp =1;
	Pid_altitude_t.ki =0.12;//0.12f;
	Pid_altitude_t.kd =0.3f;  //0.55
	Pid_altitude_t.max_i = 70.0f;   //
	Pid_altitude_t.max_pid =300.0f;  //
	Pid_altitude_t.f_cut_D =50.0f;

	//-----------angle PID------------
	Pid_angle_pitch_t.kp =2.0f; //1.2
	Pid_angle_pitch_t.ki =0;
	Pid_angle_pitch_t.kd =0;
	Pid_angle_pitch_t.max_i = 0;   //
	Pid_angle_pitch_t.max_pid =50;  //
	Pid_angle_pitch_t.f_cut_D =80;

	Pid_angle_yaw_t.kp =2;
	Pid_angle_yaw_t.ki =0;//1;
	Pid_angle_yaw_t.kd = 0;
	Pid_angle_yaw_t.max_i = 0;
	Pid_angle_yaw_t.max_pid = 120;
	Pid_angle_yaw_t.f_cut_D =0;

	//--angle velocity PID----
	Pid_velocity_pitch_t.kp =4.5f;
	Pid_velocity_pitch_t.ki =4.6;
	Pid_velocity_pitch_t.kd =0.05;
	Pid_velocity_pitch_t.max_i = 100;
	Pid_velocity_pitch_t.max_pid = 300;
	Pid_velocity_pitch_t.f_cut_D =50;

	Pid_velocity_yaw_t.kp =3;
	Pid_velocity_yaw_t.ki =0;
	Pid_velocity_yaw_t.kd =0;
	Pid_velocity_yaw_t.max_i = 100;
	Pid_velocity_yaw_t.max_pid = 150;
	Pid_velocity_yaw_t.f_cut_D =0;


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

}
static void pidCalculate(pid__t *gain,float sensor,float control,uint32_t delta_time){
	float error,P,D;
    float RC = 1.0f / (2 *M_PIf *gain->f_cut_D);
	float gain_lpf = delta_time*(1e-06f)/(RC + delta_time*(1e-06f));
	                              
    error =  sensor - control;
    P  =  error*gain->kp;
    
    gain->I   += error*gain->ki*US2SEC(delta_time);
    gain->I   = constrainf(gain->I,-gain->max_i,gain->max_i);

    D  = (sensor - gain->pre_value)*gain->kd/US2SEC(delta_time);

    gain->D_smooth = gain->D_smooth* (1-gain_lpf) + gain_lpf*D;
    gain->PID = (P + (gain->I) + gain->D_smooth);

    gain->PID = constrainf(gain->PID,-gain->max_pid,gain->max_pid);
    gain->pre_value = sensor;
}
void pidUpdate(){

	if(throttle>MINIMUN_THROTTLE_SET){
		static uint16_t m1,m2,m3,m4;
		//angle pid
		pidCalculate(&Pid_angle_pitch_t,quad_.pitch,rx_ch2,config.dt);//rx_ch2
		pidCalculate(&Pid_angle_roll_t ,quad_.roll,rx_ch1,config.dt);//rx_ch1
		pidCalculate(&Pid_angle_yaw_t,gyro_yaw,rx_yaw,config.dt);
		//velocity pid
		pidCalculate(&Pid_velocity_pitch_t,quad_.pitch_velocity,Pid_angle_pitch_t.PID,config.dt);
		pidCalculate(&Pid_velocity_roll_t,quad_.roll_velocity,-Pid_angle_roll_t.PID,config.dt);//
		pidCalculate(&Pid_velocity_yaw_t ,quad_.yaw_velocity,Pid_angle_yaw_t.PID,config.dt);

		m1 = throttle + Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
		m2 = throttle + Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;
		m3 = throttle - Pid_velocity_pitch_t.PID + Pid_velocity_roll_t.PID - Pid_velocity_yaw_t.PID;
		m4 = throttle - Pid_velocity_pitch_t.PID - Pid_velocity_roll_t.PID + Pid_velocity_yaw_t.PID;

		static float f_cut = 50;
	    float RC = 1.0f / (2 *M_PIf*f_cut);
		float gain_lpf = config.dt*(1e-06f)/(RC + config.dt*(1e-06f));
        moto1 = moto1 + gain_lpf*(m1 - moto1);
        moto2 = moto2 + gain_lpf*(m2 - moto2);
        moto3 = moto3 + gain_lpf*(m3 - moto3);
        moto4 = moto4 + gain_lpf*(m4 - moto4);

		if(moto1<MINIMUN_THROTTLE_SET)moto1=1000;
		if(moto2<MINIMUN_THROTTLE_SET)moto2=1000;
		if(moto3<MINIMUN_THROTTLE_SET)moto3=1000;
		if(moto4<MINIMUN_THROTTLE_SET)moto4=1000;
	}else{
		Pid_angle_pitch_t.I=0.0f;
		Pid_angle_roll_t.I =0.0f;
		Pid_angle_yaw_t.I  =0.0f;
	
		Pid_velocity_pitch_t.I=0.0f;
		Pid_velocity_roll_t.I =0.0f;
		Pid_velocity_yaw_t.I  =0.0f;

		moto1 = MINIMUN_THROTTLE;
		moto2 = MINIMUN_THROTTLE;
		moto3 = MINIMUN_THROTTLE;
		moto4 = MINIMUN_THROTTLE;

		rx_yaw = gyro_yaw;
	   }
	}



