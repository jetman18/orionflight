#include "pid.h"
#include "filter.h"
#include "maths.h"
#include "timer.h"
#include "ahrs.h"
#include "imu.h"
#include "pwmwrite.h"
#include "string.h"
#include "pid.h"
#include "ibus.h"
#define MINIMUN_THROTTLE 1000U
#define MAXIMUN_THROTTLE 2000U
#define MINIMUN_THROTTLE_SET 1100U
#define PWM_CUTOFF_FEQ  70.0f
//#define ACCRO
pid__t pid_rate_pitch,pid_rate_roll, pid_rate_yaw;
float pitch_kp,roll_kp,yaw_kp;

uint16_t moto1,moto2,moto3,moto4;
void PID_init_parameters(){
    pitch_kp = 2.5f;
    roll_kp  = 2.5f;
    yaw_kp   = 3.0f;

	pid_rate_pitch.kp =3.3;
	pid_rate_pitch.ki =3;
	pid_rate_pitch.kd =0.03,
	pid_rate_pitch.max_P = 400;
	pid_rate_pitch.max_I = 300;
	pid_rate_pitch.max_D = 200;
	pid_rate_pitch.max_pid = 500;
	pid_rate_pitch.I_deadband = 3.0f;
	pid_rate_pitch.f_cut_D =10;

	memcpy(&pid_rate_roll,&pid_rate_pitch,sizeof(pid__t));

	pid_rate_yaw.kp =15;
	pid_rate_yaw.ki =0;
	pid_rate_yaw.kd =0;
	pid_rate_yaw.max_P = 400;
	pid_rate_yaw.max_I = 400;
	pid_rate_yaw.max_D = 200;
	pid_rate_yaw.max_pid = 300;
	pid_rate_yaw.I_deadband = 3.0f;
	pid_rate_yaw.f_cut_D =0;
}
void pidUpdate(){
static float roll_setpoint;
static float  pitch_setpoint;
static int pwm1,pwm2,pwm3,pwm4;
	if(throttle>1020){
        // pitch PPID
		float error_rate_pitch;
		pitch_setpoint = constrainf(rx_ch2,-45,45);
		pitch_setpoint = fapplyDeadband(pitch_setpoint,2);
		//pitch_setpoint =  pitch_setpoint - y_flow_t.PID;
		error_rate_pitch = (AHRS.pitch - pitch_setpoint)*pitch_kp;
		error_rate_pitch = constrainf(error_rate_pitch,-300.0f,300.0f);
		pidCalculate(&pid_rate_pitch,-AHRS.pitch_velocity,error_rate_pitch,config.dt);
        //roll 
		float error_rate_roll;
		roll_setpoint = constrainf(rx_ch1,-45,45);
		roll_setpoint = fapplyDeadband(roll_setpoint,2);
		//roll_setpoint =  roll_setpoint - x_flow_t.PID;
		error_rate_roll = (AHRS.roll - roll_setpoint)*roll_kp;
		error_rate_roll = constrainf(-error_rate_roll,-300.0f,300.0f);
		pidCalculate(&pid_rate_roll,-AHRS.roll_velocity,error_rate_roll,config.dt);
        //yaw
		float yaw_setpoint;
		float error_rate_yaw;
		yaw_setpoint = fapplyDeadband(rx_ch4,2);
		//yaw_setpoint = constrainf(yaw_setpoint,-45,45);
		//error_rate_yaw = (AHRS.yaw -  yaw_setpoint)* yaw_kp;
		//error_rate_yaw = constrainf(error_rate_yaw,-300.0f,300.0f);
		pidCalculate(&pid_rate_yaw,-AHRS.yaw_velocity,yaw_setpoint,config.dt);

        if(altitude_stick<1700){
		    pwm1 = (int)throttle  + (int)pid_rate_pitch.PID - (int)pid_rate_roll.PID - (int)pid_rate_yaw.PID;
		    pwm2 = (int)throttle  + (int)pid_rate_pitch.PID + (int)pid_rate_roll.PID + (int)pid_rate_yaw.PID;
		    pwm3 = (int)throttle  - (int)pid_rate_pitch.PID + (int)pid_rate_roll.PID - (int)pid_rate_yaw.PID;
		    pwm4 = (int)throttle  - (int)pid_rate_pitch.PID - (int)pid_rate_roll.PID + (int)pid_rate_yaw.PID;
        }
        /*
        else{
			pwm1 = hc04_Throttle + (int)Pid_altitude_t.PID + (int)pid_rate_pitch.PID - (int)pid_rate_roll.PID - (int)pid_rate_yaw.PID;
			pwm2 = hc04_Throttle + (int)Pid_altitude_t.PID + (int)pid_rate_pitch.PID + (int)pid_rate_roll.PID + (int)pid_rate_yaw.PID;
			pwm3 = hc04_Throttle + (int)Pid_altitude_t.PID - (int)pid_rate_pitch.PID + (int)pid_rate_roll.PID - (int)pid_rate_yaw.PID;
			pwm4 = hc04_Throttle + (int)Pid_altitude_t.PID - (int)pid_rate_pitch.PID - (int)pid_rate_roll.PID + (int)pid_rate_yaw.PID;
        }*/
        //float coss = cos_approx(AHRS.pitch*RAD)*cos_approx(AHRS.roll*RAD);
       // coss = constrainf(coss,0.85f,1.0f);

        //pwm1 = pwm1/coss;
        //pwm2 = pwm2/coss;
        //pwm3 = pwm3/coss;
        //pwm4 = pwm4/coss;

    	pwm1 = constrain(pwm1,MINIMUN_THROTTLE,MAXIMUN_THROTTLE);
    	pwm2 = constrain(pwm2,MINIMUN_THROTTLE,MAXIMUN_THROTTLE);
    	pwm3 = constrain(pwm3,MINIMUN_THROTTLE,MAXIMUN_THROTTLE);
    	pwm4 = constrain(pwm4,MINIMUN_THROTTLE,MAXIMUN_THROTTLE);

		//moto smooth
	    float RC = 1.0f/(2*M_PIf*PWM_CUTOFF_FEQ );
		float gain_lpf = config.dt*(1e-06f)/(RC + config.dt*(1e-06f));
        moto1 = moto1 + gain_lpf*(pwm1 - moto1);
        moto2 = moto2 + gain_lpf*(pwm2 - moto2);
        moto3 = moto3 + gain_lpf*(pwm3 - moto3);
        moto4 = moto4 + gain_lpf*(pwm4 - moto4);

		if(moto1<MINIMUN_THROTTLE_SET)moto1=throttle;
		if(moto2<MINIMUN_THROTTLE_SET)moto2=throttle;
		if(moto3<MINIMUN_THROTTLE_SET)moto3=throttle;
		if(moto4<MINIMUN_THROTTLE_SET)moto4=throttle;
	}
	else{
		// reset PID
	    resetPID(&pid_rate_pitch);
		resetPID(&pid_rate_roll);
		resetPID(&pid_rate_yaw);

		moto1 = MINIMUN_THROTTLE;
		moto2 = MINIMUN_THROTTLE;
		moto3 = MINIMUN_THROTTLE;
		moto4 = MINIMUN_THROTTLE;
	   }
	   writePwm(ch1,moto1);
	   writePwm(ch2,moto2);
	   writePwm(ch3,moto3);
	   writePwm(ch4,moto4);
	}


