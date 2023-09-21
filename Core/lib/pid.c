#include "pid.h"
#include "filter.h"
#include "maths.h"
#include "timer.h"

#define TOSEC    (1e-06f)
void pidCalculate(pid__t *pid_temp,float sensor,float control,uint32_t Dt_time)
{
	float error,P,D;
    float RC = 1.0f / (2 *M_PIf *pid_temp->f_cut_D);
	float gain_lpf = Dt_time*(1e-06f)/(RC + Dt_time*(1e-06f));
	                              
    error =  sensor - control;
    pid_temp->e = error;
    P  =  error*pid_temp->kp;
	P =  constrainf(P,-pid_temp->max_P,pid_temp->max_P);
    
	error  = fapplyDeadband(error,pid_temp->I_deadband);
    pid_temp->I   += error*pid_temp->ki*Dt_time*TOSEC;
    pid_temp->I   = constrainf(pid_temp->I,-pid_temp->max_I,pid_temp->max_I);

    D  = (sensor - pid_temp->pre_value)*pid_temp->kd/(Dt_time*TOSEC);
	D =  constrainf(D,-pid_temp->max_D,pid_temp->max_D);

	//Noise filtering for D-term -> low pass filter
    pid_temp->D_smooth = pid_temp->D_smooth* (1-gain_lpf) + gain_lpf*D;

    pid_temp->PID = (P + pid_temp->I + pid_temp->D_smooth);
    pid_temp->PID = constrainf(pid_temp->PID,-pid_temp->max_pid,pid_temp->max_pid);
    pid_temp->pre_value = sensor;
}
void resetPID(pid__t *t)
{
	t->I = 0.0f;
	t->pre_value = 0.0f;
	t->PID = 0.0f;
}
