#include "pid.h"
#include "lpf.h"
#include "maths.h"
#include "timeclock.h"


#define APPLY_LPF
#define NS_TO_SEC(x)  (float)((x)*0.000001f)
#define MAX_I   300
#define FCUT_LPF 100.0f    //HZ
#define PID_MAX_VAL 400.0f

void pidCalculate(pid__t *gain,float sensor,float control,float f_cut){
	float error,P,D;
    float RC = 1.0f / (2 *M_PIf * f_cut);
	float gain_lpf = gain->delta_time*0.000001f / (RC + gain->delta_time*0.000001f);

    gain->delta_time = micros() -  gain->p_time;
    gain->p_time = micros();
    if(gain->delta_time<=0){
        return;
    }

    error =  sensor - control;
    gain->e =  error;
    P  =  error*gain->kp;

    gain->I+=  error*gain->ki*NS_TO_SEC(gain->delta_time);
    gain->I = constrainf(gain->I,-MAX_I,MAX_I);

    D  = (sensor - gain->pre_value)*gain->kd/NS_TO_SEC(gain->delta_time);
    gain->D_smooth = gain->D_smooth* (1-gain_lpf) + gain_lpf*D;

    gain->pre_value = sensor;
    gain->PID = (P + gain->I + gain->D_smooth);
    gain->PID=constrainf(gain->PID,-PID_MAX_VAL,PID_MAX_VAL);
}


