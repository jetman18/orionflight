#include "pid.h"
#include "lpf.h"
#include "maths.h"
#include "timeclock.h"

#define APPLY_LPF
#define NS_TO_SEC(x)  (float)((x)*(float)(1e-07f))
#define MAX_I   300
#define PID_MAX_VAL 400.0f

void pidCalculate(pid__t *gain,float sensor,float control,uint16_t delta_time,uint16_t f_cut){
	float error,P,D;
    float RC = 1.0f / (2 *M_PIf *(float)f_cut);
	float gain_lpf = delta_time*(float)(1e-07f) / (RC + delta_time*(float)(1e-07f));

    error =  sensor - control;
    P  =  error*gain->kp;

    gain->I+=  error*gain->ki*NS_TO_SEC(delta_time);
    gain->I = constrainf(gain->I,-MAX_I,MAX_I);

    D  = (sensor - gain->pre_value)*gain->kd/NS_TO_SEC(delta_time);
    gain->D_smooth = gain->D_smooth* (1-gain_lpf) + gain_lpf*D;

    gain->pre_value = sensor;
    gain->PID = (P + gain->I + gain->D_smooth);
    gain->PID = constrainf(gain->PID,-PID_MAX_VAL,PID_MAX_VAL);
}


