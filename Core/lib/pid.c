#include "pid.h"
#include "lpf.h"
#include "maths.h"
#include "timeclock.h"

#define APPLY_LPF
#define US2SEC(x)  (float)((x)*(float)(1e-06f))  

void pidCalculate(pid__t *gain,float sensor,float control,uint16_t delta_time){
	float error,P,D;
    float RC = 1.0f / (2 *M_PIf *gain->f_cut_D);
	float gain_lpf = delta_time*(float)(1e-06f) / (RC + delta_time*(float)(1e-06f));
	                              
    error =  sensor - control;
    P  =  error*gain->kp;

    gain->I+=  error*gain->ki*US2SEC(delta_time);
    gain->I = constrainf(gain->I,-gain->max_i,gain->max_i);

    D  = (sensor - gain->pre_value)*gain->kd/US2SEC(delta_time);
#ifdef APPLY_LPF
    gain->D_smooth = gain->D_smooth* (1-gain_lpf) + gain_lpf*D;
    gain->PID = (P + gain->I + gain->D_smooth);
#elif
    gain->PID = (P + gain->I + D);
#endif
    gain->PID = constrainf(gain->PID,-gain->max_pid,gain->max_pid);
    gain->pre_value = sensor;
}


