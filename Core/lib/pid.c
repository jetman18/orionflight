#include "pid.h"
#include "lpf.h"
#include "maths.h"
#include "timeclock.h"


#define APPLY_LPF
#define NS_TO_SEC(x)  (float)((x)*0.000001f)
#define MAX_I   100
#define FCUT_LPF 100.0f    //HZ
#define PID_MAX_VAL 300.0f

static float error,P,D;
void pidCalculate(pid_gain_t *gain,float sensor,float control,uint16_t dt){

    error =  sensor - control;
    P  =  error*gain->kp;

    gain->I+=  error*gain->ki*NS_TO_SEC(dt);
    if(gain->I > MAX_I) gain->I= MAX_I;
    else if(gain->I< -MAX_I)gain->I=-MAX_I;

    D  = (sensor - gain->pre_value)*gain->kd/NS_TO_SEC(dt);
#ifdef APPLY_LPF
    D = pt1FilterApply(D,FCUT_LPF,NS_TO_SEC(dt));
#endif

    gain->pre_value = sensor;
    gain->PID = (P+ gain->I+D);
	constrainf(gain->PID,-PID_MAX_VAL,PID_MAX_VAL);
}


