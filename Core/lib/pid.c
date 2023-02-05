#include "pid.h"
#include "lpf.h"
#include "maths.h"
#include "timeclock.h"


#define APPLY_LPF
#define NS_TO_SEC(x)  (float)((x)*0.000001f)
#define MAX_I   300
#define FCUT_LPF 100.0f    //HZ
#define PID_MAX_VAL 400.0f

void pidCalculate(pid__t *gain,float sensor,float control){
	float error,P,D;

    gain->delta_time = micros() -  gain->p_time;
    if(gain->delta_time<0){
        return;
    }
    gain->p_time = micros();

    error =  sensor - control;
    P  =  error*gain->kp;

    gain->I+=  error*gain->ki*NS_TO_SEC(gain->delta_time);
    consTrainf(&gain->I,-MAX_I,MAX_I);

    D  = (sensor - gain->pre_value)*gain->kd/NS_TO_SEC(gain->delta_time);

    gain->pre_value = sensor;
    gain->PID = (P + gain->I + D);
    consTrainf(&gain->PID,-PID_MAX_VAL,PID_MAX_VAL);
}


