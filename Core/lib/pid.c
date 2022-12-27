#include "pid.h"
#include "lpf.h"
#include "maths.h"
#include "timeclock.h"


#define APPLY_LPF
#define NS_TO_SEC(x)  (float)((x)*0.000001)
#define MAX_I   200
#define FCUT_LPF 40.0f  //HZ


static float error = 0;
static float p_fbcontrol=0;
static float P,I,D;
float pidCalcutate(pid_gain_t gain,float fbcontrol,float control,uint16_t dt){

    error =  fbcontrol - control;
    P  =  error*gain.kp;

    if(dt==0)return 0.0f;
    I +=  error*gain.ki*NS_TO_SEC(dt);
    if(I > MAX_I)I= MAX_I;
    else if(I< -MAX_I)I=-MAX_I;

    D  = (fbcontrol - p_fbcontrol)*gain.kd/NS_TO_SEC(dt);
#ifdef APPLY_LPF
    D = pt1FilterApply(D,FCUT_LPF,NS_TO_SEC(dt));
#endif

    p_fbcontrol = fbcontrol;
    return (P+I+D);
}

