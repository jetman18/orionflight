#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
typedef struct{
	float kp;
	float ki;
	float kd;
}pid_gain_t;

float pidCalcutate(pid_gain_t gain,float fbcontrol,float control,uint16_t dt);

#ifdef __cplusplus
}
#endif

#endif
