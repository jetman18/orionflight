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

	float I;
	float PID;
	float pre_value;
}pid_gain_t;

void pidCalculate(pid_gain_t *gain,float sensor,float control,uint16_t dt);
#ifdef __cplusplus
}
#endif

#endif
