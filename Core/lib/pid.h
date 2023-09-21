#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"
#include "stdio.h"

typedef struct pid{
	float kp;
	float ki;
	float kd;

	float e;
	float I;
	float PID;
	float pre_value;
	float D_smooth;

    float max_P;
	float max_I;
	float max_D;
	float max_pid;

	float f_cut_D;
	float I_deadband;
}pid__t;
void pidCalculate(pid__t *gain,float sensor,float control,uint32_t delta_time);
void resetPID(pid__t *t);
#ifdef __cplusplus
}
#endif

#endif
