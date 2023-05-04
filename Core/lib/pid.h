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
	float D_smooth;

	float max_i;
	float max_pid;
	float f_cut_D;
	float D_slew_threshold;
}pid__t;
void pidUpdate();
void PID_init_param();
void pidCalculate(pid__t *gain,float sensor,float control,uint32_t delta_time);
void resetPID(pid__t *t);
#ifdef __cplusplus
}
#endif

#endif
