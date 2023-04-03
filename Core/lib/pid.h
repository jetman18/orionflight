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
}pid__t;
uint32_t getReadTime();
void  pidCalculate(pid__t *gain,float sensor,float control,uint16_t delta_time);
#ifdef __cplusplus
}
#endif

#endif
