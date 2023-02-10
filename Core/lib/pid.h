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

	float e;

    uint32_t delta_time;
	uint64_t p_time;
}pid__t;

void pidCalculate(pid__t *gain,float sensor,float control,float f_cut);
#ifdef __cplusplus
}
#endif

#endif
