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

	uint32_t p_time;
	uint16_t delta_time;
}pid__t;

void pidCalculate(pid__t *gain,float sensor,float control);
#ifdef __cplusplus
}
#endif

#endif
