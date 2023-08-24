#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
extern uint16_t moto1,moto2,moto3,moto4;
void PID_init_parameters();
void pidUpdate();
#ifdef __cplusplus
}
#endif

#endif
