#ifndef _MS5611_H_
#define _MS5611_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"

void startMeasure();
int8_t ms5611Detect();

#ifdef __cplusplus
}
#endif
#endif
