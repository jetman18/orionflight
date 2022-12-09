#ifndef _DEBUG_
#define _DEBUG_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
////////////////////////////////////////

void print_float(float n);
void print_int(int n);
void print_char(char *);

#ifdef __cplusplus
}
#endif
#endif
