#ifndef _NAVIGATION_
#define _NAVIGATION_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"




void add_coordinate();
void add_coordinate_index();
void delete_coordinate(uint8_t index);

void navigation_run();



#ifdef __cplusplus
}
#endif
#endif