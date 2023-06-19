/*
 * distance.h
 *
 *  Created on: Apr 16, 2023
 *      Author: sudo
 */

#ifndef HC_SR04_H
#define HC_SR04_H
#include "stdio.h"
#include "pid.h"
#ifdef __cplusplus
extern "C" {
#endif
extern int hc04_Throttle;
extern pid__t Pid_altitude_t;
void hc_sr04_callback();
void hc_sr04_run();
#ifdef __cplusplus
}
#endif

#endif /* LIB_DISTANCE_H_ */
