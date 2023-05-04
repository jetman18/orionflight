/*
 * distance.h
 *
 *  Created on: Apr 16, 2023
 *      Author: sudo
 */

#ifndef HC_SR04_H
#define HC_SR04_H
#include "stdio.h"

#ifdef __cplusplus
extern "C" {
#endif
void hc_sr04_callback();
int hc_sr04_get_dis();
int isHcNewdata();
void hc_sr04_send_trige();
#ifdef __cplusplus
}
#endif

#endif /* LIB_DISTANCE_H_ */
