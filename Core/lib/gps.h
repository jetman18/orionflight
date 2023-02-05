#ifndef _GPS_
#define _GPS_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
}NAV_POSLLH;

/******************************************/
int processGPS(char);
int init_gps();


#ifdef __cplusplus
}
#endif
#endif