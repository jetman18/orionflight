#ifndef _GPS_
#define _GPS_

#ifdef __cplusplus
extern "C" {
#endif

/*
10hz     B5 62 06 08 06 00 64 00 01 00 01 00 7A 12
5hz      B5 62 06 08 06 00 C8 00 01 00 01 00 DE 6A



uart1
all ubx
baud 57600
B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 E1 00 00 01 00 01 00 00 00 00 00 D6 8D



*/


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