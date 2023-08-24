#ifndef _GPS_H_
#define _GPS_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
uart1
all ubx
baud 57600
B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 E1 00 00 01 00 01 00 00 00 00 00 D6 8D
*/
#include "stm32f1xx_hal.h"
typedef struct gps_data{
    uint8_t   fix;
    int32_t    coord[2];
    uint16_t  altitude;
    uint32_t  HorizontalAcc;
    uint32_t  VerticalAcc;
    uint8_t   numSat;
    int16_t   speed;
    uint16_t  ground_course;
    uint16_t  coord_update_time;
}gpsData_t;
void gpsInit(UART_HandleTypeDef *uartt,uint32_t baudrate);
void gpsCallback(void);

#ifdef __cplusplus
}
#endif
#endif
