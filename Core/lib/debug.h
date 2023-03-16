#ifndef _DEBUG_
#define _DEBUG_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
/*log to ssd1336 screen*/
void debugInit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c);
void debugLog_val(int xx,uint8_t x,uint8_t y);
void debugLog_fval(double xx,uint8_t x,uint8_t y,int afftezero);
void debugLog_str(char* str,uint8_t x,uint8_t y);
void debug_clear(void);
void debug_updateScreen();
/* uart to cp*/
void print_float(float n);
void print_int(int n);
void print_char(char *);

#ifdef __cplusplus
}
#endif
#endif
