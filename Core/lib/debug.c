#include "debug.h"
#include "math.h"
#include "usart.h"
#include "maths.h"
#include "stm32f1xx_hal.h"
#include "./ssd1306/ssd1306.h"
#include "string.h"
static UART_HandleTypeDef *uart_port;
static I2C_HandleTypeDef  *hi2cc;
static int sig;
static int indexx;
void debugInit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c){
    uart_port = huart;
    hi2cc     = hi2c;
    ssd1306_Init(hi2cc);
};
void debugLog_val(int xx,uint8_t x,uint8_t y){
     ssd1306_SetCursor(x,y);
     ssd1306_Write_val(xx,Font_7x10,White);
}
void debugLog_fval(double xx,uint8_t x,uint8_t y,int afftezero){

     ssd1306_SetCursor(x,y);
     ssd1306_Write_fval(xx,Font_7x10,White,afftezero);
}
void debugLog_str(char* str,uint8_t x,uint8_t y){
    ssd1306_SetCursor(x,y);
    ssd1306_WriteString(str,Font_7x10,White);
}
void debug_clear(void){
     ssd1306_Fill(Black);
}
void debug_updateScreen(){
     ssd1306_UpdateScreen(hi2cc);
}

static void reverse( uint8_t *str, int len)
{
    int i = sig, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
static int intToStr(int x,  uint8_t *str, int d)
{
    while (x) {
        str[indexx++] = (x % 10) + '0';
        x = x / 10;
    }

    while (indexx < d)
        str[indexx++] = '0';
    reverse(str,indexx);
    return indexx;
}

#define afftezero 3  //float 

void print_float(float n)
{  

}

void write_char(char *str)
{
    uint16_t len=0;
    while(str[len++]);
    HAL_UART_Transmit(uart_port,(uint8_t *)str,len,100);
}
void write_int(int x)
{
    indexx=0;
    sig = 0;
    uint8_t str_[11];
    memset(str_,0,11);
    if(x<0){
        x*=-1;
        str_[0] = '-';
        indexx++;
        sig = 1;
    }
   int len = intToStr(x,str_,0);
   HAL_UART_Transmit(uart_port,str_,len,100);
}
