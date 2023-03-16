#include "debug.h"
#include "math.h"
#include "usart.h"
#include "maths.h"
#include "stm32f1xx_hal.h"
#include "./ssd1306/ssd1306.h"
#include "string.h"
static UART_HandleTypeDef *uart_port;
static I2C_HandleTypeDef  *hi2cc;
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

static void reverse( char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
static int intToStr(int x,  char *str, int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    while (i < d)
        str[i++] = '0';
  
    reverse(str, i);
    str[i] = '\0';
    return i;
}

#define afftezero 3  //float 

void print_float(float n)
{  
    float nn=n;
    n=fabs(n);
    int afterpoint=afftezero+1;
    char res[20];
    int ipart = (int)n;
    float fpart = n - (float)ipart;
    int i = intToStr(ipart, res,1);

    if(fpart!=0.0){
          if (afterpoint != 0) {
              res[i] = '.'; 
              fpart = fpart * pow(10, afterpoint);
        
              int ii=intToStr((int)fpart, res + i + 1,afterpoint);
              if(nn<0.0f){
                 for(int l=ii+i+1;l>=0;l--){
                  res[l+1]=res[l];                
                 }
                 res[0]='-';
                 i++;
              }
              HAL_UART_Transmit(uart_port,(uint8_t *)&res,i+ii,100);
          }
       }
    else {
      if(nn<0.0f){
         for(int l=i+1;l>=0;l--)res[l+1]=res[l];                
         res[0]='-';
         i++;
        }
        HAL_UART_Transmit(uart_port,(uint8_t *)&res,i,100);
    }
}
////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
void print_char(char *str){
	 int k;
   for(int i=0;i<30;i++){
      if(str[i]=='\0'){
      k=i;
      break;
     }
    }
	 HAL_UART_Transmit(uart_port,(uint8_t *)str,k,100);
}
////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
void print_int(int x)
{   char str[10]; 
    memset(str,0,10);
    int m=x;
    x=ABS(x);
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
    if(m<0){
       int y=i;
       while (i < y+1)
           str[i++] = '-';
     }

    int k = 0, j = i - 1, temp;
    while (k < j) {
        temp = str[k];
        str[k] = str[j];
        str[j] = temp;
        k++;
        j--;
    }   
 HAL_UART_Transmit(uart_port,(uint8_t *)str,i,100);
}
