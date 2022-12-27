#include "pwmwrite.h"
#include "stdlib.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#define LEN 15

void checksum(uint8_t *l,int len){
	uint16_t sum=0;
    for(int i=1;i<len-2;i++){
    	sum += l[i];
    }
    l[len-1] = (uint8_t)sum & 0xff;
    l[len-2] = (uint8_t)sum>>8;
}
void writePWM(uint16_t* m){//6 channel

	uint8_t buf[LEN];
    buf[0] = 0x5b;

		int l=1;
		for(int i=0;i<6;i++){
			buf[l]=(uint8_t)m[i]&0xff;
			buf[l+1]=m[i]>>8;
			l+=2;
		}
	 checksum(buf,LEN);

	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	 HAL_SPI_Transmit(&hspi2,buf,LEN,10);
	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);


	 //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	}

