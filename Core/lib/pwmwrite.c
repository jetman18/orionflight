#include "pwmwrite.h"
#include "stdlib.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"


uint8_t ck[2];

static void checksum(uint8_t *l,int len){
	uint16_t sum=0;
    for(int i=1;i<len+1;i++){
    	sum += l[i];
    }
    ck[0] = (uint8_t)sum & 0xff;
    ck[1] = (uint8_t)sum>>8;

}
void writePWM(uint16_t* m){//6 channel
		uint8_t buf[15];
        buf[0] = 0x5b;

		int l=1;
		for(int i=0;i<6;i++){
			buf[l]=(uint8_t)m[i]&0xff;
			buf[l+1]=m[i]>>8;
			l+=2;
		}
	 checksum(buf,12);
	 buf[13]=ck[1];
	 buf[14]=ck[0];

	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	 HAL_SPI_Transmit(&hspi1,buf,15,1);
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);

	}

