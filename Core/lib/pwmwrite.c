#include "pwmwrite.h"
#include "stdlib.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#define LEN 15

#define RELOAD_AFFTER 2500

/**timer pwm
 *
 */
TIM_HandleTypeDef htimm;
void initPWM(TIM_HandleTypeDef *htim){
	HAL_TIM_PWM_Start(htim,ch1);
	HAL_TIM_PWM_Start(htim,ch2);
	HAL_TIM_PWM_Start(htim,ch3);
	HAL_TIM_PWM_Start(htim,ch4);
	__HAL_TIM_SetAutoreload(htim,RELOAD_AFFTER);
	htimm = *htim;
}

void writePwm(uint32_t Channel,uint16_t dulty){
	__HAL_TIM_SetCompare (&htimm,Channel,dulty);
}







/**external pwm board
 *
 *
 *
 */
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

	// HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	// HAL_SPI_Transmit(&hspi2,buf,LEN,10);
	// HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);


	}

