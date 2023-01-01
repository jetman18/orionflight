
#ifndef PWMWRITE_H
#define PWMWRITE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "gpio.h"

enum pwm_channel{
	ch1 = TIM_CHANNEL_1,
	ch2 = TIM_CHANNEL_2,
	ch3 = TIM_CHANNEL_3,
	ch4 = TIM_CHANNEL_4
};
void initPWM(TIM_HandleTypeDef *htim);
void writePwm(uint32_t Channel,uint16_t dulty);
void writePWM(uint16_t* m);
#ifdef __cplusplus
}
#endif

#endif /* LIB_PWMWIRITE_H_ */
