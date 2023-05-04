#ifndef _IQR_HANDLER_H_
#define _IQR_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include "gpio.h"
#include "mavlink_handler.h"
#include "hc_sr04.h"
#include "ibus.h"
#include "scheduler.h"
// IQR function
//----------------------------------IQR--Handle-----------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* ibus */
    if(huart == &huart2)
	{
        ibusCallback();
    }                                                                                                                           //
    else if(huart == &huart1){
      	mavlinkCallback();
     }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// ibus
    if(huart == &huart2)
	{
    	mav_tx_cpl_callback();
    }                                                                                                                          //
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_8)
    {
     hc_sr04_callback();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim4)
	{
        timeCallback();
	}
}
//----------------------------------IQR--Handle-----------------------------



#ifdef __cplusplus
}
#endif

#endif
