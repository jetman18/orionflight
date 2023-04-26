#ifndef MAVLINK_HANDLER
#define MAVLINK_HANDLER
#include "usart.h"
#include"MAVLink/common/mavlink.h"

#ifdef __cplusplus
extern "C" {
#endif
void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate);
void mavlinkCallback();
void mav_tx_cpl_callback();
void mavlink_send();
#ifdef __cplusplus
}
#endif
#endif
