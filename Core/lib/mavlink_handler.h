#ifndef MAVLINK_HANDLER
#define MAVLINK_HANDLER
#include "usart.h"
#include"MAVLink/common/mavlink.h"

#ifdef __cplusplus
extern "C" {
#endif
void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate);
void mavlinkCallback();
mavlink_attitude_t get_attitude_();
mavlink_named_value_int_t named_value_int_();
mavlink_named_value_float_t named_value_float_();
int mav_pack_attitude(float roll,float pitch ,float yaw, float rollspeed, float pitchspeed, float yawspeed);
int mav_pack_named_value_int(char* strr,int val);
int mav_pack_named_value_float(char* strr,float val);
void mav_tx_cpl_callback();
void mavlink_send();
#ifdef __cplusplus
}
#endif
#endif
