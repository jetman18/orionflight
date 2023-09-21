#include"mavlink_handler.h"
#include "timer.h"
#include "imu.h"
#define MAX_LENGHT 100

static mavlink_named_value_int_t  val_int;
static mavlink_named_value_float_t  val_float;
static mavlink_attitude_t attitude;
static mavlink_message_t msg;
static mavlink_status_t msg_status;
static uint8_t data;
static uint8_t index_;
uint8_t sys_id,com_id;
static UART_HandleTypeDef *uart;
uint8_t buffer[MAX_LENGHT];
static int isTxcpl;

void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate){
    isTxcpl = 1;
    index_ =0;
	sys_id  = syss_id;
    com_id  = comm_id;
	uart = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt); //reInit
	HAL_UART_Receive_IT(uart, &data,1);
}
void mavlinkCallback(){
    if (mavlink_parse_char(MAVLINK_COMM_0,data,&msg, &msg_status))
    {
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_ATTITUDE:
            mavlink_msg_attitude_decode((const mavlink_message_t*)&msg,&attitude);
            break;
        case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        	mavlink_msg_named_value_int_decode((const mavlink_message_t*)&msg,&val_int);
            break;
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        	mavlink_msg_named_value_float_decode((const mavlink_message_t*)&msg,&val_float);
            break;
        }
    }
    HAL_UART_Receive_IT(uart, &data,1);
}

char *name ="roll";
void mavlink_send(){
	if(isTxcpl){
		uint32_t boot_time = millis();
		mavlink_message_t msg_send;
		mavlink_msg_named_value_float_pack(sys_id,com_id,&msg_send,boot_time,(const char *)name,quad_.roll);
		int len = mavlink_msg_to_send_buffer((uint8_t *)&buffer[index_],&msg_send);
		HAL_UART_Transmit_DMA(uart,buffer,len);
		isTxcpl = 0;
	}
}

void mav_tx_cpl_callback()
{
	isTxcpl = 1;
}

