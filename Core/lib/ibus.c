#include "maths.h"
#include "ibus.h"
#include "scheduler.h"
#include "usart.h"
#include "imu.h"
#define IBUS_BUFFSIZE 32
#define IBUS_MAX_CHANNEL 8
#define IBUS_SYNCBYTE 0x20
#define false 0
#define true 1
float  rx_ch1,rx_ch2,rx_ch4,vr_1,vr_2;
uint16_t throttle,altitude_stick,ch3_;
static int ibusFrameDone = false;
static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];
static uint8_t ibus[IBUS_BUFFSIZE] = { 0, };
static uint8_t rx_buff;
static UART_HandleTypeDef *uart;
float rx_yaw;
static uint32_t p_time,delta_t;

//#define ACCRO

#ifdef ACCRO
	const float ch1_scale = 0.6f;
	const float ch2_scale = 0.6f;
	const float max_rate  = 400.0f;
#else
	const float ch1_scale = 0.2f;
	const float ch2_scale = 0.2f;
	const float max_rate  = 50.0f;
#endif

//const float ch3_scale = 0.23f;  // -> 200mm/s
const float ch4_scale = 0.6;
void ibusInit(UART_HandleTypeDef *uartt,uint32_t baudrate){
	uart = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt); //reInit
	HAL_UART_Receive_IT(uart, &rx_buff,1);
}
void ibusGet(){
    if(ibusFrameComplete()){
		rx_ch1=ibusReadf(CH1,ch1_scale);
		rx_ch2=ibusReadf(CH2,ch2_scale);
		rx_ch1 = constrainf(rx_ch1,-max_rate,max_rate);//40
		rx_ch2 = constrainf(rx_ch2,-max_rate,max_rate);
		//rx_ch3=ibusReadf(CH3,ch3_scale);
		throttle=ibusReadRawRC(CH3);
		ch3_ = throttle;
		ch3_ -=1000;
		ch3_ = constrain(ch3_,0,1000);
		//altitude scale
        ch3_ *=1.5f;
		altitude_stick=ibusReadRawRC(CH5);
		vr_1 = (ibusReadRawRC(CH6)-1000)/200.0f;
		vr_2 = (ibusReadRawRC(CH7)-1000)/200.0f;
		rx_ch4=ibusReadf(CH4,ch4_scale);
     }
	//float temp = (float)(1e-06f)*config.dt;
	//rx_yaw = rx_yaw + rx_ch4*temp;
}
// Receive ISR callback
void ibusDataReceive(uint16_t c)
{
    uint32_t ibusTime;
    static uint32_t ibusTimeLast;
    static uint8_t ibusFramePosition;

    ibusTime = micros();

    if ((ibusTime - ibusTimeLast) > 3000)
        ibusFramePosition = 0;

    ibusTimeLast = ibusTime;

    if (ibusFramePosition == 0 && c != IBUS_SYNCBYTE)
        return;

    ibus[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == IBUS_BUFFSIZE - 1) {
        ibusFrameDone = true;
        delta_t = micros() - p_time;
        p_time  = micros();
    } else {
        ibusFramePosition++;
    }
}

int ibusFrameComplete(void)
{
    uint8_t i;
    uint16_t chksum, rxsum;

    if (ibusFrameDone) {
        ibusFrameDone = false;

        chksum = 0xFFFF;

        for (i = 0; i < 30; i++)
            chksum -= ibus[i];

        rxsum = ibus[30] + (ibus[31] << 8);

        if (chksum == rxsum) {
            ibusChannelData[0] = (ibus[ 3] << 8) + ibus[ 2];
            ibusChannelData[1] = (ibus[ 5] << 8) + ibus[ 4];
            ibusChannelData[2] = (ibus[ 7] << 8) + ibus[ 6];
            ibusChannelData[3] = (ibus[ 9] << 8) + ibus[ 8];
            ibusChannelData[4] = (ibus[11] << 8) + ibus[10];
            ibusChannelData[5] = (ibus[13] << 8) + ibus[12];
            ibusChannelData[6] = (ibus[15] << 8) + ibus[14];
            ibusChannelData[7] = (ibus[17] << 8) + ibus[16];
            return true;
        }
    }
    return false;
}

uint16_t ibusReadRawRC(uint8_t chan){
    return ibusChannelData[chan];
}
float ibusReadf(uint8_t chan,float gain){
	int16_t val = ibusChannelData[chan];
     return 	((float)(val-1500)*gain);
}

uint32_t getReadTime(){
    return delta_t;
};

void ibusCallback()
{
    ibusDataReceive(rx_buff);
    HAL_UART_Receive_IT(uart, &rx_buff,1);
}

