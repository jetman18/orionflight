#include "ppmreceiver.h"
#include "timeclock.h"
#include "gpio.h"
#include "maths.h"
#define   NUM_CHANNEL 6

uint16_t rawValues[NUM_CHANNEL];
const uint16_t blankTime = 2100;
uint8_t pulseCounter=0;
uint32_t microsAtLastPulse=0;
void ppmcallback(){

    uint32_t previousMicros = microsAtLastPulse;
    microsAtLastPulse = micros();
    uint32_t time = microsAtLastPulse - previousMicros;

    if (time > blankTime) {
        // Blank detected: restart from channel 1
        pulseCounter = 0;
    }
    else {
        // Store times between pulses as channel values
        if (pulseCounter < NUM_CHANNEL) {
            rawValues[pulseCounter] = time;
            ++pulseCounter;
        }
    }
}

/*ppm receiver*/
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14) // If The INT Source Is EXTI 14
    {
    	callBackFuncition(&rx);

    	if(isRxupdate()){
    		Ch1 = ChannelTodec(rx.ch[0],1500,ch1_gain);
    		Ch2 = ChannelTodec(rx.ch[1],1500,ch2_gain);
    		Ch4 = ChannelTodec(rx.ch[3],1500,ch4_gain);
    	}
    }

}
*/
