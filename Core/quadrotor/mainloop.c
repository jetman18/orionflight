#include "iqr_handler.h"
#include "log.h"
#include "i2c.h"
#include "imu.h"
#include "qmc5883.h"
#include "usart.h"
#include "gpio.h"
#include "scheduler.h"
#include "pwmwrite.h"
#include "pid.h"
#include "bmp280.h"
#include "ibus.h"
#include "gps.h"
#include "hc_sr04.h"
#include "opticalflow.h"
#include "../quadrotor/config.h"
#include "mavlink_handler.h"
#define SYS_ID 1
const uint16_t dt = 2000;
void main_loop(){
//-----------------INIT MODULES--------------------
	initPWM(&htim4);
	initTimeloop(&htim3);
	ibusInit(&huart2,115200);
	//mavlinkInit(SYS_ID,0,&huart2,115200);
	MPU_i2c_init(&hi2c2);
	motoIdle();
	//qmc5883_init(&hi2c1);
	PID_init_param();
	hc_sr04_start();
	flowInit(&huart1,19200);
/*--------------------------------------------*/
while(1){
	ibusGet();
	imu_update();
	FEQUENCY_DIV(20,ON){
		optical_flow_run();
		hc_sr04_run();
	}
	pidUpdate();
	FEQUENCY_DIV(100,ON){
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
	}
	pwm2esc();
    loop_run(dt);
   }
}
