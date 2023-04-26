#include "MAVLink/common/mavlink.h"
#include "iqr_handler.h"
#include "log.h"
#include "i2c.h"
#include "imu.h"
#include "qmc5883.h"
#include "usart.h"
#include "gpio.h"
#include "timeclock.h"
#include "scheduler.h"
#include "pwmwrite.h"
#include "ppmreceiver.h"
#include "pid.h"
#include "maths.h"
#include "lpf.h"
#include "bmp280.h"
#include "ibus.h"
#include <string.h>
#include "gps.h"
#include "distance.h"
#include "./ssd1306/ssd1306.h"
#include "opticalflow.h"
#include "../quadrotor/config.h"
#include "mavlink_handler.h"
#define BIAS_ALTITUDE 80
#define GYRO_DELAY 4
#define SYS_ID 1
float buffe1[GYRO_DELAY];
float buffe2[GYRO_DELAY];
//BMP280_HandleTypedef bmp280;
//static float temperature,pressure,altitude;
//------test var-------------------------------
extern attitude_t quad_;
uint32_t iii,ii;
uint32_t mil;
int set_altitude;
const uint16_t dt = 2000;
void main_loop(){
//-----------------INIT MODULES--------------------
	initPWM(&htim4);
	motoIdle();
	initTimeloop(&htim3);
	ibusInit(&huart2,115200);
	//mavlinkInit(SYS_ID,0,&huart2,115200);
	MPU_i2c_init(&hi2c2);
	qmc5883_init(&hi2c2);
	PID_init_param();
	/*--GPS--*/
	//gpsInit(&huart3,57600);
	//bmp280 sensor
	//bmp280_init_default_params(&bmp280.params);
    //bmp280.addr = BMP280_I2C_ADDRESS_0;
    //bmp280.i2c = &hi2c1;
	//bmp280_init(&bmp280,&bmp280.params);

	/*flow*/
	//flowInit(&huart2,115200);
/*--------------------------------------------*/
while(1){
	//ibusGet();
	//imu_update();
	//compass_get_heading_();
	//pidUpdate();
	//pwm2esc();

	//FEQUENCY_DIV(100,ENABLE)mavlink_send();
	write_int(&huart1,quad_.roll*100);
	FEQUENCY_DIV(250,ENABLE)
	{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
	}
    loop_start(dt);
   }
}
