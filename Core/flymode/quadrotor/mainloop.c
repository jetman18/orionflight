#include "mainloop.h"
#include "mpu6500.h"
#include "qmc5883.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "timeclock.h"
#include "scheduler.h"
#include "pwmwrite.h"
#include "ppmreceiver.h"
#include "pid.h"
#include "debug.h"
#include "maths.h"
#include "lpf.h"
#include "bmp280.h"
#include "ibus.h"
#include "config.h"
#include <string.h>
#include "./ssd1306/ssd1306.h"
/******************************************/
static float 	Ch1,Ch2,Ch4;
uint16_t throttle;
pid__t pitch_t,roll_t,yaw_t;
int16_t moto1,moto2,moto3,moto4;
int16_t plfmt1,plfmt2,plfmt3,plfmt4;
euler_angle_t mpu;
MAG_t t;
BMP280_HandleTypedef bmp280;
float temperature,pressure,altitude;

uint32_t k1,k2;
extern int16_t calib_axi[3];
extern int16_t max_val[3];
extern int16_t min_val[3];
static void screen_dpl();

void main_loop(){
	initTimeloop(&htim4);
	ibusInit(&huart1);
	MPU_spi_init(&hspi1,GPIOA,GPIO_PIN_4);

	initPWM(&htim2);
	motoIdle();

	ssd1306_Init(&hi2c2);
	ssd1306_Fill(Black);
	ssd1306_SetCursor(10,0);
	ssd1306_WriteString("Calibrating ...",Font_7x10,White);
	ssd1306_UpdateScreen(&hi2c2);

	//qmc5883_init(&hi2c2);
	//magnet_sensor_calibrate();
	//screen_dpl();

	//bmp280 sensor
	bmp280_init_default_params(&bmp280.params);
    bmp280.addr = BMP280_I2C_ADDRESS_0;
    bmp280.i2c = &hi2c2;
	bmp280_init(&bmp280,&bmp280.params);

	ssd1306_Fill(Black);
	while(1){

		bmp280_read_fixed(&bmp280, &temperature, &pressure,&altitude);
		/*
		ssd1306_Fill(Black);
		ssd1306_SetCursor(10,0);
		ssd1306_Write_fval(altitude,Font_7x10,White,2);
		ssd1306_SetCursor(10,10);
	    ssd1306_Write_fval(pressure,Font_7x10,White,2);
		ssd1306_UpdateScreen(&hi2c2);
		ssd1306_SetCursor(10,20);
	    ssd1306_Write_fval(temperature,Font_7x10,White,2);
	    ssd1306_UpdateScreen(&hi2c2);
		HAL_Delay(10);

		/*
		bmp280_read_fixed(&bmp280, &temperature, &pressure,&altitude);


		ssd1306_SetCursor(10,5);
		ssd1306_Write_val(t.mx,Font_7x10,White);
		ssd1306_SetCursor(60,5);
		ssd1306_Write_val((int)pitch,Font_7x10,White);

		ssd1306_SetCursor(10,15);
	    ssd1306_Write_val(t.my,Font_7x10,White);
		ssd1306_SetCursor(60,15);
		ssd1306_Write_val((int)roll,Font_7x10,White);

		ssd1306_SetCursor(10,25);
		ssd1306_Write_val(t.mz,Font_7x10,White);
		*/
		//ssd1306_SetCursor(60,25);
		//ssd1306_Write_val(max_val[2]- calib_axi[2],Font_7x10,White);


	}


	memset(&pitch_t, 0, sizeof(pid__t));
    memset(&roll_t, 0, sizeof(pid__t));
    memset(&yaw_t, 0, sizeof(pid__t));

    const float ch1_gain = 0.11f;
    const float ch2_gain = 0.11f;
    const float ch4_gain =0.000005f;

	const uint16_t dttime = 1000;  //1khz loop

	/*init pid gain*/
	pitch_t.kp =4.3f;   //3.3f
	pitch_t.ki =4.7f;
	pitch_t.kd =1.5;//1.7f;  //1.0f

	roll_t.kp = pitch_t.kp;
	roll_t.ki = pitch_t.ki;
	roll_t.kd = pitch_t.kd;

	yaw_t.kp = 120000;//1000;
	yaw_t.ki = 10000;
	yaw_t.kd = 0;
  //throttle > MINIMUM_THROTLE
	while(throttle > MINIMUM_THROTLE){
		HAL_Delay(100);
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
while(1){


    if(ibusFrameComplete()){
		Ch1=ibusReadf(CH1,ch1_gain);
		Ch2=ibusReadf(CH2,ch2_gain);
		throttle=ibusReadRawRC(CH3);
		Ch4=ibusReadf(CH4,ch4_gain);
    }

	   mpu_update(&mpu,dttime);

	   //bmp280_read_fixed(&bmp280, &temperature, &pressure,&altitude);

	   /*pid update 500hz*/
	   FEQUENCY_DIV(2,ENABLE){
		if(throttle>MINIMUM_THROTLE){
			// pid calculate

			pidCalculate(&pitch_t,mpu.pitch,Ch2,70);//Ch2
			pidCalculate(&roll_t ,mpu.roll ,Ch1,70);//Ch1
			pidCalculate(&yaw_t  ,mpu.yaw  ,Ch4,60);//Ch4

			moto1 = throttle + (-pitch_t.PID - roll_t.PID - yaw_t.PID);
			moto2 = throttle + (-pitch_t.PID + roll_t.PID + yaw_t.PID);
			moto3 = throttle + ( pitch_t.PID + roll_t.PID - yaw_t.PID);
			moto4 = throttle + ( pitch_t.PID - roll_t.PID + yaw_t.PID);

		}
		else {
			   pitch_t.I=0.0f;
			   roll_t.I =0.0f;
			   yaw_t.I  =0.0f;

			   moto1 = 1000;
			   moto2 = 1000;
			   moto3 = 1000;
			   moto4 = 1000;
		   }
			//update throttle to esc
			writePwm(ch1,moto1);
			writePwm(ch2,moto2);
			writePwm(ch3,moto3);
			writePwm(ch4,moto4);
		}
     /*blink led 13*/
	 FEQUENCY_DIV(500,ENABLE){
		 HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	 }
     /*loop feq */
    looptime(dttime);
   }
}
void screen_dpl(){
	/*ssd1306*/  //128*64

		ssd1306_Fill(Black);

		ssd1306_SetCursor(10,0);
		ssd1306_Write_val(max_val[0] - calib_axi[0],Font_7x10,White);
		ssd1306_SetCursor(80,0);
		ssd1306_Write_val(min_val[0] - calib_axi[0],Font_7x10,White);

	    ssd1306_SetCursor(10,15);
	    ssd1306_Write_val(max_val[1] - calib_axi[1],Font_7x10,White);
	    ssd1306_SetCursor(80,15);
	    ssd1306_Write_val(min_val[1] - calib_axi[1],Font_7x10,White);

	    ssd1306_SetCursor(10,30);
	    ssd1306_Write_val(max_val[2],Font_7x10,White);
	    ssd1306_SetCursor(80,30);
	    ssd1306_Write_val(min_val[2],Font_7x10,White);

	    ssd1306_SetCursor(20,45);
	    ssd1306_WriteString("max",Font_11x18,White);

	    ssd1306_SetCursor(70,45);
	    ssd1306_WriteString("min",Font_11x18,White);

	    ssd1306_UpdateScreen(&hi2c2);
	    HAL_Delay(5000);
		while(1){
			static int count =0;
			count ++;
			if(count>12){
				//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
				//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
				count =0;
			}

			float pitch;
		    float roll;
		        if(ibusFrameComplete()){
				   pitch =ibusReadf(CH5,0.2);
				   roll = ibusReadf(CH6,0.2);
		        }

			qmc_get_values(&t,pitch,roll);

			ssd1306_Fill(Black);

			ssd1306_SetCursor(10,5);
			ssd1306_Write_val(t.mx,Font_7x10,White);
			ssd1306_SetCursor(60,5);
			ssd1306_Write_val((int)pitch,Font_7x10,White);

			ssd1306_SetCursor(10,15);
		    ssd1306_Write_val(t.my,Font_7x10,White);
			ssd1306_SetCursor(60,15);
			ssd1306_Write_val((int)roll,Font_7x10,White);

			ssd1306_SetCursor(10,25);
			ssd1306_Write_val(t.mz,Font_7x10,White);
			//ssd1306_SetCursor(60,25);
			//ssd1306_Write_val(max_val[2]- calib_axi[2],Font_7x10,White);

			ssd1306_UpdateScreen(&hi2c2);

		}

}



