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
#include "gps.h"
#include "./ssd1306/ssd1306.h"
/******************************************/
const float ch1_gain = 0.11f;
const float ch2_gain = 0.11f;
const float ch4_gain =0.000005f;
static const uint16_t dttime = 1000U;
faxis3_t t;
static float  Ch1,Ch2,Ch4;
static uint16_t throttle;
pid__t pitch_t,roll_t,yaw_t;
static uint16_t moto1,moto2,moto3,moto4;
static euler_angle_t mpu;
BMP280_HandleTypedef bmp280;
static float temperature,pressure,altitude;
static void screen_dpl();
static void bipbip(int bip);

void main_loop(){
	initTimeloop(&htim4);
	debugInit(&huart1,&hi2c1);
	/*--ibus--*/
	ibusInit(&huart1,115200);
	/*--MPU--*/
	MPU_spi_init(&hspi1,GPIOA,GPIO_PIN_4);
	/*--GPS--*/
	gpsInit(&huart3,57600);
    /*--pwm-out--*/
	initPWM(&htim2);
	motoIdle();

	//bmp280 sensor
	//bipbip(4);
	//bmp280_init_default_params(&bmp280.params);
    //bmp280.addr = BMP280_I2C_ADDRESS_0;
    //bmp280.i2c = &hi2c1;
	//bmp280_init(&bmp280,&bmp280.params);

	qmc5883_init(&hi2c1);
	screen_dpl();

	/*init pid gain*/
	pitch_t.kp =4.3f;   
	pitch_t.ki =4.7f;
	pitch_t.kd =1.5f;

	roll_t.kp = pitch_t.kp;
	roll_t.ki = pitch_t.ki;
	roll_t.kd = pitch_t.kd;

	yaw_t.kp = 120000;
	yaw_t.ki = 10000;
	yaw_t.kd = 0;

while(1){
    static int start_ = 0;
    if(ibusFrameComplete()){
		Ch1=ibusReadf(CH1,ch1_gain);
		Ch2=ibusReadf(CH2,ch2_gain);
		throttle=ibusReadRawRC(CH3);
		Ch4=ibusReadf(CH4,ch4_gain);
		if(!start_){
			if(ibusReadRawRC(CH3)<1100 &&  ibusReadRawRC(CH4)<1100 &&
			   ibusReadRawRC(CH2)>1900 &&  ibusReadRawRC(CH1)<1100){
               start_ =1;
               bipbip(3);
		    }
        }
    }
	   mpu_update(&mpu,dttime);
	   //bmp280_read_fixed(&bmp280, &temperature, &pressure,&altitude);

	   /*pid rate 500hz*/
	   FEQUENCY_DIV(2,ENABLE){
		if( start_ && throttle>1030){
			// pid calculate
			float Pitch = mpu.pitch/10.0f;
			float Roll = mpu.roll/10.0f;
			float Yaw = mpu.yaw/10.0f;
			pidCalculate(&pitch_t,Pitch,Ch2,2000,70);//Ch2
			pidCalculate(&roll_t ,Roll,Ch1,2000,70);//Ch1
			pidCalculate(&yaw_t  ,Yaw,Ch4,2000,0);//Ch4

			moto1 = throttle - pitch_t.PID - roll_t.PID - yaw_t.PID;
			moto2 = throttle - pitch_t.PID + roll_t.PID + yaw_t.PID;
			moto3 = throttle + pitch_t.PID + roll_t.PID - yaw_t.PID;
			moto4 = throttle + pitch_t.PID - roll_t.PID + yaw_t.PID;

		}
		else{
			//reset I
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
	 /*
	 FEQUENCY_DIV(500,ENABLE){
		 HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	 }
	 */
    looptime(dttime);
   }
}

static void screen_dpl(){

		while(1){
            /*
            static int8_t t = 0;
            int8_t fix = gpsGetdata().fix;
			uint8_t numsat = gpsGetdata().numSat;
			float longitude = gpsGetdata().coord[LON];
			float latitude =  gpsGetdata().coord[LAT];

			if(fix != 0 && t == 0){
				bipbip(4);
				t=1;
			}

			ssd1306_Fill(Black);


			ssd1306_Line(63,2,63,36,White);
			ssd1306_Line(0,36,128,36,White);

			ssd1306_SetCursor(2,0);
			ssd1306_WriteString("Ms:",Font_7x10,White);
			ssd1306_SetCursor(34,0);
			ssd1306_Write_val(gpsGetdata().coord_update_time,Font_7x10,White);

			ssd1306_SetCursor(85,2);
			ssd1306_WriteString("m/s",Font_7x10,White);
			ssd1306_SetCursor(80,14);
			ssd1306_Write_fval(gpsGetdata().speed/100,Font_11x18,White,1);

		    ssd1306_SetCursor(2,12);
			ssd1306_WriteString("Sat:",Font_7x10,White);
			ssd1306_SetCursor(34,12);
			ssd1306_Write_val(numsat,Font_7x10,White);

			ssd1306_SetCursor(2,25);
			ssd1306_WriteString("Fix:",Font_7x10,White);
			ssd1306_SetCursor(34,25);
			ssd1306_Write_val(fix,Font_7x10,White);

		    ssd1306_SetCursor(2,41);
			ssd1306_WriteString("Lon:",Font_7x10,White);
			ssd1306_SetCursor(34,41);
			ssd1306_Write_fval(longitude,Font_7x10,White,6);

		    ssd1306_SetCursor(2,53);
			ssd1306_WriteString("Lat:",Font_7x10,White);
			ssd1306_SetCursor(34,53);
			ssd1306_Write_fval(latitude,Font_7x10,White,6);




		    ssd1306_Fill(Black);
		    static float all;
		    bmp280_read_fixed(&bmp280, &temperature, &pressure,&altitude);
            all = all*0.7f + 0.3f*altitude;
			ssd1306_SetCursor(5,5);
			ssd1306_WriteString("t:",Font_7x10,White);
			ssd1306_SetCursor(30,5);
			ssd1306_Write_fval(temperature,Font_7x10,White,2);

		    ssd1306_SetCursor(5,18);
			ssd1306_WriteString("H:",Font_7x10,White);
			ssd1306_SetCursor(30,18);
			ssd1306_Write_val(all,Font_7x10,White);

		    ssd1306_SetCursor(5,28);
			ssd1306_WriteString("P:",Font_7x10,White);
			ssd1306_SetCursor(30,28);
			ssd1306_Write_val(pressure,Font_7x10,White);
 qmc_get_3axil_values(&t);
*/          static MAG_t t;
			//get_AccAngle(&mpu);
			qmc_get_raw(&t);
		    //qmc_get_values(&t,mpu.pitch,mpu.roll);
		    print_int(t.mx);
		    print_char(" ");
		    print_int(t.my);
		    print_char(" ");
		    print_int(t.mz);
		    print_char("\n");
		    HAL_Delay(25);

			//ssd1306_Fill(Black);

			//ssd1306_SetCursor(40,5);
			//ssd1306_WriteString("P:",Font_7x10,White);
			//ssd1306_SetCursor(60,5);
			//ssd1306_Write_val(mpu.pitch,Font_7x10,White);

			//ssd1306_SetCursor(40,18);
			//ssd1306_WriteString("R:",Font_7x10,White);
			//ssd1306_SetCursor(60,18);
			//ssd1306_Write_val(mpu.roll,Font_7x10,White);
/*
			ssd1306_SetCursor(3,5);
			ssd1306_Write_fval(t.mx,Font_7x10,White,1);

			ssd1306_SetCursor(3,18);
			ssd1306_Write_fval(t.my,Font_7x10,White,1);

			ssd1306_SetCursor(3,28);
			ssd1306_Write_fval(t.mz,Font_7x10,White,1);

			ssd1306_SetCursor(3,38);
			ssd1306_Write_fval(t.compas,Font_7x10,White,1);

			ssd1306_UpdateScreen(&hi2c1);
			*/

		}

}

static void bipbip(int bip){
	for(int ii=0;ii<bip;ii++){
		for(int i=0;i<200;i++){

			 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
			 delay_us(115);  //120
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
			 delay_us(300);  //120
		}
		delay_ms(100);
	}
}


//reDefine iqr function
/*----------------------------------IQR--Handle-----------------------------*/
                                                                            //
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)                     //
{                                                                           //
	/* ibus */                                                              //
    if(huart == &huart1){                                                   //
    	ibusCallback(&huart1);                                              //
    }                                                                       //
    /* gps */                                                               //
    else if(huart == &huart3){                                              //
    	gpsCallback();                                                      //
    }                                                                       //
}                                                                           //
/*----------------------------------IQR--Handle-----------------------------*/

