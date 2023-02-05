/*
 * qmc5883.cpp
 *
 *  Created on: 8 thg 1, 2023
 *      Author: sudo
 */

#include "qmc5883.h"
#include "maths.h"
#include "i2c.h"
#include "math.h"
#include "lpf.h"
#include "timeclock.h"
#include "./ssd1306/ssd1306.h"

static int16_t calib_axi[3];
const int16_t  calibrate_xyz[3]={-93,81,400};
// lab test
 /*L1    -613  -148  384
  *L2    -380  -311  647
  *L4    -312  -421  685
  *L5    -345  -496  754
  *L6    -426  -292  862
  *L7    -128  -418  89
  *home test
  *L1    -145  -119 828
  *L2    -210  -181 820
  *L3     279   572 366
  * */


const uint8_t qmc_addres = (0x0d<<1);
I2C_HandleTypeDef *qmc_i2cport;
uint16_t interval;

int16_t z_calib_value;
void qmc5883_init(I2C_HandleTypeDef *i2cport){
	qmc_i2cport = i2cport;
    uint8_t buf[2];
    buf[0]=0x0b;
    buf[1]=0X01;
    HAL_I2C_Master_Transmit(qmc_i2cport,qmc_addres,buf,2, 1);
    buf[0]=0x09;
    buf[1]=0X1D;
    HAL_I2C_Master_Transmit(qmc_i2cport,qmc_addres,buf,2, 1);

    int32_t z_value;
    for(int i=0;i<100;i++){
    	HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x04,1,buf,2,1);
    	z_value += ((int16_t)(buf[1])<<8|buf[0]) - calibrate_xyz[2];
       HAL_Delay(5);
    }
    z_calib_value = z_value/100.0f;
}
void qmc_get_raw(MAG_t *t){
	uint8_t buf[6];
	HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
	t->mx=((int16_t)buf[1]<<8|buf[0]) - calibrate_xyz[0];
	t->my=((int16_t)buf[3]<<8|buf[2]) - calibrate_xyz[1];
	t->mz=((int16_t)buf[5]<<8|buf[4]) - calibrate_xyz[2];

	t->compas=atan2_approx(t->mx,t->my)*180.0f/3.1415f;
	if(t->compas<0)t->compas=360.0f + t->compas;
}

void qmc_get_3axil_values(MAG_t *t){
      static uint8_t count=0;
	  static int16_t  mx,my,mz;
	  uint8_t buf[6]={0};
      switch(count){
      	  case 0:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
      		mx=((int16_t)buf[1]<<8|buf[0]) - calibrate_xyz[0];
      		my=((int16_t)buf[3]<<8|buf[2]) - calibrate_xyz[1];
      		mz=((int16_t)buf[5]<<8|buf[4]) - calibrate_xyz[2];
            count++;
      		break;
      	  case 1:
      		t->compas=atan2_approx(my,mx)*180.0f/3.1415f;
      		//if(t->compas<0)t->compas=360.0f + t->compas;
      		count =0;
      		break;
      }
}


float mx_,my_;
void qmc_get_values(MAG_t *t,float pitch,float roll){
      static uint8_t count_mag=0;
	  static int16_t mx,my,mz;
	  uint8_t buf[2];

      switch(count_mag){
      	  case 0:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,2,1);
      		mx=((int16_t)buf[1]<<8|buf[0]) - calibrate_xyz[0];
            count_mag++;
      		break;
      	  case 1:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x02,1,buf,2,1);
      		my=((int16_t)buf[1]<<8|buf[0]) - calibrate_xyz[1];
            count_mag++;
      		break;
      	  case 2:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x04,1,buf,2,1);
      		mz=((int16_t)buf[1]<<8|buf[0]) - calibrate_xyz[2] - z_calib_value;
            count_mag++;
      		break;
      	  case 3:
      		pitch *=0.01745f;
      		roll  *=0.01745f;

      		mx*=1;   //1
      		my*=1;  //-1
      		mz*=1;   //1

      		mx = mx*cos_approx(pitch) + my*sin_approx(pitch)*sin_approx(roll) - mz*sin_approx(pitch)*cos_approx(roll);
      		my = my*cos_approx(roll) +  mz*sin_approx(roll);

      		t->mx = mx;
			t->my = my;
			t->mz = mz;
      		t->compas=atan2_approx((float)my,(float)mx)*180.0f/3.1415f;
      		//if(t->compas<0)t->compas=360.0f + t->compas;
      		count_mag =0;
      		break;
      }

}


int16_t max_val[] = {-32767,-32767,-32767};
int16_t min_val[] = {32767, 32767, 32767};
void magnet_sensor_calibrate(){

	int16_t mx,my,mz;
	uint8_t buf[6];
	for(int i=0;i<6000;i++){
			HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
			mx=(int16_t)(buf[1])<<8|buf[0];
			my=(int16_t)(buf[3])<<8|buf[2];
			mz=(int16_t)(buf[5])<<8|buf[4];

			if(mx > max_val[0]) max_val[0] = mx;
			if(mx < min_val[0]) min_val[0] = mx;

			if(my > max_val[1]) max_val[1] = my;
			if(my < min_val[1]) min_val[1] = my;

			if(mz > max_val[2]) max_val[2] = mz;
			if(mz < min_val[2]) min_val[2] = mz;
	        HAL_Delay(5);
    }
	for(int i=0;i<3;i++){
		if((max_val[i]*min_val[i]) < 0){
			calib_axi[i] = (max_val[i] + min_val[i])/2;
		}
		else{
		    calib_axi[i] = (max_val[i] - min_val[i])/2;
		}
	}

	ssd1306_Init(&hi2c2);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen(&hi2c2);
	HAL_Delay(100);

	ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("ssd1306", Font_11x18, White);
    ssd1306_UpdateScreen(&hi2c2);
	while(1){
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}
//------------------------------------------------
