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


static int16_t calib_axi[3];
const int16_t  calibrate_xyz[3]={340,-404,693};
// board only
 /*L1    -613  -148  384
  *L2    -380  -311  647
  *L3    -219   30   747
  *L4    -312  -421  685
  *L5    -345  -496  754
  *L6    -426  -292  862
  *L7    -128  -418  89
  * */
const uint8_t qmc_addres = (0x0d<<1);
I2C_HandleTypeDef qmc_i2cport;
void qmc5883_init(I2C_HandleTypeDef *i2cport){
	qmc_i2cport = *i2cport;
    uint8_t buf[2];
    buf[0]=0x0b;
    buf[1]=0X01;
    HAL_I2C_Master_Transmit(&qmc_i2cport,qmc_addres,buf,2, 1);
    buf[0]=0x09;
    buf[1]=0X1D;
    HAL_I2C_Master_Transmit(&qmc_i2cport,qmc_addres,buf,2, 1);
}
void qmc_get_raw(MAG_t *t){
	uint8_t buf[6];
	HAL_I2C_Mem_Read(&qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
	t->mx=((int16_t)buf[1]<<8|buf[0]) - calibrate_xyz[0];
	t->my=((int16_t)buf[3]<<8|buf[2]) - calibrate_xyz[1];
	t->mz=((int16_t)buf[5]<<8|buf[4]) - calibrate_xyz[2];

	t->compas=atan2_approx(t->mx,t->my)*180.0f/3.1415f;
	if(t->compas<0)t->compas=360.0f + t->compas;
}

void qmc_get_values(MAG_t *t,float pitch,float roll){

	  float mx,my;
	  uint8_t buf[6];

		HAL_I2C_Mem_Read(&qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
		t->mx=((int16_t)buf[1]<<8|buf[0]) - calibrate_xyz[0];
		t->my=((int16_t)buf[3]<<8|buf[2]) - calibrate_xyz[1];
		t->mz=((int16_t)buf[5]<<8|buf[4]) - calibrate_xyz[2];

		mx = (float)t->mx*cos_approx(-pitch) + (float)t->my*sin_approx(-pitch)*sin_approx(roll) + (float)t->mz*sin_approx(-pitch)*cos_approx(roll);
		my = (float)t->my*(cos_approx(roll) + (float)t->mz*sin_approx(roll));

		t->compas=atan2_approx(my,mx)*180.0f/3.1415f;
		if(t->compas<0)t->compas=360.0f + t->compas;

}

void magnet_sensor_calibrate(){
	int16_t max_val[] = {-32767,-32767,-32767};
	int16_t min_val[] = {32767, 32767, 32767};
	int16_t mx,my,mz;
	uint8_t buf[6];
	for(int i=0;i<6000;i++){
			HAL_I2C_Mem_Read(&qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
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
	while(1){
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}
//------------------------------------------------
