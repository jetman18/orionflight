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

int16_t calib_axi[3];
int16_t maxval[] = {0,0,0};
const int16_t  calibrate_xyz[3]={-93,81,400};
const uint8_t qmc_addres = (0x0d<<1);
I2C_HandleTypeDef *qmc_i2cport;

float combined_bias[3]={130.445,78.9098,-129.84};

float clMatix[9]={7.898715 , -0.299080, -0.012913,
		          -0.299080, 8.613594,   0.031860,
				  -0.012913, 0.031860,   8.744643};

void qmc5883_init(I2C_HandleTypeDef *i2cport){
	qmc_i2cport = i2cport;
    uint8_t buf[2];
    buf[0]=0x0b;
    buf[1]=0X01;
    HAL_I2C_Master_Transmit(qmc_i2cport,qmc_addres,buf,2, 1);
    buf[0]=0x09;
    buf[1]=0X1D;
    HAL_I2C_Master_Transmit(qmc_i2cport,qmc_addres,buf,2, 1);
}

void qmc_get(axis3_t *t,float pitch,float roll){
	  uint8_t buf[6]={0};
	  HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
	  t->x=((int16_t)buf[1]<<8|buf[0]) ;
	  t->y=((int16_t)buf[3]<<8|buf[2]);
	  t->z=((int16_t)buf[5]<<8|buf[4]);

	  /*
	  pitch  *= 0.01745f;
	  roll   *= 0.01745f;
	  float cosP = cos_approx(pitch);
	  float sinP = sin_approx(pitch);

	  float cosR = cos_approx(roll);
	  float sinR = sin_approx(roll);

      int16_t m_x = t->x*cosP - t->z*sinP;
      int16_t m_y = t->x*sinP*sinR + t->y*cosR - t->z*cosP*sinR;
	  t->x=m_x;
	  t->y=m_y;
*/
}


int qmc_get_Heading(float *heading,float pitch,float roll){
	static uint8_t count_mag=0;
	static int16_t mx,my,mz;
	static int16_t offset_mx = -230;
	static int16_t offset_my = -200;
	static int16_t offset_mz = 0;
	static float sum_=0,offset1 =0,offset2=0;
    static float heading_;
	uint8_t buf[2];
      switch(count_mag){
      	  case 0:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,2,1);
      		mx=((int16_t)buf[1]<<8|buf[0]) - offset_mx;
            count_mag++;
            return 0;

      	  case 1:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x02,1,buf,2,1);
      		my=((int16_t)buf[1]<<8|buf[0]) - offset_my;
            count_mag++;
            return 0;

      	  case 2:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x04,1,buf,2,1);
      		mz=((int16_t)buf[1]<<8|buf[0]) - offset_mz;
            count_mag++;
            return 0;

      	  case 3:
      		pitch  *= 0.01745f;
      		roll   *= 0.01745f;
      		float cosP = cos_approx(pitch);
			float sinP = sin_approx(pitch);
 
		    float cosR = cos_approx(roll);
      		float sinR = sin_approx(roll);

            float m_x = mx*cosP - mz*sinP;
            float m_y = mx*sinP*sinR + my*cosR - mz*cosP*sinR;
            offset1 = heading_;
            heading_  = atan2_approx(m_y,m_x)*180/3.1415f;
      		if(heading_<0)heading_ = 360 + heading_;
            offset2 = heading_ - offset1;
            if(offset2<-340)sum_ +=360;
            else if(offset2>340)sum_ -=360;
      		*heading = heading_ + sum_;

      		count_mag =0;
      		return 1;
      }
	  return 0;
}

int16_t max_val[] = {-32767,-32767,-32767};
int16_t min_val[] = {32767, 32767, 32767};
void magnet_sensor_calibrate(){

	int16_t mx,my,mz;
	uint8_t buf[6];
	for(int i=0;i<4000;i++){
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

}
//------------------------------------------------
