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
void qmc_get_raw(MAG_t *t){
	uint8_t buf[6];
	HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
	t->mx=((int16_t)buf[1]<<8|buf[0]) - 61875;
	t->my=((int16_t)buf[3]<<8|buf[2]) - 54970;
	t->mz=((int16_t)buf[5]<<8|buf[4]) - 54775;

	//t->compas=atan2_approx(t->mx,t->my)*180.0f/3.1415f;
	//if(t->compas<0)t->compas=360.0f + t->compas;
}
/*
void qmc_get_3axil_values(faxis3_t *t,float pitch,float roll){
	  uint8_t buf[6]={0};
	  float mx,my,mz;
	  float x,y,z;
	  HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
	  mx=((int16_t)buf[1]<<8|buf[0]) - 61000 - 875;
	  my=((int16_t)buf[3]<<8|buf[2]) - 55000 + 25;
	  mz=((int16_t)buf[5]<<8|buf[4]) - 54000 - 775;


	  mx -=combined_bias[0];
	  my -=combined_bias[1];
	  mz -=combined_bias[2];

	  x = mx*clMatix[0] + my*clMatix[1] + mz*clMatix[2];
	  y = mx*clMatix[3] + my*clMatix[4] + mz*clMatix[5];
	  z = mx*clMatix[6] + my*clMatix[7] + mz*clMatix[8];

     // pitch*=0.0174f;
     // roll*=0.0174f;

     t->x = x;//*cos(pitch) - mz*sin(pitch);
     t->y = y;//*sin(pitch)*cos(roll) + my*cos(roll) - mz*cos(pitch)*sin(roll);
     t->z = z;
}
*/
void qmc_get_3axil_values(faxis3_t *t,float pitch,float roll){
	  uint8_t buf[6]={0};
	  int16_t mx,my,mz;
	  HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
	  mx=((int16_t)buf[1]<<8|buf[0]) - 61000 - 875;
	  my=((int16_t)buf[3]<<8|buf[2]) - 55000 + 25;
	  mz=((int16_t)buf[5]<<8|buf[4]) - 54000 - 775;

	  mz*=1.1;

      pitch*=0.0174f;
      roll*=0.0174f;

      t->x = mx*cos(pitch) - mz*sin(pitch);
      t->y = mx*sin(pitch)*sin(roll) + my*cos(roll) - mz*cos(pitch)*sin(roll);
      t->z = mz;
}
/*
void qmc_get_values(MAG_t *t,float pitch,float roll){
      static uint8_t count_mag=0;
	  static int16_t mx,my,mz;
	  uint8_t buf[2];

      switch(count_mag){
      	  case 0:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,2,1);
      		mx=((int16_t)buf[1]<<8|buf[0]) - 61875;
            count_mag++;
      		break;

      	  case 1:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x02,1,buf,2,1);
      		my=((int16_t)buf[1]<<8|buf[0]) - 54970;
            count_mag++;
      		break;

      	  case 2:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x04,1,buf,2,1);
      		mz=((int16_t)buf[1]<<8|buf[0]) - 54775 ;
            count_mag++;
      		break;

      	  case 3:
      		pitch  *= 0.01745f;
      		roll  *= 0.01745f;

      		float cosP = cos_approx(pitch);
      		float cosR = cos_approx(roll);
      		float sinP = sin_approx(pitch);
      		float sinR = sin_approx(roll);

            t->mx = mx*cosP - mz*sinP;
            t->my = mx*sinP*sinR + my*cosR - mz*cosP*sinR;
            t->mz = mz;
			count_mag++;
			break;

            case 4:
      		t->compas=atan2_approx((float)t->my,(float)t->mx)*180.0f/3.1415f;
      		if(t->compas<0)t->compas=360.0f + t->compas;
      		count_mag =0;
      		break;
      }

}
*/


int16_t qmc_get_Heading(float pitch,float roll){
	static uint8_t count_mag=0;
	static int16_t mx,my,mz;
	static int16_t heading;//0-3600
	static int16_t offset_mx = 61875;
	static int16_t offset_my = 54970;
	static int16_t offset_mz = 54775;

	uint8_t buf[2];
      switch(count_mag){
      	  case 0:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,2,1);
      		mx=((int16_t)buf[1]<<8|buf[0]) - offset_mx;
            count_mag++;
      		break;
      	  case 1:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x02,1,buf,2,1);
      		my=((int16_t)buf[1]<<8|buf[0]) - offset_my;
            count_mag++;
      		break;
      	  case 2:
      		HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x04,1,buf,2,1);
      		mz=((int16_t)buf[1]<<8|buf[0]) - offset_mz;
            count_mag++;
      		break;
      	  case 3:
      		pitch  *= 0.01745f;
      		roll   *= 0.01745f;
      		float cosP = cos_approx(pitch);
			float sinP = sin_approx(pitch);
 
		    float cosR = cos_approx(roll);
      		float sinR = sin_approx(roll);

            int16_t m_x = mx*cosP - mz*sinP;
            int16_t m_y = mx*sinP*sinR + my*cosR - mz*cosP*sinR;
			mx=m_x;
			my=m_y;

			count_mag++;
			break;

          case 4:
      		heading = atan2_approx((float)my,(float)mx)*1800/3.1415f;
      		if(heading<0)heading=3600 + heading;
      		count_mag =0;
      		break;
      }
	  return heading;
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
