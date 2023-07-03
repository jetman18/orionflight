#include <log.h>
#include "imu.h"
#include "stm32f1xx_hal.h"
#include "maths.h"
#include "i2c.h"
#include "math.h"
#include "filter.h"
#include "scheduler.h"
#include "axis.h"
#include "../quadrotor/config.h"
#define I2C
//#define SPI
#define ACCSMOOTH
#define GYRO_DATA_REG 0x43
#define ACC_DATA_REG  0x3b
#define IMU_DEV_REG   0x68
#define RESET_REG     0x00


static int16_t  acc__[3];
static int16_t gyr_offs_x = 45;
static int16_t gyr_offs_y = 121;
static int16_t gyr_offs_z =-13;
static I2C_HandleTypeDef *mpu_i2cport;
static faxis3_t vect = {0,0,1};
static float k_ = 1;
imu_config_t config;
attitude_t quad_;
float acc_vect_offs_x, acc_vect_offs_y, acc_vect_offs_z;

static void pre_config()
{
	config.dt = 2000; //us
	config.acc_f_cut = 100;
	config.gyro_f_cut =100;
    config.acc_slew_threshold=0;//
	config.gyro_slew_threshold=0;// deg/sec
	config.cpl_gain = 0.0001f;
	config.imu_adrr = (0x68<<1);
	config.offset_cycle =1000;
	config.imu_acc_data_res = ACC_DATA_REG;
	config.imu_gyro_data_res = GYRO_DATA_REG;
    config.imu_gyro_regsiter_config = 0x1b;
	config.imu_acc_regsiter_config = 0x1c;
	config.imu_acc_res_cgf_val  = (ACC_4G<<3);
	config.imu_gyro_res_cgf_val = (GYRO_1000<<3);
	config.imu_gyro_Sensitivity_Scale_Factor = 32.8f;
}
void normalizeV(axis3_t *src)
{
    int32_t sum;
	sum = src->x*src->x + src->y*src->y + src->z*src->z;
    int32_t length = sqrt(sum);
    if (length != 0) {
        src->x = src->x / length;
        src->y = src->y / length;
        src->z = src->z / length;
    }
}

static int gyro_read_raw(axis3_t *k){
	  axis3_t p_val =*k;
	  uint8_t buffe[6];
	  buffe[0]=config.imu_gyro_data_res;
#ifdef I2C
	  HAL_I2C_Master_Transmit(mpu_i2cport,config.imu_adrr,buffe,1,1);
	  HAL_I2C_Master_Receive(mpu_i2cport,config.imu_adrr,buffe,6,1);
#endif
#ifdef SPI
	  buffe[0] |=0x80;
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&mpu_spi_port,&buffe[0],1,100);
	  HAL_SPI_Receive(&mpu_spi_port,buffe,6,100);
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_SET);
#endif

	  k->x = (int16_t)buffe[0]<<8|buffe[1];
	  k->y = (int16_t)buffe[2]<<8|buffe[3];
	  k->z = (int16_t)buffe[4]<<8|buffe[5];

	  if((p_val.x == k->x)&&(p_val.y == k->y)&&(p_val.z == k->z)){
		return 1;
	  }
	  return 0;
	}

static int acc_read_raw(axis3_t *k){
	axis3_t p_val =*k;
	uint8_t buffe[6];
	buffe[0] =config.imu_acc_data_res;// acc address
#ifdef I2C
	  HAL_I2C_Master_Transmit(mpu_i2cport,config.imu_adrr,buffe,1,100);
	  HAL_I2C_Master_Receive(mpu_i2cport,config.imu_adrr,buffe,6,100);
#endif
#ifdef SPI
	  buffe[0] |=0x80;
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&mpu_spi_port,buffe,1,100);
	  HAL_SPI_Receive(&mpu_spi_port,buffe,6,100);
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_SET);
#endif

	  acc__[X] = (int16_t)buffe[0]<<8|buffe[1];
	  acc__[Y] = (int16_t)buffe[2]<<8|buffe[3];
	  acc__[Z] = (int16_t)buffe[4]<<8|buffe[5];

	  k->x = acc__[X];
	  k->y = acc__[Y];
	  k->z = acc__[Z];
      if((p_val.x == k->x)&&(p_val.y == k->y)&&(p_val.z == k->z)){
		return 1;
	  }
	  return 0;
}

static void gyro_zero_offset(){
	static int32_t store_gyro[3];
	axis3_t gyro_;
	uint16_t count_ = 0;

	for(int i=0;i<config.offset_cycle;i++){
		if(!gyro_read_raw(&gyro_)){
			count_++;
			store_gyro[X] += gyro_.x;
	    	store_gyro[Y] += gyro_.y;
	    	store_gyro[Z] += gyro_.z;
		}
		delay_ms(1);
	}

    if(count_ != 0){
      gyr_offs_x = store_gyro[X]/count_;
      gyr_offs_y = store_gyro[Y]/count_;
      gyr_offs_z = store_gyro[Z]/count_;
    }
}
#ifdef SPI
void MPU_spi_init(SPI_HandleTypeDef *spiportt,GPIO_TypeDef  *gpio_port,uint16_t pin){
	mpu_gpio_port = gpio_port;
	mpu_spi_port = *spiportt;
	mpu_cs_pin = pin;

    uint8_t data[2];
	data[0]=0x6b;
	data[1]=0x00;
	HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&mpu_spi_port,data,2,100);
	HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_SET);

	data[0]=0x1b;
	data[1]=0x00;
	HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&mpu_spi_port,data,2,100);
	HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_SET);

	data[0]=0x1c;
	data[1]=0x00;
	HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&mpu_spi_port,data,2,100);
	HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_SET);
	///get offset values
	get_offset();
}
#endif

void MPU_i2c_init(I2C_HandleTypeDef *i2cport)
{
	uint8_t buffer[6];
	mpu_i2cport = i2cport;
    pre_config();
    buffer[0] = 0x6b; 
  	buffer[1] =  RESET_REG;
  	HAL_I2C_Master_Transmit(mpu_i2cport,config.imu_adrr,buffer,2,1);

  	buffer[0] = config.imu_gyro_regsiter_config; 
  	buffer[1] = config.imu_gyro_res_cgf_val; 
  	HAL_I2C_Master_Transmit(mpu_i2cport,config.imu_adrr,buffer,2,1);

  	buffer[0] = config.imu_acc_regsiter_config; 
  	buffer[1] = config.imu_acc_res_cgf_val;  
  	HAL_I2C_Master_Transmit(mpu_i2cport,config.imu_adrr,buffer,2,1);
  	HAL_Delay(3000);
  	gyro_zero_offset();
}

static void gyro_read(faxis3_t *angle){
	axis3_t p;
	static float gyro_v[3];
	if(gyro_read_raw(&p)){
		return;
	}
	float RC = 1.0f / (2 *M_PIf *config.gyro_f_cut);
    float temp = (float)config.dt*(1e-06f);
	float gain_lpf =temp / (RC + temp);

	float x_  = ((float)(p.x - gyr_offs_x))/config.imu_gyro_Sensitivity_Scale_Factor;
	float y_ =  ((float)(p.y - gyr_offs_y))/config.imu_gyro_Sensitivity_Scale_Factor;
	float z_  = ((float)(p.z - gyr_offs_z))/config.imu_gyro_Sensitivity_Scale_Factor;

    gyro_v[X] = gyro_v[X] + gain_lpf*(x_ - gyro_v[X]);
    gyro_v[Y] = gyro_v[Y] + gain_lpf*(y_ - gyro_v[Y]);
    gyro_v[Z] = gyro_v[Z] + gain_lpf*(z_ - gyro_v[Z]);

    angle->x = gyro_v[X];
    angle->y = gyro_v[Y];
    angle->z = gyro_v[Z];

}

static void rotateBody2Earth(faxis3_t *vector,faxis3_t delta)
{
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;
	faxis3_t vec;
    
	float temp = config.dt*(float)(1e-06f)*RAD;
	float angleX = delta.x*temp;
	float angleY = delta.y*temp;
	float angleZ = delta.z*temp;

    cosx = cos_approx(angleX);
    sinx = sin_approx(angleX);
    cosy = cos_approx(angleY);
    siny = sin_approx(angleY);
    cosz = cos_approx(angleZ);
    sinz = sin_approx(angleZ);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    vec.x = vector->x * mat[0][0] + vector->y * mat[1][0] + vector->z * mat[2][0];
    vec.y = vector->x * mat[0][1] + vector->y * mat[1][1] + vector->z * mat[2][1];
    vec.z = vector->x * mat[0][2] + vector->y * mat[1][2] + vector->z * mat[2][2];
   
    vector->x =  vec.x;
	vector->y =  vec.y;
	vector->z =  vec.z;

}
static float invSqrt_(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	//y = y * (1.5f - (halfx * y * y));
	return y;
}
void get_Acc_Angle(attitude_t *m){
	axis3_t  acce;
	faxis3_t acc;
	uint32_t sum;
	float length;
    acc_read_raw(&acce);
	sum = acce.x*acce.x + acce.y*acce.y + acce.z*acce.z;
	if(sum == 0){
		return;
	}
	length = invSqrt_((float)sum);
    acc.x = acce.x*length;
    acc.y = acce.y*length;
    acc.z = acce.z*length;
	m->pitch  = atan2_approx(acc.y,acc.z)*180/M_PIf;
	m->roll   = atan2_approx(-acc.x, (1/invSqrt_(acc.y * acc.y + acc.z * acc.z)))*180/M_PIf;
}


static axis3_t  acce;
static faxis3_t accSmooth;
void imu_update(){
	static faxis3_t gyro,acc;
	uint32_t sum;
	float length;
    gyro_read(&gyro);
    quad_.pitch_velocity = -gyro.y;
	quad_.roll_velocity  = gyro.x;
	quad_.yaw_velocity   = gyro.z;
	rotateBody2Earth(&vect,gyro);
    acc_read_raw(&acce);
    quad_.raw_acc_x = acce.x;
    quad_.raw_acc_y = acce.y;
    quad_.raw_acc_z = acce.z;
	sum = acce.x*acce.x + acce.y*acce.y + acce.z*acce.z;
	if(sum == 0){
		return;
	}
	length = invSqrt_((float)sum);
    acc.x = acce.x*length;
    acc.y = acce.y*length;
    acc.z = acce.z*length;

	float RC = 1.0f / (2 *M_PIf *config.acc_f_cut);
	float gain_lpf =(float)config.dt*(1e-06f) / (RC + config.dt*(1e-06f));
	
    accSmooth.x = accSmooth.x + gain_lpf*(acc.x - accSmooth.x);
	accSmooth.y = accSmooth.y + gain_lpf*(acc.y - accSmooth.y);
	accSmooth.z = accSmooth.z + gain_lpf*(acc.z - accSmooth.z);

    //complimentary filter
	if(k_>config.cpl_gain)k_ = k_ - k_*0.02f;
    vect.x = vect.x + k_*(acc.x - vect.x);
    vect.y = vect.y + k_*(acc.y - vect.y);
    vect.z = vect.z + k_*(acc.z - vect.z);

    float trueRoll  = atan2_approx(accSmooth.y,-accSmooth.z)*180/M_PIf;
    float truePitch = atan2_approx(-accSmooth.x, (1/invSqrt_(accSmooth.y*accSmooth.y + accSmooth.z*accSmooth.z)))*180/M_PIf;

	float roll_   = atan2_approx(vect.y,-vect.z)*180/M_PIf;
	float pitch_  = atan2_approx(-vect.x, (1/invSqrt_(vect.y * vect.y + vect.z * vect.z)))*180/M_PIf;
	quad_.true_roll  = trueRoll;
	quad_.true_pitch = truePitch ;
	quad_.roll   = roll_  - 0.5f;
    quad_.pitch  = pitch_ + 1.0f;
    quad_.yaw   += gyro.z*config.dt*(1e-06f);
    quad_.error_pitch =  truePitch - pitch_;
	quad_.error_roll =   trueRoll - roll_;
}

