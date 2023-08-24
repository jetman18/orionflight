#include "log.h"
#include "imu.h"
#include "stm32f1xx_hal.h"
#include "maths.h"
#include "i2c.h"
#include "math.h"
#include "filter.h"
#include "../quadrotor/scheduler.h"
#include "axis.h"
#include "../quadrotor/config.h"
#include "mpu6500.h"
#include "qmc5883.h"
//#define SPI
#define ACCSMOOTH
#define OFFSET_CYCLE    1000

int16_t gyr_offs_x;
int16_t gyr_offs_y;
int16_t gyr_offs_z;
attitude_t quad_;

//IMU configuration parameters
imu_config_t config ={
  .gyro_f_cut =100,
  .acc_f_cut = 100,
  .cpl_gain = 0.0001f,
  .gyro_slew_threshold=0,
  .acc_slew_threshold=0,
  .dt = 2000, //us
  .gyr_lsb = 32.8f
};

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
// gyro read and calibrate
void gyro_read(faxis3_t *angle){
	axis3_t p;
	static float gyro_v[3];
	if(gyro_read_raw(&p)){
		return;
	}
	float RC = 1.0f / (2 *M_PIf *config.gyro_f_cut);
    float temp = (float)config.dt*(1e-06f);
	float gain_lpf =temp / (RC + temp);


	float x_  = ((float)(p.x))/config.gyr_lsb;
	float y_ =  ((float)(p.y))/config.gyr_lsb;
	float z_  = ((float)(p.z))/config.gyr_lsb;

    gyro_v[X] = gyro_v[X] + gain_lpf*(x_ - gyro_v[X]);
    gyro_v[Y] = gyro_v[Y] + gain_lpf*(y_ - gyro_v[Y]);
    gyro_v[Z] = gyro_v[Z] + gain_lpf*(z_ - gyro_v[Z]);

    angle->x = gyro_v[X];
    angle->y = gyro_v[Y];
    angle->z = gyro_v[Z];
}
static int32_t store_gyro[3];
axis3_t gyro_;
int16_t count_ = 0;
void gyro_zero_offset(){


	for(int i=0;i<OFFSET_CYCLE;i++){
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

void mpu_init(){
   mpu6500_init();
   HAL_Delay(3000);
   gyro_zero_offset();
}

static void rotateB2E(faxis3_t *vector,faxis3_t delta)
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


static faxis3_t accSmooth;
axis3_t gyro,acce;
static axis3_t temp;
void imu_update(){
	qmc_read_raw(&temp);
	//gyro_read(&gyro);

	/*
    static faxis3_t vect = {0,0,1};
    static float k_ = 1;
	static faxis3_t gyro,acc;
    float Dt = config.dt*(1e-06f);
	uint32_t sum;
	float length; 
    gyro_read(&gyro);
    quad_.pitch_velocity = gyro.y;
	quad_.roll_velocity  = gyro.x;
	quad_.yaw_velocity   = gyro.z;
	rotateB2E(&vect,gyro);
    acc_read_raw(&acce);
    quad_.acc_x = acce.x;
    quad_.acc_y = acce.y;
    quad_.acc_z = acce.z;
	sum = acce.x*acce.x + acce.y*acce.y + acce.z*acce.z;
	if(sum == 0){
		return;
	}
	length = invSqrt_((float)sum);
    acc.x = acce.x*length;
    acc.y = acce.y*length;
    acc.z = acce.z*length;

    
	float RC = 1.0f / (2 *M_PIf *config.acc_f_cut);
	float gain_lpf =Dt / (RC + Dt);
	
    accSmooth.x = accSmooth.x + gain_lpf*(acc.x - accSmooth.x);
	accSmooth.y = accSmooth.y + gain_lpf*(acc.y - accSmooth.y);
	accSmooth.z = accSmooth.z + gain_lpf*(acc.z - accSmooth.z);

	if(k_>config.cpl_gain)k_ = k_ - k_*0.02f;
    vect.x = vect.x + k_*(acc.x - vect.x);
    vect.y = vect.y + k_*(acc.y - vect.y);
    vect.z = vect.z + k_*(acc.z - vect.z);
    

	float roll_   = atan2_approx(vect.y,-vect.z)*180/M_PIf;
	float pitch_  = atan2_approx(-vect.x, (1/invSqrt_(vect.y * vect.y + vect.z * vect.z)))*180/M_PIf;
	quad_.roll   = roll_  - 0.5f;
    quad_.pitch  = pitch_ + 1.0f;
    quad_.yaw   += gyro.z*config.dt*(1e-06f);
    */

}
