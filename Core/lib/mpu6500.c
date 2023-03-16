#include "mpu6500.h"
#include "stm32f1xx_hal.h"
#include "maths.h"
#include "i2c.h"
#include "math.h"
#include "lpf.h"
#include "../flymode/quadrotor/config.h"
#include "spi.h"
#include "timeclock.h"
#include "debug.h"
#include "axis.h"
/*hel*/
const float gain_cp =0.0002f;
const float f_cut = 200.0f;
#define LSB_gyr  131.0f
//#define ACCSMOOTH

static int16_t gyr_offs_x, gyr_offs_y, gyr_offs_z;
static float acc_vect_offs_x, acc_vect_offs_y, acc_vect_offs_z;
static SPI_HandleTypeDef mpu_spi_port;
static I2C_HandleTypeDef mpu_i2cport;
static GPIO_TypeDef *mpu_gpio_port = NULL;
static uint16_t mpu_cs_pin;

static faxis3_t vect = {0,0,1}; // x y z
static const uint8_t mpu_address =(0x68<<1);

static void normalizeV(axis3_t *src)
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

static int mpu_read_gyro(axis3_t *k){
	  axis3_t p_val =*k;
	  uint8_t buffe[6];
	  buffe[0]= 0x43;// gyro address
	  buffe[0] |=0x80;
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&mpu_spi_port,&buffe[0],1,100);
	  HAL_SPI_Receive(&mpu_spi_port,buffe,6,100);
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_SET);

	  k->x=(int16_t)buffe[0]<<8|buffe[1];
	  k->y=(int16_t)buffe[2]<<8|buffe[3];
	  k->z=(int16_t)buffe[4]<<8|buffe[5];

	  if((p_val.x == k->x)&&(p_val.y == k->y)&&(p_val.z == k->z)){
		return 1;
	  }
	  return 0;

	}


//get acc raw value
static int mpu_read_acc(axis3_t *k){
	axis3_t p_val =*k;
	  uint8_t buffe[6];
	  buffe[0] = 0x3b;// acc address
	  buffe[0] |=0x80;
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&mpu_spi_port,buffe,1,100);
	  HAL_SPI_Receive(&mpu_spi_port,buffe,6,100);
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_SET);

	  k->x=(int16_t)buffe[0]<<8|buffe[1];
	  k->y=(int16_t)buffe[2]<<8|buffe[3];
	  k->z=(int16_t)buffe[4]<<8|buffe[5];

      if((p_val.x == k->x)&&(p_val.y == k->y)&&(p_val.z == k->z)){
		return 1;
	  }
	  return 0;
}

static void get_offset(){
	static int32_t contan_gyro[3];
	static int32_t contan_acc[3];
	axis3_t gyro_;
	axis3_t acc_;
	uint16_t k1 = 0;
	uint16_t k2 = 0;

	for(int i=0;i<1000;i++){
		// gyro
		if(!mpu_read_gyro(&gyro_)){
			k1++;
			contan_gyro[X] += gyro_.x;
	    	contan_gyro[Y] += gyro_.y;
	    	contan_gyro[Z] += gyro_.z;
		}
        // acc
		if(!mpu_read_acc(&acc_)){
			k2++;
			contan_acc[X] += acc_.x;
	        contan_acc[Y] += acc_.y;
	        contan_acc[Z] += acc_.z;
		}
	    HAL_Delay(1);
	}

    if(k1 != 0){
      gyr_offs_x = contan_gyro[X]/k1;
      gyr_offs_y = contan_gyro[Y]/k1;
      gyr_offs_z = contan_gyro[Z]/k1;
    }
	if(k2 != 0){
      contan_acc[X] = contan_acc[X]/k2;
      contan_acc[Y] = contan_acc[Y]/k2;
      contan_acc[Z] = contan_acc[Z]/k2;

	  uint32_t sum =contan_acc[X]*contan_acc[X] +contan_acc[Y]*contan_acc[Y] + contan_acc[Z]*contan_acc[Z];
      float length = sqrtf((float)sum);
		if (length != 0.0f) {
			acc_vect_offs_x = contan_acc[X]/length;
			acc_vect_offs_y = contan_acc[Y]/length;
			acc_vect_offs_z = contan_acc[Z]/length;

			vect.x = acc_vect_offs_x;
			vect.y = acc_vect_offs_y;
			vect.z = acc_vect_offs_z;
		}
    }

    //fail to read mpu
    if((k1 == 0) && (k2 == 0)){
    	while(1){
    		HAL_Delay(1000);
			debug_clear();
			debugLog_str("mpu fail to calibrate !",5,5);
			debug_updateScreen();
    	}
    }
}

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
	/*get offset values*/
	get_offset();

}

static void gyro_read(faxis3_t *angle,uint16_t dt){
	float lsb2rad = dt*(float)(1e-07f)*0.00013323f;
	axis3_t p;
	if(mpu_read_gyro(&p)){
		return ;
	};
    angle->x  = (float)(p.x - gyr_offs_x)*lsb2rad;
    angle->y  = (float)(p.y - gyr_offs_y)*lsb2rad;
    angle->z  = (float)(p.z - gyr_offs_z)*lsb2rad;
}

static void rotateV(faxis3_t *vector,faxis3_t delta)
{
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;
	faxis3_t vec;

    cosx = cos_approx(delta.x);
    sinx = sin_approx(delta.x);
    cosy = cos_approx(delta.y);
    siny = sin_approx(delta.y);
    cosz = cos_approx(delta.z);
    sinz = sin_approx(delta.z);

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
void get_AccAngle(euler_angle_t *m){
	axis3_t  acce;
	faxis3_t acc;
	uint32_t sum;
	float length;

    mpu_read_acc(&acce);
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
void mpu_update(euler_angle_t *m,uint16_t dt){
	faxis3_t gyro,acc;
	static faxis3_t accSmooth;
	axis3_t  acce;
	uint32_t sum;
	float length;

    gyro_read(&gyro,dt);
	rotateV(&vect,gyro);
    mpu_read_acc(&acce);
	sum = acce.x*acce.x + acce.y*acce.y + acce.z*acce.z;
	if(sum == 0){
		return;
	}
	length = invSqrt_((float)sum);
    acc.x = acce.x*length;
    acc.y = acce.y*length;
    acc.z = acce.z*length;

#ifdef ACCSMOOTH
	float RC = 1.0f / (2 *M_PIf * f_cut);
	float gain_lpf =(float)dt*0.000001f / (RC + dt*0.000001f);
	
    accSmooth.x += gain_lpf*(acc.x-accSmooth.x);
	accSmooth.y += gain_lpf*(acc.y-accSmooth.y);
	accSmooth.z += gain_lpf*(acc.z-accSmooth.z);
#else
    accSmooth.x = acc.x;
	accSmooth.y = acc.y;
	accSmooth.z = acc.z;
#endif

    /* Apply complimentary filter */
    vect.x +=gain_cp*(accSmooth.x - vect.x);
    vect.y +=gain_cp*(accSmooth.y - vect.y);
    vect.z +=gain_cp*(accSmooth.z - vect.z);

	/*calculate angles*/
	m->pitch  = atan2_approx(vect.y,vect.z)*1800/M_PIf;
    m->roll   = atan2_approx(-vect.x, (1/invSqrt_(vect.y * vect.y + vect.z * vect.z)))*1800/M_PIf;
    m->yaw    = gyro.z*10;

}

