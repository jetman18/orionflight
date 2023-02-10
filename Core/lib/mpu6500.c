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
#define LSB_gyr  131.0f
#define ACCSMOOTH

/*
#define GAIN 0.0005f  //0.0005
#define YAW_GAIN 0.01f  //0.0005
#define DEFAULT_SAMPLE_FREQ	250.0f	// sample frequency in Hz
#define twoKpDef	(2.0f * 15.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
#define ACC_FEQ_CUT  1000  //hz

float twoKp = twoKpDef;	// 2 * proportional gain (Kp)
float twoKi = twoKiDef;	// 2 * integral gain (Ki)
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;
float anglesComputed = 0.0f;
float invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;
*/

int16_t gyr_offs_x, gyr_offs_y, gyr_offs_z;
float acc_vect_offs_x, acc_vect_offs_y, acc_vect_offs_z;
SPI_HandleTypeDef mpu_spi_port;
I2C_HandleTypeDef mpu_i2cport;
GPIO_TypeDef *mpu_gpio_port = NULL;
uint16_t mpu_cs_pin;

faxis3_t vect = {0,0,0}; // x y z
const uint8_t mpu_address =(0x68<<1);


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
#ifdef MPU_VIA_I2C
	  uint8_t buffe[6];
	  buffe[0]= 0x43;// gyro address
	  HAL_I2C_Master_Transmit(&mpu_i2cport,mpu_address,buffe,1,1);
	  HAL_I2C_Master_Receive(&mpu_i2cport,mpu_address,buffe,6,1);

	  k->x=(int16_t)(buffe[0]<<8)|buffe[1];
	  k->y=(int16_t)(buffe[2]<<8)|buffe[3];
	  k->z=(int16_t)(buffe[4]<<8)|buffe[5];
#endif
#ifdef MPU_VIA_SPI
	  uint8_t buffe[6];
	  buffe[0]= 0x43;// gyro address
	  buffe[0] |=0x80;
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&mpu_spi_port,&buffe[0],1,100);
	  HAL_SPI_Receive(&mpu_spi_port,buffe,6,100);
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_SET);

	  k->x=(int16_t)(buffe[0]<<8)|buffe[1];
	  k->y=(int16_t)(buffe[2]<<8)|buffe[3];
	  k->z=(int16_t)(buffe[4]<<8)|buffe[5];
#endif
      if((p_val.x == k->x)&&(p_val.y == k->y)&&(p_val.z == k->z)){
		return 1;
	  }
	  return 0;

	}


//get acc raw value
static int mpu_read_acc(axis3_t *k){
	axis3_t p_val =*k;
#ifdef MPU_VIA_I2C
	uint8_t buffe[6];
	  buffe[0] = 0x3b;// acc address
	  HAL_I2C_Master_Transmit(&mpu_i2cport,mpu_address,buffe,1,1);
	  HAL_I2C_Master_Receive(&mpu_i2cport,mpu_address,buffe,6,1);

	  k->x=(int16_t)(buffe[0]<<8)|buffe[1];
	  k->y=(int16_t)(buffe[2]<<8)|buffe[3];
	  k->z=(int16_t)(buffe[4]<<8)|buffe[5];
#endif
#ifdef MPU_VIA_SPI
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
#endif
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
	int16_t k1 = 0;
	int16_t k2 = 0;

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

	  int32_t sum =contan_acc[X]*contan_acc[X] +contan_acc[Y]*contan_acc[Y] + contan_acc[Z]*contan_acc[Z];
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
    		HAL_Delay(100);
    		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	}
    }
}

void MPU_i2c_init(I2C_HandleTypeDef *i2c){
	mpu_i2cport = *i2c;
    uint8_t buffer[6];
    buffer[0] = 0x6B;
	buffer[1] = 0x00;

	HAL_I2C_Master_Transmit(&mpu_i2cport,mpu_address,buffer,2,1);
	// Configure gyro(500dps full scale)
	buffer[0] = 0x1B;
	buffer[1] = 0x08;
	HAL_I2C_Master_Transmit(&mpu_i2cport,mpu_address,buffer,2,1);
	// Configure accelerometer(+/- 8g)
	buffer[0] = 0x1C;
	buffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&mpu_i2cport,mpu_address,buffer,2,1);
	get_offset();
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
	float lsb2rad = dt*0.000001f*0.00013323f;
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
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void mpu_update(euler_angle_t *m,uint16_t dt){
    const float gain_cp =0.999f;
	const float f_cut = 100.0f;

	float RC = 1.0f / (2 *M_PIf * f_cut);
	float gain_lpf =(float)dt*0.000001f / (RC + dt*0.000001f);

	faxis3_t gyro,acc;
	static faxis3_t accSmooth;
	axis3_t  acce;
	int32_t sum;
	float length;

    gyro_read(&gyro,dt);
	rotateV(&vect,gyro);
    mpu_read_acc(&acce);
	sum = acce.x*acce.x + acce.y*acce.y + acce.z*acce.z;
    length = invSqrt_((float)sum);
    acc.x = acce.x*length;
    acc.y = acce.y*length;
    acc.z = acce.z*length;

#ifdef ACCSMOOTH
    accSmooth.x = accSmooth.x*gain_lpf + (1-gain_lpf)*acc.x;
	accSmooth.y = accSmooth.y*gain_lpf + (1-gain_lpf)*acc.y;
	accSmooth.z = accSmooth.z*gain_lpf + (1-gain_lpf)*acc.z;
#else
    accSmooth.x = acc.x;
	accSmooth.y = acc.y;
	accSmooth.z = acc.z;
#endif
    /* Apply complimentary filter */
    vect.x = vect.x*gain_cp + (1-gain_cp)*accSmooth.x;
    vect.y = vect.y*gain_cp + (1-gain_cp)*accSmooth.y;
    vect.z = vect.z*gain_cp + (1-gain_cp)*accSmooth.z;

    vect.x = constrainf(vect.x,-1.0f,1.0f);
    vect.y = constrainf(vect.y,-1.0f,1.0f);
    vect.z = constrainf(vect.z,-1.0f,1.0f);

	/*calculate angles*/
	m->pitch  = atan2_approx(vect.y,vect.z)*180/M_PIf;
    m->roll   = atan2_approx(-vect.x, sqrtf(vect.y * vect.y + vect.z * vect.z))*180/M_PIf;
    m->yaw    = gyro.z;

}

/*
void computeAnglesFromQuaternion(euler_angle_t *m)
{
	m->roll = 57.29577*atan2_approx(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	m->pitch =  57.29577*asinf(-2.0f * (q1*q3 - q0*q2));
	m->yaw =  57.29577*atan2_approx(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	anglesComputed = 1;
}


void updateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt_(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * invSampleFreq;
			integralFBy += twoKi * halfey * invSampleFreq;
			integralFBz += twoKi * halfez * invSampleFreq;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * invSampleFreq);		// pre-multiply common factors
	gy *= (0.5f * invSampleFreq);
	gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt_(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid
	// (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		updateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt_(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt_(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * invSampleFreq;
			integralFBy += twoKi * halfey * invSampleFreq;
			integralFBz += twoKi * halfez * invSampleFreq;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * invSampleFreq);		// pre-multiply common factors
	gy *= (0.5f * invSampleFreq);
	gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt_(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}
*/
