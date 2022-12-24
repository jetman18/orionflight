#include "sensor.h"
#include "stm32f1xx_hal.h"
#include "maths.h"
#include "i2c.h"
#include "math.h"
#include "lpf.h"
#include "../flymode/quadrotor/config.h"
#include "spi.h"
/*hel*/
#define LSB_gyr  65.5f
#define kalman_gain 0.001f
#define DEFAULT_SAMPLE_FREQ	333.33f	// sample frequency in Hz
#define twoKpDef	(2.0f * 15.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
#define QMC_ADDR (0x0d<<1)

/*******************************************/
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

/*  configution mpu6500  */
////////

static float acc_pitch_offset,acc_roll_offset;

static int16_t gyr_offs_x, gyr_offs_y, gyr_offs_z;
static int16_t acc_offs_x, acc_offs_y, acc_offs_z;


/**
 *  get gyro raw value
 */
void mpu_get_gyro(IMU_raw_t *k){
#ifdef MPU_VIA_I2C
	  uint8_t buffe[6];
	  buffe[0]= 0x43;// gyro address
	  HAL_I2C_Master_Transmit(I2C_PORT,0x68<<1,buffe,1,1);
	  HAL_I2C_Master_Receive(I2C_PORT,0x68<<1,buffe,6,1);

	  k->gyrox=(int16_t)(buffe[0]<<8)|buffe[1];
	  k->gyroy=(int16_t)(buffe[2]<<8)|buffe[3];
	  k->gyroz=(int16_t)(buffe[4]<<8)|buffe[5];
#endif
#ifdef MPU_VIA_SPI
	  uint8_t buffe[6];
	  buffe[0]= 0x43;// gyro address
	  buffe[0] |=0x80;
	  HAL_GPIO_WritePin(GPIO_PORT,GPIO_CS_PIN,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(SPI_PORT,&buffe[0],1,100);
	  HAL_SPI_Receive(SPI_PORT,buffe,6,100);
	  HAL_GPIO_WritePin(GPIO_PORT,GPIO_CS_PIN,GPIO_PIN_SET);

	  k->gyrox=(int16_t)(buffe[0]<<8)|buffe[1];
	  k->gyroy=(int16_t)(buffe[2]<<8)|buffe[3];
	  k->gyroz=(int16_t)(buffe[4]<<8)|buffe[5];
#endif
	}

/**
 *  get acc raw value
 */
void mpu_get_acc(IMU_raw_t *k){
#ifdef MPU_VIA_I2C
	uint8_t buffe[6];
	  buffe[0] = 0x3b;// acc address
	  HAL_I2C_Master_Transmit(I2C_PORT,0x68<<1,buffe,1,1);
	  HAL_I2C_Master_Receive(I2C_PORT,0x68<<1,buffe,6,1);

	  k->accx=(int16_t)(buffe[0]<<8)|buffe[1];
	  k->accy=(int16_t)(buffe[2]<<8)|buffe[3];
	  k->accz=(int16_t)(buffe[4]<<8)|buffe[5];
#endif
#ifdef MPU_VIA_SPI
	  uint8_t buffe[6];
	  buffe[0] = 0x3b;// acc address
	  buffe[0] |=0x80;
	  HAL_GPIO_WritePin(GPIO_PORT,GPIO_CS_PIN,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(SPI_PORT,buffe,1,100);
	  HAL_SPI_Receive(SPI_PORT,buffe,6,100);
	  HAL_GPIO_WritePin(GPIO_PORT,GPIO_CS_PIN,GPIO_PIN_SET);

	  k->accx=(int16_t)buffe[0]<<8|buffe[1];
	  k->accy=(int16_t)buffe[2]<<8|buffe[3];
	  k->accz=(int16_t)buffe[4]<<8|buffe[5];
#endif

}

static void get_offset(){
	static uint16_t k1,k2;
	float pitch_acc,roll_acc;
	IMU_raw_t data;
	static int32_t contan_gyro[3];
	static int32_t contan_acc[3];

	for(uint16_t i=0;i<2000;i++){
		
        //  acc offset
		mpu_get_acc(&data);
		if((data.accx+data.accy+data.accz)!=0)k1+=1;

        contan_acc[0] += data.accx;
        contan_acc[1] += data.accy;
        contan_acc[2] += data.accz;

		roll_acc   =-atan2_approx(data.accx,data.accz)*1/RAD;
		pitch_acc  = atan2_approx(data.accy,data.accz)*1/RAD;
		
		acc_pitch_offset += pitch_acc;
		acc_roll_offset  += roll_acc;
			
		// gyro offset		
		mpu_get_gyro(&data);
		if((data.gyrox+data.gyroy+data.gyroz)!=0)k2++;
	    contan_gyro[0] += data.gyrox;
	    contan_gyro[1] += data.gyroy;
	    contan_gyro[2] += data.gyroz;
	}

	  acc_offs_x = contan_acc[0]/k1;
      acc_offs_y = contan_acc[1]/k1;
      acc_offs_z = contan_acc[2]/k1;

	  gyr_offs_x = contan_gyro[0]/k2;
      gyr_offs_y = contan_gyro[1]/k2;
      gyr_offs_z = contan_gyro[2]/k2;

	  acc_pitch_offset /=(float)k1;
	  acc_roll_offset  /=(float)k1;
}



void MPU_init(){  
#ifdef MPU_VIA_I2C
    uint8_t buffer[6];

    buffer[0] = 0x6B;
	buffer[1] = 0x00;
	HAL_I2C_Master_Transmit(I2C_PORT,0x68<<1,buffer,2,1);
	// Configure gyro(500dps full scale)
	buffer[0] = 0x1B;
	buffer[1] = 0x08;
	HAL_I2C_Master_Transmit(I2C_PORT,0x68<<1,buffer,2,1);
	// Configure accelerometer(+/- 8g)
	buffer[0] = 0x1C;
	buffer[1] = 0x18;
	HAL_I2C_Master_Transmit(I2C_PORT,0x68<<1,buffer,2,1);
#endif
#ifdef MPU_VIA_SPI
    uint8_t data[2];

	data[0]=0x6b;
	data[1]=0x00;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,data,2,100);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

	data[0]=0x1b;
	data[1]=0x08;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,data,2,100);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

	data[0]=0x1c;
	data[1]=0x10;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,data,2,100);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);


#endif
	get_offset();
	// Finish setup MPU-6050 register
}
void MPU_update(euler_angle_t *m,int delta_t){
	static float Pitch_acc,Roll_acc;
	static IMU_raw_t p;
	float lsb2degre =(delta_t*0.000001)/LSB_gyr;
	// gyro calibrate

	mpu_get_gyro(&p);

    m->pitch += (float)(p.gyrox -gyr_offs_x)*lsb2degre;
    m->roll  += (float)(p.gyroy -gyr_offs_y)*lsb2degre;
    m->yaw   = (float)(p.gyroz -gyr_offs_z)*lsb2degre;
	
	if(m->pitch>180.0f)m->pitch  -= 360.0f;
	else if(m->pitch<-180.0f)m->pitch += 360.0f;
	
    if(m->roll>180.0f)m->roll -= 360.0f;
    else if(m->roll<-180.0f)m->roll += 360.0f;
	
	if(m->yaw>360.0f)m->yaw  -= 360.0f;
	else if(m->yaw<0.0f)m->yaw  += 360.0f;

	m->pitch += m->roll   * sin_approx((p.gyroz -gyr_offs_z)*lsb2degre*(float)RAD);
	m->roll  -= m->pitch  * sin_approx((p.gyroz -gyr_offs_z)*lsb2degre*(float)RAD);

    //  acc calibrate
	mpu_get_acc(&p);
	Roll_acc   =(-atan2_approx(p.accx,p.accz)*1/RAD - acc_roll_offset);
	Pitch_acc  = (atan2_approx(p.accy,p.accz)*1/RAD - acc_pitch_offset);
	
	m->pitch +=kalman_gain*(Pitch_acc-m->pitch);
	m->roll  +=kalman_gain*(Roll_acc-m->roll);
	//m->yaw   +=kalman_gain*(MAG_yaw-m->yaw);

    /*
	
    acc
	 xx = p.accx-acc_offs_x;
     yy = p.accy- acc_offs_y;
	 xc = xx*cos_approx(m->roll*RAD) + yy*sin_approx(m->roll*RAD)*sin_approx(m->pitch*RAD) +p.accz*sin_approx(m->roll*RAD)*cos_approx(m->pitch*RAD);
     yc = yy*cos_approx(m->pitch*RAD) - p.accz*sin_approx(m->pitch*RAD);
	 */

}

/**
 *@qmc5883l
 *@
 */
void qmc5883_init(){
    uint8_t buf[2];
    buf[0]=0x0b;
    buf[1]=0X01;
    HAL_I2C_Master_Transmit(I2C_PORT,QMC_ADDR,buf,2, 1);
    buf[0]=0x09;
    buf[1]=0X1D;
    HAL_I2C_Master_Transmit(I2C_PORT,QMC_ADDR,buf,2, 1);
}
void qmc_get_values(MAG_t *t,float pitch,float roll){
	  uint8_t datas;
	  static float mx,my;
	  static uint8_t buf[6];
	  HAL_I2C_Mem_Read(I2C_PORT, 0x1A, 0x06, 1,&datas, 1, 1);
	  if((datas && 0x01)==0x01){
			HAL_I2C_Mem_Read(I2C_PORT,QMC_ADDR,0x00,1,buf,6,1);
			t->mx=(int16_t)buf[1]<<8|(int16_t)buf[0];
			t->my=(int16_t)buf[3]<<8|(int16_t)buf[2];
			t->mz=(int16_t)buf[5]<<8|(int16_t)buf[4];
            mx = t->mx*cos_approx(-pitch) + t->my*sin_approx(-pitch)*sin_approx(roll) + t->mz*sin_approx(-pitch)*cos_approx(-roll);
	        my = t->my*(cos_approx(-roll) - t->mz*sin_approx(-roll));

	        t->compas=atan2_approx(my,mx)*180.0f/3.1415f;
			if(t->compas<0)t->compas=360.0f + t->compas;
		  }
}
void magnet_sensor_calibrate(){
	uint8_t datas;
	int16_t max_val[] = {-32768,-32768,-32768};
	int16_t min_val[] = {32768, 32768, 32768};

	for(int i=0;i<1000;i++){
		HAL_I2C_Mem_Read(I2C_PORT,0x1A, 0x06, 1,&datas, 1,1);
		if((datas&0x01)==1){
			uint8_t buf[6];
			float mx,my,mz;
			HAL_I2C_Mem_Read(I2C_PORT,QMC_ADDR,0x00,1,buf,6,1);
			mx=(int16_t)(buf[1])<<8|(int16_t)buf[0];
			my=(int16_t)(buf[3])<<8|(int16_t)buf[2];
			mz=(int16_t)(buf[5])<<8|(int16_t)buf[4];

			if(mx > max_val[0]) max_val[0] = mx;
			else if(mx < min_val[0]) min_val[0] = mx;

			if(my > max_val[1]) max_val[1] = my;
			else if(my < min_val[1]) min_val[1] = my;

			if(mz > max_val[2]) max_val[2] = mz;
			else if(mz < min_val[2]) min_val[2] = mz;
	    }
	HAL_Delay(10);
    }
}
//-------------------------------------------------------------------------------------------

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

void computeAnglesFromQuaternion(euler_angle_t *m)
{
	m->roll = atan2_approx(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	m->pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	m->yaw = atan2_approx(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
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

