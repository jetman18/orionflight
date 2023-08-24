#include "mpu6500.h"
#include "i2c.h"
#include "axis.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "gpio.h"

//#define SPI
#define I2C
/*------------- mpu6500 registers ----------------*/
#define GYRO_DATA_REG                        0x43
#define ACC_DATA_REG                         0x3b
#define IMU_DEV_REG                          0x68<<1
#define RESET_REG                            0x00
#define GYRO_REG_CONFIG                      0x1b
#define ACC_REG_CONFIG                       0x1c
#define CONFIGURATION                        0x26
#define MPU_RA_WHO_AM_I                      0x75
#define MPU6500_WHO_AM_I_CONST               0x70
#define MPU_RA_PWR_MGMT_1                    0x6B
/*------------- mpu6500 registers ----------------*/

#define SPI_MPU_GPIO_CS_PIN          GPIO_PIN_4
#define SPI_MPU_GPIO_PORT            GPIOA
#define SPI_PORT                     hspi1
#define SPI_TIME_OUT                 10 //ms
#define I2C_PORT                     hi2c2
#define I2C_TIME_OUT                 10 //ms
#define SPI_ON  (HAL_GPIO_WritePin(SPI_MPU_GPIO_PORT,SPI_MPU_GPIO_CS_PIN,GPIO_PIN_RESET))
#define SPI_OFF (HAL_GPIO_WritePin(SPI_MPU_GPIO_PORT,SPI_MPU_GPIO_CS_PIN,GPIO_PIN_SET))
//acce
typedef enum{
    ACC_2G = 0,
    ACC_4G,
    ACC_8G,
    ACC_16G
}ACC_SCALE_;
typedef enum{
    FCHOISE_OFF= 0,
	FCHOISE_ON  // maximum bandwidth 1.13khz
}ACCEL_FCHOICE_B;
typedef enum{
    ACC_LPF460HZ = 0,
    ACC_LPF184HZ,
    ACC_LPF92HZ,
    ACC_LPF41HZ,
	ACC_LPF20HZ,
	ACC_LPF10HZ,
	ACC_LPF5HZ,
}A_DLPF_CFG;

typedef enum{
    HZ_250 = 0,
    HZ_184,
    HZ_92,
    HZ_41,
	HZ_20,
	HZ_10,
	HZ_5,
	HZ_3600
}DLPF_CFG;

// gyro
typedef enum{ 
	Fchoise0 = 0,  // use lpf -> DLPF_CFG
    Fchoise1,      // not use lpf ; Bandwidth 8800 hz
    Fchoise2       // not use lpf ; Bandwidth 3600 hz
}FCHOICE_B;
typedef enum{
    GYRO_250dps = 0,
    GYRO_500dps,
    GYRO_1000dps,
    GYRO_2000dps
}GYRO_SCALE_;

static void SPI_write(uint8_t *data,uint8_t len);
static void SPI_read(uint8_t reg_addr,uint8_t *data,uint8_t len);
static void I2C_write(uint8_t *data,uint8_t len);
static void I2C_read(uint8_t *data,uint8_t len);

int gyro_read_raw(axis3_t *k){
	  axis3_t p_val = *k;
	  uint8_t buffe[6];
	  buffe[0] = (uint8_t)GYRO_DATA_REG;
#ifdef I2C
	  HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,buffe,1,1);
	  HAL_I2C_Master_Receive(&I2C_PORT,IMU_DEV_REG,buffe,6,1);
#endif
#ifdef SPI
	  buffe[0] |= (uint8_t)0x80;
	  HAL_GPIO_WritePin(SPI_MPU_GPIO_PORT,SPI_MPU_GPIO_CS_PIN,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&SPI_PORT,&buffe[0],1,1);
	  HAL_SPI_Receive(&SPI_PORT,buffe,6,1);
	  HAL_GPIO_WritePin(SPI_MPU_GPIO_PORT,SPI_MPU_GPIO_CS_PIN,GPIO_PIN_SET);
#endif

	  k->x = (int16_t)buffe[0]<<8|buffe[1];
	  k->y = (int16_t)buffe[2]<<8|buffe[3];
	  k->z = (int16_t)buffe[4]<<8|buffe[5];

	  if((p_val.x == k->x)&&(p_val.y == k->y)&&(p_val.z == k->z)){
		return 1;
	  }
	  return 0;
	}

int acc_read_raw(axis3_t *k){
	axis3_t p_val =*k;
	uint8_t buffe[6];
	buffe[0] = (uint8_t)ACC_DATA_REG;
#ifdef I2C
	HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,buffe,1,1);
    HAL_I2C_Master_Receive(&I2C_PORT,IMU_DEV_REG,buffe,6,1);
#endif
#ifdef SPI
/*
	  buffe[0] |= (uint8_t)0x80;
	  HAL_GPIO_WritePin(SPI_MPU_GPIO_PORT,SPI_MPU_GPIO_CS_PIN,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&SPI_PORT,&buffe[0],1,1);
	  HAL_SPI_Receive(&SPI_PORT,buffe,6,1);
	  HAL_GPIO_WritePin(SPI_MPU_GPIO_PORT,SPI_MPU_GPIO_CS_PIN,GPIO_PIN_SET);
*/
      SPI_read(buffe[0],buffe,14);
#endif
	  k->x = (int16_t)buffe[0]<<8|buffe[1];
	  k->y = (int16_t)buffe[2]<<8|buffe[3];
	  k->z = (int16_t)buffe[4]<<8|buffe[5];

      if((p_val.x == k->x)&&(p_val.y == k->y)&&(p_val.z == k->z)){
		return 1;
	  }
	  return 0;
}
void mpu_read_all(axis3_t *acc, axis3_t *gyr){
	uint8_t buffe[14];
	int16_t temp;
	buffe[0] = (uint8_t)ACC_DATA_REG;
#ifdef I2C
	HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,buffe,1,1);
    HAL_I2C_Master_Receive(&I2C_PORT,IMU_DEV_REG,buffe,14,1);
#endif
#ifdef SPI
    SPI_read(buffe[0],buffe,14);
#endif
	acc->x = (int16_t)buffe[0]<<8|buffe[1];
	acc->y = (int16_t)buffe[2]<<8|buffe[3];
	acc->z = (int16_t)buffe[4]<<8|buffe[5];
	temp   = (int16_t)buffe[6]<<8|buffe[7];
	gyr->x = (int16_t)buffe[8]<<8|buffe[9];
	gyr->y = (int16_t)buffe[10]<<8|buffe[11];
	gyr->z = (int16_t)buffe[12]<<8|buffe[12];
}
// user configurations
void mpu6500_init(){
#ifdef SPI
    uint8_t data[2];
	data[0] = MPU_RA_PWR_MGMT_1;
	data[1] = 0x00;
	SPI_write(&data,2);
    //  gyro configure
	data[0] = (uint8_t)GYRO_REG_CONFIG;
	data[1] = (uint8_t)(GYRO_1000dps<<3);
    SPI_write(&data,2);  
    //  acce configure
	data[0] = (uint8_t)ACC_REG_CONFIG;
	data[1] = (uint8_t)(ACC_2G<<3);
    SPI_write(&data,2);
#endif
#ifdef I2C
	uint8_t buffer[6];
    buffer[0] = MPU_RA_PWR_MGMT_1; 
  	buffer[1] = RESET_REG;
  	HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,buffer,2,1);
    
  	//buffer[0] = CONFIGURATION;
  	//buffer[1] = (HZ_5<<0);
  	//HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,buffer,2,1);

	buffer[0] = GYRO_REG_CONFIG;
  	buffer[1] = (GYRO_1000dps<<3);
  	HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,buffer,2,1);

  	buffer[0] = ACC_REG_CONFIG;
  	buffer[1] = (ACC_16G<<3);
  	HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,buffer,2,1);
#endif
}

int16_t get_axis_register(uint8_t addr){
	uint8_t buffe[2];
	HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,&addr,1,1);
    HAL_I2C_Master_Receive(&I2C_PORT,IMU_DEV_REG,buffe,2,1);
    int16_t temp  = (int16_t)buffe[0]<<8|buffe[1];
    return temp;
}

void write_axis_register(uint8_t addr,uint8_t val){
	uint8_t buffe[2];
	buffe[0] = addr;
	buffe[1] = val;
	HAL_I2C_Master_Transmit(&I2C_PORT,IMU_DEV_REG,buffe,2,1);
}
static void SPI_write(uint8_t *data,uint8_t len){
    SPI_ON;
	HAL_SPI_Transmit(&SPI_PORT,data,len,SPI_TIME_OUT);
	SPI_OFF;
}
static void SPI_read(uint8_t reg_addr,uint8_t *data,uint8_t len){
    SPI_ON;
	reg_addr |= 0x80;
	HAL_SPI_Transmit(&SPI_PORT,&reg_addr,1,SPI_TIME_OUT);
	HAL_SPI_Receive(&SPI_PORT,data,len,SPI_TIME_OUT);
	SPI_OFF;
}
