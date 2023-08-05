#include "mpu6500.h"
#include "i2c.h"
#include "axis.h"
#include "stm32f1xx_hal.h"

#define I2C
#define GYRO_DATA_REG   0x43
#define ACC_DATA_REG    0x3b
#define IMU_DEV_REG     0x68<<1
#define RESET_REG       0x00
#define GYRO_REG_CONFIG 0x1b
#define ACC_REG_CONFIG  0x1c


static I2C_HandleTypeDef *mpu_i2cport;

int gyro_read_raw(axis3_t *k){
	  axis3_t p_val =*k;
	  uint8_t buffe[6];
	  buffe[0] = GYRO_DATA_REG;
#ifdef I2C
	  HAL_I2C_Master_Transmit(mpu_i2cport,IMU_DEV_REG,buffe,1,1);
	  HAL_I2C_Master_Receive(mpu_i2cport,IMU_DEV_REG,buffe,6,1);
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


int acc_read_raw(axis3_t *k){
	axis3_t p_val =*k;
	uint8_t buffe[6];
	buffe[0] = ACC_DATA_REG;// acc address
#ifdef I2C
	  HAL_I2C_Master_Transmit(mpu_i2cport,IMU_DEV_REG,buffe,1,1);
	  HAL_I2C_Master_Receive(mpu_i2cport,IMU_DEV_REG,buffe,6,1);
#endif
#ifdef SPI
	  buffe[0] |=0x80;
	  HAL_GPIO_WritePin(mpu_gpio_port,mpu_cs_pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&mpu_spi_port,buffe,1,100);
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

// user configurations
void MPU_i2c_init(I2C_HandleTypeDef *i2cport)
{
	uint8_t buffer[6];
	mpu_i2cport = i2cport;
    buffer[0] = 0x6b; 
  	buffer[1] =  RESET_REG;
  	HAL_I2C_Master_Transmit(mpu_i2cport,IMU_DEV_REG,buffer,2,1);
    
  	buffer[0] = GYRO_REG_CONFIG;
  	buffer[1] = (GYRO_1000<<3); 
  	HAL_I2C_Master_Transmit(mpu_i2cport,IMU_DEV_REG,buffer,2,1);

  	buffer[0] = ACC_REG_CONFIG; 
  	buffer[1] = (ACC_2G<<3);
  	HAL_I2C_Master_Transmit(mpu_i2cport,IMU_DEV_REG,buffer,2,1);
}
