
#include "bmp280.h"
#include "math.h"
#include "timeclock.h"
/**
 * BMP280 registers
 */
#define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_HUM_CALIB   0x88

#define BMP280_RESET_VALUE     0xB6


void bmp280_init_default_params(bmp280_params_t *params) {
	params->mode = BMP280_MODE_NORMAL;
	params->filter = BMP280_FILTER_16;
	params->oversampling_pressure = BMP280_STANDARD;
	params->oversampling_temperature = BMP280_STANDARD;
	params->standby = BMP280_STANDBY_62;
}

static bool read_register16(BMP280_HandleTypedef *dev, uint8_t addr, uint16_t *value) {
	uint16_t tx_buff;
	uint8_t rx_buff[2];
	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, rx_buff, 2, 5000)
			== HAL_OK) {
		*value = (uint16_t) ((rx_buff[1] << 8) | rx_buff[0]);
		return true;
	} else
		return false;

}

static inline int read_data(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t *value,
		uint8_t len) {
	uint16_t tx_buff;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, value, len, 5000) == HAL_OK)
		return 0;
	else
		return 1;

}

static bool read_calibration_data(BMP280_HandleTypedef *dev) {

	if (read_register16(dev, 0x88, &dev->dig_T1)
			&& read_register16(dev, 0x8a, (uint16_t *) &dev->dig_T2)
			&& read_register16(dev, 0x8c, (uint16_t *) &dev->dig_T3)
			&& read_register16(dev, 0x8e, &dev->dig_P1)
			&& read_register16(dev, 0x90, (uint16_t *) &dev->dig_P2)
			&& read_register16(dev, 0x92, (uint16_t *) &dev->dig_P3)
			&& read_register16(dev, 0x94, (uint16_t *) &dev->dig_P4)
			&& read_register16(dev, 0x96, (uint16_t *) &dev->dig_P5)
			&& read_register16(dev, 0x98, (uint16_t *) &dev->dig_P6)
			&& read_register16(dev, 0x9a, (uint16_t *) &dev->dig_P7)
			&& read_register16(dev, 0x9c, (uint16_t *) &dev->dig_P8)
			&& read_register16(dev, 0x9e,
					(uint16_t *) &dev->dig_P9)) {

		return true;
	}

	return false;
}


static int write_register8(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t value) {
	uint16_t tx_buff;

	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Write(dev->i2c, tx_buff, addr, 1, &value, 1, 10000) == HAL_OK)
		return false;
	else
		return true;
}

bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params) {

	if (dev->addr != BMP280_I2C_ADDRESS_0
			&& dev->addr != BMP280_I2C_ADDRESS_1) {

		return false;
	}

	if (read_data(dev, BMP280_REG_ID, &dev->id, 1)) {
		return false;
	}

	if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID) {

		return false;
	}

	// Soft reset.
	if (write_register8(dev, BMP280_REG_RESET, BMP280_RESET_VALUE)) {
		return false;
	}

	// Wait until finished copying over the NVP data.
	while (1) {
		uint8_t status;
		if (!read_data(dev, BMP280_REG_STATUS, &status, 1)&& (status & 1) == 0)
			break;
	}

	if (!read_calibration_data(dev)) {
		return false;
	}

	if (dev->id == BME280_CHIP_ID ) {
		return false;
	}

	uint8_t config = (params->standby << 5) | (params->filter << 2);
	if (write_register8(dev, BMP280_REG_CONFIG, config)) {
		return false;
	}

	if (params->mode == BMP280_MODE_FORCED) {
		params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
	}

	uint8_t ctrl = (params->oversampling_temperature << 5)
			| (params->oversampling_pressure << 2) | (params->mode);

	if (write_register8(dev, BMP280_REG_CTRL, ctrl)) {
		return false;
	}

	return true;
}

bool bmp280_force_measurement(BMP280_HandleTypedef *dev) {
	uint8_t ctrl;
	if (read_data(dev, BMP280_REG_CTRL, &ctrl, 1))
		return false;
	ctrl &= ~0b11;  // clear two lower bits
	ctrl |= BMP280_MODE_FORCED;
	if (write_register8(dev, BMP280_REG_CTRL, ctrl)) {
		return false;
	}
	return true;
}

bool bmp280_is_measuring(BMP280_HandleTypedef *dev) {
	uint8_t status;
	if (read_data(dev, BMP280_REG_STATUS, &status, 1))
		return false;
	if (status & (1 << 3)) {
		return true;
	}
	return false;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
static inline int32_t compensate_temperature(BMP280_HandleTypedef *dev, int32_t adc_temp, int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) dev->dig_T1 << 1)))
			* (int32_t) dev->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) dev->dig_T1)
			* ((adc_temp >> 4) - (int32_t) dev->dig_T1)) >> 12)
			* (int32_t) dev->dig_T3) >> 14;

	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(BMP280_HandleTypedef *dev, int32_t adc_press,int32_t fine_temp) {
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t) dev->dig_P5) << 17);
	var2 = var2 + (((int64_t) dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dev->dig_P3) >> 8)
			+ ((var1 * (int64_t) dev->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dev->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) dev->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) dev->dig_P7 << 4);
	return p/256;
}

bool bmp280_read_fixed(BMP280_HandleTypedef *dev, float *temp,float *press,float *altitude) {
	static int32_t adc_pressure;
	static int32_t adc_temp;
	static int32_t fine_temp;
	static int bmp_count=0;
	static int32_t temperature;
	static int32_t pressure;
	uint8_t data[3];
	if(bmp_count == 0){
		if (read_data(dev, 0xf7, data,3)) {
			return false;
		}
		adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
		bmp_count++;
		return true;
	}

	else if(bmp_count == 1){
		if (read_data(dev, 0xfA, data,3)) {
			return false;
		}
		adc_temp = data[0] << 12 | data[1] << 4 | data[2] >> 4;
		bmp_count++;
		return true;
	 }

	else if(bmp_count == 2){
		 temperature = compensate_temperature(dev, adc_temp, &fine_temp);
		 bmp_count++;
		 return true;
	}
	else if(bmp_count == 3){
		 pressure = compensate_pressure(dev, adc_pressure, fine_temp);
		 bmp_count++;
		 return true;
	}
	else if(bmp_count == 4){
		*press = (float)pressure;
		*temp  = (float)temperature/100;
		*altitude =((44330 * (1.0 - powf((float)pressure/(102416), 0.1903))))*100;

		 bmp_count = 0;
		 return true;
	}

	return true;
}
/*
bool bmp280_read_fixed(BMP280_HandleTypedef *dev, float *temp,float *press,float *altitude) {
	int32_t adc_pressure;
	int32_t adc_temp;
	int32_t fine_temp;
	int32_t temperature;
	int32_t pressure;
	uint8_t data[3];

	    read_data(dev, 0xf7, data,3);
	    adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
		read_data(dev, 0xfA, data,3);
		adc_temp = data[0] << 12 | data[1] << 4 | data[2] >> 4;

		 temperature = compensate_temperature(dev, adc_temp, &fine_temp);

		 pressure = compensate_pressure(dev, adc_pressure, fine_temp);

		*press = (float)pressure;
		*temp  = (float)temperature/100;
		*altitude =(44330 * (1.0 - powf((float)pressure/(101325*100), 0.1903)));
return 0;
}
*/

