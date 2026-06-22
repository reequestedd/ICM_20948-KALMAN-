/*
* icm20948.c
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi
*  Fixed: SLV4 untuk init, SLV0 untuk runtime auto-fetch
*/

#include "icm20948.h"

static float gyro_scale_factor;
static float accel_scale_factor;

/* Static Functions */
static void     cs_high();
static void     cs_low();
static void     select_user_bank(userbank ub);
static uint8_t  read_single_icm20948_reg(userbank ub, uint8_t reg);
static void     write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val);
static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);
static void     write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len);

/* AK09916 via SLV4 (one-shot, untuk init saja) */
static uint8_t  ak09916_read_via_slv4(uint8_t reg);
static void     ak09916_write_via_slv4(uint8_t reg, uint8_t val);

uint8_t ak09916_id;
uint8_t icm20948_id;

/* ================================================================
 *  ICM-20948 INIT
 * ================================================================ */
void icm20948_init()
{
	while(!icm20948_who_am_i());

	icm20948_device_reset();
	icm20948_wakeup();

	icm20948_clock_source(1);
	icm20948_odr_align_enable();

	icm20948_spi_slave_enable();

	icm20948_gyro_low_pass_filter(0);
	icm20948_accel_low_pass_filter(0);

	icm20948_gyro_sample_rate_divider(0);
	icm20948_accel_sample_rate_divider(0);

	icm20948_gyro_calibration();
	icm20948_accel_calibration();

	icm20948_gyro_full_scale_select(_250dps);
	icm20948_accel_full_scale_select(_2g);
}

/* ================================================================
 *  AK09916 INIT
 *  - SLV4 untuk semua read/write ke AK09916 saat init
 *  - SLV0 TIDAK DISENTUH sampai step terakhir (auto-fetch setup)
 *  - Ini menghindari konflik SLV0 yang selama ini jadi masalah
 * ================================================================ */
void ak09916_init()
{
	// Step 1: I2C Master Reset
	uint8_t user_ctrl = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	user_ctrl |= 0x02;
	write_single_icm20948_reg(ub_0, B0_USER_CTRL, user_ctrl);
	HAL_Delay(100);

	// Step 2: I2C Master Enable (preserve SPI bit)
	user_ctrl = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	user_ctrl |= 0x20;
	write_single_icm20948_reg(ub_0, B0_USER_CTRL, user_ctrl);
	HAL_Delay(50);

	// Step 3: I2C Master Clock = 7 (~345kHz)
	write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, 0x07);
	HAL_Delay(10);

	// Step 4: LP_CONFIG — enable I2C Master duty cycle
	write_single_icm20948_reg(ub_0, B0_LP_CONFIG, 0x40);
	HAL_Delay(10);

	// Step 5: I2C_MST_ODR_CONFIG — ~137Hz
	write_single_icm20948_reg(ub_3, B3_I2C_MST_ODR_CONFIG, 0x03);
	HAL_Delay(10);

	// Step 6: Cek AK09916 ID via SLV4
	ak09916_id = 0;
	for (int retry = 0; retry < 10; retry++)
	{
		ak09916_id = ak09916_read_via_slv4(MAG_WIA2);
		if (ak09916_id == AK09916_ID) break;
		HAL_Delay(50);
	}
	while (ak09916_id != AK09916_ID);

	// Step 7: Soft reset AK09916 via SLV4
	ak09916_write_via_slv4(MAG_CNTL3, 0x01);
	HAL_Delay(200);

	// Step 8: Set Continuous Mode 100Hz via SLV4
	ak09916_write_via_slv4(MAG_CNTL2, 0x08);
	HAL_Delay(100);

	// Step 9: Verify CNTL2
	uint8_t cntl2_check = ak09916_read_via_slv4(MAG_CNTL2);
	if ((cntl2_check & 0x1F) != 0x08)
	{
		ak09916_write_via_slv4(MAG_CNTL2, 0x08);
		HAL_Delay(100);
	}

	// Step 10: Setup SLV0 auto-fetch (SLV0 bersih, belum pernah dipakai)
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x00);
	HAL_Delay(10);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, MAG_ST1);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x89);
	write_single_icm20948_reg(ub_3, B3_I2C_MST_DELAY_CTRL, 0x01);
	HAL_Delay(200);
}

/* ================================================================
 *  SENSOR READ
 * ================================================================ */
void icm20948_gyro_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);
	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_accel_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);
	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor;
}

bool ak09916_mag_read(axises* data)
{
	uint8_t* buf = read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, 9);

	if (!(buf[0] & 0x01)) return false;
	if (buf[8] & 0x08) return false;

	data->x = (int16_t)(buf[2] << 8 | buf[1]);
	data->y = (int16_t)(buf[4] << 8 | buf[3]);
	data->z = (int16_t)(buf[6] << 8 | buf[5]);

	return true;
}

void icm20948_gyro_read_dps(axises* data)
{
	icm20948_gyro_read(data);
	data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(axises* data)
{
	icm20948_accel_read(data);
	data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;
}

bool ak09916_mag_read_uT(axises* data)
{
	axises temp;
	bool new_data = ak09916_mag_read(&temp);
	if(!new_data) return false;

	data->x = (float)(temp.x * 0.15);
	data->y = (float)(temp.y * 0.15);
	data->z = (float)(temp.z * 0.15);

	return true;
}

/* ================================================================
 *  SUB FUNCTIONS
 * ================================================================ */
bool icm20948_who_am_i()
{
	icm20948_id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);
	return (icm20948_id == ICM20948_ID);
}

bool ak09916_who_am_i()
{
	ak09916_id = ak09916_read_via_slv4(MAG_WIA2);
	return (ak09916_id == AK09916_ID);
}

void icm20948_device_reset()
{
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	HAL_Delay(100);
}

void ak09916_soft_reset()
{
	ak09916_write_via_slv4(MAG_CNTL3, 0x01);
	HAL_Delay(100);
}

void icm20948_wakeup()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_sleep()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_spi_slave_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x10;
	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_reset()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x02;
	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x20;
	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
	HAL_Delay(100);
}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
	write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, config);
}

void icm20948_clock_source(uint8_t source)
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_odr_align_enable()
{
	write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;
	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;
	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}

void ak09916_operation_mode_setting(operation_mode mode)
{
	ak09916_write_via_slv4(MAG_CNTL2, mode);
	HAL_Delay(100);
}

void icm20948_gyro_calibration()
{
	axises temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_gyro_read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF;
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;

	write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

void icm20948_accel_calibration()
{
	axises temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;

	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_accel_read(&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;
	}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];

	write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);

	switch(full_scale)
	{
		case _250dps:  new_val |= 0x00; gyro_scale_factor = 131.0;  break;
		case _500dps:  new_val |= 0x02; gyro_scale_factor = 65.5;   break;
		case _1000dps: new_val |= 0x04; gyro_scale_factor = 32.8;   break;
		case _2000dps: new_val |= 0x06; gyro_scale_factor = 16.4;   break;
	}

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);

	switch(full_scale)
	{
		case _2g:  new_val |= 0x00; accel_scale_factor = 16384; break;
		case _4g:  new_val |= 0x02; accel_scale_factor = 8192;  break;
		case _8g:  new_val |= 0x04; accel_scale_factor = 4096;  break;
		case _16g: new_val |= 0x06; accel_scale_factor = 2048;  break;
	}

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}


/* ================================================================
 *  SPI LOW-LEVEL
 * ================================================================ */
static void cs_high()
{
	HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, SET);
}

static void cs_low()
{
	HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, RESET);
}

static void select_user_bank(userbank ub)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | REG_BANK_SEL;
	write_reg[1] = ub;
	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
	cs_high();
}

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
	uint8_t read_reg = READ | reg;
	uint8_t reg_val;
	select_user_bank(ub);
	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
	HAL_SPI_Receive(ICM20948_SPI, &reg_val, 1, 1000);
	cs_high();
	return reg_val;
}

static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | reg;
	write_reg[1] = val;
	select_user_bank(ub);
	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 1000);
	cs_high();
}

static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
	uint8_t read_reg = READ | reg;
	static uint8_t reg_val[16];
	select_user_bank(ub);
	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
	HAL_SPI_Receive(ICM20948_SPI, reg_val, len, 1000);
	cs_high();
	return reg_val;
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = WRITE | reg;
	select_user_bank(ub);
	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, &write_reg, 1, 1000);
	HAL_SPI_Transmit(ICM20948_SPI, val, len, 1000);
	cs_high();
}


/* ================================================================
 *  SLV4 — One-shot read/write ke AK09916
 *  Polling I2C_MST_STATUS bit 6 (SLV4_DONE)
 * ================================================================ */
static uint8_t ak09916_read_via_slv4(uint8_t reg)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV4_ADDR, 0x80 | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV4_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV4_CTRL, 0x80);

	for (int i = 0; i < 100; i++)
	{
		HAL_Delay(1);
		uint8_t status = read_single_icm20948_reg(ub_0, B0_I2C_MST_STATUS);
		if (status & 0x40)
		{
			return read_single_icm20948_reg(ub_3, B3_I2C_SLV4_DI);
		}
	}
	return 0xFF;
}

static void ak09916_write_via_slv4(uint8_t reg, uint8_t val)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV4_ADDR, 0x00 | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV4_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV4_DO, val);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV4_CTRL, 0x80);

	for (int i = 0; i < 100; i++)
	{
		HAL_Delay(1);
		uint8_t status = read_single_icm20948_reg(ub_0, B0_I2C_MST_STATUS);
		if (status & 0x40)
			return;
	}
}


/* ================================================================
 *  PUBLIC WRAPPERS (untuk debug dari main.c)
 * ================================================================ */
uint8_t icm20948_read_reg(userbank ub, uint8_t reg)
{
	return read_single_icm20948_reg(ub, reg);
}

void icm20948_read_multi(userbank ub, uint8_t reg, uint8_t* buf, uint8_t len)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub, reg, len);
	for(int i = 0; i < len; i++) buf[i] = temp[i];
}

void icm20948_write_reg(userbank ub, uint8_t reg, uint8_t val)
{
	write_single_icm20948_reg(ub, reg, val);
}
