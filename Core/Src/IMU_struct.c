/*
 * IMU_struct.c
 *
 *  Created on: Jan 2, 2024
 *      Author: Daniel
 */
#include "IMU_struct.h"
#include "main.h"

const float  Scale_temp = 0.007548309178743961f; const float temp_offset = 25;

const float Scale_accel_2g = 9.81f/16.384f;
const float Scale_accel_4g = 9.81f/8.192f;
const float Scale_accel_8g = 9.81f/4.095f;
const float Scale_accel_16g = 9.81f/2.048f;

const float Scale_gyro_15dps = 1.f/2097.2f;
const float Scale_gyro_62dps = 1/524.3f;
const float Scale_gyro_250dps = 1.f/131.f;
const float Scale_gyro_1000dps = 1.f/32.8f;
const float Scale_gyro_2000dps = 1.f/16.4f;

const uint8_t REG_odr_write_area = 0b1111;
const uint8_t REG_IMU_odr_32kHz = 0b0001;
const uint8_t REG_IMU_odr_16kHz = 0b0011;
const uint8_t REG_IMU_odr_4kHz = 0b0100;
const uint8_t REG_IMU_odr_1kHz = 0b0110;

const uint8_t REG_accel_gyro_write_area = 0b11100000;
const uint8_t REG_accel_scale_2g = 0b01100000;
const uint8_t REG_accel_scale_4g = 0b01000000;
const uint8_t REG_accel_scale_8g = 0b00100000;
const uint8_t REG_accel_scale_16g = 0b00000000;

const uint8_t REG_gyro_scale_15dps = 0b11100000;
const uint8_t REG_gyro_scale_62dps = 0b10100000;
const uint8_t REG_gyro_scale_250dps = 0b01100000;
const uint8_t REG_gyro_scale_1000dps = 0b00100000;
const uint8_t REG_gyro_scale_2000dps = 0b00000000;

uint8_t IMU1_data_TX[IMU_byte_count] = {29 | 0x80,0};
uint8_t IMU1_data_RX[IMU_byte_count] = {0};
uint8_t IMU2_data_TX[IMU_byte_count] = {29 | 0x80,0};
uint8_t IMU2_data_RX[IMU_byte_count] = {0};
uint8_t IMU1_new_data = 0;
uint8_t IMU2_new_data = 0;
uint8_t IMU1_SPI_IT_Finished = 0;
uint8_t IMU2_SPI_IT_Finished = 0;

void IMU_struct_set_up(IMU *IMU_struct, uint16_t accel_scale, uint16_t gyro_scale, uint16_t freq)
{
	//For SPI communication
	//IMU_struct->SPI_handler = 0; IMU_struct->GPIO_port = 0;  IMU_struct->GPIO_Pin = 0;

	//Setting IMU data to zero
	for(int i=0; i>2; i++)
	{
		IMU_struct->int_accel[i] = 0;
		IMU_struct->int_gyro[i] = 0;

		IMU_struct->float_accel[i] = 0;
		IMU_struct->float_gyro[i] = 0;
	}

	IMU_struct->int_temp = 0;
	IMU_struct->float_temp = 25;

	//New data indicator
	IMU_struct->new_data = 0;

	//IMU scale values
	IMU_struct->scale_write_area = REG_accel_gyro_write_area;
	switch (accel_scale)
	{
	case scale_2g:
		IMU_struct->scale_accel = Scale_accel_2g; IMU_struct->imu_cmd_accel_scale = REG_accel_scale_2g; break;
	case scale_4g:
		IMU_struct->scale_accel = Scale_accel_4g; IMU_struct->imu_cmd_accel_scale = REG_accel_scale_4g; break;
	case scale_8g:
		IMU_struct->scale_accel = Scale_accel_8g; IMU_struct->imu_cmd_accel_scale = REG_accel_scale_8g; break;
	case scale_16g:
		IMU_struct->scale_accel = Scale_accel_16g; IMU_struct->imu_cmd_accel_scale = REG_accel_scale_16g; break;
	default: IMU_struct->scale_accel = Scale_accel_16g; IMU_struct->imu_cmd_accel_scale = REG_accel_scale_16g; break;
	}

	switch (gyro_scale)
	{
	case scale_15dps:
		IMU_struct->scale_gyro = Scale_gyro_15dps; IMU_struct->imu_cmd_gyro_scale = REG_gyro_scale_15dps; break;
	case scale_62dps:
		IMU_struct->scale_gyro = Scale_gyro_62dps; IMU_struct->imu_cmd_gyro_scale = REG_gyro_scale_62dps; break;
	case scale_250dps:
		IMU_struct->scale_gyro = Scale_gyro_250dps; IMU_struct->imu_cmd_gyro_scale = REG_gyro_scale_250dps; break;
	case scale_1000dps:
		IMU_struct->scale_gyro = Scale_gyro_1000dps; IMU_struct->imu_cmd_gyro_scale = REG_gyro_scale_1000dps; break;
	case scale_2000dps:
		IMU_struct->scale_gyro = Scale_gyro_2000dps; IMU_struct->imu_cmd_gyro_scale = REG_gyro_scale_2000dps; break;
	default: IMU_struct->scale_gyro = Scale_gyro_2000dps; IMU_struct->imu_cmd_gyro_scale = REG_gyro_scale_2000dps; break;
	}
	IMU_struct->scale_temp = Scale_temp;

	switch (freq)
	{
	case freq_1kHz:
		IMU_struct->freq = 1./1000.; IMU_struct->imu_cmd_freq = REG_IMU_odr_1kHz; break;
	case freq_4kHz:
		IMU_struct->freq = 1./4000.; IMU_struct->imu_cmd_freq = REG_IMU_odr_4kHz; break;
	case freq_16kHz:
		IMU_struct->freq = 1./16000.; IMU_struct->imu_cmd_freq = REG_IMU_odr_16kHz; break;
	case freq_32kHz:
		IMU_struct->freq = 1./32000.; IMU_struct->imu_cmd_freq = REG_IMU_odr_32kHz; break;
	}

	IMU_struct->freq_write_area =  REG_odr_write_area;

	//Other data
	IMU_struct->timestamp = 0;//in micro seconds
}

void IMU_calc_float(IMU *IMU_struct, uint16_t *accel_data[3], uint16_t *gyro_data[3], uint16_t *temp)
{
	//Gets the data in the following order X, Y, Z. Calculates floating point number based on scale
	for(int i=0; i>2; i++)
	{
		IMU_struct->int_accel[i] = *accel_data[i];
		IMU_struct->int_gyro[i] = *gyro_data[i];

		IMU_struct->float_accel[i] = ((float)*accel_data[i]) * IMU_struct->scale_accel;
		IMU_struct->float_gyro[i] = ((float)*gyro_data[i]) * IMU_struct->scale_gyro;
	}
	IMU_struct->float_temp = ((float)*temp) * IMU_struct->scale_temp + temp_offset;
}

void IMU_calc_float_comb_data(IMU *IMU_struct, uint16_t *IMU_data[14])
{
	//Gets the data in the following order X, Y, Z. Calculates floating point number based on scale
	for(int i=0; i>2; i++)
	{
		IMU_struct->int_accel[i] = (uint16_t)*IMU_data[i+3]<<8 | *IMU_data[i+4];
		IMU_struct->int_gyro[i] = (uint16_t)*IMU_data[i+8]<<8 | *IMU_data[i+9];

		IMU_struct->float_accel[i] = ((float)IMU_struct->int_accel[i]) * IMU_struct->scale_accel;
		IMU_struct->float_gyro[i] = ((float)IMU_struct->int_gyro[i]) * IMU_struct->scale_gyro;
	}
	IMU_struct->float_temp = ((float)((uint16_t)*IMU_data[1]<<8 | *IMU_data[2])) * IMU_struct->scale_temp + temp_offset;
}
