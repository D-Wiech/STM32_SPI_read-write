/*
 * IMU_struct.h
 *
 *  Created on: Jan 2, 2024
 *      Author: Daniel
 */

#ifndef INC_IMU_STRUCT_H_
#define INC_IMU_STRUCT_H_

#include "main.h"

//General Struct for IMUs
typedef struct
{
	//For SPI communication
	SPI_HandleTypeDef SPI_handler; GPIO_TypeDef *GPIO_port;  uint16_t GPIO_Pin;

	//IMU binary data
	uint16_t int_accel[3]; uint16_t int_gyro[3]; uint16_t int_temp;

	//New data indicator
	uint8_t new_data;

	//IMU float data
	float float_accel[3]; float float_gyro[3]; float float_temp;

	//IMU scale values
	float scale_accel; float scale_gyro; float scale_temp;

	//IMU inverse sample freq
	float freq;

	//Other data
	uint32_t timestamp;//in micro seconds
	uint8_t set_up_done;
	uint8_t imu_cmd_gyro_scale;
	uint8_t imu_cmd_accel_scale;
	uint8_t scale_write_area;
	uint8_t imu_cmd_freq; uint8_t freq_write_area;
} IMU;

enum IMU_accel_scale{
	scale_2g = 1,//2G
	scale_4g,//4G
	scale_8g,//8G
	scale_16g//16G
};

enum IMU_gyro_scale{
	scale_15dps = 1,//15,6dps
	scale_62dps,//62,5dps
	scale_250dps,//250dps
	scale_1000dps,//1000dps
	scale_2000dps//2000dps
};

enum IMU_freq{
	freq_1kHz = 1,
	freq_4kHz,
	freq_16kHz,
	freq_32kHz
};

//IMU scale
#define IMU1_accel_scale scale_4g
#define IMU2_accel_scale scale_16g
#define IMU1_gyro_scale scale_15dps
#define IMU2_gyro_scale scale_2000dps
#define IMU1_freq freq_4kHz
#define IMU2_freq freq_4kHz
#define IMU_byte_count 15
extern uint8_t IMU1_data_TX[IMU_byte_count];
extern uint8_t IMU1_data_RX[IMU_byte_count];
extern uint8_t IMU2_data_TX[IMU_byte_count];
extern uint8_t IMU2_data_RX[IMU_byte_count];
extern uint8_t IMU1_new_data;
extern uint8_t IMU2_new_data;

void IMU_struct_set_up(IMU *IMU_struct, uint16_t accel_scale, uint16_t gyro_scale, uint16_t freq);

void IMU_calc_float(IMU *IMU_struct, uint16_t *accel_data[3], uint16_t *gyro_data[3], uint16_t *temp);

void IMU_calc_float_comb_data(IMU *IMU_struct, uint16_t *IMU_data[14]);

#endif /* INC_IMU_STRUCT_H_ */
