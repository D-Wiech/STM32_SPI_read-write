/*
 * IMU_cmds.c
 *
 *  Created on: Jan 2, 2024
 *      Author: Daniel
 */

#include "IMU_cmds.h"
#include "IMU_struct.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

HAL_StatusTypeDef IMU_send_cmd(IMU *IMU_struct, uint8_t TX[], uint8_t RX[], uint16_t length)
{
	//Sends and Receives two bytes to the Sensor
	//HAL_SPI_Abort(&IMU_struct->SPI_handler);
	uint32_t Timeout = 3;
	HAL_GPIO_WritePin(IMU_struct->GPIO_port, IMU_struct->GPIO_Pin, GPIO_PIN_RESET );
	const HAL_StatusTypeDef return_Value = HAL_SPI_TransmitReceive(&IMU_struct->SPI_handler, TX, RX, length, Timeout);
	HAL_GPIO_WritePin(IMU_struct->GPIO_port, IMU_struct->GPIO_Pin, GPIO_PIN_SET );

	return return_Value;
}

HAL_StatusTypeDef IMU_write_reg(IMU *IMU_struct, uint8_t *REG_add, uint8_t *REG_val, uint8_t *REG_write_area)
{
	//Write an register -> REG_write_area should contain a 1 where the register is being written to
	const uint8_t length = 2;
	uint8_t TXData[2] = {*REG_add | 0x80, 0}; // 0x80 results in a Read-operation
	uint8_t RXData[2] = {0, 0};

	if(IMU_send_cmd(&IMU_struct, TXData, RXData, length) == HAL_OK)
		{
			TXData[0] = *REG_add; // 0x80 results in a Read-operation
			TXData[1]  = ((RXData[1] & (!*REG_write_area)) | *REG_val);

			return IMU_send_cmd(&IMU_struct, TXData, RXData, length);
		}
	return HAL_ERROR;
}

HAL_StatusTypeDef IMU_WHO_AM_I(IMU *IMU_struct)
{
	//Looks wheather there is an IMU
	const uint8_t length = 2;
	uint8_t TXData[2] = {117 | 0x80, 0}; // 0x80 results in a Read-operation
	uint8_t RXData[2] = {0, 0};

	IMU_send_cmd(IMU_struct, TXData, RXData, length);

	if(RXData[1] == 0x47)
	{
		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef IMU_activate(IMU *IMU_struct)
{
	//Activates the IMU
	const uint8_t length = 2;
	uint8_t TXData[2] = {0, 0};
	uint8_t RXData[2] = {0, 0};
	HAL_StatusTypeDef return_Value = HAL_OK;

	return_Value = IMU_WHO_AM_I(IMU_struct);

	HAL_Delay(1);

	TXData[0] = 78; // PWR MGMT0 register (Write)
	TXData[1] = 0b00001111; // IMU activation
	if(return_Value == HAL_OK)
	{
		IMU_send_cmd(IMU_struct, TXData, RXData, length);
	}
	else
	{
		return_Value = HAL_ERROR;
	}
	HAL_Delay(1);
	return return_Value;
}

HAL_StatusTypeDef IMU_deactivate(IMU *IMU_struct)
{
	//Deactivates the IMU
	uint8_t length = 2;
	uint8_t TXData[2] = {0, 0};
	uint8_t RXData[2] = {0, 0};
	HAL_StatusTypeDef return_Value = HAL_OK;

	return_Value = IMU_WHO_AM_I(IMU_struct);

	HAL_Delay(1);

	TXData[0] = 78; // PWR MGMT0 Register (Schreiben)
	TXData[1] = 0b00000000; // Gyro und Accel ausschalten;
	if(return_Value == HAL_OK)
	{
		IMU_send_cmd(IMU_struct, TXData, RXData, length);
	}
	else
	{
		return_Value = HAL_ERROR;
		debug_uart4_write_text("IMU deactivating error\n");
	}
	HAL_Delay(1);
	return return_Value;
}

HAL_StatusTypeDef IMU_register_bank_change(IMU *IMU_struct, uint8_t Reg_Bank)
{
	//Changes the Register Bank to make different Settings available
	//Allows only possible values
	if(Reg_Bank > 5)
	{
		return HAL_ERROR;
	}
	uint8_t length = 3;
	uint8_t TXData[2] = {118, Reg_Bank};//Writes the Input to Register 118 (REG_BANK_SEL)
	uint8_t RXData[2] = {0, 0};
	HAL_StatusTypeDef return_Value = HAL_OK;

	IMU_send_cmd(IMU_struct, TXData, RXData, length);
	return return_Value;
}



HAL_StatusTypeDef IMU_setup(IMU *IMU_struct)
{
	//Sets up the Gyro/Accel
	uint8_t length = 2;
	uint8_t TXData[2] = {0, 0};
	uint8_t RXData[2] = {0, 0};

	while(IMU_deactivate(IMU_struct) != HAL_OK)
	{
		HAL_Delay(10);//Gyro and Accel must be turned of for changing settings)
	}

	//Sensor Reset and Gyro and Accel ODR set
	TXData[0] = 17;//Writes the Device_Config_Register
	TXData[1] = 0b00000001;//Writes the Reset command
	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};
	HAL_Delay(1);



	TXData[0] = 79;//Writes GYRO_CONFIG0
	//TXData[1] = 0b00000111;//200Hz
	//TXData[1] = 0b00000110;//Set the GYRO scale to 2000dps and ODR 0110(LSB_1kHz)
	//TXData[1] = 0b00000001;//Set the GYRO scale to 2000dps and ODR 0001(LSB_32kHz)
	//TXData[1] = 0b00000010;//16kHz
	TXData[1] = IMU_struct->imu_cmd_freq |IMU_struct->imu_cmd_gyro_scale;
	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	TXData[0] = 80;//Writes ACCEL_CONFIG0
	TXData[1] = IMU_struct->imu_cmd_freq |IMU_struct->imu_cmd_accel_scale;
	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	//Filter set-up
	TXData[0] = 82;//Writes the GYRO_ACCEl_CONFIG0 Register
	TXData[1] = 0x33;//Set the Gyro/Accel BW to ODR/8(0x33), ODR/5(0x22),  ODR/16(0x55)

	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	//Interrupt set up (Data RDY)
	TXData[0] = 20;//Writes the INT_CONFIG Register
	TXData[1] = 0b00000010;//Set the INT1_Pin to PUSH-PULL Output

	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	TXData[0] = 99;//Writes the INT_CONFIG0 Register
	TXData[1] = 0b00110000;//Set the Interrupt clear on Status Read and Data Read

	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	TXData[0] = 100;//Writes the INT_CONFIG1 Register
	TXData[1] = 0b01100000;//Set the Interrupt duration to 8us and disables INT_Async_Reset

	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	TXData[0] = 101;//Writes the INT_SOURCE0 Register
	TXData[1] = 0b00001000;//Enables the Dataready interrupt

	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	//FIFO Config
	TXData[0] = 95;//Writes the FIFIO_CONFIG1 Register
	//TXData[1] = 0b00010000;//Set the FIFO Packet size to 20byte (HiRes), disables a partial read of the FIFO and disables FSYNC for the FIFO

	//if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	TXData[0] = 22;//Writes the FIFIO_CONFIG Register
	//TXData[1] = 0b01000000;//Set Stream to FIFO

	//if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	TXData[0] = 84;//Writes the TMST_Config Register
	//TXData[1] = 0b00110101;//Set TMST_RES to 1us, disables TMST_FSYNC and enables TMST_TO_REGS_EN

	//if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	TXData[0] = 76;//Writes the INTF_CONFIG0 Register
	TXData[1] = 0b10110011;//Set FIFO_Count_REC to bytes and disables I2C

	if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	TXData[0] = 75;//Writes SIGNAL_PATH_RESET
	//TXData[1] = 0b00000010;//FLUSH the FIFO
	//if(IMU_send_cmd(IMU_struct, TXData, RXData, length) != HAL_OK){return HAL_ERROR;};

	if(IMU_activate(IMU_struct) != HAL_OK)
	{
		Error_Handler();
	}
	IMU_struct->set_up_done = 1;
	return HAL_OK;
}
