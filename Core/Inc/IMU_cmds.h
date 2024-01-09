/*
 * IMU_cmds.h
 *
 *  Created on: Jan 2, 2024
 *      Author: Daniel
 */

#ifndef INC_IMU_CMDS_H_
#define INC_IMU_CMDS_H_

#include "main.h"
#include "IMU_struct.h"

HAL_StatusTypeDef IMU_send_cmd(IMU *IMU_struct, uint8_t TX[], uint8_t RX[], uint16_t length);

HAL_StatusTypeDef IMU_WHO_AM_I(IMU *IMU_struct);

HAL_StatusTypeDef IMU_activate(IMU *IMU_struct);

HAL_StatusTypeDef IMU_deactivate(IMU *IMU_struct);

HAL_StatusTypeDef IMU_register_bank_change(IMU *IMU_struct, uint8_t Reg_Bank);

HAL_StatusTypeDef IMU_write_reg(IMU *IMU_struct, uint8_t *REG_add, uint8_t *REG_val, uint8_t *REG_write_area);

HAL_StatusTypeDef IMU_setup(IMU *IMU_struct);

#endif /* INC_IMU_CMDS_H_ */
