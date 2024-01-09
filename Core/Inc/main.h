/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_cordic.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "stm32h7xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Druck_INT_Pin GPIO_PIN_4
#define Druck_INT_GPIO_Port GPIOB
#define Druck_INT_EXTI_IRQn EXTI4_IRQn
#define GYRO_INT1_Pin GPIO_PIN_7
#define GYRO_INT1_GPIO_Port GPIOC
#define GYRO_INT1_EXTI_IRQn EXTI9_5_IRQn
#define GYRO1_NSS_Pin GPIO_PIN_4
#define GYRO1_NSS_GPIO_Port GPIOA
#define GYRO2_MOSI_Pin GPIO_PIN_15
#define GYRO2_MOSI_GPIO_Port GPIOB
#define GYRO1_SCK_Pin GPIO_PIN_5
#define GYRO1_SCK_GPIO_Port GPIOA
#define GYRO2_MISO_Pin GPIO_PIN_14
#define GYRO2_MISO_GPIO_Port GPIOB
#define GYRO1_MISO_Pin GPIO_PIN_6
#define GYRO1_MISO_GPIO_Port GPIOA
#define GYRO2_SCK_Pin GPIO_PIN_13
#define GYRO2_SCK_GPIO_Port GPIOB
#define GYRO1_MOSI_Pin GPIO_PIN_7
#define GYRO1_MOSI_GPIO_Port GPIOA
#define GYRO2_NSS_Pin GPIO_PIN_12
#define GYRO2_NSS_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_12
#define USER_LED_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
