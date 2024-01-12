/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dts.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "math.h"
#include "STM32_Cordic.h"
#include "stdio.h"
#include "IMU_struct.h"
#include "IMU_cmds.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

const float PI = 3.14159265359;

//Magnetometer variables 0xD << 1
const uint16_t QMC5833_ADDR = (0xD << 1);
int Magnetometer_Set_up_done = 0;// 0 set up not yet done | 1 set up is done
uint8_t raw_int_Magnetometer_data[6] = {0};
int16_t int_Megnetometer_data[3] = {0};
float  Magnetometer_data[3] = {0};
int new_Magnetometer_data = 0;//Indicates new Magnetometer data

//Pressure sensor variables
const uint16_t BMP388_ADDR = (0b1110110);

//IMU variables


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef debug_uart4_write_text(char *pData);
HAL_StatusTypeDef debug_uart4_write_float(float number);
HAL_StatusTypeDef debug_uart4_write_int(int number);
void Debug_send_Tempreture(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CORDIC_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC3_Init();
  MX_DTS_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  //UART Debuging prep
  uint8_t buffer[50];
  memset(buffer, 0, sizeof(buffer));

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //Start USER LED
  TIM4->CCR1 = 6874; // f = 2Hz

  //Starten des empfangens einer UART Nachricht
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rxbuffer, sizeof(rxbuffer));

  HAL_DTS_Start(&hdts);//Starts the digital tempreture sensor

  //Initialisation of the IMU structs
  IMU IMU1_data;
  IMU_struct_set_up(&IMU1_data, IMU1_accel_scale, IMU1_gyro_scale, IMU1_freq);
  IMU1_data.SPI_handler = hspi1;
  IMU1_data.GPIO_port = GYRO1_NSS_GPIO_Port;
  IMU1_data.GPIO_Pin = GYRO1_NSS_Pin;

  IMU IMU2_data;
  IMU_struct_set_up(&IMU2_data, IMU2_accel_scale, IMU2_gyro_scale, IMU2_freq);
  IMU2_data.SPI_handler = hspi2;
  IMU2_data.GPIO_port = GYRO2_NSS_GPIO_Port;
  IMU2_data.GPIO_Pin = GYRO2_NSS_Pin;

	uint16_t length = 2;
	uint8_t TXData[2] = {117 | 0x80, 0}; // 0x80 results in a Read-operation
	uint8_t RXData[2] = {0, 111};

	uint8_t ERROR_NOTICE = 0;
	uint32_t SPI1_ERROR_COUNT = 0;
	uint32_t SPI2_ERROR_COUNT = 0;

	HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  RXData[1] = 25;

	  if(IMU_send_cmd(&IMU1_data, TXData, RXData, length) != HAL_OK)
	  {
		  HAL_Delay(50);
	  }
	  debug_uart4_write_text("WHO AM I (1): ");debug_uart4_write_int((int)RXData[1]);

	  if(RXData[1] != 71)
	  {
		  SPI1_ERROR_COUNT++;
		  ERROR_NOTICE = 1;
	  }

	  RXData[1] = 25;
	  IMU_send_cmd(&IMU2_data, TXData, RXData, length);
	  debug_uart4_write_text("; WHO AM I (2): ");debug_uart4_write_int((int)RXData[1]);

	  if(RXData[1] != 71)
	  {
		  SPI2_ERROR_COUNT++;
		  ERROR_NOTICE = 1;
	  }

	  if(ERROR_NOTICE == 1)
	  {
		  debug_uart4_write_text("  ");debug_uart4_write_int((int)SPI1_ERROR_COUNT);debug_uart4_write_text(" SPI1 errors, ");debug_uart4_write_int((int)SPI2_ERROR_COUNT);debug_uart4_write_text(" SPI2 errors");
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Debug_send_Tempreture();
	  HAL_Delay(250);
	  int i = 0;
	  /*
	  while(i < 0xFFFFFF)
	  {
		  i++;
	  }
	  */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 37;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 24;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Debug_send_Tempreture(void)
{
	//Digital temperature sensor has to be enabeled before call
	int32_t Temperature;
	  debug_uart4_write_text("\nMCU temp: ");
	  HAL_DTS_GetTemperature(&hdts, &Temperature);//acquires a sensor reading
	  debug_uart4_write_int((int)Temperature);
	  debug_uart4_write_text("° \n");
}

HAL_StatusTypeDef debug_uart4_write_int(int number)
{
	//Sends a number over UART
	char temp[15] = {0};
	sprintf(temp, "%d", number );
	if (HAL_UART_Transmit(&huart4, temp, 10, 10000) != HAL_OK)
	  {
	    return HAL_ERROR;
	  }
	return HAL_OK;
}

HAL_StatusTypeDef debug_uart4_write_text(char *pData)
{
	//Sends a text message over UART
	if (HAL_UART_Transmit(&huart4, pData, strlen(pData), 10000) != HAL_OK)
	{
	  return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef debug_uart4_write_float(float number)
{
	//Sends a number over UART
	char temp[15];
	memset(temp, 0, 15);
	sprintf(temp, "%f", number );
	if (HAL_UART_Transmit(&huart4, temp, 10, 10000) != HAL_OK)
	  {
	    return HAL_ERROR;
	  }
	return HAL_OK;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
