/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "winder.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint64_t overflow_count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3)
		overflow_count += 65536;
}

// needed to send printf to the serial port
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void getShortCompileDate(char *result) {
	char *date = __DATE__;  // e.g., "Jul  4 2025"

	strcpy(result, "0-/--/--");

	if (strncmp(date, "Jan", 3) == 0) {
		result[1] = '1';
	} else if (strncmp(date, "Feb", 3) == 0) {
		result[1] = '2';
	} else if (strncmp(date, "Mar", 3) == 0) {
		result[1] = '3';
	} else if (strncmp(date, "Apr", 3) == 0) {
		result[1] = '4';
	} else if (strncmp(date, "May", 3) == 0) {
		result[1] = '5';
	} else if (strncmp(date, "Jun", 3) == 0) {
		result[1] = '6';
	} else if (strncmp(date, "Jul", 3) == 0) {
		result[1] = '7';
	} else if (strncmp(date, "Aug", 3) == 0) {
		result[1] = '8';
	} else if (strncmp(date, "Sep", 3) == 0) {
		result[1] = '9';
	} else if (strncmp(date, "Oct", 3) == 0) {
		result[0] = '1';
		result[1] = '0';
	} else if (strncmp(date, "Nov", 3) == 0) {
		result[0] = '1';
		result[1] = '1';
	} else if (strncmp(date, "Dec", 3) == 0) {
		result[0] = '1';
		result[1] = '2';
	} else {
		result[0] = 'X';
		result[1] = 'X';
	}

	strncpy(&result[3], &date[4], 2);
	if (result[3] == ' ')
		result[3] = '0';

	strncpy(&result[6], &date[9], 2);
}

uint64_t getTicks(void) {
	uint64_t count;
	uint16_t tim_cnt;
	uint64_t of;

	__disable_irq();

	of = overflow_count;
	tim_cnt = __HAL_TIM_GET_COUNTER(&htim3);

	// Check if overflow happened during this read
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) &&
	__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE)) {
		// Timer overflowed but interrupt not yet handled
		of += 65536;
	}

	__enable_irq();

	count = of + tim_cnt;
	return count;
}

void pulseAndWait(uint64_t pulseEnd) {

	HAL_GPIO_WritePin(BarrelPulse_GPIO_Port, BarrelPulse_Pin, 1);

	uint64_t endUp = getTicks() + 360; // 10 microseconds
	while (getTicks() < endUp);

	HAL_GPIO_WritePin(BarrelPulse_GPIO_Port, BarrelPulse_Pin, 0);

	while (getTicks() < pulseEnd);
}

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	uint8_t msg[] = "Hello over USART2\r\n";
	HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, HAL_MAX_DELAY);

	printf("qwewqewq\r\n");

	// get the date as mm/dd/yy
	char date[9];
	getShortCompileDate(date);
	printf("Hello from the Winder -- Build: %s %s\r\n", date, __TIME__);

	lcd_init(&hi2c1);
	lcd_set_cursor(0, 0);
	lcd_write_string("Winder ");
	lcd_write_string(date);

	lcd_set_cursor(7, 1);
	lcd_write_string(__TIME__);


	lcd_set_cursor(0, 2);
	lcd_write_string("Hello World :)");

	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	float clock = 36000000;
	float a = 300000;
	float vMax = 21760/2;

	float v;
	float pulse;

	float finalPulseWidth = clock / vMax;
	printf("finalPulseWidth =%.3f\r\n",(double)finalPulseWidth);

	float x = 1234.56f;
	uint32_t t1 = getTicks();  // TIM2 running at 1 MHz for example
	for(int n=0 ; n<1000 ; n++) {v = sqrtf(x);}
	uint32_t t2 = getTicks();

	printf("sqrtf time = %lu us\r\n", t2 - t1);
	HAL_Delay(5000);

	printf("Starting movement\r\n");

	uint64_t pulseStart = getTicks();

	uint32_t s = 0;
	do {
		s++;
		v = sqrt(2.0f * s * a);
		pulse = clock / v;
		//printf("accel pulse =%.3f\r\n",(double)pulse);
		pulseStart += pulse;
		pulseAndWait(pulseStart);

	} while (pulse > finalPulseWidth);

	//printf("accelerated s=%lu\r\n",s);

	uint32_t middlePulses = (21760*3) - (s * 2);

	for (uint32_t i = 0; i < middlePulses; i++) {
		pulseStart += finalPulseWidth;
		pulseAndWait(pulseStart);
	}

	//printf("middle done\r\n");


	for (uint32_t s2=0; s2<s; s2++) {
		v = sqrt(2.0f * (s-s2) * a);
		pulse = clock / v;
		//printf("decel pulse =%.3f\r\n",(double)pulse);

		pulseStart += pulse;
		pulseAndWait(pulseStart);
	}

	printf("movement complete s=%lu\r\n", s);

	while (1) {
		uint64_t start = getTicks();
		HAL_Delay(10000);
		uint64_t end = getTicks();

		int diff = (int) (end - start);
		printf("delta:%d\r\n", diff);

	}
	main_menu();
//	  if(HAL_GPIO_ReadPin(ButtonLeft_GPIO_Port, ButtonLeft_Pin) == 0) {
//		  printf("ButtonLeft\r\n");
//	  }
//
//	  if(HAL_GPIO_ReadPin(ButtonRight_GPIO_Port, ButtonRight_Pin) == 0) {
//		  printf("ButtonRight\r\n");
//	  }
//
//	  if(HAL_GPIO_ReadPin(ButtonTop_GPIO_Port, ButtonTop_Pin) == 0) {
//		  printf("ButtonTop\r\n");
//	  }
//
//	  if(HAL_GPIO_ReadPin(ButtonBottom_GPIO_Port, ButtonBottom_Pin) == 0) {
//		  printf("ButtonBottom\r\n");
//	  }
//
//	  if(HAL_GPIO_ReadPin(ButtonCenter_GPIO_Port, ButtonCenter_Pin) == 0) {
//		  printf("ButtonCenter\r\n");
//	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RedLight_GPIO_Port, RedLight_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CarriageDir_Pin|CarriagePulse_Pin|GreenLight_Pin|YellowLight_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BarrelDir_Pin|BarrelPulse_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RedLight_Pin */
  GPIO_InitStruct.Pin = RedLight_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RedLight_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ButtonCenter_Pin */
  GPIO_InitStruct.Pin = ButtonCenter_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ButtonCenter_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CarriageDir_Pin CarriagePulse_Pin GreenLight_Pin YellowLight_Pin */
  GPIO_InitStruct.Pin = CarriageDir_Pin|CarriagePulse_Pin|GreenLight_Pin|YellowLight_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BarrelDir_Pin BarrelPulse_Pin */
  GPIO_InitStruct.Pin = BarrelDir_Pin|BarrelPulse_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ButtonLeft_Pin ButtonBottom_Pin */
  GPIO_InitStruct.Pin = ButtonLeft_Pin|ButtonBottom_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitAlarm_Pin */
  GPIO_InitStruct.Pin = LimitAlarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LimitAlarm_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ButtonRight_Pin ButtonTop_Pin */
  GPIO_InitStruct.Pin = ButtonRight_Pin|ButtonTop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
while (1) {
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
