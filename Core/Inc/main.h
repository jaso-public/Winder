/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

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
#define RedLight_Pin GPIO_PIN_0
#define RedLight_GPIO_Port GPIOF
#define ButtonCenter_Pin GPIO_PIN_1
#define ButtonCenter_GPIO_Port GPIOF
#define EncoderA_Pin GPIO_PIN_0
#define EncoderA_GPIO_Port GPIOA
#define EncoderB_Pin GPIO_PIN_1
#define EncoderB_GPIO_Port GPIOA
#define LimitSwitch_Pin GPIO_PIN_3
#define LimitSwitch_GPIO_Port GPIOA
#define LimitSwitch_EXTI_IRQn EXTI3_IRQn
#define CarriageDir_Pin GPIO_PIN_4
#define CarriageDir_GPIO_Port GPIOA
#define CarriagePulse_Pin GPIO_PIN_7
#define CarriagePulse_GPIO_Port GPIOA
#define BarrelDir_Pin GPIO_PIN_0
#define BarrelDir_GPIO_Port GPIOB
#define BarrelPulse_Pin GPIO_PIN_1
#define BarrelPulse_GPIO_Port GPIOB
#define OpticalSensor_Pin GPIO_PIN_8
#define OpticalSensor_GPIO_Port GPIOA
#define GreenLight_Pin GPIO_PIN_9
#define GreenLight_GPIO_Port GPIOA
#define YellowLight_Pin GPIO_PIN_10
#define YellowLight_GPIO_Port GPIOA
#define ButtonBottom_Pin GPIO_PIN_11
#define ButtonBottom_GPIO_Port GPIOA
#define ButtonLeft_Pin GPIO_PIN_12
#define ButtonLeft_GPIO_Port GPIOA
#define Led_Pin GPIO_PIN_3
#define Led_GPIO_Port GPIOB
#define ButtonRight_Pin GPIO_PIN_4
#define ButtonRight_GPIO_Port GPIOB
#define ButtonTop_Pin GPIO_PIN_5
#define ButtonTop_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


int32_t readEncoderValue();

void Timer2_IRQHandler(void);
void Barrel_IRQHandler(void);
void Carriage_IRQHandler(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
