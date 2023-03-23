/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DISP_5V_CONTROL_Pin GPIO_PIN_13
#define DISP_5V_CONTROL_GPIO_Port GPIOD
#define OUT_PK1_Pin GPIO_PIN_8
#define OUT_PK1_GPIO_Port GPIOC
#define OUT_PK2_Pin GPIO_PIN_9
#define OUT_PK2_GPIO_Port GPIOC
#define OUT_PK3_Pin GPIO_PIN_8
#define OUT_PK3_GPIO_Port GPIOA
#define OUT_PK4_Pin GPIO_PIN_9
#define OUT_PK4_GPIO_Port GPIOA
#define OUT_PK5_Pin GPIO_PIN_11
#define OUT_PK5_GPIO_Port GPIOA
#define OUT_PK6_Pin GPIO_PIN_12
#define OUT_PK6_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
