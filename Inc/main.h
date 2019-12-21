/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define SWITCH_Pin GPIO_PIN_1
#define SWITCH_GPIO_Port GPIOA
#define SWITCH_EXTI_IRQn EXTI1_IRQn
#define EN_A_Pin GPIO_PIN_2
#define EN_A_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_3
#define A1_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_4
#define A2_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_5
#define A3_GPIO_Port GPIOA
#define BAT_STAT_Pin GPIO_PIN_6
#define BAT_STAT_GPIO_Port GPIOA
#define BUZ_Pin GPIO_PIN_12
#define BUZ_GPIO_Port GPIOB
#define K1_Pin GPIO_PIN_5
#define K1_GPIO_Port GPIOB
#define K0_Pin GPIO_PIN_6
#define K0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LED_COUNT 16
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
