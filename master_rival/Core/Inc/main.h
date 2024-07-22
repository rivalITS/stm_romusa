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
#include "stm32f4xx_hal.h"

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
#define PWM_LIFT_Pin GPIO_PIN_5
#define PWM_LIFT_GPIO_Port GPIOE
#define ENC0_A_Pin GPIO_PIN_0
#define ENC0_A_GPIO_Port GPIOA
#define ENC0_B_Pin GPIO_PIN_1
#define ENC0_B_GPIO_Port GPIOA
#define Motor_s_bak_Pin GPIO_PIN_5
#define Motor_s_bak_GPIO_Port GPIOC
#define pneu_Pin GPIO_PIN_0
#define pneu_GPIO_Port GPIOB
#define vac_Pin GPIO_PIN_2
#define vac_GPIO_Port GPIOB
#define retryBut_Pin GPIO_PIN_9
#define retryBut_GPIO_Port GPIOE
#define LIFT_D1_Pin GPIO_PIN_10
#define LIFT_D1_GPIO_Port GPIOE
#define LIFT_D2_Pin GPIO_PIN_13
#define LIFT_D2_GPIO_Port GPIOE
#define ARM_D1_Pin GPIO_PIN_12
#define ARM_D1_GPIO_Port GPIOB
#define ARM_D2_Pin GPIO_PIN_13
#define ARM_D2_GPIO_Port GPIOB
#define PWM_ARM_Pin GPIO_PIN_15
#define PWM_ARM_GPIO_Port GPIOB
#define mini_pump_Pin GPIO_PIN_8
#define mini_pump_GPIO_Port GPIOD
#define buzz_Pin GPIO_PIN_9
#define buzz_GPIO_Port GPIOD
#define ENC1_A_Pin GPIO_PIN_6
#define ENC1_A_GPIO_Port GPIOC
#define ENC1_B_Pin GPIO_PIN_7
#define ENC1_B_GPIO_Port GPIOC
#define startBut_Pin GPIO_PIN_3
#define startBut_GPIO_Port GPIOB
#define prox1_Pin GPIO_PIN_4
#define prox1_GPIO_Port GPIOB
#define prox2_Pin GPIO_PIN_5
#define prox2_GPIO_Port GPIOB
#define mapBut_Pin GPIO_PIN_7
#define mapBut_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
