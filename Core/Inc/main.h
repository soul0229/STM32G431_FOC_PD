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
#include "stm32g4xx_hal.h"

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
#define KEY0_Pin GPIO_PIN_13
#define KEY0_GPIO_Port GPIOC
#define KEY0_EXTI_IRQn EXTI15_10_IRQn
#define KEY1_Pin GPIO_PIN_14
#define KEY1_GPIO_Port GPIOC
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define KEY2_Pin GPIO_PIN_15
#define KEY2_GPIO_Port GPIOC
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define VV_B_Pin GPIO_PIN_1
#define VV_B_GPIO_Port GPIOF
#define HALL1_U_Pin GPIO_PIN_0
#define HALL1_U_GPIO_Port GPIOA
#define HALL1_V_Pin GPIO_PIN_1
#define HALL1_V_GPIO_Port GPIOA
#define HALL1_W_Pin GPIO_PIN_2
#define HALL1_W_GPIO_Port GPIOA
#define VU_A_Pin GPIO_PIN_3
#define VU_A_GPIO_Port GPIOA
#define VW_B_Pin GPIO_PIN_4
#define VW_B_GPIO_Port GPIOA
#define V_POWER_Pin GPIO_PIN_5
#define V_POWER_GPIO_Port GPIOA
#define HALL2_U_Pin GPIO_PIN_6
#define HALL2_U_GPIO_Port GPIOA
#define HALL2_V_Pin GPIO_PIN_7
#define HALL2_V_GPIO_Port GPIOA
#define HALL2_W_Pin GPIO_PIN_0
#define HALL2_W_GPIO_Port GPIOB
#define VV_A_Pin GPIO_PIN_1
#define VV_A_GPIO_Port GPIOB
#define VU_B_Pin GPIO_PIN_2
#define VU_B_GPIO_Port GPIOB
#define VW_A_Pin GPIO_PIN_12
#define VW_A_GPIO_Port GPIOB
#define MOTOR1_UN_Pin GPIO_PIN_13
#define MOTOR1_UN_GPIO_Port GPIOB
#define MOTOR1_VN_Pin GPIO_PIN_14
#define MOTOR1_VN_GPIO_Port GPIOB
#define MOTOR1_WN_Pin GPIO_PIN_15
#define MOTOR1_WN_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define MOTOR1_UP_Pin GPIO_PIN_8
#define MOTOR1_UP_GPIO_Port GPIOA
#define MOTOR1_VP_Pin GPIO_PIN_9
#define MOTOR1_VP_GPIO_Port GPIOA
#define MOTOR1_WP_Pin GPIO_PIN_10
#define MOTOR1_WP_GPIO_Port GPIOA
#define MOTOR2_UP_Pin GPIO_PIN_15
#define MOTOR2_UP_GPIO_Port GPIOA
#define MOTOR2_UN_Pin GPIO_PIN_10
#define MOTOR2_UN_GPIO_Port GPIOC
#define MOTOR2_VN_Pin GPIO_PIN_11
#define MOTOR2_VN_GPIO_Port GPIOC
#define MOTOR2_WN_Pin GPIO_PIN_5
#define MOTOR2_WN_GPIO_Port GPIOB
#define MOTOR2_VP_Pin GPIO_PIN_8
#define MOTOR2_VP_GPIO_Port GPIOB
#define MOTOR2_WP_Pin GPIO_PIN_9
#define MOTOR2_WP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
