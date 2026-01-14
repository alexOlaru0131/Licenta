/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define M3_IN1_Pin GPIO_PIN_0
#define M3_IN1_GPIO_Port GPIOC
#define M3_IN2_Pin GPIO_PIN_1
#define M3_IN2_GPIO_Port GPIOC
#define M4_IN3_Pin GPIO_PIN_2
#define M4_IN3_GPIO_Port GPIOC
#define M4_IN4_Pin GPIO_PIN_3
#define M4_IN4_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SMPS_EN_Pin GPIO_PIN_4
#define SMPS_EN_GPIO_Port GPIOA
#define SMPS_V1_Pin GPIO_PIN_5
#define SMPS_V1_GPIO_Port GPIOA
#define SMPS_PG_Pin GPIO_PIN_6
#define SMPS_PG_GPIO_Port GPIOA
#define SMPS_SW_Pin GPIO_PIN_7
#define SMPS_SW_GPIO_Port GPIOA
#define M1_ENC_Pin GPIO_PIN_1
#define M1_ENC_GPIO_Port GPIOB
#define M1_ENC_EXTI_IRQn EXTI1_IRQn
#define M2_ENC_Pin GPIO_PIN_2
#define M2_ENC_GPIO_Port GPIOB
#define M2_ENC_EXTI_IRQn EXTI2_IRQn
#define M3_ENC_Pin GPIO_PIN_10
#define M3_ENC_GPIO_Port GPIOB
#define M3_ENC_EXTI_IRQn EXTI15_10_IRQn
#define M4_ENC_Pin GPIO_PIN_11
#define M4_ENC_GPIO_Port GPIOB
#define M4_ENC_EXTI_IRQn EXTI15_10_IRQn
#define LD4_Pin GPIO_PIN_13
#define LD4_GPIO_Port GPIOB
#define M2_IN4_Pin GPIO_PIN_6
#define M2_IN4_GPIO_Port GPIOC
#define M2_IN3_Pin GPIO_PIN_7
#define M2_IN3_GPIO_Port GPIOC
#define M1_IN2_Pin GPIO_PIN_8
#define M1_IN2_GPIO_Port GPIOC
#define M1_IN1_Pin GPIO_PIN_9
#define M1_IN1_GPIO_Port GPIOC
#define M4_PWM_GEN_Pin GPIO_PIN_8
#define M4_PWM_GEN_GPIO_Port GPIOA
#define M3_PWM_GEN_Pin GPIO_PIN_9
#define M3_PWM_GEN_GPIO_Port GPIOA
#define M2_PWM_GEN_Pin GPIO_PIN_10
#define M2_PWM_GEN_GPIO_Port GPIOA
#define M1_PWM_GEN_Pin GPIO_PIN_11
#define M1_PWM_GEN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
