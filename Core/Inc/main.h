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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DCMot_right_ENB_GPO_Pin GPIO_PIN_1
#define DCMot_right_ENB_GPO_GPIO_Port GPIOC
#define IPS_right_GPI_Pin GPIO_PIN_2
#define IPS_right_GPI_GPIO_Port GPIOC
#define IPS_left_GPI_Pin GPIO_PIN_3
#define IPS_left_GPI_GPIO_Port GPIOC
#define DCMotor_right_IN1B_T2C1_PWM_Pin GPIO_PIN_0
#define DCMotor_right_IN1B_T2C1_PWM_GPIO_Port GPIOA
#define DCMotor_right_IN2B_T2C2_PWM_Pin GPIO_PIN_1
#define DCMotor_right_IN2B_T2C2_PWM_GPIO_Port GPIOA
#define SS_right_T15C1_ICDM_Pin GPIO_PIN_2
#define SS_right_T15C1_ICDM_GPIO_Port GPIOA
#define SS_left_T15C2_ICDM_Pin GPIO_PIN_3
#define SS_left_T15C2_ICDM_GPIO_Port GPIOA
#define uServo_T16C1_PWM_Pin GPIO_PIN_6
#define uServo_T16C1_PWM_GPIO_Port GPIOA
#define USS_TRIG_T17C1_PWM_Pin GPIO_PIN_7
#define USS_TRIG_T17C1_PWM_GPIO_Port GPIOA
#define DCMot_left_ENA_GPO_Pin GPIO_PIN_10
#define DCMot_left_ENA_GPO_GPIO_Port GPIOA
#define LTS_left_GPI_Pin GPIO_PIN_10
#define LTS_left_GPI_GPIO_Port GPIOC
#define LTS_right_GPI_Pin GPIO_PIN_11
#define LTS_right_GPI_GPIO_Port GPIOC
#define LTS_middle_GPI_Pin GPIO_PIN_12
#define LTS_middle_GPI_GPIO_Port GPIOC
#define DCMotor_left_IN1A_T3C1_PWM_Pin GPIO_PIN_4
#define DCMotor_left_IN1A_T3C1_PWM_GPIO_Port GPIOB
#define DCMotor_left_IN2A_T3C2_PWM_Pin GPIO_PIN_5
#define DCMotor_left_IN2A_T3C2_PWM_GPIO_Port GPIOB
#define USS_ECHO_T8C1_ICDM_Pin GPIO_PIN_6
#define USS_ECHO_T8C1_ICDM_GPIO_Port GPIOB
#define RGB_Lights_T4C4_PWM_Pin GPIO_PIN_9
#define RGB_Lights_T4C4_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
