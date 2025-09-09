/** 
  ******************************************************************************
  * @file    x_nucleo_ihm04a1_stm32l0xx.h
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    September 29, 2016
  * @brief   Header for BSP driver for x-nucleo-ihm04a1 Nucleo extension board 
  *  (based on L6206)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef X_NUCLEO_IHM04A1_STM32L0XX_H
#define X_NUCLEO_IHM04A1_STM32L0XX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_nucleo.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup X_NUCLEO_IHM04A1_STM32L0XX
  * @{   
  */   
   
/* Exported Constants --------------------------------------------------------*/
   
/** @defgroup IHM04A1_Exported_Constants IHM04A1 Exported Constants
  * @{
  */   
   
/******************************************************************************/
/* USE_STM32L0XX_NUCLEO                                                       */
/******************************************************************************/

 /** @defgroup Constants_For_STM32L0XX_NUCLEO  Constants for STM32L0XX NUCLEO
* @{
*/   
/// Interrupt line used for L6206 Over Current Detection and over Temperature On Bridge A
#define EXTI_FLAG_A_IRQn           (EXTI4_15_IRQn)

/// Interrupt line used for L6206 Over Current Detection and over Temperature On Bridge B
#define EXTI_FLAG_B_IRQn           (EXTI0_1_IRQn)   
   
/// Timer used for PWM_1A
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A      (TIM22)

/// Timer used for PWM_2A
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A      (TIM22)

/// Timer used for PWM_1B
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B      (TIM2)

/// Timer used for PWM_2B
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B      (TIM2)   

/// Channel Timer used for PWM_1A
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1A      (TIM_CHANNEL_1)

/// Channel Timer used for PWM_2A
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2A      (TIM_CHANNEL_2)

/// Channel Timer used for PWM_1B
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1B      (TIM_CHANNEL_1)

/// Channel Timer used for PWM_2B
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2B      (TIM_CHANNEL_2)

/// HAL Active Channel Timer used for PWM_1A
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1A      (HAL_TIM_ACTIVE_CHANNEL_1)

/// HAL Active Channel Timer used for PWM_2A
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2A      (HAL_TIM_ACTIVE_CHANNEL_2)

/// HAL Active Channel Timer used for PWM_1B
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1B      (HAL_TIM_ACTIVE_CHANNEL_1)

/// HAL Active Channel Timer used for PWM_2B
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2B      (HAL_TIM_ACTIVE_CHANNEL_2)
   
/// Timer Clock Enable for PWM_1A
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A_CLCK_ENABLE()  __TIM22_CLK_ENABLE()

/// Timer Clock Enable for PWM_2A
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A_CLCK_ENABLE()  __TIM22_CLK_ENABLE()

/// Timer Clock Enable for PWM_1B
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B_CLCK_ENABLE()  __TIM2_CLK_ENABLE()

/// Timer Clock Enable for PWM_2B
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B_CLCK_ENABLE()  __TIM2_CLK_ENABLE()
   
/// Timer Clock Enable for PWM_1A
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A_CLCK_DISABLE()  __TIM22_CLK_DISABLE()

/// Timer Clock Enable for PWM_2A
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A_CLCK_DISABLE()  __TIM22_CLK_DISABLE()

/// Timer Clock Enable for PWM_1B
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B_CLCK_DISABLE()  __TIM2_CLK_DISABLE()

/// Timer Clock Enable for PWM_2B
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B_CLCK_DISABLE()  __TIM2_CLK_DISABLE()

/// PWM1A GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM1A  (GPIO_AF4_TIM22)

/// PWM2A GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM2A  (GPIO_AF4_TIM22)

/// PWM1A GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM1B  (GPIO_AF2_TIM2)

/// PWM2A GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM2B  (GPIO_AF2_TIM2)
   
 /**
* @}
*/

/******************************************************************************/
/* Independent plateform definitions                                          */
/******************************************************************************/

   /** @defgroup Constants_For_All_Nucleo_Platforms Constants For All Nucleo Platforms
* @{
*/   

/// GPIO Pin used for the PWM of the L6206 Brige A Input 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_1A_PIN  (GPIO_PIN_4)
/// GPIO Port used for the PWM of the L6206 Brige A Input 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_1A_PORT  (GPIOB)

/// GPIO Pin used for the PWM of the L6206 Brige A Input 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_2A_PIN  (GPIO_PIN_5)
/// GPIO Port used for the PWM of the L6206 Brige A Input 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_2A_PORT  (GPIOB)
   
/// GPIO Pin used for the PWM of the L6206 Brige BInput 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_1B_PIN  (GPIO_PIN_0)
/// GPIO Port used for the PWM of the L6206 Brige B Input 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_1B_PORT  (GPIOA)

/// GPIO Pin used for the PWM of the L6206 Brige B Input 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_2B_PIN  (GPIO_PIN_1)
/// GPIO Port used for the PWM of the L6206 Brige B Input 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_2B_PORT  (GPIOA)
   
/// GPIO Pin used for the L6206 Bridge A Enable pin and OCD and OVT alarms
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PIN  (GPIO_PIN_10)
/// GPIO port used for the L6206 Bridge A Enable pin and OCD and OVT alarms
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PORT (GPIOA)
/// Flag of bridge A interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PRIORITY             (3)

/// GPIO Pin used for the L6206 Bridge B Enable pin and OCD and OVT alarms
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PIN  (GPIO_PIN_1)
/// GPIO port used for the L6206 Bridge B Enable pin and OCD and OVT alarms
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PORT (GPIOC)  
/// Flag of bridge B interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PRIORITY             (3)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* X_NUCLEO_IHM04A1_STM32L0XX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
