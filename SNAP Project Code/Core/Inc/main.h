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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
#define BATT_ADC_Pin GPIO_PIN_1
#define BATT_ADC_GPIO_Port GPIOA
#define RGB_RED_Pin GPIO_PIN_2
#define RGB_RED_GPIO_Port GPIOA
#define RGB_BLUE_Pin GPIO_PIN_3
#define RGB_BLUE_GPIO_Port GPIOA
#define RGB_GREEN_Pin GPIO_PIN_4
#define RGB_GREEN_GPIO_Port GPIOA
#define RESET_SHT31_Pin GPIO_PIN_5
#define RESET_SHT31_GPIO_Port GPIOA
#define MT3608_EN_Pin GPIO_PIN_6
#define MT3608_EN_GPIO_Port GPIOA
#define OPAMP_ADC_Pin GPIO_PIN_7
#define OPAMP_ADC_GPIO_Port GPIOA
#define BATT_STAT_Pin GPIO_PIN_9
#define BATT_STAT_GPIO_Port GPIOA
#define BATT_STAT_EXTI_IRQn EXTI9_5_IRQn
#define BUTT_BT_Pin GPIO_PIN_0
#define BUTT_BT_GPIO_Port GPIOB
#define BUTT_1_Pin GPIO_PIN_1
#define BUTT_1_GPIO_Port GPIOB
#define BUTT_2_Pin GPIO_PIN_4
#define BUTT_2_GPIO_Port GPIOE
#define LED_WP7_EN_Pin GPIO_PIN_10
#define LED_WP7_EN_GPIO_Port GPIOA
#define PWM_VREF_Pin GPIO_PIN_15
#define PWM_VREF_GPIO_Port GPIOA
#define LED_4273_EN_Pin GPIO_PIN_4
#define LED_4273_EN_GPIO_Port GPIOB
#define LED_4261_EN_Pin GPIO_PIN_5
#define LED_4261_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
