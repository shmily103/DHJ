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
void System_FaultShutdown(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_C1_Pin GPIO_PIN_1
#define ADC_C1_GPIO_Port GPIOA
#define ADC_C2_Pin GPIO_PIN_2
#define ADC_C2_GPIO_Port GPIOA
#define ADC_C3_Pin GPIO_PIN_3
#define ADC_C3_GPIO_Port GPIOA
#define ADC_C4_Pin GPIO_PIN_4
#define ADC_C4_GPIO_Port GPIOA
#define TFT_SCK_Pin GPIO_PIN_5
#define TFT_SCK_GPIO_Port GPIOA
#define TFT_SDA_Pin GPIO_PIN_7
#define TFT_SDA_GPIO_Port GPIOA
#define TFT_RES_Pin GPIO_PIN_0
#define TFT_RES_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_1
#define TFT_DC_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_10
#define TFT_CS_GPIO_Port GPIOB
#define TFT_BL_Pin GPIO_PIN_11
#define TFT_BL_GPIO_Port GPIOB
#define CH_EN_Pin GPIO_PIN_12
#define CH_EN_GPIO_Port GPIOB
#define Touch_Pin GPIO_PIN_14
#define Touch_GPIO_Port GPIOB
#define Touch_EXTI_IRQn EXTI15_10_IRQn
#define Pulse_PWM_Pin GPIO_PIN_8
#define Pulse_PWM_GPIO_Port GPIOA
#define C4_Pin GPIO_PIN_9
#define C4_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_10
#define C3_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_11
#define C2_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_12
#define C1_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
#define EC11_E_Pin GPIO_PIN_3
#define EC11_E_GPIO_Port GPIOB
#define EC11_E_EXTI_IRQn EXTI3_IRQn
#define EC11_B_Pin GPIO_PIN_4
#define EC11_B_GPIO_Port GPIOB
#define EC11_A_Pin GPIO_PIN_5
#define EC11_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
