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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Bin1_Pin GPIO_PIN_5
#define Bin1_GPIO_Port GPIOE
#define Bin2_Pin GPIO_PIN_6
#define Bin2_GPIO_Port GPIOE
#define IR_L_Pin GPIO_PIN_0
#define IR_L_GPIO_Port GPIOC
#define IR_R_Pin GPIO_PIN_1
#define IR_R_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOE
#define US_Trig_Pin GPIO_PIN_14
#define US_Trig_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOD
#define OLED_RES_Pin GPIO_PIN_12
#define OLED_RES_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOD
#define US_Echo_Pin GPIO_PIN_7
#define US_Echo_GPIO_Port GPIOC
#define ENCODER_A1_Pin GPIO_PIN_15
#define ENCODER_A1_GPIO_Port GPIOA
#define ENCODER_A2_Pin GPIO_PIN_3
#define ENCODER_A2_GPIO_Port GPIOB
#define ENCODER_B1_Pin GPIO_PIN_4
#define ENCODER_B1_GPIO_Port GPIOB
#define ENCODER_B2_Pin GPIO_PIN_5
#define ENCODER_B2_GPIO_Port GPIOB
#define Ain2_Pin GPIO_PIN_8
#define Ain2_GPIO_Port GPIOB
#define Ain1_Pin GPIO_PIN_9
#define Ain1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
