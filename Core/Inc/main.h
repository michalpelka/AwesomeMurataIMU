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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define DBG_Pin GPIO_PIN_2
#define DBG_GPIO_Port GPIOA
#define DRY_Pin GPIO_PIN_3
#define DRY_GPIO_Port GPIOA
#define DRY_EXTI_IRQn EXTI3_IRQn
#define EXTERNSN_Pin GPIO_PIN_4
#define EXTERNSN_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_0
#define SPI_CS_GPIO_Port GPIOB
#define NMEA_IN_Pin GPIO_PIN_11
#define NMEA_IN_GPIO_Port GPIOB
#define PPS_IN_Pin GPIO_PIN_6
#define PPS_IN_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
