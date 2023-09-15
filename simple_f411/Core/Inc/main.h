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
#define HOME_Pin GPIO_PIN_0
#define HOME_GPIO_Port GPIOA
#define HOME_EXTI_IRQn EXTI0_IRQn
#define MOV_Pin GPIO_PIN_1
#define MOV_GPIO_Port GPIOA
#define MOVE_EN_Pin GPIO_PIN_2
#define MOVE_EN_GPIO_Port GPIOA
#define RAMP_Pin GPIO_PIN_3
#define RAMP_GPIO_Port GPIOA
#define RDY_Pin GPIO_PIN_4
#define RDY_GPIO_Port GPIOA
#define REC_Pin GPIO_PIN_0
#define REC_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_1
#define SPI1_CS_GPIO_Port GPIOB
#define E_MUX_Pin GPIO_PIN_2
#define E_MUX_GPIO_Port GPIOB
#define ENA_stepper_Pin GPIO_PIN_10
#define ENA_stepper_GPIO_Port GPIOB
#define S0_MUX_Pin GPIO_PIN_12
#define S0_MUX_GPIO_Port GPIOB
#define S1_MUX_Pin GPIO_PIN_13
#define S1_MUX_GPIO_Port GPIOB
#define S2_MUX_Pin GPIO_PIN_14
#define S2_MUX_GPIO_Port GPIOB
#define S3_MUX_Pin GPIO_PIN_15
#define S3_MUX_GPIO_Port GPIOB
#define COM_IO_MUX_Pin GPIO_PIN_8
#define COM_IO_MUX_GPIO_Port GPIOA
#define PUL_stepper_Pin GPIO_PIN_6
#define PUL_stepper_GPIO_Port GPIOB
#define DIR_stepper_Pin GPIO_PIN_9
#define DIR_stepper_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
