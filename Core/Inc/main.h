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
//#define B1_Pin GPIO_PIN_13
//#define B1_GPIO_Port GPIOC
//#define B1_EXTI_IRQn EXTI15_10_IRQn
//#define ANALOG_C0_Pin GPIO_PIN_0
//#define ANALOG_C0_GPIO_Port GPIOC
//#define ANALOG_C3_Pin GPIO_PIN_3
//#define ANALOG_C3_GPIO_Port GPIOC
//#define ANALOG_A1_Pin GPIO_PIN_1
//#define ANALOG_A1_GPIO_Port GPIOA
//#define LD2_Pin GPIO_PIN_5
//#define LD2_GPIO_Port GPIOA
//#define OUTPUT_C4_Pin GPIO_PIN_4
//#define OUTPUT_C4_GPIO_Port GPIOC
//#define ANALOG_C5_Pin GPIO_PIN_5
//#define ANALOG_C5_GPIO_Port GPIOC
//#define ANALOG_B0_Pin GPIO_PIN_0
//#define ANALOG_B0_GPIO_Port GPIOB
//#define ANALOG_B1_Pin GPIO_PIN_1
//#define ANALOG_B1_GPIO_Port GPIOB
//#define ANALOG_B2_Pin GPIO_PIN_2
//#define ANALOG_B2_GPIO_Port GPIOB
//#define SPI2_CS1_Pin GPIO_PIN_14
//#define SPI2_CS1_GPIO_Port GPIOB
//#define SPI2_CS2_Pin GPIO_PIN_15
//#define SPI2_CS2_GPIO_Port GPIOB
//#define ANALOG_C6_Pin GPIO_PIN_6
//#define ANALOG_C6_GPIO_Port GPIOC
//#define SPI3_CS1_Pin GPIO_PIN_8
//#define SPI3_CS1_GPIO_Port GPIOC
//#define OUTPUT_C9_Pin GPIO_PIN_9
//#define OUTPUT_C9_GPIO_Port GPIOC
//#define INPUT_A9_Pin GPIO_PIN_9
//#define INPUT_A9_GPIO_Port GPIOA
//#define OUTPUT_A10_Pin GPIO_PIN_10
//#define OUTPUT_A10_GPIO_Port GPIOA
//#define OUTPUT_A11_Pin GPIO_PIN_11
//#define OUTPUT_A11_GPIO_Port GPIOA
//#define INPUT_A12_Pin GPIO_PIN_12
//#define INPUT_A12_GPIO_Port GPIOA
//#define TMS_Pin GPIO_PIN_13
//#define TMS_GPIO_Port GPIOA
//#define TCK_Pin GPIO_PIN_14
//#define TCK_GPIO_Port GPIOA
//#define OUTPUT_A15_Pin GPIO_PIN_15
//#define OUTPUT_A15_GPIO_Port GPIOA
//#define OUTPUT_C10_Pin GPIO_PIN_10
//#define OUTPUT_C10_GPIO_Port GPIOC
//#define OUTPUT_C11_Pin GPIO_PIN_11
//#define OUTPUT_C11_GPIO_Port GPIOC
//#define OUTPUT_D2_Pin GPIO_PIN_2
//#define OUTPUT_D2_GPIO_Port GPIOD
//#define OUTPUT_B3_Pin GPIO_PIN_3
//#define OUTPUT_B3_GPIO_Port GPIOB
//#define OUTPUT_B4_Pin GPIO_PIN_4
//#define OUTPUT_B4_GPIO_Port GPIOB
//#define INPUT_B5_Pin GPIO_PIN_5
//#define INPUT_B5_GPIO_Port GPIOB
//#define INPUT_B6_Pin GPIO_PIN_6
//#define INPUT_B6_GPIO_Port GPIOB
//#define INPUT_B7_Pin GPIO_PIN_7
//#define INPUT_B7_GPIO_Port GPIOB
//#define INPUT_B8_Pin GPIO_PIN_8
//#define INPUT_B8_GPIO_Port GPIOB
//#define INPUT_B9_Pin GPIO_PIN_9
//#define INPUT_B9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
