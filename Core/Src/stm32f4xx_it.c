/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc2;
extern CAN_HandleTypeDef hcan2;
extern DAC_HandleTypeDef hdac;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;
extern WWDG_HandleTypeDef hwwdg;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
//void DebugMon_Handler(void)
//{
//  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
//
//  /* USER CODE END DebugMonitor_IRQn 0 */
//  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
//
//  /* USER CODE END DebugMonitor_IRQn 1 */
//}
//
///******************************************************************************/
///* STM32F4xx Peripheral Interrupt Handlers                                    */
///* Add here the Interrupt Handlers for the used peripherals.                  */
///* For the available peripheral interrupt handler names,                      */
///* please refer to the startup file (startup_stm32f4xx.s).                    */
///******************************************************************************/
//
///**
//  * @brief This function handles Window watchdog interrupt.
//  */
//void WWDG_IRQHandler(void)
//{
//  /* USER CODE BEGIN WWDG_IRQn 0 */
//
//  /* USER CODE END WWDG_IRQn 0 */
//  HAL_WWDG_IRQHandler(&hwwdg);
//  /* USER CODE BEGIN WWDG_IRQn 1 */
//
//  /* USER CODE END WWDG_IRQn 1 */
//}
//
///**
//  * @brief This function handles RCC global interrupt.
//  */
//void RCC_IRQHandler(void)
//{
//  /* USER CODE BEGIN RCC_IRQn 0 */
//
//  /* USER CODE END RCC_IRQn 0 */
//  /* USER CODE BEGIN RCC_IRQn 1 */
//
//  /* USER CODE END RCC_IRQn 1 */
//}
//
///**
//  * @brief This function handles ADC1, ADC2 and ADC3 interrupts.
//  */
//void ADC_IRQHandler(void)
//{
//  /* USER CODE BEGIN ADC_IRQn 0 */
//
//  /* USER CODE END ADC_IRQn 0 */
//  HAL_ADC_IRQHandler(&hadc2);
//  /* USER CODE BEGIN ADC_IRQn 1 */
//
//  /* USER CODE END ADC_IRQn 1 */
//}
//
///**
//  * @brief This function handles TIM4 global interrupt.
//  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
//
///**
//  * @brief This function handles I2C2 event interrupt.
//  */
//void I2C2_EV_IRQHandler(void)
//{
//  /* USER CODE BEGIN I2C2_EV_IRQn 0 */
//
//  /* USER CODE END I2C2_EV_IRQn 0 */
//  HAL_I2C_EV_IRQHandler(&hi2c2);
//  /* USER CODE BEGIN I2C2_EV_IRQn 1 */
//
//  /* USER CODE END I2C2_EV_IRQn 1 */
//}
//
///**
//  * @brief This function handles I2C2 error interrupt.
//  */
//void I2C2_ER_IRQHandler(void)
//{
//  /* USER CODE BEGIN I2C2_ER_IRQn 0 */
//
//  /* USER CODE END I2C2_ER_IRQn 0 */
//  HAL_I2C_ER_IRQHandler(&hi2c2);
//  /* USER CODE BEGIN I2C2_ER_IRQn 1 */
//
//  /* USER CODE END I2C2_ER_IRQn 1 */
//}
//
///**
//  * @brief This function handles SPI2 global interrupt.
//  */
//void SPI2_IRQHandler(void)
//{
//  /* USER CODE BEGIN SPI2_IRQn 0 */
////
//  /* USER CODE END SPI2_IRQn 0 */
//  HAL_SPI_IRQHandler(&hspi2);
//  /* USER CODE BEGIN SPI2_IRQn 1 */
////
//  /* USER CODE END SPI2_IRQn 1 */
//}
//
///**
//  * @brief This function handles USART2 global interrupt.
//  */
//void USART2_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
////
//  /* USER CODE END USART2_IRQn 0 */
//  HAL_UART_IRQHandler(&huart2);
//  /* USER CODE BEGIN USART2_IRQn 1 */
////
//  /* USER CODE END USART2_IRQn 1 */
//}
//
///**
//  * @brief This function handles EXTI line[15:10] interrupts.
//  */
//void EXTI15_10_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
//
//  /* USER CODE END EXTI15_10_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
//  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
//
//  /* USER CODE END EXTI15_10_IRQn 1 */
//}
//
///**
//  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
//  */
//void TIM6_DAC_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
//
//  /* USER CODE END TIM6_DAC_IRQn 0 */
//  HAL_DAC_IRQHandler(&hdac);
//  HAL_TIM_IRQHandler(&htim6);
//  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
//
//  /* USER CODE END TIM6_DAC_IRQn 1 */
//}
//
///**
//  * @brief This function handles CAN2 TX interrupt.
//  */
//void CAN2_TX_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN2_TX_IRQn 0 */
//
//  /* USER CODE END CAN2_TX_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan2);
//  /* USER CODE BEGIN CAN2_TX_IRQn 1 */
//
//  /* USER CODE END CAN2_TX_IRQn 1 */
//}
//
///**
//  * @brief This function handles CAN2 RX0 interrupt.
//  */
//void CAN2_RX0_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
//
//  /* USER CODE END CAN2_RX0_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan2);
//  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */
//
//  /* USER CODE END CAN2_RX0_IRQn 1 */
//}
//
///**
//  * @brief This function handles CAN2 RX1 interrupt.
//  */
//void CAN2_RX1_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN2_RX1_IRQn 0 */
//
//  /* USER CODE END CAN2_RX1_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan2);
//  /* USER CODE BEGIN CAN2_RX1_IRQn 1 */
//
//  /* USER CODE END CAN2_RX1_IRQn 1 */
//}
//
///**
//  * @brief This function handles CAN2 SCE interrupt.
//  */
//void CAN2_SCE_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN2_SCE_IRQn 0 */
//
//  /* USER CODE END CAN2_SCE_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan2);
//  /* USER CODE BEGIN CAN2_SCE_IRQn 1 */
//
//  /* USER CODE END CAN2_SCE_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
