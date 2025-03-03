/***********************************************************************************************************************
 * Main_Controller
 * peripheral_initialization.h
 *
 * wilson
 * 6/30/22
 * 7:27 PM
 *
 * Description:
 *
 **********************************************************************************************************************/


#ifndef MAIN_CONTROLLER_HAL_H
#define MAIN_CONTROLLER_HAL_H

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_uart.h"
/* third-party includes */

/* layer_0 includes */
//#include "hal_spi_old.h"
#include "hal_wrapper.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define ANALOG_C0_Pin GPIO_PIN_0
#define ANALOG_C0_GPIO_Port GPIOC
#define ANALOG_C3_Pin GPIO_PIN_3
#define ANALOG_C3_GPIO_Port GPIOC
#define ANALOG_A1_Pin GPIO_PIN_1
#define ANALOG_A1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define OUTPUT_C4_Pin GPIO_PIN_4
#define OUTPUT_C4_GPIO_Port GPIOC
#define ANALOG_C5_Pin GPIO_PIN_5
#define ANALOG_C5_GPIO_Port GPIOC
#define ANALOG_B0_Pin GPIO_PIN_0
#define ANALOG_B0_GPIO_Port GPIOB
#define ANALOG_B1_Pin GPIO_PIN_1
#define ANALOG_B1_GPIO_Port GPIOB
#define ANALOG_B2_Pin GPIO_PIN_2
#define ANALOG_B2_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_14
#define SPI2_CS1_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_15
#define SPI2_CS2_GPIO_Port GPIOB
#define ANALOG_C6_Pin GPIO_PIN_6
#define ANALOG_C6_GPIO_Port GPIOC
#define SPI3_CS1_Pin GPIO_PIN_8
#define SPI3_CS1_GPIO_Port GPIOC
#define OUTPUT_C9_Pin GPIO_PIN_9
#define OUTPUT_C9_GPIO_Port GPIOC
#define INPUT_A9_Pin GPIO_PIN_9
#define INPUT_A9_GPIO_Port GPIOA
#define OUTPUT_A10_Pin GPIO_PIN_10
#define OUTPUT_A10_GPIO_Port GPIOA
#define OUTPUT_A11_Pin GPIO_PIN_11
#define OUTPUT_A11_GPIO_Port GPIOA
#define INPUT_A12_Pin GPIO_PIN_12
#define INPUT_A12_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define OUTPUT_A15_Pin GPIO_PIN_15
#define OUTPUT_A15_GPIO_Port GPIOA
#define OUTPUT_C10_Pin GPIO_PIN_10
#define OUTPUT_C10_GPIO_Port GPIOC
#define OUTPUT_C11_Pin GPIO_PIN_11
#define OUTPUT_C11_GPIO_Port GPIOC
#define OUTPUT_D2_Pin GPIO_PIN_2
#define OUTPUT_D2_GPIO_Port GPIOD
#define OUTPUT_B3_Pin GPIO_PIN_3
#define OUTPUT_B3_GPIO_Port GPIOB
#define OUTPUT_B4_Pin GPIO_PIN_4
#define OUTPUT_B4_GPIO_Port GPIOB
#define INPUT_B5_Pin GPIO_PIN_5
#define INPUT_B5_GPIO_Port GPIOB
#define INPUT_B6_Pin GPIO_PIN_6
#define INPUT_B6_GPIO_Port GPIOB
#define INPUT_B7_Pin GPIO_PIN_7
#define INPUT_B7_GPIO_Port GPIOB
#define INPUT_B8_Pin GPIO_PIN_8
#define INPUT_B8_GPIO_Port GPIOB
#define INPUT_B9_Pin GPIO_PIN_9
#define INPUT_B9_GPIO_Port GPIOB

namespace w_hal
{
//    extern spi spi_2;

//    spi* get_spi_2_object();
    void i2c_build_packet_array_from_converted_bytes(uint8_t* arg_i2c_packet_array, uint8_t arg_global_id, const uint8_t* arg_converted_bytes);

    void tim2_init();
    void sys_clk_config();
    void adc2_init();
    void rtc_init();
    void gpio_init();
    void uart2_init();
//void MX_TIM2_Init();

    void spi1_init();
    void spi2_init();
    void spi3_init();

    void can2_init();
    void dac_init();
    void i2c2_init();
    void iwdg_init();

    SPI_HandleTypeDef* spi2_get_handle();

    HAL_StatusTypeDef rcc_oscillator_config(RCC_OscInitTypeDef  *RCC_OscInitStruct);
}




void error_handler();
//void Error_Handler();

SPI_HandleTypeDef* get_spi_1_handle();
RTC_HandleTypeDef* get_rtc_handle();
TIM_HandleTypeDef* get_timer_2_handle();
uint32_t get_timer_2_count();
uint32_t get_timer_count(w_hal::timer_handle_t* arg_timer_handle);


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);



//UART_HandleTypeDef* get_usart_2_handle();
//void initialize_peripherals();

//void MX_USART2_UART_Init();
//
//void MX_RTC_Init();
//
//
//void SystemClock_Config();

#endif //MAIN_CONTROLLER_HAL_H
