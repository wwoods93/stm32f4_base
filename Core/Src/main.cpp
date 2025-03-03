/***********************************************************************************************************************
 * stm32_cpp_spi_lib
 * main.cpp
 *
 * wilson
 * 11/28/24
 * 1:18 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <memory>
#include <cstring>
#include <cstdio>
/* stm32 includes */
#include "stm32f4xx_it.h"
/* third-party includes */
#include "cmsis_os.h"
/* layer_0 includes */
#include "../layer_0/w_hal.h"
#include "../layer_0/hal_spi.h"
//#include "../layer_0/hal_spi_old.h"
#include "../layer_0/rtosal.h"
#include "../Inc/device.h"
#include "../Inc/serial_monitor.h"
/* main header */
#include "main.h"


static constexpr uint8_t SYSTEM_RUN = 1U;






serial_monitor debug_serial_monitor;

osThreadId_t client_task_handle;
osThreadId_t spi_task_handle;
osThreadId_t heartbeat_task_handle;
osTimerId_t comms_handler_tick_handle;

const osThreadAttr_t client_task_attributes     = { .name = "client_task",      .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t spi_task_attributes        = { .name = "spi_task",         .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t heartbeat_task_attributes  = { .name = "heartbeat_task",   .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };



const osTimerAttr_t comms_handler_tick_attributes = { .name = "comms_handler_tick" };


void MX_SPI2_Init();

[[noreturn]] void start_client_task(void *argument);
[[noreturn]] void start_spi_task(void *argument);
[[noreturn]] void start_heartbeat_task(void *argument);

spi spi_2;
void comms_handler_tick_callback(void *argument);

int main()
{
    HAL_Init();
    w_hal::sys_clk_config();
    w_hal::gpio_init();
    w_hal::uart2_init();
    w_hal::tim2_init();
    w_hal::rtc_init();
    w_hal::spi2_init();
    HAL_TIM_Base_Start(get_timer_2_handle());

    osKernelInitialize();

    comms_handler_tick_handle = osTimerNew(comms_handler_tick_callback, osTimerPeriodic, nullptr, &comms_handler_tick_attributes);
    client_task_handle = osThreadNew(start_client_task, nullptr, &client_task_attributes);
    spi_task_handle = osThreadNew(start_spi_task, nullptr, &spi_task_attributes);
    heartbeat_task_handle = osThreadNew(start_heartbeat_task, nullptr, &heartbeat_task_attributes);

    rtosal::initialize();

    osKernelStart();

    while (1)
    {

    }
}

[[noreturn]] void start_client_task(void *argument)
{
    osEventFlagsId_t initialization_event_flags_handle = get_initialization_event_flags_handle();
    rtosal::event_flag_wait(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG, rtosal::OS_FLAGS_ANY, rtosal::OS_WAIT_FOREVER);

    while (SYSTEM_RUN)
    {

    }
}

[[noreturn]] void start_spi_task(void *argument)
{
    osEventFlagsId_t initialization_event_flags_handle = get_initialization_event_flags_handle();

    static uint32_t spi_task_count = 0U;


    uint8_t tx_bytes[8] = { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0C, 0x0E };
    uint8_t rx_bytes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };


    rtosal::event_flag_set(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG);

    w_hal::gpio_write_pin(PORT_B, GPIO_PIN_14, 1U);

    while (SYSTEM_RUN)
    {
        if (get_timer_count(get_timer_2_handle()) - spi_task_count > 300000)
        {

            rx_bytes[0] = spi_2.etl_test(1);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            w_hal::gpio_write_pin(PORT_B, GPIO_PIN_14, 0U);
            HAL_SPI_Transmit(w_hal::spi2_get_handle(), tx_bytes, 8, 100U);
            w_hal::gpio_write_pin(PORT_B, GPIO_PIN_14, 1U);
            spi_task_count = get_timer_count(get_timer_2_handle());
        }

//        if (w_hal::gpio_read_pin(PORT_B, GPIO_PIN_14) == 0U)
//        {
//            HAL_SPI_Receive(&hspi2, rx_bytes, 8, 100U);
//        }


    }
}


[[noreturn]] void start_heartbeat_task(void *argument)
{
    static uint32_t count = 0U;

    while (SYSTEM_RUN)
    {
//        rtosal::thread_yield();
    }
}

void comms_handler_tick_callback(void *argument)
{

}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

void SPI2_IRQHandler()
{
    HAL_SPI_IRQHandler(w_hal::spi2_get_handle());
}

void USART2_IRQHandler()
{
//    HAL_UART_IRQHandler(&huart2);
}

void DebugMon_Handler()
{

}

void WWDG_IRQHandler()
{
//    HAL_WWDG_IRQHandler(&hwwdg);
}

void RCC_IRQHandler()
{
}


void ADC_IRQHandler()
{
//    HAL_ADC_IRQHandler(&hadc2);
}

void I2C2_EV_IRQHandler()
{
//    HAL_I2C_EV_IRQHandler(&hi2c2);
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler()
{
//    HAL_I2C_ER_IRQHandler(&hi2c2);
}

void EXTI15_10_IRQHandler()
{
//    HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}

void TIM6_DAC_IRQHandler()
{
//    HAL_DAC_IRQHandler(&hdac);
//    HAL_TIM_IRQHandler(&htim6);
}

void CAN2_TX_IRQHandler()
{
//    HAL_CAN_IRQHandler(&hcan2);
}

void CAN2_RX0_IRQHandler()
{
//    HAL_CAN_IRQHandler(&hcan2);
}

void CAN2_RX1_IRQHandler()
{
//    HAL_CAN_IRQHandler(&hcan2);
}

void CAN2_SCE_IRQHandler()
{
//    HAL_CAN_IRQHandler(&hcan2);
}


void callback_spi_peripheral_tx_rx_complete(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}
void callback_spi_controller_error(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}



#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
