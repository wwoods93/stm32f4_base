/***********************************************************************************************************************
 * Main_Controller
 * hal_peripheral.cpp
 *
 * wilson
 * 10/11/24
 * 1:52 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* stm32 includes */
#include "stm32f4xx.h"

/* layer_0 includes */
#include "hal.h"
#include "hal_spi_old.h"

SPI_HandleTypeDef hspi1;

SPI_HandleTypeDef hspi3;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;


TIM_HandleTypeDef htim6;



WWDG_HandleTypeDef hwwdg;

static TIM_HandleTypeDef ms_timer_base_timer_handle;
static TIM_HandleTypeDef us_base_timer_handle;

namespace hal
{
    void i2c_build_packet_array_from_converted_bytes(uint8_t* arg_i2c_packet_array, uint8_t arg_global_id, const uint8_t* arg_converted_bytes)
    {
        arg_i2c_packet_array[0] = arg_global_id;
        arg_i2c_packet_array[1] = arg_converted_bytes[0];
        arg_i2c_packet_array[2] = arg_converted_bytes[1];
        arg_i2c_packet_array[3] = arg_converted_bytes[2];
        arg_i2c_packet_array[4] = arg_converted_bytes[3];
    }

    void timer_2_initialize()
    {
        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        htim2.Instance = TIM2;
        htim2.Init.Prescaler = 31U;
        htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim2.Init.Period = 4294967295U;
        htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        {
            error_handler();
        }
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        {
            error_handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        {
            error_handler();
        }
    }

}

RTC_HandleTypeDef* get_rtc_handle()
{
    return &hrtc;
}

TIM_HandleTypeDef* get_timer_2_handle()
{
    return &htim2;
}

uint32_t get_timer_2_count()
{
    return htim2.Instance->CNT;
}

uint32_t get_timer_count(hal::timer_handle_t* arg_timer_handle)
{
    return arg_timer_handle->Instance->CNT;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}


void SystemClock_Config()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 128;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (hal_rcc_oscillator_config(&RCC_OscInitStruct) != HAL_OK)
    {
        error_handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        error_handler();
    }
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

void MX_ADC2_Init()
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        error_handler();
    }

    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        error_handler();
    }
}

void MX_CAN2_Init()
{
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 16;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan2.Init.TimeTriggeredMode = DISABLE;
    hcan2.Init.AutoBusOff = DISABLE;
    hcan2.Init.AutoWakeUp = DISABLE;
    hcan2.Init.AutoRetransmission = DISABLE;
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
        error_handler();
    }
}

void MX_DAC_Init()
{
    DAC_ChannelConfTypeDef sConfig = {0};
    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
        error_handler();
    }

    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        error_handler();
    }
}

void MX_I2C2_Init()
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        error_handler();
    }
}

void MX_IWDG_Init()
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        error_handler();
    }
}



void MX_RTC_Init()
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        error_handler();
    }

    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        error_handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x1;
    sDate.Year = 0x24;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
        error_handler();
    }
}


void MX_SPI1_Init()
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        error_handler();
    }
}

void MX_SPI3_Init()
{
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        error_handler();
    }
}

void MX_TIM2_Init()
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 32-1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        error_handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        error_handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        error_handler();
    }
}

void MX_GPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();


    HAL_GPIO_WritePin(GPIOA, LD2_Pin|OUTPUT_A11_Pin|OUTPUT_A15_Pin, GPIO_PIN_RESET);


    HAL_GPIO_WritePin(GPIOC, OUTPUT_C4_Pin|OUTPUT_C9_Pin|OUTPUT_C10_Pin|OUTPUT_C11_Pin, GPIO_PIN_RESET);


    HAL_GPIO_WritePin(GPIOB, SPI2_CS1_Pin|SPI2_CS2_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(SPI3_CS1_GPIO_Port, SPI3_CS1_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(OUTPUT_A10_GPIO_Port, OUTPUT_A10_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(OUTPUT_D2_GPIO_Port, OUTPUT_D2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, OUTPUT_B3_Pin|OUTPUT_B4_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ANALOG_C0_Pin|ANALOG_C3_Pin|ANALOG_C5_Pin|ANALOG_C6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ANALOG_A1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ANALOG_A1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OUTPUT_C4_Pin|OUTPUT_C9_Pin|OUTPUT_C10_Pin|OUTPUT_C11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ANALOG_B0_Pin|ANALOG_B1_Pin|ANALOG_B2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI2_CS1_Pin|SPI2_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI3_CS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI3_CS1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = INPUT_A9_Pin|INPUT_A12_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OUTPUT_A10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(OUTPUT_A10_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OUTPUT_A11_Pin|OUTPUT_A15_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OUTPUT_D2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OUTPUT_D2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OUTPUT_B3_Pin|OUTPUT_B4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = INPUT_B5_Pin|INPUT_B6_Pin|INPUT_B7_Pin|INPUT_B8_Pin
                          |INPUT_B9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void error_handler()
{
    __disable_irq();
    while (1)
    {

    }
}


HAL_StatusTypeDef hal_rcc_oscillator_config(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
    uint32_t tickstart, pll_config;

    /* Check Null pointer */
    if(RCC_OscInitStruct == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(IS_RCC_OSCILLATORTYPE(RCC_OscInitStruct->OscillatorType));
    /*------------------------------- HSE Configuration ------------------------*/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
    {
        /* Check the parameters */
        assert_param(IS_RCC_HSE(RCC_OscInitStruct->HSEState));
        /* When the HSE is used as system clock or clock source for PLL in these cases HSE will not disabled */
#if defined(STM32F446xx)
        if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSE)                                                                     ||\
      ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)) ||\
      ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLLR) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)))
#else
            if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSE)                                                                     ||\
      ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)))
#endif /* STM32F446xx */
        {
            if((__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
            {
                return HAL_ERROR;
            }
        }
        else
        {
            /* Set the new HSE configuration ---------------------------------------*/
            __HAL_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

            /* Check the HSE State */
            if((RCC_OscInitStruct->HSEState) != RCC_HSE_OFF)
            {
                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till HSE is ready */
                while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till HSE is bypassed or disabled */
                while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }
    /*----------------------------- HSI Configuration --------------------------*/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
    {
        /* Check the parameters */
        assert_param(IS_RCC_HSI(RCC_OscInitStruct->HSIState));
        assert_param(IS_RCC_CALIBRATION_VALUE(RCC_OscInitStruct->HSICalibrationValue));

        /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
#if defined(STM32F446xx)
        if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSI)                                                                     ||\
      ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)) ||\
      ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLLR) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)))
#else
            if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSI)                                                                     ||\
      ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)))
#endif /* STM32F446xx */
        {
            /* When HSI is used as system clock it will not disabled */
            if((__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET) && (RCC_OscInitStruct->HSIState != RCC_HSI_ON))
            {
                return HAL_ERROR;
            }
                /* Otherwise, just the calibration is allowed */
            else
            {
                /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
                __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
            }
        }
        else
        {
            /* Check the HSI State */
            if((RCC_OscInitStruct->HSIState)!= RCC_HSI_OFF)
            {
                /* Enable the Internal High Speed oscillator (HSI). */
                __HAL_RCC_HSI_ENABLE();

                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till HSI is ready */
                while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }

                /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
                __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
            }
            else
            {
                /* Disable the Internal High Speed oscillator (HSI). */
                __HAL_RCC_HSI_DISABLE();

                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till HSI is ready */
                while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }
    /*------------------------------ LSI Configuration -------------------------*/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
    {
        /* Check the parameters */
        assert_param(IS_RCC_LSI(RCC_OscInitStruct->LSIState));

        /* Check the LSI State */
        if((RCC_OscInitStruct->LSIState)!= RCC_LSI_OFF)
        {
            /* Enable the Internal Low Speed oscillator (LSI). */
            __HAL_RCC_LSI_ENABLE();

            /* Get Start Tick*/
            tickstart = HAL_GetTick();

            /* Wait till LSI is ready */
            while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
            {
                if((HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
                {
                    return HAL_TIMEOUT;
                }
            }
        }
        else
        {
            /* Disable the Internal Low Speed oscillator (LSI). */
            __HAL_RCC_LSI_DISABLE();

            /* Get Start Tick*/
            tickstart = HAL_GetTick();

            /* Wait till LSI is ready */
            while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) != RESET)
            {
                if((HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
                {
                    return HAL_TIMEOUT;
                }
            }
        }
    }
    /*------------------------------ LSE Configuration -------------------------*/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
    {
        FlagStatus       pwrclkchanged = RESET;

        /* Check the parameters */
        assert_param(IS_RCC_LSE(RCC_OscInitStruct->LSEState));

        /* Update LSE configuration in Backup Domain control register    */
        /* Requires to enable write access to Backup Domain of necessary */
        if(__HAL_RCC_PWR_IS_CLK_DISABLED())
        {
            __HAL_RCC_PWR_CLK_ENABLE();
            pwrclkchanged = SET;
        }

        if(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
        {
            /* Enable write access to Backup domain */
            SET_BIT(PWR->CR, PWR_CR_DBP);

            /* Wait for Backup domain Write protection disable */
            tickstart = HAL_GetTick();

            while(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
            {
                if((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
                {
                    return HAL_TIMEOUT;
                }
            }
        }

        /* Set the new LSE configuration -----------------------------------------*/
        __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
        /* Check the LSE State */
        if((RCC_OscInitStruct->LSEState) != RCC_LSE_OFF)
        {
            /* Get Start Tick*/
            tickstart = HAL_GetTick();

            /* Wait till LSE is ready */
            while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
            {
                if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
                {
                    return HAL_TIMEOUT;
                }
            }
        }
        else
        {
            /* Get Start Tick*/
            tickstart = HAL_GetTick();

            /* Wait till LSE is ready */
            while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != RESET)
            {
                if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
                {
                    return HAL_TIMEOUT;
                }
            }
        }

        /* Restore clock configuration if changed */
        if(pwrclkchanged == SET)
        {
            __HAL_RCC_PWR_CLK_DISABLE();
        }
    }
    /*-------------------------------- PLL Configuration -----------------------*/
    /* Check the parameters */
    assert_param(IS_RCC_PLL(RCC_OscInitStruct->PLL.PLLState));
    if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
    {
        /* Check if the PLL is used as system clock or not */
        if(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
        {
            if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
            {
                /* Check the parameters */
                assert_param(IS_RCC_PLLSOURCE(RCC_OscInitStruct->PLL.PLLSource));
                assert_param(IS_RCC_PLLM_VALUE(RCC_OscInitStruct->PLL.PLLM));
                assert_param(IS_RCC_PLLN_VALUE(RCC_OscInitStruct->PLL.PLLN));
                assert_param(IS_RCC_PLLP_VALUE(RCC_OscInitStruct->PLL.PLLP));
                assert_param(IS_RCC_PLLQ_VALUE(RCC_OscInitStruct->PLL.PLLQ));
                assert_param(IS_RCC_PLLR_VALUE(RCC_OscInitStruct->PLL.PLLR));

                /* Disable the main PLL. */
                __HAL_RCC_PLL_DISABLE();

                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }

                /* Configure the main PLL clock source, multiplication and division factors. */
                WRITE_REG(RCC->PLLCFGR, (RCC_OscInitStruct->PLL.PLLSource                                            | \
                                 RCC_OscInitStruct->PLL.PLLM                                                 | \
                                 (RCC_OscInitStruct->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos)                       | \
                                 (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos)        | \
                                 (RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)                       | \
                                 (RCC_OscInitStruct->PLL.PLLR << RCC_PLLCFGR_PLLR_Pos)));
                /* Enable the main PLL. */
                __HAL_RCC_PLL_ENABLE();

                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                /* Disable the main PLL. */
                __HAL_RCC_PLL_DISABLE();

                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
        else
        {
            /* Check if there is a request to disable the PLL used as System clock source */
            if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF)
            {
                return HAL_ERROR;
            }
            else
            {
                /* Do not return HAL_ERROR if request repeats the current configuration */
                pll_config = RCC->PLLCFGR;
#if defined (RCC_PLLCFGR_PLLR)
                if (((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != (RCC_OscInitStruct->PLL.PLLM) << RCC_PLLCFGR_PLLM_Pos) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (RCC_OscInitStruct->PLL.PLLN) << RCC_PLLCFGR_PLLN_Pos) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U)) << RCC_PLLCFGR_PLLP_Pos))
#else
                    if (((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != (RCC_OscInitStruct->PLL.PLLM) << RCC_PLLCFGR_PLLM_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (RCC_OscInitStruct->PLL.PLLN) << RCC_PLLCFGR_PLLN_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U)) << RCC_PLLCFGR_PLLP_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLQ) != (RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)))
#endif
                {
                    return HAL_ERROR;
                }
            }
        }
    }
    return HAL_OK;
}