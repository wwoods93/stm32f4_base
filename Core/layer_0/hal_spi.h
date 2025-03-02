/***********************************************************************************************************************
 * stm32_cpp_spi_lib
 * hal_spi.h
 *
 * wilson
 * 2/26/25
 * 12:12 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef STM32_CPP_SPI_LIB_HAL_SPI_H
#define STM32_CPP_SPI_LIB_HAL_SPI_H

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */
#include "hal.h"
#include "hal_spi_definitions.h"
/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */




class spi
{
    public:

    static constexpr uint32_t  CONFIG_STATUS_BIT_OK                      = (uint32_t)(0x00000000U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_INSTANCE                = (0x00000001U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_MODULE                  = (0x00000002U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_CLOCK_PHASE             = (0x00000004U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_CLOCK_POLARITY          = (0x00000008U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_MODE                    = (0x00000010U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_BAUD_RATE               = (0x00000020U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_FIRST_BIT_SETTING       = (0x00000040U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_CHIP_SELECT             = (0x00000080U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_DIRECTION               = (0x00000100U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_FRAME_FORMAT            = (0x00000200U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_CRC                     = (0x00000400U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_TIMEOUT_TIMER           = (0x00000800U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_DATA_SIZE               = (0x00001000U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_MSP_INIT                = (0x00002000U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_ISR_INIT                = (0x00004000U);
    static constexpr uint32_t  CONFIG_STATUS_BIT_RX_PTR_INIT             = (0x00008000U);

    static constexpr uint8_t  ERROR_NONE                                                = (0x00000000U);
    static constexpr uint8_t  ERROR_MODE_FAULT                                          = (0x00000001U);
    static constexpr uint8_t  ERROR_DURING_CRC_CALCULATION                              = (0x00000002U);
    static constexpr uint8_t  ERROR_OVERRUN                                             = (0x00000004U);
    static constexpr uint8_t  ERROR_TI_MODE_FRAME_FORMAT                                = (0x00000008U);
    static constexpr uint8_t  ERROR_DMA_TRANSFER                                        = (0x00000010U);
    static constexpr uint8_t  ERROR_WAITING_FOR_FLAG                                    = (0x00000020U);
    static constexpr uint8_t  ERROR_DURING_ABORT                                        = (0x00000040U);
    static constexpr uint8_t  ERROR_CALLBACK_INVALID                                    = (0x00000080U);

    typedef enum
    {
        RESOURCE_UNLOCKED                     = 0x00U,
        RESOURCE_LOCKED                       = 0x01U
    } lock_t;

    typedef enum
    {
        DATA_REG_ID                         = 0x00U,
        STATUS_REG_ID                       = 0x01U,
        CONTROL_REG_1_ID                    = 0x02U,
        CONTROL_REG_2_ID                    = 0x03U,
    } register_id_t;

    typedef enum
    {
        RESOURCE_STATUS_RESET                 = 0x00U,
        RESOURCE_STATUS_READY                 = 0x01U,
        RESOURCE_STATUS_BUSY                  = 0x02U,
        RESOURCE_STATUS_BUSY_TX               = 0x03U,
        RESOURCE_STATUS_BUSY_RX               = 0x04U,
        RESOURCE_STATUS_BUSY_TX_RX            = 0x05U,
        RESOURCE_STATUS_ERROR                 = 0x06U,
        RESOURCE_STATUS_ABORT                 = 0x07U
    } module_status_t;

    typedef struct
    {
        hal::gpio_t     port;
        uint16_t        pin;
    } chip_select_t;

    typedef struct
    {
        volatile uint32_t CONTROL_REG_1;
        volatile uint32_t CONTROL_REG_2;
        volatile uint32_t STATUS_REG;
        volatile uint32_t DATA_REG;
        volatile uint32_t CRC_POLYNOMIAL_REG;
        volatile uint32_t CRC_RX_REG;
        volatile uint32_t CRC_TX_REG;
        volatile uint32_t I2S_CONFIG_REG;
        volatile uint32_t I2S_PRESCALER_REG;
    } hal_spi_t;

    typedef struct
    {
        uint32_t mode;
        uint32_t direction;
        uint32_t data_size;
        uint32_t clock_polarity;
        uint32_t clock_phase;
        uint32_t chip_select_setting;
        uint32_t baud_rate_prescaler;
        uint32_t first_bit_setting;
        uint32_t ti_mode;
        uint32_t crc_calculation;
        uint32_t crc_polynomial;
    } config_t;


    typedef struct module_struct
    {
        hal_spi_t                   register_map;
        config_t                    config;
        volatile module_status_t    status;
        volatile uint32_t           error_bit_field;
        volatile uint32_t           config_status_bit_field;
        chip_select_t               chip_select;
        uint8_t                     tx_buffer[128];
        uint8_t                     rx_buffer[128];
        uint8_t                     tx_index;
        uint8_t                     rx_index;
        volatile uint16_t           tx_counter;
        volatile uint16_t           rx_counter;
        uint8_t                     rx_data_ready_flag;
        lock_t                      lock;
        hal::gpio_t*                chip_select_port;
        uint16_t                    chip_select_pin;
    } resource_t;




    resource_t resource;

    uint8_t etl_test(uint8_t arg);
    void transmit_receive_interrupt(uint8_t arg_tx_bytes[], uint8_t arg_rx_bytes[], uint8_t arg_size);
    friend void spi_irq_handler(spi& arg_object);
    void irq_handler();



    private:

        void rx_isr();

        void enable_module();
        void disable_module();
        void enable_interrupt(uint32_t arg_interrupts);
        void disable_interrupt(uint32_t arg_interrupts);
        [[nodiscard]] bit_status_t check_interrupt_source(uint32_t arg_interrupt) const;
        void set_register_bit(register_id_t arg_register, uint32_t arg_bit);
        void clear_register_bit(register_id_t arg_register, uint32_t arg_bit);
        [[nodiscard]] bit_status_t get_status_register_bit(uint32_t arg_bit) const;
        void set_error_bit(uint32_t arg_bit);
        void set_config_status_bit(uint32_t arg_bit);
        void clear_config_status_bit_field();
        void clear_mode_fault_flag();
        void clear_overrun_flag() const;


};

inline void spi::enable_module()
{
    resource.register_map.CONTROL_REG_1 |= SPI_CR1_BIT_SPI_ENABLE;
}

inline void spi::disable_module()
{
    resource.register_map.CONTROL_REG_1 &= (~SPI_CR1_BIT_SPI_ENABLE);
}


inline void spi::enable_interrupt(uint32_t arg_interrupts)
{
    resource.register_map.CONTROL_REG_2 |= arg_interrupts;
}

inline void spi::disable_interrupt(uint32_t arg_interrupts)
{
    resource.register_map.CONTROL_REG_2 &= (~arg_interrupts);
}

inline bit_status_t spi::check_interrupt_source(uint32_t arg_interrupt) const
{
    bit_status_t bit_status = BIT_RESET;
    if ((resource.register_map.CONTROL_REG_2 & arg_interrupt) == arg_interrupt)
    {
        bit_status = BIT_SET;
    }

    return bit_status;
}


inline void spi::set_register_bit(register_id_t arg_register, uint32_t arg_bit)
{
    switch(arg_register)
    {
        case CONTROL_REG_1_ID:
        {
            resource.register_map.CONTROL_REG_1 |= arg_bit;
            break;
        }
        case CONTROL_REG_2_ID:
        {
            resource.register_map.CONTROL_REG_2 |= arg_bit;
            break;
        }
        default:
        {
            break;
        }
    }
}

inline void spi::clear_register_bit(register_id_t arg_register, uint32_t arg_bit)
{
    switch(arg_register)
    {
        case STATUS_REG_ID:
        {
            resource.register_map.STATUS_REG &= (~arg_bit);
            break;
        }
        case CONTROL_REG_1_ID:
        {
            resource.register_map.CONTROL_REG_1 &= (~arg_bit);
            break;
        }
        case CONTROL_REG_2_ID:
        {
            resource.register_map.CONTROL_REG_2 &= (~arg_bit);
            break;
        }
        default:
        {
            break;
        }
    }
}

inline bit_status_t spi::get_status_register_bit(uint32_t arg_bit) const
{
    bit_status_t bit_status = BIT_RESET;
    if ((resource.register_map.STATUS_REG & arg_bit & SPI_SR_BITS_MASK) == (arg_bit & SPI_SR_BITS_MASK))
    {
        bit_status = BIT_SET;
    }
    return bit_status;
}

inline void spi::set_error_bit(uint32_t arg_bit)
{
    resource.error_bit_field |= arg_bit;
}

inline void spi::set_config_status_bit(uint32_t arg_bit)
{
    resource.config_status_bit_field |= arg_bit;
}

inline void spi::clear_config_status_bit_field()
{
    resource.config_status_bit_field = CONFIG_STATUS_BIT_OK;
}

inline void spi::clear_mode_fault_flag()
{
    uint32_t register_contents = resource.register_map.STATUS_REG;
    UNUSED_CAST_VOID(register_contents);
    clear_register_bit(CONTROL_REG_1_ID, SPI_CR1_BIT_SPI_ENABLE);
}

inline void spi::clear_overrun_flag() const
{
    uint32_t data_reg_contents = resource.register_map.DATA_REG;
    uint32_t status_reg_contents = resource.register_map.STATUS_REG;
    UNUSED_CAST_VOID(data_reg_contents);
    UNUSED_CAST_VOID(status_reg_contents);
}


#endif //STM32_CPP_SPI_LIB_HAL_SPI_H
