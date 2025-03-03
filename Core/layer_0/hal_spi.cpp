/***********************************************************************************************************************
 * stm32_cpp_spi_lib
 * hal_spi.cpp
 *
 * wilson
 * 2/26/25
 * 12:12 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */
#include "w_hal.h"
/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* hal_spi header */
#include "hal_spi.h"
#include "../etl/include/etl/vector.h"
#include "../etl/include/etl/numeric.h"


/*
 * irq_handler()
 *
 * if (interrupt is RX_BUFFER_NOT_EMPTY)
 *      if (status register RX_BUFFER_NOT_EMPTY bit is SET)
 *          if (status register OVERRUN bit is NOT SET)
 *                rx_isr()
 *
 * if (interrupt is TX_BUFFER_EMPTY)
 *      if (status register TX_BUFFER_EMPTY bit is SET)
 *                tx_isr()
 *
 * if (interrupt is ERROR)
 *      if (status register OVERRUN bit is SET)
 *              if (module->status != BUSY_TX)
 *                      set_error_bit(OVERRUN)
 *              clear_overrun_flag()
 *              disable_interrupt(RX_BUFFER_NOT_EMPY | TX_BUFFER_EMPTY | ERROR)
 *
 *      if (status register MODE_FAULT is SET)
 *              set_error_bit(MODE_FAULT)
 *              clear_mode_fault_flag()
 *              disable interrupts(RX_BUFFER_NOT_EMPY | TX_BUFFER_EMPTY | ERROR)
 *
 *      handle_transaction_error()
 *      module->status = ready
 */

uint8_t spi::etl_test(uint8_t arg)
{
    volatile uint8_t num = 0U;
    etl::vector<uint8_t, 10> v1(10);
    for (uint8_t n  = 0U; n < 10; ++n)
    {
         v1[n] = arg + n;
    }
    while (num > 2 * arg)
    {
        --num;
    }
    return num;
}



void spi::transmit_receive_interrupt(uint8_t arg_tx_bytes[], uint8_t arg_rx_bytes[], uint8_t arg_size)
{




    resource.status = RESOURCE_STATUS_BUSY_TX_RX;
    resource.error_bit_field = spi::ERROR_NONE;
    for (uint8_t n = 0U; n < arg_size; ++n)
    {
        resource.tx_buffer[n] = arg_tx_bytes[n];
        resource.rx_buffer[n] = 0U;
    }
    resource.tx_counter = arg_size;
    resource.rx_counter = arg_size;
    resource.chip_select.port = *PORT_B;
    resource.chip_select_pin = PIN_14;


}

void spi::irq_handler()
{
    if (check_interrupt_source(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE))
    {
        if (get_status_register_bit(SPI_SR_BIT_RX_BUFFER_NOT_EMPTY) == BIT_SET)
        {
            if (get_status_register_bit(SPI_SR_BIT_OVERRUN) == BIT_RESET)
            {
                if (resource.config.data_size == SPI_CONFIG_DATA_SIZE_8_BIT)
                {
                    resource.rx_buffer[resource.rx_index] = (uint8_t)resource.register_map.DATA_REG;
                    ++resource.rx_index;
                    --resource.rx_counter;
                }

                if (resource.rx_counter == 0U)
                {
                    disable_interrupt(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
                }
            }
        }
    }

    if (check_interrupt_source(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE))
    {
        if (get_status_register_bit(SPI_SR_BIT_TX_BUFFER_EMPTY) == BIT_SET)
        {
            if (resource.config.data_size == SPI_CONFIG_DATA_SIZE_8_BIT)
            {
                resource.register_map.DATA_REG = resource.tx_buffer[resource.tx_index];
                ++resource.tx_index;
                --resource.tx_counter;
            }

            if (resource.tx_counter == 0U)
            {
                disable_interrupt(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
            }
        }
    }

    if (check_interrupt_source(SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE) == BIT_SET)
    {
        disable_interrupt(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);

        if (get_status_register_bit(SPI_SR_BIT_OVERRUN) == BIT_SET)
        {
            if (resource.status != spi::RESOURCE_STATUS_BUSY_TX)
            {
                set_error_bit(spi::ERROR_OVERRUN);
            }
            clear_overrun_flag();
        }

        if (get_status_register_bit(SPI_SR_BIT_MODE_FAULT) == BIT_SET)
        {
            set_error_bit(spi::ERROR_MODE_FAULT);
            clear_mode_fault_flag();
        }

        return;
    }

    if (resource.rx_counter == 0U && resource.tx_counter == 0U)
    {
        while (get_status_register_bit(SPI_SR_BIT_TX_BUFFER_EMPTY) == BIT_RESET);
        while (get_status_register_bit(SPI_SR_BIT_RESOURCE_BUSY) == BIT_SET);
        disable_interrupt(SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);
        disable_module();
        return;
    }

}


/*
 * rx_isr()
 *
 *      rx_buffer[index] = module->register_map->DATA_REG
 *      ++index
 *      --transfer_counter
 *
 *      if (transfer_counter == 0)
 *              disable_interrupts(RX_BUFFER_NOT_EMPTY | ERROR)
 *              wait for flag(s) to clear:      SR_BIT_RESOURCE_BUSY (for TX_RX controller)
 *                                              SR_BIT_RESOURCE_BUSY (for TX_RX controller or RX peripheral)
 *              clear_overrun_flag()
 *              if (error)
 *                  handle_error()
 *
 *              module->status = ready
 *
 *
 * tx_isr()
 *
 *      module->register_map->DATA_REG = tx_buffer[index]
 *      ++index
 *      --transfer_counter
 *      if (transfer_counter == 0)
 *              disable_interrupt(TX_BUFFER_EMPTY | ERROR (?))
 *              wait for flag(s) to clear: TX_BUFFER_EMPTY | RESOURCE_BUSY
 *
 *              clear_overrun_flag()
 *              if (error)
 *                  handle_error()
 *
 *              module->status = ready
 *
 */
void spi_irq_handler(spi &arg_object)
{
    arg_object.irq_handler();
}


