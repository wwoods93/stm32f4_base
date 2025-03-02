/***********************************************************************************************************************
 * Main_Controller
 * layer_1_rtosal.cpp
 *
 * wilson
 * 11/6/22
 * 2:46 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
#include <cstring>
/* stm32 includes */

/* third-party includes */
#include "cmsis_os2.h"
/* hal includes */

/* driver includes */

/* rtos abstraction includes */
#include "hal_spi_old.h"
/* sys op includes */

/* meta structure includes */
/* layer_1_rtosal header */
#include "rtosal.h"

osMessageQueueId_t spi_2_client_tx_queue_handle;
osMessageQueueId_t spi_2_client_rx_queue_handle;
const osMessageQueueAttr_t spi_2_client_tx_queue_attributes = { .name = "spi_2_extrusion_task_tx_queue" };
const osMessageQueueAttr_t spi_2_client_rx_queue_attributes = { .name = "spi_2_extrusion_task_rx_queue" };

osEventFlagsId_t initialization_event_flags_handle;
const osEventFlagsAttr_t initialization_event_flags_attributes = { .name = "initialization_event_flags" };

osMessageQueueId_t get_spi_2_client_tx_queue_handle()
{
    return spi_2_client_tx_queue_handle;
}

osMessageQueueId_t get_spi_2_client_rx_queue_handle()
{
    return spi_2_client_rx_queue_handle;
}

osEventFlagsId_t get_initialization_event_flags_handle()
{
    return initialization_event_flags_handle;
}


namespace rtosal
{
    void initialize()
    {
//        spi_2_client_tx_queue_handle = osMessageQueueNew((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(spi::packet_t), &spi_2_client_tx_queue_attributes);
//        spi_2_client_rx_queue_handle = osMessageQueueNew((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(spi::packet_t), &spi_2_client_rx_queue_attributes);
        initialization_event_flags_handle = osEventFlagsNew(&initialization_event_flags_attributes);
    }

    #if (USE_CMSIS_OS2 == 1U)
        uint32_t get_rtos_kernel_tick_frequency()
        {
            return osKernelGetTickFreq();
        }

        uint32_t get_rtos_kernel_tick_count()
        {
            return osKernelGetTickCount();
        }
    #endif

    void build_common_packet(common_packet_t& arg_packet, int16_t arg_channel_id, uint8_t (&arg_bytes)[8], uint8_t (&arg_bytes_per_tx)[8])
    {
        memset(&arg_packet, '\0', sizeof(common_packet_t));
        arg_packet.status = 0xFF; // packet active
        arg_packet.channel_id = arg_channel_id;
        memcpy(&arg_packet.bytes_per_transaction, arg_bytes_per_tx, sizeof(arg_packet.bytes_per_transaction));
        memcpy(&arg_packet.bytes, arg_bytes, sizeof(arg_packet.bytes));
    }
}
