/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#include <string.h>
#include "bootloader_app.h"
#include "bootloader_util.h"
#include "timeslot.h"
#include "rbc_mesh.h"
#include "dfu_types_mesh.h"
#include "toolchain.h"
#include "bl_if.h"
#include "nrf_flash.h"
#include "mesh_packet.h"
#include "rbc_mesh_common.h"
/*****************************************************************************
* Local defines
*****************************************************************************/
#define IRQ_ENABLED            0x01     /**< Field that identifies if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS  32       /**< Maximum number of interrupts available. */



/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef enum
{
    FLASH_OP_TYPE_NONE,
    FLASH_OP_TYPE_WRITE,
    FLASH_OP_TYPE_ERASE
} flash_op_type_t;

typedef struct
{
    flash_op_type_t type;
    union
    {
        struct
        {
            uint32_t start_addr;
            uint8_t* p_data;
            uint32_t length;
        } write;
        struct
        {
            uint32_t start_addr;
            uint32_t length;
        } erase;
    } params;
} flash_op_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static bl_if_cmd_handler_t          m_cmd_handler = NULL;           /**< Command handler in shared code space */
static flash_op_t                   m_flash_op;                     /**< Flash operation in progress. */
static timer_event_t                m_timer_evt;                    /**< Timer event for scheduler. */
static timer_event_t                m_tx_timer_evt;                 /**< TX event for scheduler. */
static bool                         m_tx_scheduled;                 /**< Whether the TX event is scheduled. */
static dfu_tx_t                     m_tx_slots[DFU_TX_SLOTS];       /**< TX slots for concurrent transmits. */
static prng_t                       m_prng;                         /**< PRNG for time delays. */  
static tc_tx_config_t               m_tx_config;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq;

    interrupt_setting_mask = NVIC->ISER[0];

    /* Loop through and disable all interrupts. */
    for (irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

static uint32_t flash_operation_execute(void)
{
    uint32_t error_code;
    switch (m_flash_op.type)
    {
        case FLASH_OP_TYPE_WRITE:
            error_code = sd_flash_write(
                    (uint32_t*) m_flash_op.params.write.start_addr,
                    (uint32_t*) m_flash_op.params.write.p_data,
                    m_flash_op.params.write.length / 4); /* sd takes length in words */
            break;
        case FLASH_OP_TYPE_ERASE:
            error_code = sd_flash_page_erase(m_flash_op.params.erase.start_addr / (NRF_FICR->CODEPAGESIZE));
            break;
        default:
            error_code = NRF_ERROR_INTERNAL;
            break;
    }
    if (error_code == NRF_SUCCESS)
    {
        timeslot_restart();
    }
    
    return error_code;
}

static uint32_t bootloader_event_handler(bl_evt_t* p_evt)
{
    //rbc_mesh_event_t app_evt;
    switch (p_evt->type)
    {
        case BL_EVT_TYPE_ECHO:
            _LOG("Echo: %s\n", p_evt->params.echo.str);
            break;

        case BL_EVT_TYPE_FLASH_ERASE:
            _LOG("Erase flash at: 0x%x (length %d)\n", p_evt->params.flash.erase.start_addr, p_evt->params.flash.erase.length);
            if (m_flash_op.type != FLASH_OP_TYPE_NONE)
            {
                return NRF_ERROR_BUSY;
            }
            if (p_evt->params.flash.erase.start_addr & (NRF_FICR->CODEPAGESIZE - 1))
            {
                return NRF_ERROR_INVALID_ADDR;
            }
            if (p_evt->params.flash.erase.length & (NRF_FICR->CODEPAGESIZE - 1))
            {
                return NRF_ERROR_INVALID_LENGTH;
            }
            m_flash_op.type = FLASH_OP_TYPE_ERASE;
            m_flash_op.params.erase.start_addr  = p_evt->params.flash.erase.start_addr;
            m_flash_op.params.erase.length      = p_evt->params.flash.erase.length;
            return flash_operation_execute();
        case BL_EVT_TYPE_FLASH_WRITE:
        {
            _LOG("Write flash at: 0x%x (length %d)\n", p_evt->params.flash.write.start_addr, p_evt->params.flash.write.length);
            if (m_flash_op.type != FLASH_OP_TYPE_NONE)
            {
                return NRF_ERROR_BUSY;
            }
            if (p_evt->params.flash.write.start_addr & 0x03)
            {
                return NRF_ERROR_INVALID_ADDR;
            }
            if (p_evt->params.flash.write.length & 0x03)
            {
                return NRF_ERROR_INVALID_LENGTH;
            }
            m_flash_op.type = FLASH_OP_TYPE_WRITE;
            m_flash_op.params.write.start_addr  = p_evt->params.flash.write.start_addr;
            m_flash_op.params.write.p_data      = p_evt->params.flash.write.p_data;
            m_flash_op.params.write.length      = p_evt->params.flash.write.length;
            mesh_packet_t* p_packet = mesh_packet_get_aligned(p_evt->params.flash.write.p_data);
            if (p_packet)
            {
                mesh_packet_ref_count_inc(p_packet);
            }
            return flash_operation_execute();
        }

        case BL_EVT_TYPE_TX_RADIO:
            _LOG("RADIO TX! SLOT %d, count %d, interval: %s, handle: %x\n", 
                p_evt->params.tx.radio.tx_slot, 
                p_evt->params.tx.radio.tx_count,
                p_evt->params.tx.radio.interval_type == BL_RADIO_INTERVAL_TYPE_EXPONENTIAL ? "exponential" : "periodic",
                p_evt->params.tx.radio.p_dfu_packet->packet_type
            );
            break;
        case BL_EVT_TYPE_TX_SERIAL:
            _LOG("SERIAL TX!\n");
            break;
        
        case BL_EVT_TYPE_TIMER_SET:
            _LOG("TIMER event: @%d us\n", p_evt->params.timer.set.delay_us);
            break;
        case BL_EVT_TYPE_TIMER_ABORT:
            _LOG("TIMER abort: %d\n", p_evt->params.timer.abort.index);
            break;

        default:
            _LOG("Got unsupported event: 0x%x\n", p_evt->type);
            return NRF_ERROR_NOT_SUPPORTED;
    }
    return NRF_SUCCESS;
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
uint32_t bootloader_start(dfu_type_t type, fwid_union_t* p_fwid)
{
    if (NRF_UICR->BOOTLOADERADDR != 0xFFFFFFFF)
    {
        interrupts_disable();

#ifdef SOFTDEVICE_PRESENT
        sd_power_reset_reason_clr(0x0F000F);
        sd_power_gpregret_set(RBC_MESH_GPREGRET_CODE_FORCED_REBOOT);
        sd_nvic_SystemReset();
#else
        NRF_POWER->RESETREAS = 0x0F000F; /* erase reset-reason to avoid wrongful state-readout on reboot */
        NRF_POWER->GPREGRET = RBC_MESH_GPREGRET_CODE_FORCED_REBOOT;
        NVIC_SystemReset();
#endif
        return NRF_SUCCESS; /* unreachable */
    }
    else
    {
        /* the UICR->BOOTLOADERADDR isn't set, and we have no way to find the bootloader-address. */
        return NRF_ERROR_FORBIDDEN;
    }
}

uint32_t bootloader_init(void)
{
    m_cmd_handler = *((bl_if_cmd_handler_t*) (0x20000000 + ((uint32_t) (NRF_FICR->SIZERAMBLOCKS * NRF_FICR->NUMRAMBLOCK) - 4)));
    if (m_cmd_handler == NULL ||
        (uint32_t) m_cmd_handler >= (NRF_FICR->CODESIZE * NRF_FICR->CODEPAGESIZE))
    {
        m_cmd_handler = NULL;
        return NRF_ERROR_NOT_SUPPORTED;
    }

    bl_cmd_t init_cmd =
    {
        .type = BL_CMD_TYPE_INIT,
        .params.init =
        {
            .bl_if_version = BL_IF_VERSION,
            .event_callback = bootloader_event_handler,
            .timer_count = 1,
            .tx_slots = 8
        }
    };

    return m_cmd_handler(&init_cmd);
}

uint32_t bootloader_enable(void)
{
    if (m_cmd_handler == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    bl_cmd_t enable_cmd =
    {
        .type = BL_CMD_TYPE_ENABLE,
        .params = {0}
    };

    return m_cmd_handler(&enable_cmd);
}

uint32_t bootloader_rx(mesh_adv_data_t* p_adv)
{
    if (m_cmd_handler == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_adv->handle <= RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    dfu_packet_t* p_dfu = (dfu_packet_t*) (&p_adv->handle);
    bl_cmd_t rx_cmd =
    {
        .type = BL_CMD_TYPE_RX,
        .params.rx.p_dfu_packet = p_dfu,
        .params.rx.length = p_adv->adv_data_length - 3
    };

    return m_cmd_handler(&rx_cmd);
}

uint32_t bootloader_dfu_abort(void)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t bootloader_dfu_finalize(void)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t bootloader_cmd_send(bl_cmd_t* p_cmd)
{
    if (m_cmd_handler == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return m_cmd_handler(p_cmd);
}

void bootloader_flash_operation_end(bool success)
{
    if (m_cmd_handler == NULL)
    {
        return;
    }
    if (m_flash_op.type == FLASH_OP_TYPE_WRITE)
    {
        if (success)
        {
            bl_cmd_t status_cmd =
            {
                .type = BL_CMD_TYPE_FLASH_WRITE_COMPLETE,
                .params.flash.write.start_addr  = m_flash_op.params.write.start_addr,
                .params.flash.write.p_data      = m_flash_op.params.write.p_data,
                .params.flash.write.length      = m_flash_op.params.write.length,
            };
            m_flash_op.type = FLASH_OP_TYPE_NONE;
            m_cmd_handler(&status_cmd);
            mesh_packet_t* p_packet = mesh_packet_get_aligned(status_cmd.params.flash.write.p_data);
            if (p_packet)
            {
                mesh_packet_ref_count_dec(p_packet);
            }
        }
        else
        {
            flash_operation_execute();
        }
    }
    else if (m_flash_op.type == FLASH_OP_TYPE_ERASE)
    {
        if (success)
        {
            m_flash_op.params.erase.start_addr += NRF_FICR->CODEPAGESIZE;
            m_flash_op.params.erase.length     -= NRF_FICR->CODEPAGESIZE;
        }
        
        if (m_flash_op.params.erase.length == 0)
        {
            m_flash_op.type = FLASH_OP_TYPE_NONE;
            bl_cmd_t status_cmd =
            {
                .type = BL_CMD_TYPE_FLASH_ERASE_COMPLETE,
                .params.flash.erase.start_addr  = m_flash_op.params.erase.start_addr,
                .params.flash.erase.length      = m_flash_op.params.erase.length,
            };
            m_cmd_handler(&status_cmd);
        }
        else
        {
            flash_operation_execute();
        }
    }
}

