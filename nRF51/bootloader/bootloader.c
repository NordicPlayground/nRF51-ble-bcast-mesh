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
#include "bootloader.h"
#include "bootloader_rtc.h"
#include "transport.h"
#include "bootloader_util.h"
#include "bootloader_info.h"
#include "dfu_mesh.h"
#include "dfu_types_mesh.h"
#include "nrf_mbr.h"
#include "nrf_flash.h"
#include "mesh_aci.h"
#include "app_error.h"
#include "serial_handler.h"
#include "bootloader_app_bridge.h"
/*****************************************************************************
* Local defines
*****************************************************************************/
#define IRQ_ENABLED                 (0x01)     /**< Field that identifies if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS       (32)       /**< Maximum number of interrupts available. */
/*****************************************************************************
* Local typedefs
*****************************************************************************/

/*****************************************************************************
* Static globals
*****************************************************************************/

/*****************************************************************************
* Static functions
*****************************************************************************/
static void set_timeout(uint32_t time)
{
#ifndef NO_TIMEOUTS
    NRF_RTC0->EVENTS_COMPARE[RTC_BL_STATE_CH] = 0;
    NRF_RTC0->CC[RTC_BL_STATE_CH] = (NRF_RTC0->COUNTER + time) & RTC_MASK;
    NRF_RTC0->INTENSET = (1 << (RTC_BL_STATE_CH + RTC_INTENSET_COMPARE0_Pos));
#endif
}

static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq;

    /* Fetch the current interrupt settings. */
    interrupt_setting_mask = NVIC->ISER[0];

    /* Loop from interrupt 0 for disabling of all interrupts. */
    for (irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            /* The interrupt was enabled, hence disable it. */
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

/** Interrupt indicating new serial command */
#ifdef SERIAL
void SWI2_IRQHandler(void)
{
    mesh_aci_command_check();
}
#endif

static void rx_cb(mesh_packet_t* p_packet)
{
    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(p_packet);
    if (p_adv_data && p_adv_data->handle > RBC_MESH_APP_MAX_HANDLE)
    {
        bl_cmd_t rx_cmd;
        rx_cmd.type = BL_CMD_TYPE_RX;
        rx_cmd.params.rx.p_dfu_packet = (dfu_packet_t*) &p_adv_data->handle;
        rx_cmd.params.rx.length = p_adv_data->adv_data_length - 3;
        bl_cmd_handler(&rx_cmd);
    }
}

static uint32_t bl_evt_handler(bl_evt_t* p_evt)
{
    bl_cmd_t rsp_cmd;
    switch (p_evt->type)
    {
        case BL_EVT_TYPE_ABORT:
            bootloader_abort(p_evt->params.abort.reason);
            break;
        case BL_EVT_TYPE_TX_RADIO:
        {
            mesh_packet_t* p_packet = NULL;
            if (!mesh_packet_acquire(&p_packet))
            {
                return NRF_ERROR_NO_MEM;
            }

            mesh_packet_set_local_addr(p_packet);
            p_packet->header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
            p_packet->header.length = DFU_PACKET_OVERHEAD + p_evt->params.tx.radio.length;
            ((ble_ad_t*) p_packet->payload)->adv_data_type = MESH_ADV_DATA_TYPE;
            ((ble_ad_t*) p_packet->payload)->data[0] = (MESH_UUID & 0xFF);
            ((ble_ad_t*) p_packet->payload)->data[1] = (MESH_UUID >> 8) & 0xFF;
            ((ble_ad_t*) p_packet->payload)->adv_data_length = DFU_PACKET_ADV_OVERHEAD + p_evt->params.tx.radio.length;
            memcpy(&p_packet->payload[4], p_evt->params.tx.radio.p_dfu_packet, p_evt->params.tx.radio.length);

            bool success = transport_tx(p_packet,
                                        p_evt->params.tx.radio.tx_slot,
                                        p_evt->params.tx.radio.tx_count,
                                        (tx_interval_type_t) p_evt->params.tx.radio.interval_type);
            mesh_packet_ref_count_dec(p_packet);

            if (!success)
            {
                return NRF_ERROR_INTERNAL;
            }
            break;
        }
        case BL_EVT_TYPE_TX_ABORT:
            transport_tx_abort(p_evt->params.tx.abort.tx_slot);
            break;
        case BL_EVT_TYPE_TX_SERIAL:
        {
#ifdef SERIAL            
            serial_evt_t serial_evt;
            serial_evt.opcode = SERIAL_EVT_OPCODE_DFU;
            memcpy(&serial_evt.params.dfu.packet, p_evt->params.tx.serial.p_dfu_packet, p_evt->params.tx.serial.length);
            serial_evt.length = SERIAL_PACKET_OVERHEAD + p_evt->params.tx.serial.length;
            if (!serial_handler_event_send(&serial_evt))
            {
                return NRF_ERROR_INTERNAL;
            }
            break;
#endif            
        }
        case BL_EVT_TYPE_TIMER_SET:
            set_timeout(US_TO_RTC_TICKS(p_evt->params.timer.set.delay_us));
            break;
        case BL_EVT_TYPE_FLASH_WRITE:
            nrf_flash_store((uint32_t*) p_evt->params.flash.write.start_addr,
                                        p_evt->params.flash.write.p_data,
                                        p_evt->params.flash.write.length, 0);

            /* respond immediately */
            rsp_cmd.type                            = BL_CMD_TYPE_FLASH_WRITE_COMPLETE;
            rsp_cmd.params.flash.write.start_addr   = p_evt->params.flash.write.start_addr;
            rsp_cmd.params.flash.write.p_data       = p_evt->params.flash.write.p_data;
            rsp_cmd.params.flash.write.length       = p_evt->params.flash.write.length;
            bl_cmd_handler(&rsp_cmd);
            break;
        case BL_EVT_TYPE_FLASH_ERASE:
            nrf_flash_erase((uint32_t*) p_evt->params.flash.erase.start_addr,
                                        p_evt->params.flash.erase.length);

            /* respond immediately */
            rsp_cmd.type                            = BL_CMD_TYPE_FLASH_ERASE_COMPLETE;
            rsp_cmd.params.flash.erase.start_addr   = p_evt->params.flash.erase.start_addr;
            rsp_cmd.params.flash.erase.length       = p_evt->params.flash.erase.length;
            bl_cmd_handler(&rsp_cmd);
            break;
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }
    return NRF_SUCCESS;
}

/** Check if the bank of the given type matches the firmware we're running. */
static bool bank_was_flashed(bl_info_type_t bank_type, bl_info_type_t segment_type)
{
    bl_info_entry_t* p_bank_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, bank_type);
    if (!p_bank_entry)
    {
        return false; /* no bank of this type */
    }
    
    bl_info_entry_t* p_segment_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, segment_type);
    if (!p_segment_entry)
    {
        return false; /* no segment of this type */
    }
    if (p_segment_entry->segment.length < p_bank_entry->bank.length)
    {
        return false; /* bank wouldn't fit */
    }
    
    /* check that the fwid is different */
    bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_VERSION);
    if (!p_fwid_entry)
    {
        return false;
    }
    switch (bank_type)
    {
        case BL_INFO_TYPE_BANK_APP:
            if (memcmp(&p_fwid_entry->version.app, &p_bank_entry->bank.fwid.app, sizeof(app_id_t)) == 0)
            {
                return false;
            }
            break;
        case BL_INFO_TYPE_BANK_BL:
            if (memcmp(&p_fwid_entry->version.bootloader, &p_bank_entry->bank.fwid.bootloader, sizeof(bl_id_t)) == 0)
            {
                return false;
            }
            break;
        case BL_INFO_TYPE_BANK_SD:
            if (p_fwid_entry->version.sd == p_bank_entry->bank.fwid.sd)
            {
                return false;
            }
            break;
        default:
            return false;
    }
            
    /* compare the bank and the fw word by word */
    for (uint32_t i = 0; i < p_bank_entry->bank.length; i += 4)
    {
        if (*((uint32_t*) (p_segment_entry->segment.length + i)) != *((uint32_t*) ((uint32_t) p_bank_entry->bank.p_bank_addr + i)))
        {
            /* no match */
            return false;
        }
    }
    
    return true;
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void bootloader_init(void)
{
    rtc_init();

    bootloader_app_bridge_init();
    
    bl_cmd_t init_cmd;
    init_cmd.type = BL_CMD_TYPE_INIT;
    init_cmd.params.init.bl_if_version = BL_IF_VERSION;
    init_cmd.params.init.event_callback = bl_evt_handler;
    init_cmd.params.init.timer_count = 1;
    init_cmd.params.init.tx_slots = TRANSPORT_TX_SLOTS;
    bl_cmd_handler(&init_cmd);

    
#ifdef SERIAL
    mesh_aci_init();
#endif

    transport_init(rx_cb, RBC_MESH_ACCESS_ADDRESS_BLE_ADV);
    
    /* If we've flashed a bank, we should check to see if it worked. */
    if (NRF_POWER->GPREGRET == RBC_MESH_GPREGRET_CODE_BANK_FLASH || !dfu_mesh_app_is_valid())
    {
        bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_VERSION);
        bl_info_entry_t* p_flag_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_FLAGS);

        bl_info_entry_t new_fwid_entry;
        memcpy(&new_fwid_entry, p_fwid_entry, sizeof(BL_INFO_LEN_FWID));
        bl_info_entry_t new_flag_entry;
        memcpy(&new_flag_entry, p_flag_entry, sizeof(BL_INFO_LEN_FLAGS));
        bool new_fw = false;
        
        
        if (bank_was_flashed(BL_INFO_TYPE_BANK_BL, BL_INFO_TYPE_SEGMENT_BL))
        {
            new_fw = true;
            bl_info_entry_t* p_bank_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_BANK_BL);
            if (p_bank_entry)
            {
                memcpy(&new_fwid_entry.version.bootloader, &p_bank_entry->bank.fwid.bootloader, sizeof(bl_id_t));
                new_flag_entry.flags.bl_intact = true;
            }
        }
        if (bank_was_flashed(BL_INFO_TYPE_BANK_SD, BL_INFO_TYPE_SEGMENT_SD))
        {
            new_fw = true;
            bl_info_entry_t* p_bank_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_BANK_SD);
            if (p_bank_entry)
            {
                new_fwid_entry.version.sd = p_bank_entry->bank.fwid.sd;
                new_flag_entry.flags.sd_intact = true;
            }
        }
        if (bank_was_flashed(BL_INFO_TYPE_BANK_APP, BL_INFO_TYPE_SEGMENT_APP))
        {
            new_fw = true;
            bl_info_entry_t* p_bank_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_BANK_APP);
            if (p_bank_entry)
            {
                memcpy(&new_fwid_entry.version.app, &p_bank_entry->bank.fwid.app, sizeof(app_id_t));
                new_flag_entry.flags.app_intact = true;
            }
        }
        
        bootloader_info_entry_put(BL_INFO_TYPE_VERSION, &new_fwid_entry, BL_INFO_LEN_FWID);
        bootloader_info_entry_put(
        
    }
}

void bootloader_enable(void)
{
    bl_cmd_t enable_cmd;
    enable_cmd.type = BL_CMD_TYPE_ENABLE;
    bl_cmd_handler(&enable_cmd);
    transport_start();
}

uint32_t bootloader_cmd_send(bl_cmd_t* p_bl_cmd)
{
    return bl_cmd_handler(p_bl_cmd);
}

void bootloader_abort(bl_end_t end_reason)
{
    bl_info_entry_t* p_segment_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_SEGMENT_APP);
    switch (end_reason)
    {
        case BL_END_SUCCESS:
        case BL_END_ERROR_TIMEOUT:
        case BL_END_FWID_VALID:
        case BL_END_ERROR_MBR_CALL_FAILED:
            if (p_segment_entry && dfu_mesh_app_is_valid((uint32_t*) p_segment_entry->segment.start))
            {
                interrupts_disable();

                sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };

                volatile uint32_t err_code = sd_mbr_command(&com);
                APP_ERROR_CHECK(err_code);

                err_code = sd_softdevice_vector_table_base_set(p_segment_entry->segment.start);
                APP_ERROR_CHECK(err_code);

                bootloader_util_app_start(p_segment_entry->segment.start);
            }
            break;
        case BL_END_ERROR_INVALID_PERSISTENT_STORAGE:
            APP_ERROR_CHECK_BOOL(false);
        default:
            NVIC_SystemReset();
            break;
    }
}


bl_info_entry_t* info_entry_get(bl_info_type_t type)
{
    bl_cmd_t get_cmd;
    get_cmd.type = BL_CMD_TYPE_INFO_GET;
    get_cmd.params.info.get.type = type;
    get_cmd.params.info.get.p_entry = NULL;
    if (bl_cmd_handler(&get_cmd) != NRF_SUCCESS)
    {
        return NULL;
    }

    return get_cmd.params.info.get.p_entry;
}
