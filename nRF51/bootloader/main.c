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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "bl_if.h"
#include "bootloader_mesh.h"
#include "bootloader_info.h"
#include "bootloader_rtc.h"
#include "dfu_types_mesh.h"
#include "rbc_mesh.h"
#include "transport.h"
#include "mesh_aci.h"
#include "nrf_flash.h"
#include "bootloader_app_bridge.h"
#include "bootloader_util.h"
#include "nrf_mbr.h"

#include "app_error.h"
#include "nrf_gpio.h"


#define IRQ_ENABLED                 (0x01)     /**< Field that identifies if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS       (32)       /**< Maximum number of interrupts available. */

/* Magic UICR overwrite to convince the MBR to start in bootloader. */
#if defined(__CC_ARM)
extern uint32_t __Vectors;
uint32_t* m_uicr_bootloader_start_address
    __attribute__((at(NRF_UICR_BOOT_START_ADDRESS))) = &__Vectors;
#elif defined(__GNUC__)
extern volatile uint32_t __Vectors;
volatile uint32_t* m_uicr_bootloader_start_address
    __attribute__((section(".uicrBootStartAddress"))) = &__Vectors;
#else
#error "Unsupported toolchain."
#endif

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
#ifdef DEBUG_LEDS
    __disable_irq();
    NRF_GPIO->OUTSET = (1 << 7);
    NRF_GPIO->OUTCLR = (1 << 23);
#endif
    __BKPT(0);
    while (1);
}

void HardFault_Handler(uint32_t pc, uint32_t lr)
{
#ifdef DEBUG_LEDS
    NRF_GPIO->OUTSET = (1 << 7);
    NRF_GPIO->OUTCLR = (1 << 23);
#endif
    __BKPT(0);
    while (1);
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
        bootloader_rx((dfu_packet_t*) &p_adv_data->handle, p_adv_data->adv_data_length - 3, false);
    }
}

static void init_leds(void)
{
#ifdef DEBUG_LEDS
    nrf_gpio_range_cfg_output(21, 24);
    NRF_GPIO->OUT = (1 << 22) | (1 << 21) | (1 << 24);
#endif
}

static void init_clock(void)
{
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (!NRF_CLOCK->EVENTS_HFCLKSTARTED);
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (!NRF_CLOCK->EVENTS_LFCLKSTARTED);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

    NRF_CLOCK->TASKS_CAL = 1;
    while (!NRF_CLOCK->EVENTS_DONE);
    NRF_CLOCK->EVENTS_DONE = 0;
}

static bool fw_is_verified(void)
{
    bl_info_entry_t* p_flag_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_FLAGS);
    if (p_flag_entry)
    {
        return (p_flag_entry->flags.sd_intact &&
                p_flag_entry->flags.app_intact &&
                p_flag_entry->flags.bl_intact);
    }

    return true;
}

static bool app_is_valid(uint32_t* p_app_start)
{
    bl_info_entry_t* p_fwid_entry    = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_FLAGS);
    
    return (p_fwid_entry != NULL &&
            *p_app_start != 0xFFFFFFFF &&
            p_fwid_entry->version.app.app_version != APP_VERSION_INVALID &&
            fw_is_verified());
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


static uint32_t bl_evt_handler(bl_evt_t* p_evt)
{
    bl_cmd_t rsp_cmd;
    switch (p_evt->type)
    {
        case BL_EVT_TYPE_ABORT:
        {
            bl_info_entry_t* p_segment_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_FLAGS);
            switch (p_evt->params.abort.reason)
            {
                case BL_END_SUCCESS:
                case BL_END_ERROR_TIMEOUT:
                case BL_END_FWID_VALID:
                case BL_END_ERROR_MBR_CALL_FAILED:
                    if (p_segment_entry && app_is_valid((uint32_t*) p_segment_entry->segment.start))
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
            break;
        }
        case BL_EVT_TYPE_FLASH_WRITE:
            nrf_flash_store((uint32_t*) p_evt->params.flash.write.start_addr,
                                        p_evt->params.flash.write.p_data,
                                        p_evt->params.flash.write.length, 0);
        
            rsp_cmd.type                            = BL_CMD_TYPE_FLASH_WRITE_COMPLETE;
            rsp_cmd.params.flash.write.start_addr   = p_evt->params.flash.write.start_addr;
            rsp_cmd.params.flash.write.p_data       = p_evt->params.flash.write.p_data;
            rsp_cmd.params.flash.write.length       = p_evt->params.flash.write.length;
            bl_cmd_handler(&rsp_cmd);
            break;
        case BL_EVT_TYPE_FLASH_ERASE:
            nrf_flash_erase((uint32_t*) p_evt->params.flash.erase.start_addr,
                                        p_evt->params.flash.erase.length);
        
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

int main(void)
{
    init_clock();

    NVIC_SetPriority(SWI2_IRQn, 2);
    NVIC_EnableIRQ(SWI2_IRQn);
    __enable_irq();

    init_leds();
    rtc_init();
#ifdef SERIAL
    mesh_aci_init();
#endif
    transport_init(rx_cb, RBC_MESH_ACCESS_ADDRESS_BLE_ADV);
    if (bootloader_info_init((uint32_t*) BOOTLOADER_INFO_ADDRESS,
                             (uint32_t*) (BOOTLOADER_INFO_ADDRESS - PAGE_SIZE))
        != NRF_SUCCESS)
    {
        bootloader_abort(BL_END_ERROR_INVALID_PERSISTENT_STORAGE);
    }

    bl_cmd_t init_cmd;
    init_cmd.type = BL_CMD_TYPE_INIT;
    init_cmd.params.init.bl_if_version = BL_IF_VERSION;
    init_cmd.params.init.event_callback = bl_evt_handler;
    init_cmd.params.init.timer_count = 1;
    bl_cmd_handler(&init_cmd);
    
    /* check whether we should go to application */
    if (NRF_POWER->GPREGRET == RBC_MESH_GPREGRET_CODE_GO_TO_APP)
    {
        bootloader_abort(BL_END_SUCCESS);
    }
    NRF_POWER->GPREGRET = RBC_MESH_GPREGRET_CODE_GO_TO_APP;

    bootloader_start();
    transport_start();

    while (1)
    {
        __WFE();
    }
}
