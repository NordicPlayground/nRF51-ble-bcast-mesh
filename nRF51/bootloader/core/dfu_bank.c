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
#include "dfu_bank.h"
#include "rbc_mesh.h"
#include "bootloader_info.h"
#include "bootloader_app_bridge.h"
#include "nrf_mbr.h"
#include "dfu_types_mesh.h"
#include "nrf_error.h"
#include "nrf.h"

#if NORDIC_SDK_VERSION >= 11
#include "nrf_nvic.h"
#endif

#include "rtt_log.h"
#include "dfu_util.h"
#include "toolchain.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

/*****************************************************************************
* Local typedefs
*****************************************************************************/

/*****************************************************************************
* Static globals
*****************************************************************************/
static bl_info_bank_t*  mp_bank_entry;
static dfu_type_t       m_dfu_type;
static bool             m_waiting_for_idle;
/*****************************************************************************
* Static functions
*****************************************************************************/

static void flash_bank_entry(void)
{
    bl_info_bank_t* p_bank_entry = mp_bank_entry; /* make local copy to avoid race conditions */
    if (p_bank_entry == NULL)
    {
        return;
    }

    bl_info_entry_t bank_entry_replacement;
    memcpy(&bank_entry_replacement, p_bank_entry, sizeof(bl_info_bank_t));
    switch (p_bank_entry->state)
    {
        case BL_INFO_BANK_STATE_IDLE:
            {
                m_waiting_for_idle = true;
                bank_entry_replacement.bank.state = BL_INFO_BANK_STATE_FLASH_FW;
                bootloader_info_entry_overwrite((bl_info_type_t) (BL_INFO_TYPE_BANK_BASE + m_dfu_type), &bank_entry_replacement);

                /* Wait for this to take effect before moving on, as the
                   potential mbr commands in the flash_fw state may trigger
                   sudden reboots. */
                return;
            }

        case BL_INFO_BANK_STATE_FLASH_FW:
            switch (m_dfu_type)
            {
                case DFU_TYPE_BOOTLOADER:
                    /* Check to see if the bank transfer has been executed */
                    if (memcmp(p_bank_entry->p_bank_addr,
                                (uint32_t*) bootloader_info_entry_get(BL_INFO_TYPE_SEGMENT_BL)->segment.start,
                                p_bank_entry->length) != 0)
                    {
                        /* move the bank with MBR. BOOTLOADERADDR() must
                           have been set. */
                        sd_mbr_command_t sd_mbr_cmd;

                        sd_mbr_cmd.command               = SD_MBR_COMMAND_COPY_BL;
                        sd_mbr_cmd.params.copy_bl.bl_src = p_bank_entry->p_bank_addr;
                        sd_mbr_cmd.params.copy_bl.bl_len = p_bank_entry->length / sizeof(uint32_t);
                        APP_ERROR_CHECK(sd_mbr_command(&sd_mbr_cmd));
                        return; /* Can't be reached, only here for readability. */
                    }
                    else
                    {
                        bank_entry_replacement.bank.state = BL_INFO_BANK_STATE_FLASH_META;
                        bootloader_info_entry_overwrite(BL_INFO_TYPE_BANK_BL, &bank_entry_replacement);
                    }
                    break;

                case DFU_TYPE_SD:
                    /* Check to see if the bank transfer has been executed */
                    if (memcmp(p_bank_entry->p_bank_addr,
                                (uint32_t*) bootloader_info_entry_get(BL_INFO_TYPE_SEGMENT_SD)->segment.start,
                                p_bank_entry->length) != 0)
                    {
                        /* move the bank with MBR. */
                        sd_mbr_command_t sd_mbr_cmd;

                        sd_mbr_cmd.command               = SD_MBR_COMMAND_COPY_SD;
                        sd_mbr_cmd.params.copy_sd.src    = p_bank_entry->p_bank_addr;
                        sd_mbr_cmd.params.copy_sd.len    = p_bank_entry->length / sizeof(uint32_t);
                        sd_mbr_cmd.params.copy_sd.dst    = (uint32_t*) 0x1000;
                        APP_ERROR_CHECK(sd_mbr_command(&sd_mbr_cmd));
                        return; /* Can't be reached, only here for readability. */
                    }
                    else
                    {
                        bank_entry_replacement.bank.state = BL_INFO_BANK_STATE_FLASH_META;
                        bootloader_info_entry_overwrite((bl_info_type_t) (BL_INFO_TYPE_BANK_BASE + m_dfu_type), &bank_entry_replacement);
                    }
                    break;

                case DFU_TYPE_APP:
                    /* This nukes the call stack and any flash-callbacks on the
                       app side. If we're in the application, we have to jump
                       to bootloader. */
                    if (bootloader_is_in_application())
                    {
                        /* All paths leading to this call warns about this
                           reset. We'll come back to finalize the transfer
                           after the reset. */
                        __LOG("IN APP MODE. RESET!\n");

#if 1 //def SOFTDEVICE_PRESENT
                        sd_power_reset_reason_clr(0x0F000F);

#if NORDIC_SDK_VERSION >= 11
                        sd_power_gpregret_set(0, RBC_MESH_GPREGRET_CODE_GO_TO_APP);
#else
                        sd_power_gpregret_set(RBC_MESH_GPREGRET_CODE_GO_TO_APP);
#endif
                        sd_nvic_SystemReset();
#else
                        NRF_POWER->RESETREAS = 0x0F000F; /* erase reset-reason to avoid wrongful state-readout on reboot */
                        NRF_POWER->GPREGRET = RBC_MESH_GPREGRET_CODE_GO_TO_APP;
                        NVIC_SystemReset();
#endif
                    }
                    else
                    {
                        /* Erase, Flash the FW, flash FW flag, flash the signature, erase the bank entry. */
                        bl_info_entry_t* p_app_entry = bootloader_info_entry_get(BL_INFO_TYPE_SEGMENT_APP);

                        APP_ERROR_CHECK_BOOL(p_app_entry != NULL);
                        APP_ERROR_CHECK_BOOL(IS_PAGE_ALIGNED(p_app_entry->segment.start));

                        /* Erase existing FW */
                        bl_evt_t flash_evt;
                        flash_evt.type = BL_EVT_TYPE_FLASH_ERASE;
                        flash_evt.params.flash.erase.start_addr = p_app_entry->segment.start;
                        flash_evt.params.flash.erase.length = ((p_bank_entry->length + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1)); /* Pad the rest of the page */
                        if (bootloader_evt_send(&flash_evt) != NRF_SUCCESS)
                        {
                            m_waiting_for_idle = true;
                            return;
                        }

                        /* Flash bank */
                        flash_evt.type = BL_EVT_TYPE_FLASH_WRITE;
                        flash_evt.params.flash.write.p_data = (uint8_t*) p_bank_entry->p_bank_addr;
                        flash_evt.params.flash.write.length = p_bank_entry->length;
                        flash_evt.params.flash.write.start_addr = p_app_entry->segment.start;
                        if (bootloader_evt_send(&flash_evt) != NRF_SUCCESS)
                        {
                            m_waiting_for_idle = true;
                            return;
                        }

                        /* Update state */
                        bank_entry_replacement.bank.state = BL_INFO_BANK_STATE_FLASH_META;
                        bootloader_info_entry_overwrite((bl_info_type_t) (BL_INFO_TYPE_BANK_BASE + m_dfu_type), &bank_entry_replacement);
                    }
                    break;

                default:
                    APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                    break;
            }
            /* deliberate fallthrough */
        case BL_INFO_BANK_STATE_FLASH_META:
            {
                bl_info_entry_t fwid_entry;
                bl_info_type_t signature_type;
                bl_info_entry_t flags_entry;
                bl_info_entry_t* p_old_fwid_entry  = bootloader_info_entry_get(BL_INFO_TYPE_VERSION);
                bl_info_entry_t* p_old_flags_entry = bootloader_info_entry_get(BL_INFO_TYPE_FLAGS);
                APP_ERROR_CHECK_BOOL(p_old_fwid_entry);
                APP_ERROR_CHECK_BOOL(p_bank_entry);

                memcpy(&fwid_entry, p_old_fwid_entry, sizeof(bl_info_version_t));
                memcpy(&flags_entry, p_old_flags_entry, sizeof(bl_info_flags_t));
                switch (m_dfu_type)
                {
                    case DFU_TYPE_SD:
                        fwid_entry.version.sd = p_bank_entry->fwid.sd;
                        signature_type = BL_INFO_TYPE_SIGNATURE_SD;
                        flags_entry.flags.sd_intact = true;
                        break;
                    case DFU_TYPE_BOOTLOADER:
                        fwid_entry.version.bootloader.id  = p_bank_entry->fwid.bootloader.id;
                        fwid_entry.version.bootloader.ver = p_bank_entry->fwid.bootloader.ver;
                        signature_type = BL_INFO_TYPE_SIGNATURE_BL;
                        flags_entry.flags.bl_intact = true;
                        break;
                    case DFU_TYPE_APP:
                        fwid_entry.version.app.company_id   = p_bank_entry->fwid.app.company_id;
                        fwid_entry.version.app.app_id       = p_bank_entry->fwid.app.app_id;
                        fwid_entry.version.app.app_version  = p_bank_entry->fwid.app.app_version;
                        signature_type = BL_INFO_TYPE_SIGNATURE_APP;
                        flags_entry.flags.app_intact = true;
                        break;
                    default:
                        APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                        return;
                }
                if (!bootloader_info_entry_put(BL_INFO_TYPE_VERSION,
                            &fwid_entry,
                            BL_INFO_LEN_FWID))
                {
                    m_waiting_for_idle = true;
                    return;
                }
                if (p_bank_entry->has_signature)
                {
                    if (!bootloader_info_entry_put(signature_type,
                                (bl_info_entry_t*) p_bank_entry->signature,
                                BL_INFO_LEN_SIGNATURE))
                    {
                        m_waiting_for_idle = true;
                        return;
                    }
                }
                if (!bootloader_info_entry_put(BL_INFO_TYPE_FLAGS,
                            &flags_entry,
                            BL_INFO_LEN_FLAGS))
                {
                    m_waiting_for_idle = true;
                    return;
                }

                /* Update state */
                __LOG("Bank: Set state to FLASHED\n");
                bank_entry_replacement.bank.state = BL_INFO_BANK_STATE_FLASHED;
                bootloader_info_entry_overwrite((bl_info_type_t) (BL_INFO_TYPE_BANK_BASE + m_dfu_type), &bank_entry_replacement);

            }
            /* deliberate fallthrough */
        case BL_INFO_BANK_STATE_FLASHED:
            /* We may invalidate the bank entry in the device page now,
               it's all redundant. */
            __LOG("Bank: Invalidate.\n");
            if (bootloader_info_entry_invalidate((bl_info_type_t) (BL_INFO_TYPE_BANK_BASE + m_dfu_type)) == NRF_SUCCESS)
            {
                __LOG("Bank invalidated.\n");
                mp_bank_entry = NULL; /* reset the static bank pointer, as we no longer need it. */
            }
            else
            {
                m_waiting_for_idle = true;
            }
            break;

    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void dfu_bank_scan(bool* p_bank_flash_started)
{
    for (uint32_t i = 1; i <= 4; i <<= 1)
    {
        bl_info_entry_t* p_bank_entry = bootloader_info_entry_get((bl_info_type_t) (BL_INFO_TYPE_BANK_BASE + i));
        if (!p_bank_entry)
        {
            continue;
        }
        if (p_bank_entry->bank.state != BL_INFO_BANK_STATE_IDLE)
        {
            __LOG("Flash of bank of type %d already started (state 0x%x), continue where we left off!\n", i, p_bank_entry->bank.state);
            m_dfu_type = (dfu_type_t) i;
            mp_bank_entry = &p_bank_entry->bank;
            flash_bank_entry();
            if (p_bank_flash_started)
            {
                *p_bank_flash_started = true;
            }

            return;
        }
    }
    mp_bank_entry = NULL;
    if (p_bank_flash_started)
    {
        *p_bank_flash_started = false;
    }
}

uint32_t dfu_bank_flash(dfu_type_t dfu_type)
{
    if (dfu_type != DFU_TYPE_SD &&
        dfu_type != DFU_TYPE_APP &&
        dfu_type != DFU_TYPE_BOOTLOADER)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (mp_bank_entry != NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    bl_info_entry_t* p_bank_entry = bootloader_info_entry_get((bl_info_type_t) (BL_INFO_TYPE_BANK_BASE + dfu_type));
    if (!p_bank_entry || *p_bank_entry->bank.p_bank_addr == 0xFFFFFFFF)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    __LOG("Flash bank of type %d\n", dfu_type);
    m_dfu_type = dfu_type;
    mp_bank_entry = &p_bank_entry->bank;
    flash_bank_entry();

    return NRF_SUCCESS;
}

bool dfu_bank_is_available(dfu_type_t dfu_type)
{
    bl_info_entry_t* p_entry = bootloader_info_entry_get((bl_info_type_t) (BL_INFO_TYPE_BANK_BASE + dfu_type));
    return (p_entry && p_entry->bank.state == BL_INFO_BANK_STATE_IDLE);
}

void dfu_bank_on_flash_idle(void)
{
    if (mp_bank_entry != NULL && m_waiting_for_idle)
    {
        m_waiting_for_idle = false;
        flash_bank_entry();
    }
}

bool dfu_bank_transfer_in_progress(void)
{
    return (mp_bank_entry != NULL);
}
