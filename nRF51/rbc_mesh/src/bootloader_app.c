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

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


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

#include <stddef.h>
#include "bootloader_app.h"
#include "bootloader_util.h"
#include "rbc_mesh.h"
#include "dfu_types_mesh.h"
#include "toolchain.h"

#define IRQ_ENABLED            0x01     /**< Field that identifies if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS  32       /**< Maximum number of interrupts available. */

bootloader_prepare_cb_t m_prepare_cb = NULL;

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

uint32_t bootloader_start(void)
{
    if (NRF_UICR->BOOTLOADERADDR != 0xFFFFFFFF)
    {
        interrupts_disable();
        
        if (m_prepare_cb)
        {
            m_prepare_cb();
        }

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

void bootloader_prepare_callback_set(bootloader_prepare_cb_t prepare_callback)
{
    /* or would it work with an event? */
    m_prepare_cb = prepare_callback;
}
