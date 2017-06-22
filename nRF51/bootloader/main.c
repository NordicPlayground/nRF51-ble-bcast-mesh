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
#include <stdarg.h>

#include "bl_if.h"
#include "bootloader.h"
#include "bootloader_info.h"
#include "rtt_log.h"
#include "rbc_mesh.h"
#include "dfu_bank.h"
#include "boards.h"

#include "app_error.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"

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

void app_error_handler_bare(uint32_t error_code)
{
    __disable_irq();
#ifdef DEBUG_LEDS
    NRF_GPIO->OUTCLR = LED_4;
#endif
    __BKPT(0);
    while (1);
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    __LOG(RTT_CTRL_TEXT_RED "APP ERROR %d, @%s:L%d\n", error_code, p_file_name, line_num);
    app_error_handler_bare(error_code);
}


void HardFault_Handler(uint32_t pc, uint32_t lr)
{
    __LOG(RTT_CTRL_TEXT_RED "HARDFAULT pc=0x%x\n", pc);
    __disable_irq();
#ifdef DEBUG_LEDS
    NRF_GPIO->OUTCLR = LED_4;
#endif
    __BKPT(0);
    while (1);
}

static void init_leds(void)
{
#ifdef DEBUG_LEDS
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    NRF_GPIO->OUT = LEDS_MASK;
#endif
}

static void init_clock(void)
{
    if (!NRF_CLOCK->HFCLKRUN)
    {
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
        while (!NRF_CLOCK->EVENTS_HFCLKSTARTED);
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    }
    if (!NRF_CLOCK->LFCLKRUN)
    {
        NRF_CLOCK->TASKS_LFCLKSTART = 1;
        while (!NRF_CLOCK->EVENTS_LFCLKSTARTED);
        NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    }
    NRF_CLOCK->TASKS_CAL = 1;
    while (!NRF_CLOCK->EVENTS_DONE);
    NRF_CLOCK->EVENTS_DONE = 0;
}

int main(void)
{
    init_clock();

    NVIC_SetPriority(SWI2_IRQn, 2);
    NVIC_EnableIRQ(SWI2_IRQn);
    __enable_irq();
#ifdef RTT_LOG
    SEGGER_RTT_Init();
    __LOG("= START | %s | ===========================================================\n", __TIME__);
#endif

    init_leds();
    bootloader_init();

    /* Wait for any ongoing bank transfers to finish. */
    while (dfu_bank_transfer_in_progress())
    {
        /* may safely while-loop here, as the bank-transfer finishes in an IRQ. */
        __WFE();
    }

    /* check whether we should go to application */
    if (NRF_POWER->GPREGRET == RBC_MESH_GPREGRET_CODE_GO_TO_APP)
    {
        bootloader_abort(DFU_END_SUCCESS);
    }
    NRF_POWER->GPREGRET = RBC_MESH_GPREGRET_CODE_GO_TO_APP;

    bootloader_enable();

    while (1)
    {
        __WFE();
    }
}

