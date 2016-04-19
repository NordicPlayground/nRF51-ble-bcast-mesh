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
#include <stdio.h>
#include <string.h>
#include "bootloader_info_app.h"
#include "app_error.h"
#include "nrf51.h"

#define WORD_ALIGN(data) data = (((uint32_t) data + 4) & 0xFFFFFFFC)

#define HEADER_LEN       (4)

#ifdef DEBUG
#undef DEBUG
#define DEBUG 0
#endif

#define PIN_INVALIDATE      (0)
#define PIN_RESET           (1)
#define PIN_ENTRY_GET       (2)
#define PIN_SET_LEN         (3)
#define PIN_ENTRY_PUT       (4)
#define PIN_INIT            (5)

#if defined(NRF51) && DEBUG
#define PIN_SET(x) NRF_GPIO->OUTSET = (1 << (x))
#define PIN_CLEAR(x) NRF_GPIO->OUTCLR = (1 << (x))
#define PIN_TICK(x) do{ PIN_SET(x); _NOP(); _NOP(); _NOP(); _NOP(); PIN_CLEAR(x); while (0)
#else
#define PIN_SET(x)
#define PIN_CLEAR(x)
#define PIN_TICK(x)
#endif

typedef struct
{
    uint16_t len;
    uint16_t type;
} bootloader_info_header_t;

static bootloader_info_t* mp_bl_info_page;
/******************************************************************************
* Static functions
******************************************************************************/
static inline bootloader_info_header_t* bootloader_info_iterate(bootloader_info_header_t* p_info_header)
{
    return (bootloader_info_header_t*) (((uint32_t) p_info_header) + ((uint32_t) p_info_header->len) * 4);
}
/******************************************************************************
* Interface functions
******************************************************************************/
uint32_t bootloader_info_init(uint32_t* p_bl_info_page)
{
    if ((uint32_t) p_bl_info_page & (PAGE_SIZE - 1))
    {
        return NRF_ERROR_INVALID_ADDR; /* must be page aligned */
    }

    mp_bl_info_page = (bootloader_info_t*) p_bl_info_page;

    return NRF_SUCCESS;
}

bl_info_entry_t* bootloader_info_entry_get(uint32_t* p_bl_info_page, bl_info_type_t type)
{
    if (mp_bl_info_page &&
        mp_bl_info_page->metadata.entry_header_length != 0xFF)
    {
        bootloader_info_header_t* p_header =
            (bootloader_info_header_t*)
            ((uint32_t) p_bl_info_page + ((bootloader_info_t*) p_bl_info_page)->metadata.metadata_len);
        uint32_t iterations = 0;
        bl_info_type_t iter_type = (bl_info_type_t) p_header->type;
        while (iter_type != type)
        {
            p_header = bootloader_info_iterate(p_header);
            if ((uint32_t) p_header > ((uint32_t) p_bl_info_page) + PAGE_SIZE ||
                (
                 (iter_type == BL_INFO_TYPE_LAST) &&
                 (iter_type != type)
                ) ||
                ++iterations > PAGE_SIZE / 2)
            {
                PIN_CLEAR(PIN_ENTRY_GET);
                return NULL; /* out of bounds */
            }
            iter_type = (bl_info_type_t) p_header->type;
        }
        return (bl_info_entry_t*) ((uint32_t) p_header + 4 * mp_bl_info_page->metadata.entry_header_length);
    }
    else
    {
        return NULL;
    }
}

