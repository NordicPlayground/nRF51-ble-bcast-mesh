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

#include "journal.h"
#include "dfu.h"

#include "nrf_flash.h"
#include "dfu_types_mesh.h"
#include "boards.h"

#define JOURNAL_INVALIDATE_ADDR  ((uint32_t*) 0x3FF00)
#define JOURNAL_COMPLETED_ADDR   ((uint32_t*) 0x3FF80)

#define TO_WORD(index) ((index) / 32)
#define TO_OFFSET(index) ((index & 0x1F))
#define GET_FLAG(index, bitfield) !!(*(bitfield + TO_WORD(index)) & (1 << TO_OFFSET(index)))

static uint32_t g_start_addr;
static uint32_t g_size;

void journal_init(uint32_t start_address, uint32_t size)
{
    g_start_addr = start_address;
    g_size = size;
}

void journal_invalidate(uint16_t page_index)
{
    page_index -= g_start_addr / PAGE_SIZE;
    uint32_t field = ~(1 << TO_OFFSET(page_index));
    nrf_flash_store(JOURNAL_INVALIDATE_ADDR, (uint8_t*) &field, 4, TO_WORD(page_index) * 4);
}

void journal_complete(uint16_t page_index)
{
    page_index -= g_start_addr / PAGE_SIZE;
    uint32_t field = ~(1 << TO_OFFSET(page_index));
    nrf_flash_store(JOURNAL_COMPLETED_ADDR, (uint8_t*) &field, 4, TO_WORD(page_index) * 4);
}

bool journal_is_finished(void)
{
    for (uint32_t i = 0; i <= g_size / PAGE_SIZE; ++i)
    {
        if (GET_FLAG(i, JOURNAL_COMPLETED_ADDR))
        {
            return false;
        }
    }
    return true;
}

bool journal_is_invalid(void)
{
    for (uint32_t i = 0; i <= g_size / PAGE_SIZE; ++i)
    {
        if (!GET_FLAG(i, JOURNAL_INVALIDATE_ADDR))
        {
            return true;
        }
    }
    return false;
}

