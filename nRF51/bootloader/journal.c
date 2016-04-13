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
#include "journal.h"
#include "dfu_mesh.h"
#include "nrf_flash.h"
#include "dfu_types_mesh.h"

#define TO_WORD(index) ((index) / 32)
#define TO_OFFSET(index) ((index) & 0x1F)
#define GET_FLAG(index, bitfield) !!(*(bitfield + TO_WORD(index)) & (1 << TO_OFFSET(index)))

static uint32_t* mp_invalidate_field;
static uint32_t* mp_complete_field;

static void mark_single(uint32_t* p_field, uint32_t* p_page)
{
    uint32_t page_index = ((uint32_t) p_page) / PAGE_SIZE;
    uint32_t field = ~(1 << TO_OFFSET(page_index));
    nrf_flash_store(p_field, (uint8_t*) &field, 4, TO_WORD(page_index) * 4);
}

static void mark_multiple(uint32_t* p_field, uint32_t* p_first, uint32_t count)
{
    uint32_t page_index = ((uint32_t) p_first) / PAGE_SIZE;
    uint32_t fields[32];
    memset(fields, 0xFF, 32 * 4);
    for (uint32_t page = 0; page < count; ++page)
    {
        fields[TO_WORD(page_index + page)] &= ~(1 << TO_OFFSET(page_index + page));
    }
    nrf_flash_store(p_field, (uint8_t*) fields, 4 * (1 + TO_WORD(page_index + count)), 0);
}

void journal_init(uint32_t* p_invalidate_field, uint32_t* p_complete_field)
{
    mp_invalidate_field = p_invalidate_field;
    mp_complete_field = p_complete_field;
}

void journal_invalidate(uint32_t* p_page)
{
    mark_single(mp_invalidate_field, p_page);
}

void journal_invalidate_multiple(uint32_t* p_first, uint32_t count)
{
    mark_multiple(mp_invalidate_field, p_first, count);
}

void journal_complete(uint32_t* p_page)
{
    mark_single(mp_complete_field, p_page);
}

void journal_complete_multiple(uint32_t* p_first, uint32_t count)
{
    mark_multiple(mp_complete_field, p_first, count);
}

bool journal_is_complete(uint32_t* p_start, uint32_t length)
{
    /* complete if all pages are tagged as complete */
    for (uint32_t i = (uint32_t) p_start / PAGE_SIZE; i < ((uint32_t) p_start + length) / PAGE_SIZE; ++i)
    {
        if (GET_FLAG(i, mp_complete_field))
        {
            return false;
        }
    }
    return true;
}

bool journal_is_invalid(uint32_t* p_start, uint32_t length)
{
    if (mp_invalidate_field == NULL || mp_complete_field == NULL)
    {
        return false; /* no reason to believe that the journal is invalid */
    }
    
    /* invalid if at least one page is invalid, and the entire thing isn't complete. */
    for (uint32_t i = (uint32_t) p_start / PAGE_SIZE; i < ((uint32_t) p_start + length) / PAGE_SIZE; ++i)
    {
        if (!GET_FLAG(i, mp_invalidate_field))
        {
            return !journal_is_complete(p_start, length);
        }
    }
    return false;
}

