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
#include "cache.h"
#include "journal.h"

static dfu_record_t* gp_cache;
static uint32_t g_cache_size;
static uint32_t g_index;
static uint8_t g_assembly_page[PAGE_SIZE];

#define FULL_ADDRESS(short_addr)    (((uint32_t) (short_addr)) << 8)
#define PAGE_INDEX(full_addr)       ((full_addr) >> 10)
#define PAGE_LOCAL_ADDR(full_addr)  ((full_addr) & 0x03FF)

#define INVALID_PAGE_INDEX          (0xFFFF)
#define INVALID_SHORT_ADDR          (0xFFFF)
#define CACHE_SIZE_MARGIN           (g_cache_size / 10)

static void cache_invalidate(void)
{
    for (uint32_t i = 0; i < g_cache_size; ++i)
    {
        gp_cache[i].short_addr = INVALID_SHORT_ADDR;
    }
}

static uint16_t get_first_page_index(bool first_round)
{
    static uint32_t index = 0;
    if (first_round)
    {
        index = 0;
    }
    
    for (; index < g_index; ++index)
    {
        if (gp_cache[index].short_addr != INVALID_SHORT_ADDR)
        {
            return PAGE_INDEX(FULL_ADDRESS(gp_cache[index].short_addr));
        }
    }
    return INVALID_PAGE_INDEX;
}

static dfu_record_t* get_next_record(uint16_t page_index, bool first_round)
{
    static uint32_t index = 0;
    if (first_round)
    {
        index = 0;
    }
    for (; index < g_index; ++index)
    {
        if (PAGE_INDEX(FULL_ADDRESS(gp_cache[index].short_addr)) == page_index)
        {
            return &gp_cache[index];
        }
    }

    return NULL;
}

void cache_init(void)
{
    /** @TODO: set gp_cache pos */

    cache_invalidate();
}

void cache_record_add(dfu_record_t* p_record)
{
    memcpy(&gp_cache[g_index++], p_record, sizeof(dfu_record_t));

    if (g_index == g_cache_size - CACHE_SIZE_MARGIN)
    {
        cache_flash();
    }
}

void cache_flash(void)
{
    uint32_t page_index = get_first_page_index(true);

    while (page_index != INVALID_PAGE_INDEX)
    {
        /* load old page from flash */
        memcpy(g_assembly_page, PAGE_SIZE * page_index, PAGE_SIZE);

        /* apply records to page */
        dfu_record_t* p_rec = get_next_record(page_index, true);
        uint32_t record_count = 0;
        while (p_rec != NULL)
        {
            record_count++;

            memcpy(g_assembly_page + PAGE_LOCAL_ADDR(FULL_ADDRESS(p_rec->short_addr)), p_rec->data, DFU_RECORD_SIZE);
            p_rec->short_addr = INVALID_SHORT_ADDR; /* invalidate */

            p_rec = get_next_record(page_index, false);
        }
        
        /* erase and flash */
        journal_invalidate(page_index);
        nrf_flash_erase(page_index * PAGE_SIZE, PAGE_SIZE);
        nrf_flash_store(page_index * PAGE_SIZE, g_assembly_page, PAGE_SIZE, 0);
        journal_complete(page_index);

        /* fetch next page */
        page_index = get_first_page_index(false);
    }
    g_index = 0;
}

bool cache_record_get(uint16_t seq_num, dfu_record_t* p_record)
{
    for (uint32_t i = 0; i < g_index; ++i)
    {
        if (gp_cache[i].seq_num == seq_num)
        {
            memcpy(p_record, &gp_cache[i], sizeof(dfu_record_t));
            return true;
        }
    }
    return false;
} 

