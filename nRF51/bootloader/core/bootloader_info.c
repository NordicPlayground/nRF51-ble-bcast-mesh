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
#include "bootloader_info.h"
#include "dfu_types_mesh.h"
#include "bootloader_app_bridge.h"
#include "nrf51.h"
#include "nrf_error.h"
#include "toolchain.h"

#define WORD_ALIGN(data) data = (((uint32_t) data + 4) & 0xFFFFFFFC)

#define HEADER_LEN       (sizeof(bootloader_info_header_t))

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

#define INFO_WRITE_BUFLEN   (128)

typedef enum
{
    BL_INFO_STATE_UNINITIALIZED,
    BL_INFO_STATE_IDLE,
    BL_INFO_STATE_AWAITING_RECOVERY,
    BL_INFO_STATE_RECOVERING
} bl_info_state_t;

typedef struct
{
    uint16_t len;
    uint16_t type;
} bootloader_info_header_t;

typedef struct
{
    bootloader_info_header_t header;
    bl_info_entry_t entry;
} info_buffer_t;


static bootloader_info_t*               mp_bl_info_page;
static bootloader_info_t*               mp_bl_info_bank_page;
static uint8_t                          mp_info_entry_buffer[INFO_WRITE_BUFLEN];
static info_buffer_t*                   mp_info_entry_head;
static info_buffer_t*                   mp_info_entry_tail;
static const bootloader_info_header_t   m_invalid_header = {0xFFFF, 0x0000}; /**< Flash buffer used for invalidating entries. */
static const bootloader_info_header_t   m_last_header = {0xFFFF, BL_INFO_TYPE_LAST}; /**< Flash buffer used for flashing the "last" header. */
static bl_info_state_t                  m_state;
/******************************************************************************
* Static functions
******************************************************************************/
static void invalidate_entry(bootloader_info_header_t* p_header)
{
    /* TODO: optimization: check if the write only adds 0-bits to the current value,
       in which case we can just overwrite the current. */
    if ((uint32_t) p_header & 0x03)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_ADDR);
    }

    APP_ERROR_CHECK(flash_write((uint32_t*) p_header, (uint8_t*) &m_invalid_header, HEADER_LEN));
}

static void free_write_buffer(info_buffer_t* p_buf)
{
    /* This must happen in order. */
    APP_ERROR_CHECK_BOOL(p_buf == mp_info_entry_tail);

    /* The end-of-entries entry may have max length - treat as empty entry */
    if (mp_info_entry_tail->header.len == 0xFFFF)
    {
        mp_info_entry_head->header.len += 1;
    }
    else
    {
        mp_info_entry_head->header.len += p_buf->header.len;
    }
    mp_info_entry_tail = (info_buffer_t*) ((uint32_t) p_buf + p_buf->header.len * 4);
    if (mp_info_entry_tail == (info_buffer_t*) (mp_info_entry_buffer + INFO_WRITE_BUFLEN))
    {
        mp_info_entry_tail = (info_buffer_t*) mp_info_entry_buffer;
    }
    if (mp_info_entry_tail->header.type == BL_INFO_TYPE_LAST)
    {
        /* tail recursion */
        free_write_buffer(mp_info_entry_tail);
    }
}

static void place_head_after_entry(info_buffer_t* p_entry)
{
    mp_info_entry_head = (info_buffer_t*) ((uint32_t) p_entry + p_entry->header.len * 4);
    if (mp_info_entry_head == (info_buffer_t*) (mp_info_entry_buffer + INFO_WRITE_BUFLEN))
    {
        mp_info_entry_head = (info_buffer_t*) mp_info_entry_buffer;
    }
    if (mp_info_entry_head != mp_info_entry_tail)
    {
        mp_info_entry_head->header.len = ((uint32_t) mp_info_entry_tail - (uint32_t) mp_info_entry_head) / 4;
        if (mp_info_entry_tail < mp_info_entry_head)
        {
            mp_info_entry_head->header.len += INFO_WRITE_BUFLEN / 4;
        }
        mp_info_entry_head->header.type = BL_INFO_TYPE_INVALID;
    }
}

static info_buffer_t* get_write_buffer(uint32_t req_space)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    if (mp_info_entry_head->header.len * 4 < req_space || mp_info_entry_head->header.type != BL_INFO_TYPE_INVALID)
    {
        _ENABLE_IRQS(was_masked);
        return NULL;
    }

    if (mp_info_entry_head == mp_info_entry_tail)
    {
        /* the entire buffer is empty, let's just reset */
        mp_info_entry_head = (info_buffer_t*) mp_info_entry_buffer;
        mp_info_entry_tail = (info_buffer_t*) mp_info_entry_buffer;

        info_buffer_t* p_ret = mp_info_entry_head;
        p_ret->header.len = req_space / 4;
        p_ret->header.type = BL_INFO_TYPE_INVALID;
        place_head_after_entry(p_ret);
        _ENABLE_IRQS(was_masked);
        return p_ret;
    }

    const uint32_t space_until_rollover = ((uint32_t) mp_info_entry_buffer + INFO_WRITE_BUFLEN - (uint32_t) mp_info_entry_head);

    /* check if we have to put it on the other side of the rollover. */
    if (req_space > space_until_rollover)
    {
        /* can we fit it at the other side of the rollover? */
        if (req_space <= (uint32_t) mp_info_entry_head->header.len * 4 - space_until_rollover)
        {
            /* the last entry before the rollover is now padding, let's mark it as unused */
            uint16_t old_head_len = mp_info_entry_head->header.len;
            mp_info_entry_head->header.type = BL_INFO_TYPE_LAST;
            mp_info_entry_head->header.len = space_until_rollover / 4;

            mp_info_entry_head = (info_buffer_t*) mp_info_entry_buffer;
            info_buffer_t* p_ret = mp_info_entry_head;
            p_ret->header.len = req_space / 4;
            p_ret->header.type = BL_INFO_TYPE_INVALID;
            place_head_after_entry(p_ret);
            _ENABLE_IRQS(was_masked);
            return p_ret;
        }
        else
        {
            /* Couldn't fit the entry at either side of the rollover */
            _ENABLE_IRQS(was_masked);
            return NULL;
        }
    }
    else
    {
        /* straight forward put */
        info_buffer_t* p_ret = mp_info_entry_head;
        p_ret->header.len = req_space / 4;
        p_ret->header.type = BL_INFO_TYPE_INVALID;
        place_head_after_entry(p_ret);
        _ENABLE_IRQS(was_masked);
        return p_ret;
    }
}

static inline info_buffer_t* bootloader_info_iterate(info_buffer_t* p_buf)
{
    return (info_buffer_t*) (((uint32_t) p_buf) + ((uint32_t) p_buf->header.len) * 4);
}

static bootloader_info_header_t* bootloader_info_header_get(bl_info_entry_t* p_entry)
{
    if (p_entry == NULL || (uint32_t) p_entry == FLASH_SIZE)
    {
        return NULL;
    }
    return (bootloader_info_header_t*) ((uint32_t) p_entry - 4 * mp_bl_info_page->metadata.entry_header_length);
}

static inline info_buffer_t* bootloader_info_first_unused_get(bootloader_info_t* p_bl_info_page)
{
    bl_info_entry_t* p_entry = bootloader_info_entry_get(
            (uint32_t*) p_bl_info_page,
            BL_INFO_TYPE_LAST);

    return (info_buffer_t*) (p_entry ? bootloader_info_header_get(p_entry) : NULL);
}

#if 0
static bootloader_info_header_t* reset_with_replace(
        bl_info_type_t replace_type,
        bl_info_entry_t* p_entry,
        uint32_t entry_length)
{
    /* create new page from valid entries, assumes intact info page */
    uint8_t new_page[PAGE_SIZE];
    bootloader_info_header_t* p_replace_position = NULL;
    memset(new_page, 0xFF, PAGE_SIZE);
    memcpy(new_page, mp_bl_info_page, mp_bl_info_page->metadata.metadata_len);
    uint8_t* p_new_page_next_space = &new_page[mp_bl_info_page->metadata.metadata_len];

    bootloader_info_header_t* p_info =
        ((bootloader_info_header_t*)
         ((uint8_t*) mp_bl_info_page + mp_bl_info_page->metadata.metadata_len));

    while ((uint32_t) p_info < (uint32_t) ((uint32_t) mp_bl_info_page + PAGE_SIZE))
    {
        bl_info_type_t type = (bl_info_type_t) p_info->type;

        if (type == BL_INFO_TYPE_LAST)
        {
            break;
        }

        if (type != BL_INFO_TYPE_INVALID)
        {
            if (type == replace_type)
            {
                /* replace */
                p_replace_position = (bootloader_info_header_t*) p_new_page_next_space;
                uint32_t len = HEADER_LEN + entry_length;
                if (len & 0x03)
                {
                    WORD_ALIGN(len);
                }

                if (p_new_page_next_space + len > new_page + PAGE_SIZE)
                {
                    APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                }
                p_replace_position->len = len / 4;
                p_replace_position->type = type;
                memset(p_new_page_next_space + len - 3, 0xFF, 3); /* sanitize padding */
                memcpy(p_new_page_next_space + HEADER_LEN, p_entry, entry_length);
                p_new_page_next_space += len;
            }
            else
            {
                /* restore */
                uint32_t len = p_info->len * 4;
                if (p_new_page_next_space + len > new_page + PAGE_SIZE)
                {
                    APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                }
                memcpy(p_new_page_next_space, p_info, len);
                p_new_page_next_space += len;
            }
        }
        p_info = bootloader_info_iterate(p_info);
    }

    if (p_new_page_next_space < &new_page[PAGE_SIZE])
    {
        ((bootloader_info_header_t*) p_new_page_next_space)->type = BL_INFO_TYPE_LAST;
    }

    /* bank page */
    while (flash_erase((uint32_t*) mp_bl_info_bank_page, PAGE_SIZE) != NRF_SUCCESS);
    while (flash_write((uint32_t*) mp_bl_info_bank_page, new_page, p_new_page_next_space - &new_page[0] + 4) != NRF_SUCCESS);

    /* reflash original */
    while (flash_erase((uint32_t*) mp_bl_info_page, PAGE_SIZE) != NRF_SUCCESS);
    while (flash_write((uint32_t*) mp_bl_info_page, new_page, p_new_page_next_space - &new_page[0] + 4) != NRF_SUCCESS);

    return p_replace_position;
}
#endif
/******************************************************************************
* Interface functions
******************************************************************************/
uint32_t bootloader_info_init(uint32_t* p_bl_info_page, uint32_t* p_bl_info_bank_page)
{
    m_state = BL_INFO_STATE_UNINITIALIZED;

    if ((uint32_t) p_bl_info_page & (PAGE_SIZE - 1) ||
        (uint32_t) p_bl_info_bank_page & (PAGE_SIZE - 1))
    {
        return NRF_ERROR_INVALID_ADDR; /* must be page aligned */
    }

    mp_bl_info_page      = (bootloader_info_t*) p_bl_info_page;
    mp_bl_info_bank_page = (bootloader_info_t*) p_bl_info_bank_page;
    mp_info_entry_head = (info_buffer_t*) mp_info_entry_buffer;
    mp_info_entry_tail = (info_buffer_t*) mp_info_entry_buffer;
    mp_info_entry_head->header.len = INFO_WRITE_BUFLEN / 4;

    if (mp_bl_info_page->metadata.metadata_len == 0xFF)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    /* make sure we have an end-of-entries entry */
    if (bootloader_info_first_unused_get(mp_bl_info_page) == NULL)
    {
        /* need to restore the bank */
        info_buffer_t* p_first_unused = bootloader_info_first_unused_get(mp_bl_info_bank_page);
        if (p_first_unused == NULL)
        {
            /* bank is invalid too, no way to recover */
            return NRF_ERROR_INVALID_DATA;
        }

        APP_ERROR_CHECK(flash_erase((uint32_t*) mp_bl_info_page, PAGE_SIZE));
        APP_ERROR_CHECK(flash_write((uint32_t*) mp_bl_info_page,
                            (uint8_t*) mp_bl_info_bank_page,
                            (uint32_t) p_first_unused + 4 - (uint32_t) mp_bl_info_bank_page));
    }

    m_state = BL_INFO_STATE_IDLE;
    return NRF_SUCCESS;
}

bootloader_info_t* bootloader_info_get(void)
{
    return (bootloader_info_t*) (mp_bl_info_page);
}

bl_info_entry_t* bootloader_info_entry_get(uint32_t* p_bl_info_page, bl_info_type_t type)
{
    info_buffer_t* p_buffer =
        (info_buffer_t*)
        ((uint32_t) p_bl_info_page + ((bootloader_info_t*) p_bl_info_page)->metadata.metadata_len);

    uint32_t iterations = 0;
    bl_info_type_t iter_type = (bl_info_type_t) p_buffer->header.type;
    while (iter_type != type)
    {
        p_buffer = bootloader_info_iterate(p_buffer);
        if ((uint32_t) p_buffer > ((uint32_t) p_bl_info_page) + PAGE_SIZE ||
            (
             (iter_type == BL_INFO_TYPE_LAST) &&
             (iter_type != type)
            ) ||
            ++iterations > PAGE_SIZE / 2)
        {
            return NULL; /* out of bounds */
        }
        iter_type = (bl_info_type_t) p_buffer->header.type;
    }
    return &p_buffer->entry;
}

bl_info_entry_t* bootloader_info_entry_put(bl_info_type_t type,
                                           bl_info_entry_t* p_entry,
                                           uint32_t length)
{
    if (PAGE_ALIGN((uint32_t) p_entry) == (uint32_t) mp_bl_info_page) /* Can't come from our info page */
    {
        return NULL;
    }

    /* previous entry, to be invalidated after we've stored the current entry. */
    bootloader_info_header_t* p_old_header =
        bootloader_info_header_get(bootloader_info_entry_get((uint32_t*) mp_bl_info_page, type));

    /* find first unused space */
    info_buffer_t* p_new_buf =
        bootloader_info_first_unused_get(mp_bl_info_page);

    /* we have to be able to fit the end-of-entries entry at the last word, therefore check entry
       length + HEADER_LEN */
    if (p_new_buf == NULL ||
        ((uint32_t) p_new_buf + HEADER_LEN + length + HEADER_LEN) >= (uint32_t) ((uint32_t) mp_bl_info_page + PAGE_SIZE))
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
    }
    else
    {
        /* store new entry in the available space, and pad */
        uint32_t entry_length = ((HEADER_LEN + length + 3) & ~0x03);

        info_buffer_t* p_write_buf = get_write_buffer(entry_length + HEADER_LEN);
        APP_ERROR_CHECK_BOOL(p_write_buf != NULL);

        memset((uint8_t*) p_write_buf + entry_length - 4, 0xFF, 8); /* pad the end-of-entries entry */
        ((bootloader_info_header_t*) ((uint8_t*) p_write_buf + entry_length))->type = BL_INFO_TYPE_LAST;

        p_write_buf->header.type = type;
        p_write_buf->header.len = entry_length / 4;
        memcpy(&p_write_buf->entry, p_entry, length);

        APP_ERROR_CHECK(flash_write((uint32_t*) p_new_buf, (uint8_t*) p_write_buf, entry_length + HEADER_LEN));

        /* invalidate old entry of this type */
        if (p_old_header != NULL)
        {
            invalidate_entry(p_old_header);
        }

        if (((uint32_t) p_new_buf + HEADER_LEN + length + HEADER_LEN) >= (uint32_t) ((uint32_t) mp_bl_info_page + PAGE_SIZE))
        {

        }
    }
    return &p_new_buf->entry;
}

void bootloader_info_entry_invalidate(uint32_t* p_info_page, bl_info_type_t type)
{
    bootloader_info_header_t* p_header = bootloader_info_header_get(bootloader_info_entry_get(p_info_page, type));
    if (p_header)
    {
        invalidate_entry(p_header);
    }
}

void bootloader_info_reset(void)
{
    //TODO
    //reset_with_replace(BL_INFO_TYPE_INVALID, NULL, 0);
}

bool bootloader_info_available(void)
{
    return (m_state == BL_INFO_STATE_IDLE);
}

void bootloader_info_on_flash_op_end(flash_op_type_t type, void* p_context)
{
    if (type == FLASH_OP_TYPE_ERASE)
    {
    }
    else
    {
        if (p_context >= (void*) mp_info_entry_buffer && p_context < (void*) ((uint32_t) mp_info_entry_buffer + INFO_WRITE_BUFLEN))
        {
            free_write_buffer(p_context);
        }
        if (m_state == BL_INFO_STATE_AWAITING_RECOVERY &&
            (mp_info_entry_head == mp_info_entry_tail))
        {
            /* The flash operations are all finished, let's start recovery */
            m_state = BL_INFO_STATE_RECOVERING;
        }
    }
}
