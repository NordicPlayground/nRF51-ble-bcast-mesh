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

#ifndef DFU_UTIL_H__
#define DFU_UTIL_H__

#include <stdint.h>
#include "dfu_types_mesh.h"
#include "nrf.h"

#define IS_PAGE_ALIGNED(p) (((uint32_t)(p) & (PAGE_SIZE - 1)) == 0)
#define IS_WORD_ALIGNED(p) (((uint32_t)(p) & (WORD_SIZE - 1)) == 0)


void fwid_union_cpy(fwid_union_t* p_dst, fwid_union_t* p_src, dfu_type_t dfu_type);

/** Returns true if the two fwids are equal. */
bool fwid_union_cmp(fwid_union_t* p_a, fwid_union_t* p_b, dfu_type_t dfu_type);

/** Returns true if the two fwids have the same ID, ignoring versioning */
bool fwid_union_id_cmp(fwid_union_t* p_a, fwid_union_t* p_b, dfu_type_t dfu_type);

bool ready_packet_is_upgrade(dfu_packet_t* p_packet);

bool ready_packet_matches_our_req(dfu_packet_t* p_packet, dfu_type_t dfu_type_req, fwid_union_t* p_fwid_req);

uint32_t* addr_from_seg(uint16_t segment, uint32_t* p_start_addr);

bool app_is_newer(app_id_t* p_app_id);

bool bootloader_is_newer(bl_id_t bl_id);

bool fw_is_verified(void);

void tid_cache_entry_put(uint32_t tid);

bool tid_cache_has_entry(uint32_t tid);

bool packet_in_cache(dfu_packet_t* p_packet);

void packet_cache_put(dfu_packet_t* p_packet);

void packet_cache_flush(void);

bool section_overlap(uint32_t section_a_start, uint32_t section_a_length, 
                     uint32_t section_b_start, uint32_t section_b_length);
#endif /* DFU_UTIL_H__ */

