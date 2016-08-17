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
#ifndef DFU_TRANSFER_MESH_H__
#define DFU_TRANSFER_MESH_H__

#include <stdint.h>
#include <stdbool.h>
#include "sha256.h"

void dfu_transfer_init(void);

uint32_t dfu_transfer_start(
        uint32_t* p_start_addr,
        uint32_t* p_bank_addr,
        uint32_t size,
        bool final_transfer);

uint32_t dfu_transfer_data(uint32_t p_addr, uint8_t* p_data, uint16_t length);

bool dfu_transfer_has_entry(uint32_t* p_addr, uint8_t* p_out_buffer, uint16_t len);

bool dfu_transfer_get_oldest_missing_entry(
        uint32_t* p_start_addr,
        uint32_t** pp_entry,
        uint32_t* p_len);

uint32_t dfu_transfer_sha256(sha256_context_t* p_hash_context);

void dfu_transfer_end(void);

void dfu_transfer_flash_write_complete(uint8_t* p_write_src);


#endif /* DFU_TRANSFER_MESH_H__ */
