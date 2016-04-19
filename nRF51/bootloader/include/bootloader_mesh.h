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
#ifndef BOOTLOADER_H__
#define BOOTLOADER_H__

#include <stdint.h>
#include <stdbool.h>
#include "dfu_types_mesh.h"
#include "mesh_packet.h"

#define SD_VERSION_INVALID                  (0x0000)
#define APP_VERSION_INVALID                 (0x00000000)

typedef enum
{
    BL_STATE_FIND_FWID,
    BL_STATE_DFU_REQ,
    BL_STATE_DFU_READY,
    BL_STATE_DFU_TARGET,
    BL_STATE_VALIDATE,
    BL_STATE_RELAY_CANDIDATE,
    BL_STATE_RELAY
} bl_state_t;

typedef enum
{
    BL_END_SUCCESS,
    BL_END_FWID_VALID,
    BL_END_ERROR_PACKET_LOSS,
    BL_END_ERROR_UNAUTHORIZED,
    BL_END_ERROR_NO_START,
    BL_END_ERROR_TIMEOUT,
    BL_END_ERROR_NO_MEM,
    BL_END_ERROR_INVALID_PERSISTENT_STORAGE,
    BL_END_ERROR_SEGMENT_VIOLATION,
} bl_end_t;

void bootloader_init(void);
void bootloader_start(void);
uint32_t bootloader_rx(dfu_packet_t* p_packet, uint16_t length, bool from_serial);
void bootloader_abort(bl_end_t end_reason);
void bootloader_rtc_irq_handler(void);
void bootloader_packet_set_local_fields(mesh_packet_t* p_packet, uint8_t dfu_packet_len);

#endif /* BOOTLOADER_H__ */
