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

#ifndef _MESH_SRV_H__
#define _MESH_SRV_H__

#include "trickle.h"
#include "rbc_mesh.h"
#include "ble.h"
#include <stdint.h>
#include <stdbool.h>

/**
* @file Module handling all Softdevice GATT server related functionality.
*/

#define MESH_SRV_UUID                   (0xFEE4) /* Mesh service UUID (16bit) */
#define MESH_MD_CHAR_UUID               (0x0004) /* Mesh metadata characteristic UUID (128bit) */
#define MESH_VALUE_CHAR_UUID            (0x0005) /* Mesh value characteristic UUID (128bit) */

#define MESH_MD_CHAR_LEN                (9) /* Total length of Mesh metadata characteristic data */
#define MESH_MD_CHAR_AA_OFFSET          (0) /* Metadata characteristic Access Address offset */
#define MESH_MD_CHAR_ADV_INT_OFFSET     (4) /* Metadata characteristic Advertisement interval offset */
#define MESH_MD_CHAR_CH_OFFSET          (8) /* Metadata characteristic channel offset */

#define CONN_HANDLE_INVALID             (0xFFFF)

/**
* @brief Global mesh metadata characteristic type
*/
typedef struct
{
    uint32_t mesh_access_addr; /* operating access address */
    uint32_t mesh_interval_min_ms; /* minimum tx interval */
    uint8_t mesh_channel; /* Mesh channel */
} mesh_metadata_char_t;

uint32_t mesh_gatt_init(uint32_t access_address, uint8_t channel, uint32_t interval_min_ms);

uint32_t mesh_gatt_value_set(rbc_mesh_value_handle_t handle, uint8_t* data, uint8_t length);

void mesh_gatt_sd_ble_event_handle(ble_evt_t* p_ble_evt);

#endif /* _MESH_SRV_H__ */

