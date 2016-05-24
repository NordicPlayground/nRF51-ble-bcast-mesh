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

#ifndef _MESH_PACKET_H__
#define _MESH_PACKET_H__
#include <stdint.h>
#include <stdbool.h>
#include "rbc_mesh.h"
#include "toolchain.h"
#include "dfu_types_mesh.h"


#define MESH_UUID                           (0xFEE4)
#define MESH_ADV_DATA_TYPE                  (0x16)
#define BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH   (31)

#define MESH_PACKET_BLE_OVERHEAD            (BLE_GAP_ADDR_LEN)                                                      /* overhead before advertisement payload */
#define MESH_PACKET_ADV_OVERHEAD            (1 /* adv_type */ + 2 /* UUID */ + 2 /* handle */ + 2 /* version */)    /* overhead inside adv data */
#define MESH_PACKET_OVERHEAD                (MESH_PACKET_BLE_OVERHEAD + 1 + MESH_PACKET_ADV_OVERHEAD)               /* mesh packet total overhead */
/******************************************************************************
* Public typedefs
******************************************************************************/
typedef __packed_armcc struct
{
    uint8_t type : 4;
    uint8_t _rfu1 : 2;
    uint8_t addr_type : 1;
    uint8_t _rfu2 : 1;
    uint8_t length;
    uint8_t _rfu3;
} __packed_gcc ble_packet_header_t;

typedef __packed_armcc struct
{
    uint8_t                 adv_data_length;
    uint8_t                 adv_data_type;
    uint8_t                 data[];
} __packed_gcc ble_ad_t;

typedef __packed_armcc struct
{
    uint8_t                 adv_data_length;
    uint8_t                 adv_data_type;
    uint16_t                mesh_uuid;
    rbc_mesh_value_handle_t handle;
    uint16_t                version;
    uint8_t                 data[RBC_MESH_VALUE_MAX_LEN];
} __packed_gcc mesh_adv_data_t;

typedef __packed_armcc struct
{
    uint8_t                 adv_data_length;
    uint8_t                 adv_data_type;
    uint16_t                mesh_uuid;
    dfu_packet_t            dfu_packet;
} __packed_gcc mesh_dfu_adv_data_t;

typedef __packed_armcc struct
{
    ble_packet_header_t header;
    uint8_t addr[6];
    uint8_t payload[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH];
} __packed_gcc mesh_packet_t;

/******************************************************************************
* Interface functions
******************************************************************************/
void mesh_packet_init(void);

void mesh_packet_on_ts_begin(void);

bool mesh_packet_acquire(mesh_packet_t** pp_packet);

/** Get pointer to start of packet which p_buf_pointer is pointing into */
mesh_packet_t* mesh_packet_get_aligned(void* p_buf_pointer);

bool mesh_packet_ref_count_inc(mesh_packet_t* p_packet);

bool mesh_packet_ref_count_dec(mesh_packet_t* p_packet);

uint8_t mesh_packet_ref_count_get(mesh_packet_t* p_packet);

uint32_t mesh_packet_set_local_addr(mesh_packet_t* p_packet);

uint32_t mesh_packet_build(mesh_packet_t* p_packet,
        rbc_mesh_value_handle_t handle,
        uint16_t version,
        uint8_t* data,
        uint8_t length);

uint32_t mesh_packet_adv_data_sanitize(mesh_packet_t* p_packet);

mesh_adv_data_t* mesh_packet_adv_data_get(mesh_packet_t* p_packet);

rbc_mesh_value_handle_t mesh_packet_handle_get(mesh_packet_t* p_packet);

bool mesh_packet_has_additional_data(mesh_packet_t* p_packet);

/** Fill address field with local addr, and sanitize adv-data */
void mesh_packet_take_ownership(mesh_packet_t* p_packet);

#endif /* _MESH_PACKET_H__ */

