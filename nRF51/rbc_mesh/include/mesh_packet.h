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

#ifndef _MESH_PACKET_H__
#define _MESH_PACKET_H__
#include <stdint.h>
#include "mesh_srv.h"
#include "toolchain.h"

#define MESH_PACKET_POOL_SIZE   (16)
#define MESH_PACKET_OVERHEAD    (10)

/******************************************************************************
* Public typedefs
******************************************************************************/
typedef enum 
{
  BLE_PACKET_TYPE_ADV_IND,
  BLE_PACKET_TYPE_ADV_DIRECT_IND,
  BLE_PACKET_TYPE_ADV_NONCONN_IND,
  BLE_PACKET_TYPE_SCAN_REQ,
  BLE_PACKET_TYPE_SCAN_RSP,
  BLE_PACKET_TYPE_CONN_REQ,
  BLE_PACKET_TYPE_ADV_DISCOVER_IND
} ble_packet_type_t;

typedef __packed_armcc struct
{
  ble_packet_type_t type : 4;
  uint8_t _rfu1 : 2;
  uint8_t addr_type : 1;
  uint8_t _rfu2 : 1;
  uint8_t length;
  uint8_t _rfu3;
} __packed_gcc ble_packet_header_t;

typedef __packed_armcc struct 
{
    ble_packet_header_t header;
    uint8_t addr[6];
    __packed_armcc struct
    {
        uint8_t handle;
        uint16_t version;
        uint8_t data[MAX_VALUE_LENGTH];
    } __packed_gcc payload;
} __packed_gcc mesh_packet_t;
/******************************************************************************
* Interface functions
******************************************************************************/
void mesh_packet_init(void);

bool mesh_packet_acquire(mesh_packet_t** pp_packet);

bool mesh_packet_free(mesh_packet_t* p_packet);

void mesh_packet_on_ts_begin(void);

#endif /* _MESH_PACKET_H__ */

