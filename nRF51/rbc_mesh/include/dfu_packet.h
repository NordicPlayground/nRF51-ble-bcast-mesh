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

#ifndef DFU_PACKET_H__
#define DFU_PACKET_H__

#define DFU_PACKET_LEN_OVERHEAD     (2)

#define DFU_PACKET_LEN_FWID         (DFU_PACKET_LEN_OVERHEAD + sizeof(fwid_t))
#define DFU_PACKET_LEN_STATE_SD     (DFU_PACKET_LEN_OVERHEAD + sizeof(dfu_packet_state_t) - sizeof(fwid_union_t) + DFU_FWID_LEN_SD)
#define DFU_PACKET_LEN_STATE_BL     (DFU_PACKET_LEN_OVERHEAD + sizeof(dfu_packet_state_t) - sizeof(fwid_union_t) + DFU_FWID_LEN_BL)
#define DFU_PACKET_LEN_STATE_APP    (DFU_PACKET_LEN_OVERHEAD + sizeof(dfu_packet_state_t) - sizeof(fwid_union_t) + DFU_FWID_LEN_APP)
#define DFU_PACKET_LEN_START        (DFU_PACKET_LEN_OVERHEAD + sizeof(dfu_packet_start_t))
#define DFU_PACKET_LEN_DATA         (DFU_PACKET_LEN_OVERHEAD + sizeof(dfu_packet_data_t))
#define DFU_PACKET_LEN_DATA_REQ     (DFU_PACKET_LEN_OVERHEAD + sizeof(dfu_packet_data_req_t))
#define DFU_PACKET_LEN_DATA_RSP     (DFU_PACKET_LEN_OVERHEAD + sizeof(dfu_packet_data_rsp_t))

#define DFU_PACKET_ADV_OVERHEAD     (1 /* adv_type */ + 2 /* UUID */) /* overhead inside adv data */
#define DFU_PACKET_OVERHEAD         (MESH_PACKET_BLE_OVERHEAD + 1 + DFU_PACKET_ADV_OVERHEAD) /* dfu packet total overhead */

/** Packet types for DFU. */
typedef enum
{
    DFU_PACKET_TYPE_DATA_RSP    = 0xFFFA,
    DFU_PACKET_TYPE_DATA_REQ    = 0xFFFB,
    DFU_PACKET_TYPE_DATA        = 0xFFFC,
    DFU_PACKET_TYPE_STATE       = 0xFFFD,
    DFU_PACKET_TYPE_FWID        = 0xFFFE,
} dfu_packet_type_t;

/** DFU state packet payload */
typedef struct __attribute((packed))
{
    uint8_t dfu_type    : 4;
    uint8_t _rfu1       : 4;
    uint8_t authority   : 3;
    uint8_t flood       : 1;
    uint8_t relay_node  : 1;
    uint8_t _rfu2       : 3;
    uint32_t transaction_id;
    fwid_union_t fwid;
} dfu_packet_state_t;

/** DFU start packet payload */
typedef struct __attribute((packed))
{
    uint16_t segment;
    uint32_t transaction_id;
    uint32_t start_address;
    uint32_t length; /* in words */
    uint16_t signature_length;
    uint8_t diff        : 1;
    uint8_t single_bank : 1;
    uint8_t first       : 1;
    uint8_t last        : 1;
    uint8_t _rfu        : 4;
} dfu_packet_start_t;

/** DFU data packet payload */
typedef struct __attribute((packed))
{
    uint16_t segment;
    uint32_t transaction_id;
    uint8_t data[SEGMENT_LENGTH];
} dfu_packet_data_t;

/** DFU data request packet payload */
typedef struct __attribute((packed))
{
    uint16_t segment;
    uint32_t transaction_id;
} dfu_packet_data_req_t;

/** DFU data response packet payload */
typedef struct __attribute((packed))
{
    uint16_t segment;
    uint32_t transaction_id;
    uint8_t data[SEGMENT_LENGTH];
} dfu_packet_data_rsp_t;

/** DFU radio packet format */
typedef struct __attribute((packed))
{
    uint16_t packet_type;
    union __attribute((packed))
    {
        fwid_t                fwid;
        dfu_packet_state_t    state;
        dfu_packet_start_t    start;
        dfu_packet_data_t     data;
        dfu_packet_data_req_t req_data;
        dfu_packet_data_rsp_t rsp_data;
    } payload;
} dfu_packet_t;

#endif /* DFU_PACKET_H__ */

