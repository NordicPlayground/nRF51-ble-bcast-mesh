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

#ifndef _HANDLE_STORAGE_H__
#define _HANDLE_STORAGE_H__

#include <stdint.h>
#include <stdbool.h>
#include "timer.h"
#include "mesh_packet.h"


#define MESH_VALUE_LOLLIPOP_LIMIT       (200)

typedef struct
{
    uint16_t version;
    mesh_packet_t* p_packet;
} handle_info_t;

typedef enum
{
    HANDLE_FLAG_PERSISTENT,
    HANDLE_FLAG_TX_EVENT,
    HANDLE_FLAG_DISABLED,
    HANDLE_FLAG__MAX
} handle_flag_t;

uint32_t handle_storage_init(uint32_t min_interval_us);

uint32_t handle_storage_min_interval_set(uint32_t min_interval_us);

/**
* Get version and packet pointer of the given handle. The packet is returned
*   with a reference, which must be freed when the packet goes out of scope.
*/
uint32_t handle_storage_info_get(uint16_t handle, handle_info_t* p_info);

/** MUST BE CALLED FROM EVENT HANDLER CONTEXT */
uint32_t handle_storage_info_set(uint16_t handle, handle_info_t* p_info);

uint32_t handle_storage_local_packet_push(mesh_packet_t* p_packet);

/** MUST BE CALLED FROM EVENT HANDLER CONTEXT */
uint32_t handle_storage_flag_set(uint16_t handle, handle_flag_t flag, bool value);

uint32_t handle_storage_flag_set_async(uint16_t handle, handle_flag_t flag, bool value);

uint32_t handle_storage_flag_get(uint16_t handle, handle_flag_t flag, bool* p_value);

uint32_t handle_storage_rx_consistent(uint16_t handle, uint32_t timestamp);

uint32_t handle_storage_rx_inconsistent(uint16_t handle, uint32_t timestamp);

uint32_t handle_storage_next_timeout_get(bool* p_found_value);

/**
* Get a list of packets ready for transmit at the timestamp provided. Each
*   packet is returned with a reference, which must be freed when the packet
*   goes out of scope.
*
* @param[in] time_now Timestamp to compare TX-timeouts against.
* @param[out] pp_tx_packets An array of packet pointers to be filled by the
*   function.
* @param[in,out] p_count The maximum number of elements the given array can
*   hold. When returned, the argument contains the number of packets filled
*   into the array by the function.
*/
uint32_t handle_storage_tx_packets_get(uint32_t time_now, mesh_packet_t** pp_packets, uint32_t* p_count);

/**
* Report a successful transmit on the given handle at the given timestamp.
*/
uint32_t handle_storage_transmitted(uint16_t handle, uint32_t timestamp);


#endif /* _HANDLE_STORAGE_H__ */
