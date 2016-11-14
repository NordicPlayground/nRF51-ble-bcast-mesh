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

#ifndef PACKET_HANDLER_H__
#define PACKET_HANDLER_H__

#include "mesh_packet.h"

typedef void (*fwid_packet_handler_t)     (dfu_packet_payload_fwid_t* p_fwid_packet, uint32_t length);
typedef void (*state_packet_handler_t)    (dfu_packet_payload_state_t* p_state_packet, uint32_t length);
typedef void (*start_packet_handler_t)    (dfu_packet_payload_start_t* p_start_packet, uint32_t length);
typedef void (*data_packet_handler_t)     (dfu_packet_payload_data_t* p_data_packet, uint32_t length);
typedef void (*req_data_packet_handler_t) (dfu_packet_payload_req_data_t* p_req_data_packet, uint32_t length);
typedef void (*rsp_data_packet_handler_t) (dfu_packet_payload_rsp_data_t* p_rsp_data_packet, uint32_t length);


typedef struct
{
    fwid_packet_handler_t     handler_fwid;
    state_packet_handler_t    handler_state;
    start_packet_handler_t    handler_start;
    data_packet_handler_t     handler_data;
    req_data_packet_handler_t handler_req_data;
    rsp_data_packet_handler_t handler_rsp_data;
} dfu_state_context_t;


#endif /* PACKET_HANDLER_H__ */

