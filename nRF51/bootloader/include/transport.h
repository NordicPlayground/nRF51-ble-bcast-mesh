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
#ifndef TRANSPORT_H__
#define TRANSPORT_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_packet.h"

#define TX_REPEATS_EXPONENTIAL_MAX      (12)
#define TX_REPEATS_INF                  (0xFF)
#define TRANSPORT_TX_SLOTS              (8)

typedef enum
{
    TX_INTERVAL_TYPE_EXPONENTIAL,
    TX_INTERVAL_TYPE_REGULAR,
    TX_INTERVAL_TYPE_REGULAR_SLOW
} tx_interval_type_t;

typedef void(*release_cb_t)(mesh_packet_t* p_packet);
typedef void(*rx_cb_t)(mesh_packet_t* p_packet);

void transport_init(rx_cb_t rx_cb, uint32_t access_addr);
void transport_start(void);
bool transport_tx(mesh_packet_t* p_packet, uint8_t slot, uint8_t repeats, tx_interval_type_t type);
void transport_tx_reset(uint8_t slot);
void transport_tx_skip(uint8_t slot);
void transport_tx_abort(uint8_t slot);
void transport_rtc_irq_handler(void);
void transport_tx_evt_set(uint16_t handle, bool value);
bool transport_tx_evt_get(uint16_t handle);

#endif /* TRANSPORT_H__ */
