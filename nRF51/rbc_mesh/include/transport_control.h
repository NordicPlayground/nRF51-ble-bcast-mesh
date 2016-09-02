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

#ifndef _TRANSPORT_CONTROL_H__
#define _TRANSPORT_CONTROL_H__
#include <stdint.h>
#include "mesh_packet.h"
#include "ble.h"
#ifdef NRF51
#include "nrf51.h"
#else
#include "nrf.h"
#endif
#include "rbc_mesh.h"

/**
* @file This module takes care of all lower level packet processing and
*   schedules the radio for transmission. Acts as the link between the radio
*   and the mesh service.
*/

/**
* Transport configuration for outgoing packets.
*/
typedef struct
{
    bool                alt_access_address; /**< Whether to use the alternate access address. */
    uint8_t             first_channel;      /**< Channel offset in the channel map. */
    uint8_t             channel_map;        /**< Bitmap for channels to transmit on. */
    rbc_mesh_txpower_t  tx_power;           /**< Transmit power. */
} tc_tx_config_t;


/** @brief Function pointer type for packet peek callback. */
typedef void (*packet_peek_cb_t)(mesh_packet_t* p_packet,
                                 uint32_t crc,
                                 uint32_t timestamp,
                                 uint8_t rssi);


void tc_init(uint32_t access_address, uint8_t channel);

void tc_radio_params_set(uint32_t access_address, uint8_t channel);

void tc_on_ts_begin(void);

/**
* @brief: Assemble a packet by getting data from server based on params,
*   and place it on the radio queue.
*
* @params[in] p_packet Pointer to a BLE-packet to send.
* @params[in] p_tx_config TX configuration for the transmission.
*
* @return NRF_SUCCESS The packets was scheduled for transmission on all indicated channels.
* @return NRF_ERROR_NO_MEM One or more packets failed.
*/
uint32_t tc_tx(mesh_packet_t* p_packet, const tc_tx_config_t* p_tx_config);

void tc_packet_handler(uint8_t* data, uint32_t crc, uint32_t timestamp, uint8_t rssi);

/**
* @brief Set packet peek function pointer. Every received packet will be
*   passed to the peek function before being processed by the stack -
*   including non-mesh packets. This allows the application to read
*   out parameters like RSSI from nearby devices.
*
* @warning This is considered an advanced feature, and should be used with some
*   care. The packet memory will be invalid after the function is finished, and
*   users should not store any direct pointers to it. Also note that the
*   function is called from APP_LOW priority, which means it takes away from
*   stack-internal processing time. Excessive usage may lead to starvation of
*   internal functionality, and potentially packet drops.
*
* @param[in] packet_peek_cb Function pointer to a packet-peek function.
*/
void tc_packet_peek_cb_set(rbc_mesh_packet_peek_cb_t packet_peek_cb);

#endif /* _TRANSPORT_CONTROL_H__ */
