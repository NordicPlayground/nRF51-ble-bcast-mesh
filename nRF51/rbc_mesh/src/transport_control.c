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

#include "transport_control.h"

#include "radio_control.h"
#include "timer_control.h"
#include "mesh_srv.h"
#include "rbc_mesh.h"
#include "event_handler.h"
#include "timeslot_handler.h"
#include "rbc_mesh_common.h"
#include "app_error.h"
#include "ble_gap.h"
#include "nrf_soc.h"
#include <string.h>
#include <stdlib.h>

/**@file
*
* @brief Controller for radio_control, timer_control and timeslot_handler.
*   Acts as an abstraction of the lower transport to higher layer modules
*/

#define PACKET_TYPE_LEN             (1)
#define PACKET_LENGTH_LEN           (1)
#define PACKET_ADDR_LEN             (BLE_GAP_ADDR_LEN)

#define PACKET_TYPE_POS             (0)
#define PACKET_LENGTH_POS           (1)
#define PACKET_PADDING_POS          (2)
#define PACKET_ADDR_POS             (3)
#define PACKET_DATA_POS             (PACKET_ADDR_POS + PACKET_ADDR_LEN)


#define PACKET_TYPE_ADV_NONCONN     (0x02)

#define PACKET_TYPE_MASK            (0x0F)
#define PACKET_LENGTH_MASK          (0x3F)
#define PACKET_ADDR_TYPE_MASK       (0x40)

#define PACKET_DATA_MAX_LEN         (28)
#define PACKET_MAX_CHAIN_LEN        (1) /**@TODO: May be increased when RX
                                        callback packet chain handling is implemented.*/

/* minimum time to be left in the timeslot for there to be any point in ordering the radio */
#define RADIO_SAFETY_TIMING_US      (500)


static uint8_t tx_data[(PACKET_DATA_MAX_LEN + PACKET_DATA_POS) * PACKET_MAX_CHAIN_LEN];
static uint64_t global_time = 0;
static uint8_t step_timer_index = 0xFF;

static void search_callback(uint8_t* data);
static void trickle_step_callback(void);

/*****************************************************************************
* Static functions
*****************************************************************************/

/**
* @brief Order the radio_control module to do a RX, and report back to
*   search_callback().
*/
static void order_search(void)
{
    radio_event_t search_event;
    search_event.access_address = 1; /* RX: treat as bitfield */
    search_event.callback.rx = search_callback;
    rbc_mesh_channel_get(&search_event.channel);
    search_event.event_type = RADIO_EVENT_TYPE_RX;
    search_event.start_time = 0;

    radio_order(&search_event);
}

/**
* @brief Convert a data array into a packet object. Pulls out address, payload,
*   CRC and length.
*/
static inline void packet_create_from_data(uint8_t* data, packet_t* packet)
{
    /* advertisement package */

    packet->length = (data[PACKET_LENGTH_POS] & PACKET_LENGTH_MASK) - PACKET_ADDR_LEN;
    memcpy(packet->data, &data[PACKET_DATA_POS], packet->length);
    memcpy(packet->sender.addr, &data[PACKET_ADDR_POS], PACKET_ADDR_LEN);

    /* addr type */
    bool addr_is_random = (data[PACKET_TYPE_POS] & PACKET_ADDR_TYPE_MASK);

    if (addr_is_random)
    {
        bool is_static = ((packet->sender.addr[5] & 0xC0) == 0xC0);
        if (is_static)
        {
            packet->sender.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
        }
        else
        {
            bool is_resolvable = ((packet->sender.addr[5] & 0xC0) == 0x40);
            packet->sender.addr_type = (is_resolvable?
                BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE :
                BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE);
        }
    }
    else
    {
        packet->sender.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    }
}

/**
* @brief check whether a packet is a valid data packet, eligible for processing
*/
static inline bool packet_is_data_packet(uint8_t* data)
{
    return ((data[PACKET_TYPE_POS] & PACKET_TYPE_MASK)
        == PACKET_TYPE_ADV_NONCONN);
}

/**
* @brief Handle incoming packets
*/
static void search_callback(uint8_t* data)
{
    SET_PIN(PIN_RX);

    uint32_t checksum = (NRF_RADIO->RXCRC & 0x00FFFFFF);

    /* check if timeslot is about to end */
    uint64_t radio_time_left = timeslot_get_remaining_time();

    if (radio_time_left > RADIO_SAFETY_TIMING_US)
    {
        /* setup next RX */
        order_search();
    }

    CLEAR_PIN(PIN_RX);

    if (data == NULL || !packet_is_data_packet(data))
    {
        return;
    }


    async_event_t async_evt;
    async_evt.type = EVENT_TYPE_PACKET;
    packet_create_from_data(data, &async_evt.callback.packet);
    async_evt.callback.packet.rx_crc = checksum;
    event_handler_push(&async_evt);


    /** @TODO: add packet chain handling */
}

/**
* @brief Handle trickle timing events
*/
static void trickle_step_callback(void)
{
    TICK_PIN(6);
    /* check if timeslot is about to end */
    if (timeslot_get_remaining_time() < RADIO_SAFETY_TIMING_US)
        return;

    uint64_t time_now = global_time + timer_get_timestamp();
    trickle_time_update(time_now);

    packet_t packet;
    bool has_anything_to_send = false;

    mesh_srv_packet_assemble(&packet, PACKET_DATA_MAX_LEN * PACKET_MAX_CHAIN_LEN,
        &has_anything_to_send);

    if (has_anything_to_send)
    {
        TICK_PIN(PIN_MESH_TX);
        radio_disable();

        uint8_t packet_and_addr_type = PACKET_TYPE_ADV_NONCONN |
            ((packet.sender.addr_type == BLE_GAP_ADDR_TYPE_PUBLIC)?
            0 :
            PACKET_ADDR_TYPE_MASK);

        uint8_t* temp_data_ptr = &packet.data[0];
        uint8_t* tx_data_ptr = &tx_data[0];
        tx_data_ptr[PACKET_TYPE_POS] = packet_and_addr_type;

        /* Code structured for packet chaining, although this is yet
         to be implemented. */
        do
        {
            uint8_t min_len = ((packet.length > PACKET_DATA_MAX_LEN)?
                PACKET_DATA_MAX_LEN :
                packet.length);

            tx_data_ptr[PACKET_PADDING_POS] = 0;
            tx_data_ptr[PACKET_LENGTH_POS] = (min_len + PACKET_ADDR_LEN);
            tx_data_ptr[PACKET_TYPE_POS] = packet_and_addr_type;

            memcpy(&tx_data_ptr[PACKET_ADDR_POS], packet.sender.addr, PACKET_ADDR_LEN);
            memcpy(&tx_data_ptr[PACKET_DATA_POS], &temp_data_ptr[0], min_len);

            radio_event_t tx_event;
            tx_event.access_address = 0;
            rbc_mesh_channel_get(&tx_event.channel);
            tx_event.event_type = RADIO_EVENT_TYPE_TX;
            tx_event.packet_ptr = &tx_data_ptr[0];
            tx_event.start_time = 0;
            tx_event.callback.tx = NULL;

            radio_order(&tx_event);
            TICK_PIN(0);
        } while (0);

        order_search(); /* search for the rest of the timeslot */
    }

    /* order next processing */
    uint64_t next_time;
    uint64_t end_time = timeslot_get_end_time();
    uint32_t error_code = mesh_srv_get_next_processing_time(&next_time);

    if (error_code == NRF_SUCCESS && next_time < global_time + end_time)
    {
        timer_abort(step_timer_index);
        step_timer_index = timer_order_cb(next_time - global_time, trickle_step_callback);
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void transport_control_timeslot_begin(uint64_t global_timer_value)
{
    uint32_t aa;
    uint32_t error_code = rbc_mesh_access_address_get(&aa);
    APP_ERROR_CHECK(error_code);

    radio_init(aa);

    step_timer_index = 0xFF;

    global_time = global_timer_value;

    order_search();
    transport_control_step();

}

void transport_control_step(void)
{
    uint64_t next_time;
    uint64_t time_now = timer_get_timestamp() + global_time;
    trickle_time_update(time_now);
    uint32_t error_code = mesh_srv_get_next_processing_time(&next_time);
    if (error_code != NRF_SUCCESS)
    {
        return;
    }

    if (next_time < time_now)
    {
        async_event_t async_evt;
        async_evt.callback.generic = trickle_step_callback;
        async_evt.type = EVENT_TYPE_GENERIC;
        event_handler_push(&async_evt);
    }
    else
    {
        if (next_time < global_time + timeslot_get_end_time())
        {
            timer_abort(step_timer_index);
            step_timer_index = timer_order_cb(next_time - global_time, trickle_step_callback);
        }
    }
}

