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

#ifndef _VERSION_HANDLER_H__
#define _VERSION_HANDLER_H__
#include "ble_gap.h"
#include <stdint.h>
#include <stdbool.h>

#define MAX_VALUE_COUNT                 (155) /* The highest possible number of values stored in the mesh. */

typedef enum
{
    VH_DATA_STATUS_NEW,
    VH_DATA_STATUS_UPDATED,
    VH_DATA_STATUS_OLD,
    VH_DATA_STATUS_SAME,
    VH_DATA_STATUS_CONFLICTING,
    VH_DATA_STATUS_UNKNOWN
} vh_data_status_t;

uint32_t vh_init(uint8_t handle_count, uint32_t min_interval_us);

vh_data_status_t vh_compare_metadata(uint8_t handle, uint16_t version, uint8_t* incoming_data, uint8_t len);

uint32_t vh_rx_register(vh_data_status_t status, uint8_t handle, uint16_t version, uint64_t timestamp);

vh_data_status_t vh_local_update(uint8_t handle);

uint32_t vh_on_timeslot_begin(void);

uint32_t vh_order_update(uint64_t time_now);

uint32_t vh_tx_report(uint8_t handle, bool do_tx_event);

uint32_t vh_set_gatts_handle(uint8_t value_handle, uint8_t gatts_handle);

uint32_t vh_get_gatts_handle(uint8_t value_handle, uint8_t* gatts_handle);

uint16_t vh_get_version_delta(uint8_t handle, uint16_t version);

uint32_t vh_value_enable(uint8_t handle);

uint32_t vh_value_disable(uint8_t handle);
#endif /* _VERSION_HANDLER_H__ */

