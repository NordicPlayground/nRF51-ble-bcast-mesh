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

#include "rbc_mesh.h"
#include "timeslot_handler.h"

#include "app_uart.h"
#include "app_error.h"
#include "dfu.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "records.h"
#include "timeslot_handler.h"
#include "version_handler.h"
#include "rbc_mesh.h"
#include "event_handler.h"
#include "transport_control.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define MISSING_POOL_SIZE   (32)
#define HANDLE_COUNT        (18)
#define DATA_HANDLE_START   (3)
#define DATA_HANDLE_STOP    (HANDLE_COUNT)
#define RECOVERY_REQ_HANDLE (1)
#define RECOVERY_RSP_HANDLE (2)

typedef union
{
    struct 
    {
        uint16_t short_addr;
        uint8_t data[DFU_RECORD_SIZE];
    } data;
    
    struct 
    {
        uint16_t seq_num;
    } req;
    
    dfu_record_t rsp;
    
} dfu_packet_t;


static dfu_bootloader_info_t g_bl_info;

static void get_bootloader_info(dfu_bootloader_info_t* p_info)
{
    memcpy(p_info, 
            (dfu_bootloader_info_t*) BOOTLOADER_INFO_ADDRESS, 
            sizeof(dfu_bootloader_info_t));
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    __BKPT(0);
}

void rbc_mesh_event_handler(rbc_mesh_event_t* p_evt)
{
    if (p_evt->event_type == RBC_MESH_EVENT_TYPE_NEW_VAL ||
        p_evt->event_type == RBC_MESH_EVENT_TYPE_UPDATE_VAL || 
        p_evt->event_type == RBC_MESH_EVENT_TYPE_CONFLICTING_VAL
    )
    {
        dfu_packet_t* p_packet = (dfu_packet_t*) p_evt->data;
        
        if (p_evt->value_handle >= DATA_HANDLE_START && 
            p_evt->value_handle <= DATA_HANDLE_STOP)
        {
            /* regular dfu packet */
            dfu_record_t record;
            if (p_evt->data_len >= 8)
            {
                memcpy(&record, &p_evt->data[0], sizeof(dfu_record_t));
                records_record_add(&record, false);
            }
            else 
            {
                APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
            }
            
            if (p_evt->version_delta > 1)
            {
                /* we've missed a packet */
                records_missing_report(record.seq_num - HANDLE_COUNT);
            }
        }
        else if (p_evt->value_handle == RECOVERY_REQ_HANDLE)
        {
            /* someone is requesting a recovery */
            dfu_packet_t rsp_packet;
            if (records_record_get(p_evt->data[0], &rsp_packet.rsp))
            {
                mesh_srv_char_val_set(RECOVERY_RSP_HANDLE, (uint8_t*) &rsp_packet, sizeof(rsp_packet));
                vh_local_update(RECOVERY_RSP_HANDLE);
            }
        }
        else if (p_evt->value_handle == RECOVERY_RSP_HANDLE)
        {
            if (records_is_missing(p_packet->rsp.seq_num))
            {
                records_record_add(&p_packet->rsp, true);
            }
        }
        else
        {
            APP_ERROR_CHECK(NRF_ERROR_INVALID_ADDR);
        }
    }
}

static void init_leds(void)
{
    nrf_gpio_range_cfg_output(0, 32);
    NRF_GPIO->OUTCLR = 0xFFFFFFFF;
}

int main(void)
{
    init_leds();
    
    get_bootloader_info(&g_bl_info);
    
    tc_init(0xA541A68F, 38);
    event_handler_init();
    vh_init(HANDLE_COUNT, 100000);
    timeslot_handler_init();
    records_init(MISSING_POOL_SIZE);
    
    while (1)
    {
        __WFE();
    }
}
