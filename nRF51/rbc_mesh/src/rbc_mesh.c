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

#include "rbc_mesh.h"
#include "rbc_mesh_common.h"
#include "timeslot.h"
#include "timer_scheduler.h"
#include "event_handler.h"
#include "version_handler.h"
#include "transport_control.h"
#include "mesh_packet.h"
#include "mesh_gatt.h"
#include "dfu_app.h"
#include "fifo.h"

#include "app_error.h"
#include "nrf_sdm.h"

#include <string.h>

/*****************************************************************************
* Static globals
*****************************************************************************/
static rbc_mesh_state_t m_mesh_state;
static uint32_t         m_access_addr;
static uint8_t          m_channel;
static uint32_t         m_interval_min_ms;
static fifo_t           m_rbc_event_fifo;
static rbc_mesh_event_t m_rbc_event_buffer[RBC_MESH_APP_EVENT_QUEUE_LENGTH];

/*****************************************************************************
* Static Functions
*****************************************************************************/
#if defined(WITH_ACK_MASTER) 
static uint32_t top_queue_counter[4] __attribute__((at(0x20002F98)))={0};
static uint32_t top_queue_drop __attribute__((at(0x20002FA8))) =0;
#endif

#if defined (WITHOUT_ACK_MASTER)
static uint32_t top_queue_counter[4] __attribute__((at(0x200026B4))) ={0};
static uint32_t top_queue_drop __attribute__((at(0x200026C4))) =0;
#endif

#if defined(WITH_ACK_SLAVE)
static uint32_t top_queue_counter[4] __attribute__((at(0x200026A0)))  ={0};
static uint32_t top_queue_drop __attribute__((at(0x200026B0)))  =0;
#endif

#if defined(WITHOUT_ACK_SLAVE)
static uint32_t top_queue_counter[4] __attribute__((at(0x200026A0)))  ={0};
static uint32_t top_queue_drop __attribute__((at(0x200026B0))) =0;
#endif
/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t rbc_mesh_init(rbc_mesh_init_params_t init_params)
{
    uint8_t sd_is_enabled = 0;
    sd_softdevice_is_enabled(&sd_is_enabled);

    if (!sd_is_enabled)
    {
        return NRF_ERROR_SOFTDEVICE_NOT_ENABLED;
    }

    if (m_mesh_state != MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (init_params.interval_min_ms < RBC_MESH_INTERVAL_MIN_MIN_MS ||
        init_params.interval_min_ms > RBC_MESH_INTERVAL_MIN_MAX_MS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    timer_sch_init();
    event_handler_init();
    mesh_packet_init();
    tc_init(init_params.access_addr, init_params.channel);


    uint32_t error_code;
    error_code = vh_init(init_params.interval_min_ms * 1000, /* ms -> us */
                         init_params.access_addr,
                         init_params.channel, 
                         init_params.tx_power);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    ble_enable_params_t ble_enable;
    memset(&ble_enable, 0, sizeof(ble_enable));
    ble_enable.gatts_enable_params.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    ble_enable.gatts_enable_params.service_changed = 0;
    
#if(NORDIC_SDK_VERSION >= 11)
    ble_enable.gap_enable_params.periph_conn_count = 1;
    uint32_t ram_base = RAM_R1_BASE;
    error_code = sd_ble_enable(&ble_enable, &ram_base);
#else
    error_code = sd_ble_enable(&ble_enable);
#endif
    if (error_code != NRF_SUCCESS &&
        error_code != NRF_ERROR_INVALID_STATE)
    {
        return error_code;
    }

    error_code = mesh_gatt_init(init_params.access_addr, init_params.channel, init_params.interval_min_ms);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    timeslot_init(init_params.lfclksrc);

    m_access_addr = init_params.access_addr;
    m_channel = init_params.channel;
    m_interval_min_ms = init_params.interval_min_ms;

    m_mesh_state = MESH_STATE_RUNNING;

    m_rbc_event_fifo.array_len = RBC_MESH_APP_EVENT_QUEUE_LENGTH;
    m_rbc_event_fifo.elem_array = m_rbc_event_buffer;
    m_rbc_event_fifo.elem_size = sizeof(rbc_mesh_event_t);
    m_rbc_event_fifo.memcpy_fptr = NULL;
    fifo_init(&m_rbc_event_fifo);
    timeslot_resume();

#ifdef MESH_DFU
    return dfu_init();
#else
    return NRF_SUCCESS;
#endif
}

rbc_mesh_state_t rbc_mesh_state_get(void)
{
    return m_mesh_state;
}

uint32_t rbc_mesh_start(void)
{
    if (m_mesh_state != MESH_STATE_STOPPED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    timeslot_resume();

    m_mesh_state = MESH_STATE_RUNNING;

    return NRF_SUCCESS;
}

uint32_t rbc_mesh_stop(void)
{
    if (m_mesh_state != MESH_STATE_RUNNING)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    timeslot_stop();

    m_mesh_state = MESH_STATE_STOPPED;

    return NRF_SUCCESS;
}

uint32_t rbc_mesh_value_enable(rbc_mesh_value_handle_t handle)
{
    if (handle > RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    return vh_value_enable(handle);
}

uint32_t rbc_mesh_value_disable(rbc_mesh_value_handle_t handle)
{
    if (handle > RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    return vh_value_disable(handle);
}

uint32_t rbc_mesh_persistence_set(rbc_mesh_value_handle_t handle, bool persistent)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (handle > RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    return vh_value_persistence_set(handle, persistent);
}

uint32_t rbc_mesh_tx_event_set(rbc_mesh_value_handle_t handle, bool do_tx_event)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (handle > RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    return vh_tx_event_set(handle, do_tx_event);
}

/****** Getters and setters ******/

uint32_t rbc_mesh_value_set(rbc_mesh_value_handle_t handle, uint8_t* data, uint16_t len)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (handle > RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    /* no critical errors if this call fails, ignore return */
    mesh_gatt_value_set(handle, data, len);

    return vh_local_update(handle, data, len);
}

uint32_t rbc_mesh_value_get(rbc_mesh_value_handle_t handle, uint8_t* data, uint16_t* len)
{
    if (handle > RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    return vh_value_get(handle, data, len);
}

uint32_t rbc_mesh_access_address_get(uint32_t* access_address)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    *access_address = m_access_addr;

    return NRF_SUCCESS;
}

uint32_t rbc_mesh_channel_get(uint8_t* ch)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    *ch = m_channel;

    return NRF_SUCCESS;
}

uint32_t rbc_mesh_interval_min_ms_get(uint32_t* interval_min_ms)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    *interval_min_ms = m_interval_min_ms;

    return NRF_SUCCESS;
}

uint32_t rbc_mesh_persistence_get(rbc_mesh_value_handle_t handle, bool* is_persistent)
{
    if (handle > RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    return vh_value_persistence_get(handle, is_persistent);
}

uint32_t rbc_mesh_tx_event_flag_get(rbc_mesh_value_handle_t handle, bool* is_doing_tx_event)
{
    if (handle > RBC_MESH_APP_MAX_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    return vh_tx_event_flag_get(handle, is_doing_tx_event);
}

void rbc_mesh_tx_power_set(rbc_mesh_txpower_t tx_power)
{
    vh_tx_power_set(tx_power);
}


void rbc_mesh_ble_evt_handler(ble_evt_t* p_evt)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return;
    }

    mesh_gatt_sd_ble_event_handle(p_evt);
}

void rbc_mesh_sd_evt_handler(uint32_t sd_evt)
{
    timeslot_sd_event_handler(sd_evt);
}

/** Internal only function to push mesh events to application queue. */
uint32_t rbc_mesh_event_push(rbc_mesh_event_t* p_event)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_event == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t error_code = fifo_push(&m_rbc_event_fifo, p_event);
    
    #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)|| defined (WITH_ACK_SLAVE)|| defined (WITHOUT_ACK_SLAVE)
    
    
		
		if (error_code != NRF_SUCCESS)
			   top_queue_drop++;
		
		 switch (p_event->type)
         {
            case RBC_MESH_EVENT_TYPE_NEW_VAL:
					top_queue_counter[0]++;
				    break;
						
            case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
					top_queue_counter[1]++;
					break;
            case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
					top_queue_counter[2]++;
				    break;
			case RBC_MESH_EVENT_TYPE_TX:
					top_queue_counter[3]++;
					break;
						
			default:
					break;
		}
         
      #endif
        
    

    if (error_code == NRF_SUCCESS && p_event->params.rx.p_data != NULL)
    {
        switch (p_event->type)
        {
            case RBC_MESH_EVENT_TYPE_NEW_VAL:
            case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
                if (p_event->params.rx.p_data)
                {
                    mesh_packet_ref_count_inc((mesh_packet_t*) p_event->params.rx.p_data); /* will be aligned by packet manager */
                }
                break;
            case RBC_MESH_EVENT_TYPE_TX:
                if (p_event->params.tx.p_data)
                {
                    mesh_packet_ref_count_inc((mesh_packet_t*) p_event->params.tx.p_data); /* will be aligned by packet manager */
                }
                break;
            default:
                break;
        }
    }
    return error_code;
}

uint32_t rbc_mesh_event_get(rbc_mesh_event_t* p_evt)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (fifo_pop(&m_rbc_event_fifo, p_evt) != NRF_SUCCESS)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t rbc_mesh_event_peek(rbc_mesh_event_t* p_evt)
{
    if (m_mesh_state == MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_evt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (fifo_peek(&m_rbc_event_fifo, p_evt) != NRF_SUCCESS)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

void rbc_mesh_event_release(rbc_mesh_event_t* p_evt)
{
    switch (p_evt->type)
    {
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
            if (p_evt->params.rx.p_data != NULL)
            {
                mesh_packet_ref_count_dec((mesh_packet_t*) p_evt->params.rx.p_data);
            }
            break;
        case RBC_MESH_EVENT_TYPE_TX:
            if (p_evt->params.tx.p_data != NULL)
            {
                mesh_packet_ref_count_dec((mesh_packet_t*) p_evt->params.tx.p_data);
            }
            break;

        default:
            break;
    }
}

void rbc_mesh_packet_peek_cb_set(rbc_mesh_packet_peek_cb_t packet_peek_cb)
{
    tc_packet_peek_cb_set(packet_peek_cb);
}

