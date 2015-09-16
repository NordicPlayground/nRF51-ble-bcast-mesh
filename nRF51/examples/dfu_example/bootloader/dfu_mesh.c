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

#include "dfu_mesh.h"
#include "dfu_types_mesh.h"
#include "dfu_types.h"
#include "nrf_flash.h"
#include "nrf_sdm.h"
#include "bootloader_util.h"
#include "app_error.h"
#include "rbc_mesh.h"
#include "records.h"
#include "mesh_srv.h"
#include "version_handler.h"

#include "nrf51.h"
#include <stdbool.h>
#include <string.h>

#define MAX_NUMBER_INTERRUPTS   (32)

#define DATA_HANDLE_START       (3)
#define DATA_HANDLE_STOP        (DFU_HANDLE_COUNT)
#define RECOVERY_REQ_HANDLE     (1)
#define RECOVERY_RSP_HANDLE     (2)

#define TX_COUNT_MARGIN         (3)

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
/*****************************************************************************
* Static globals
*****************************************************************************/
static dfu_bootloader_info_t m_bl_info;
static bool m_in_dfu = false;
static uint8_t m_tx_counts[DATA_HANDLE_STOP - DATA_HANDLE_START + 1];
static uint8_t m_seq_nums[DATA_HANDLE_STOP - DATA_HANDLE_START + 1];
static uint32_t m_next_index = DATA_HANDLE_START;
static dfu_record_t m_next_record;
static bool m_next_record_ready = false;
/*****************************************************************************
* Static Functions
*****************************************************************************/
static void get_bootloader_info(dfu_bootloader_info_t* p_info)
{
    memcpy(p_info, 
            (dfu_bootloader_info_t*) BOOTLOADER_INFO_ADDRESS, 
            sizeof(dfu_bootloader_info_t));
}

static uint16_t crc16_compute(const uint8_t * p_data, uint32_t size, const uint16_t * p_crc)
{
    uint32_t i;
    uint16_t crc = (p_crc == NULL) ? 0xffff : *p_crc;

    for (i = 0; i < size; i++)
    {
        crc  = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= p_data[i];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }

    return crc;
}

static bool bank_is_valid(const uint8_t* bank_start, uint32_t size, uint16_t bank_crc)
{
    uint16_t crc = crc16_compute(bank_start, size, NULL);
    return (crc == bank_crc);
}

static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq;

    // Fetch the current interrupt settings.
    interrupt_setting_mask = NVIC->ISER[0];

    // Loop from interrupt 0 for disabling of all interrupts.
    for (irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (1 << irq))
        {
            // The interrupt was enabled, hence disable it.
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

/** @brief Put a record into a trickle value and propagate, seeder device only */
static void seed_record(void)
{
    /* if this fails, it will be picked up in the TX_EVENT */
    if (m_tx_counts[m_next_index] >= TX_COUNT_MARGIN && m_next_record_ready)
    {
        m_tx_counts[m_next_index] = 0;
        mesh_srv_char_val_set(m_next_index, (uint8_t*) &(m_next_record.short_addr), DFU_RECORD_SIZE + 2);
        vh_local_update(m_next_index);
        m_next_record_ready = false;
        
        if (++m_next_index == DATA_HANDLE_STOP)
        {
            m_next_index = DATA_HANDLE_START;
        }
    }
}

/** @brief request next record from serial, seeder device only */
static void request_next_record(void)
{
    
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
                memcpy(&record.data, p_packet->data.data, DFU_RECORD_SIZE);
                record.short_addr = p_packet->data.short_addr;
                m_seq_nums[p_evt->value_handle - DATA_HANDLE_START] +=
                               p_evt->version_delta * (DATA_HANDLE_STOP - DATA_HANDLE_START + 1);
                record.seq_num = m_seq_nums[p_evt->value_handle - DATA_HANDLE_START];
                records_record_add(&record, false);
            }
            else 
            {
                APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
            }
            
            if (p_evt->version_delta > 1)
            {
                /* we've missed a packet */
                records_missing_report(record.seq_num - DFU_HANDLE_COUNT);
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
    else if (p_evt->event_type == RBC_MESH_EVENT_TYPE_TX)
    {
        m_tx_counts[p_evt->value_handle - DATA_HANDLE_START]++;
        
        seed_record(); /* handles checks to avoid  */
    }
}
/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t dfu_transfer_begin(dfu_bootloader_info_t* bl_info)
{
    if (m_in_dfu)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    switch (bl_info->dfu_type)
    {
        case DFU_TYPE_APP:
            bl_info->bank_addr = DFU_APP_BANK_ADDRESS;
            if (bl_info->size > DFU_APP_MAX_SIZE)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            break;
        case DFU_TYPE_BOOTLOADER:
            bl_info->bank_addr = DFU_BL_BANK_ADDRESS;
            if (bl_info->size > DFU_BL_MAX_SIZE)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            break;
        case DFU_TYPE_SOFTDEVICE:
            bl_info->bank_addr = DFU_SD_BANK_ADDRESS;
            if (bl_info->size > DFU_SD_MAX_SIZE)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            break;
    }
    memset(m_tx_counts, 0, sizeof(m_tx_counts));
    /* make ram copy */
    memcpy(&m_bl_info, bl_info, sizeof(dfu_bootloader_info_t));
    
    return NRF_SUCCESS;
}

uint32_t dfu_transfer_data(uint32_t address, uint32_t length, uint8_t* data)
{
    if (!m_in_dfu)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (address < m_bl_info.start_addr ||
        address + length > m_bl_info.start_addr + m_bl_info.size)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    /* must be word aligned */
    if ((address & 0x03) != 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    if (length != DFU_RECORD_SIZE)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    
    m_next_record.seq_num++;
    m_next_record.short_addr = SHORT_ADDRESS(address);
    memcpy(m_next_record.data, data, length);
    m_next_record_ready = true;
    
    seed_record();
    
    return NRF_SUCCESS;
}

uint32_t dfu_end(void)
{
    if (!m_in_dfu)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (m_bl_info.using_crc && !bank_is_valid((uint8_t*) m_bl_info.bank_addr, m_bl_info.size, m_bl_info.image_crc))
    {
        return NRF_ERROR_INVALID_DATA;
    }

    m_in_dfu = false;
    
    if (m_bl_info.dfu_type != DFU_TYPE_BOOTLOADER)
    {
        /** @todo */
    }
    
    return NRF_SUCCESS;
}

