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

#include <string.h>
#include "bl_test.h"
#include "bl_if.h"
#include "bootloader.h"
#include "bootloader_info.h"
#include "rtt_log.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

/*****************************************************************************
* Local typedefs
*****************************************************************************/

/*****************************************************************************
* Static globals
*****************************************************************************/

/*****************************************************************************
* Static functions
*****************************************************************************/

/*****************************************************************************
* Interface functions
*****************************************************************************/

void test_fwid_old(void)
{
    __LOG(RTT_CTRL_TEXT_CYAN "TEST: %s\n", __FUNCTION__);
    bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get(BL_INFO_TYPE_VERSION);
    dfu_packet_t dfu_packet;
    dfu_packet.packet_type = DFU_PACKET_TYPE_FWID;
    memcpy(&dfu_packet.payload.fwid, &p_fwid_entry->version, sizeof(fwid_t));

    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.p_dfu_packet = &dfu_packet;
    rx_cmd.params.rx.length = DFU_PACKET_LEN_FWID;
    bootloader_cmd_send(&rx_cmd);
}

void test_fwid_new(void)
{
    __LOG(RTT_CTRL_TEXT_CYAN "TEST: %s\n", __FUNCTION__);
    bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get(BL_INFO_TYPE_VERSION);
    dfu_packet_t dfu_packet;
    dfu_packet.packet_type = DFU_PACKET_TYPE_FWID;
    memcpy(&dfu_packet.payload.fwid, &p_fwid_entry->version, sizeof(fwid_t));
    dfu_packet.payload.fwid.app.app_version++;

    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.p_dfu_packet = &dfu_packet;
    rx_cmd.params.rx.length = DFU_PACKET_LEN_FWID;
    bootloader_cmd_send(&rx_cmd);
}

void test_state_dont_care(void)
{
    __LOG(RTT_CTRL_TEXT_CYAN "TEST: %s\n", __FUNCTION__);
    bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get(BL_INFO_TYPE_VERSION);
    dfu_packet_t dfu_packet;
    memset(&dfu_packet, 0, sizeof(dfu_packet_t));
    dfu_packet.packet_type = DFU_PACKET_TYPE_STATE;
    dfu_packet.payload.state.authority = 1;
    dfu_packet.payload.state.flood = 0;
    dfu_packet.payload.state.transaction_id = 0x12345678;
    dfu_packet.payload.state.dfu_type = DFU_TYPE_APP;
    memcpy(&dfu_packet.payload.state.fwid.app, &p_fwid_entry->version.app, sizeof(app_id_t));

    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.p_dfu_packet = &dfu_packet;
    rx_cmd.params.rx.length = DFU_PACKET_LEN_STATE_APP;
    bootloader_cmd_send(&rx_cmd);
}

void test_state_req(void)
{
    __LOG(RTT_CTRL_TEXT_CYAN "TEST: %s\n", __FUNCTION__);
    bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get(BL_INFO_TYPE_VERSION);
    dfu_packet_t dfu_packet;
    memset(&dfu_packet, 0, sizeof(dfu_packet_t));
    dfu_packet.packet_type = DFU_PACKET_TYPE_STATE;
    dfu_packet.payload.state.authority = 0;
    dfu_packet.payload.state.flood = 0;
    dfu_packet.payload.state.transaction_id = 0;
    dfu_packet.payload.state.dfu_type = DFU_TYPE_APP;
    memcpy(&dfu_packet.payload.state.fwid.app, &p_fwid_entry->version.app, sizeof(app_id_t));

    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.p_dfu_packet = &dfu_packet;
    rx_cmd.params.rx.length = DFU_PACKET_LEN_STATE_APP;
    bootloader_cmd_send(&rx_cmd);
}

void test_state_high_version(void)
{
    __LOG(RTT_CTRL_TEXT_CYAN "TEST: %s\n", __FUNCTION__);
    bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get(BL_INFO_TYPE_VERSION);
    dfu_packet_t dfu_packet;
    memset(&dfu_packet, 0, sizeof(dfu_packet_t));
    dfu_packet.packet_type = DFU_PACKET_TYPE_STATE;
    dfu_packet.payload.state.authority = 7;
    dfu_packet.payload.state.flood = 0;
    dfu_packet.payload.state.transaction_id = 0x12345678;
    dfu_packet.payload.state.dfu_type = DFU_TYPE_APP;
    memcpy(&dfu_packet.payload.state.fwid.app, &p_fwid_entry->version.app, sizeof(app_id_t));
    dfu_packet.payload.state.fwid.app.app_version++;

    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.p_dfu_packet = &dfu_packet;
    rx_cmd.params.rx.length = DFU_PACKET_LEN_STATE_APP;
    bootloader_cmd_send(&rx_cmd);
}

void test_state_highauth(void)
{
    __LOG(RTT_CTRL_TEXT_CYAN "TEST: %s\n", __FUNCTION__);
    bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get(BL_INFO_TYPE_VERSION);
    dfu_packet_t dfu_packet;
    memset(&dfu_packet, 0, sizeof(dfu_packet_t));
    dfu_packet.packet_type = DFU_PACKET_TYPE_STATE;
    dfu_packet.payload.state.authority = 7;
    dfu_packet.payload.state.flood = 0;
    dfu_packet.payload.state.transaction_id = 0x12345678;
    dfu_packet.payload.state.dfu_type = DFU_TYPE_APP;
    memcpy(&dfu_packet.payload.state.fwid.app, &p_fwid_entry->version.app, sizeof(app_id_t));

    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.p_dfu_packet = &dfu_packet;
    rx_cmd.params.rx.length = DFU_PACKET_LEN_STATE_APP;
    bootloader_cmd_send(&rx_cmd);
}

void test_start(void)
{
    __LOG(RTT_CTRL_TEXT_CYAN "TEST: %s\n", __FUNCTION__);
    dfu_packet_t dfu_packet;
    memset(&dfu_packet, 0, sizeof(dfu_packet_t));
    dfu_packet.packet_type = DFU_PACKET_TYPE_DATA;
    dfu_packet.payload.start.start_address = 0xFFFFFFFF;
    dfu_packet.payload.start.length = 0x400 / 4;
    dfu_packet.payload.start.transaction_id = 0x12345678;
    dfu_packet.payload.start.first = 1;

    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.p_dfu_packet = &dfu_packet;
    rx_cmd.params.rx.length = DFU_PACKET_LEN_START;
    bootloader_cmd_send(&rx_cmd);
}

void test_data(uint16_t segment)
{
    __LOG(RTT_CTRL_TEXT_CYAN "TEST: %s\n", __FUNCTION__);
    dfu_packet_t dfu_packet;
    memset(&dfu_packet, 0, sizeof(dfu_packet_t));
    dfu_packet.packet_type = DFU_PACKET_TYPE_DATA;
    dfu_packet.payload.data.segment = segment;
    dfu_packet.payload.data.transaction_id = 0x12345678;
    dfu_packet.payload.data.data[0] = 0xFF;
    dfu_packet.payload.data.data[1] = 0xFF;
    dfu_packet.payload.data.data[2] = 0xFF;
    dfu_packet.payload.data.data[3] = 0xFF;
    for (uint32_t i = 4; i < 16; ++i)
    {
        dfu_packet.payload.data.data[i] = i;
    }

    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.p_dfu_packet = &dfu_packet;
    rx_cmd.params.rx.length = DFU_PACKET_LEN_DATA;
    bootloader_cmd_send(&rx_cmd);
}

