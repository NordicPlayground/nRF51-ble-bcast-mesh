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

/* Must predefine the max length, at is larger than the one used for nRF8001 */
#define HAL_ACI_MAX_LENGTH (37)

 /** @file
  *  @brief parsing a subset of the mesh_rbc commands into spi messages 
  *  and enqueues them to be sent.
  */
#include "hal_aci_tl.h"
#include "lib_aci.h"
#include "serial_evt.h"
#include "serial_command.h"

#include "boards.h"

#include "rbc_mesh_interface.h"

static void unaligned_memcpy(uint8_t* p_dst, uint8_t const* p_src, uint8_t len){
  while(len--)
  {
    *p_dst++ = *p_src++;
  }
}

bool rbc_mesh_echo(uint8_t* buffer, int len){
	if (len > HAL_ACI_MAX_LENGTH - 1 || len < 0)
		return false;
    
    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = len + 1; // account for opcode
    p_cmd->opcode = SERIAL_CMD_OPCODE_ECHO;
    memcpy(p_cmd->params.echo.data, buffer, len);

	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_init(
	uint32_t accessAddr,
	uint8_t chanNr,
	uint8_t handleCount,
    uint32_t advInt_ms){
	
	if(chanNr != 37 && chanNr != 38 && chanNr != 39)
		return false;
	
    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 11;
    p_cmd->opcode = SERIAL_CMD_OPCODE_INIT;
    p_cmd->params.init.access_addr = accessAddr;
    p_cmd->params.init.channel = chanNr;
    p_cmd->params.init.handle_count = handleCount;
    p_cmd->params.init.adv_int_min = advInt_ms;

	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_value_set(uint8_t handle, uint8_t* buffer, int len){

	if (len > HAL_ACI_MAX_LENGTH - 1 || len < 1)
		return false;

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = len + 2; // account for opcode and handle 
    p_cmd->opcode = SERIAL_CMD_OPCODE_VALUE_SET;
    p_cmd->params.value_set.handle = handle;
    memcpy(p_cmd->params.value_set.value, buffer, len);

	return hal_aci_tl_send(&msg_for_mesh);
}


bool rbc_mesh_value_enable(uint8_t handle){

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 2;
    p_cmd->opcode = SERIAL_CMD_OPCODE_VALUE_ENABLE;
    p_cmd->params.value_enable.handle = handle;

	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_value_disable(uint8_t handle){

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 2;
    p_cmd->opcode = SERIAL_CMD_OPCODE_VALUE_DISABLE;
    p_cmd->params.value_enable.handle = handle;

	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_value_get(uint8_t handle){

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 2;
    p_cmd->opcode = SERIAL_CMD_OPCODE_VALUE_GET;
    p_cmd->params.value_enable.handle = handle;

	return hal_aci_tl_send(&msg_for_mesh);
}


bool rbc_mesh_build_version_get(){

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 1;
    p_cmd->opcode = SERIAL_CMD_OPCODE_BUILD_VERSION_GET;
	
	return hal_aci_tl_send(&msg_for_mesh);
}


bool rbc_mesh_access_addr_get(){

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 1;
    p_cmd->opcode = SERIAL_CMD_OPCODE_ACCESS_ADDR_GET;
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_channel_get(){

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 1;
    p_cmd->opcode = SERIAL_CMD_OPCODE_CHANNEL_GET;
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_handle_count_get(){

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 1;
    p_cmd->opcode = SERIAL_CMD_OPCODE_HANDLE_COUNT_GET;
    
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_adv_int_get(){

    hal_aci_data_t msg_for_mesh;
    serial_cmd_t* p_cmd = (serial_cmd_t*) msg_for_mesh.buffer;

    p_cmd->length = 1;
    p_cmd->opcode = SERIAL_CMD_OPCODE_ADV_INT_GET;
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_evt_get(serial_evt_t* p_evt){
    hal_aci_data_t msg;
    bool status = hal_aci_tl_event_get(&msg);
    if (status)
        memcpy((uint8_t*) p_evt, msg.buffer, sizeof(serial_evt_t));

    return status;
}

void rbc_mesh_hw_init(aci_pins_t* pins){
	
  	hal_aci_tl_init(pins, false);
}

