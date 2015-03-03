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

 /** @file
  *  @brief parsing a subset of the mesh_rbc commands into spi messages 
  *  and enqueues them to be sent.
  */
#include "hal_aci_tl.h"
#include "lib_aci.h"

#include "boards.h"

#include "rbc_mesh_interface.h"

hal_aci_data_t  msg_for_mesh;

bool rbc_mesh_echo(uint8_t* buffer, int len){
	if (len > HAL_ACI_MAX_LENGTH - 1 || len < 0)
		return false;

	msg_for_mesh.buffer[0] = len + 1;	// size
	msg_for_mesh.buffer[1] = 0x02;		// cmd opcode

	int i;
	for( i = 0; i < len; i++){
		msg_for_mesh.buffer[i+2] = buffer[i];
	}
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_init(
	uint8_t* access_addr,
	uint8_t chanNr,
	uint8_t handleCount){
	
	if(chanNr != 37 && chanNr != 38 && chanNr != 39)
		return false;
	
	msg_for_mesh.buffer[0] = 11;		// size
	msg_for_mesh.buffer[1] = 0x70;		// cmd opcode
	
	msg_for_mesh.buffer[2] = access_addr[0];	//access address of network
	msg_for_mesh.buffer[3] = access_addr[1];
	msg_for_mesh.buffer[4] = access_addr[2];
	msg_for_mesh.buffer[5] = access_addr[3];
	
	msg_for_mesh.buffer[6] = chanNr;	// channel nr
	msg_for_mesh.buffer[7] = handleCount;	// amount of handles
	
	msg_for_mesh.buffer[8]  = 100;	//advertising intervall in ms, little endian
	msg_for_mesh.buffer[9]  = 0;
	msg_for_mesh.buffer[10] = 0;
	msg_for_mesh.buffer[11] = 0;
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_value_set(uint8_t handle, uint8_t* buffer, int len){

	if (len > HAL_ACI_MAX_LENGTH - 1 || len < 1)
		return false;

	msg_for_mesh.buffer[0] = len + 2;	// size
	msg_for_mesh.buffer[1] = 0x71;		// cmd opcode
	msg_for_mesh.buffer[2] = handle; 	// handle id

	int i;
	for( i = 0; i < len; i++){
		msg_for_mesh.buffer[i+3] = buffer[i];
	}
	
	return hal_aci_tl_send(&msg_for_mesh);
}


bool rbc_mesh_value_enable(uint8_t handle){

	msg_for_mesh.buffer[0] = 2;		// size
	msg_for_mesh.buffer[1] = 0x72;		// cmd opcode
	
	msg_for_mesh.buffer[2] = handle;	// handle id

	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_value_disable(uint8_t handle){

	msg_for_mesh.buffer[0] = 2;		// size
	msg_for_mesh.buffer[1] = 0x73;		// cmd opcode
	
	msg_for_mesh.buffer[2] = handle;	// handle id

	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_value_get(uint8_t handle){

	msg_for_mesh.buffer[0] = 2;		// size
	msg_for_mesh.buffer[1] = 0x7A;		// cmd opcode
	
	msg_for_mesh.buffer[2] = handle;	// handle id

	return hal_aci_tl_send(&msg_for_mesh);
}


bool rbc_mesh_build_version_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7B;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}


bool rbc_mesh_adv_addr_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7C;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_channel_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7D;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_handle_count_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7E;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_adv_int_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7F;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool rbc_mesh_evt_get(hal_aci_data_t* p_data){
	return hal_aci_tl_event_get(p_data);
}

void rbc_mesh_wait_for_answer(uint8_t* buf, int len){
	hal_aci_data_t data;
	
	while(!hal_aci_tl_event_get(&data));

	// get the x last bytes of the answer
	// (handle data comes in standart sizes, we eventually need less)
	for(int i = 0; i < len; i++){
		int offset = data.buffer[0] - len + 1;
		buf[i] = data.buffer[i+offset];
	}
}

void rbc_mesh_hw_init(aci_pins_t* pins){
	
  	hal_aci_tl_init(pins, false);
}
