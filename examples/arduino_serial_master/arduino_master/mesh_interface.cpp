#include "hal_aci_tl.h"
#include "lib_aci.h"

#include "boards.h"

hal_aci_data_t  msg_for_mesh;

bool mesh_interface_send_echo(uint8_t* buffer, int len){
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

bool mesh_interface_send_init(
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

bool mesh_interface_send_value_set(uint8_t handle, uint8_t* buffer, int len){

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


bool mesh_interface_send_value_enable(uint8_t handle){

	msg_for_mesh.buffer[0] = 2;		// size
	msg_for_mesh.buffer[1] = 0x72;		// cmd opcode
	
	msg_for_mesh.buffer[2] = handle;	// handle id

	return hal_aci_tl_send(&msg_for_mesh);
}

bool mesh_interface_send_value_disable(uint8_t handle){

	msg_for_mesh.buffer[0] = 2;		// size
	msg_for_mesh.buffer[1] = 0x73;		// cmd opcode
	
	msg_for_mesh.buffer[2] = handle;	// handle id

	return hal_aci_tl_send(&msg_for_mesh);
}

bool mesh_interface_send_value_get(uint8_t handle){

	msg_for_mesh.buffer[0] = 2;		// size
	msg_for_mesh.buffer[1] = 0x7A;		// cmd opcode
	
	msg_for_mesh.buffer[2] = handle;	// handle id

	return hal_aci_tl_send(&msg_for_mesh);
}


bool mesh_interface_send_build_version_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7B;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}


bool mesh_interface_send_adv_addr_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7C;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool mesh_interface_send_channel_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7D;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool mesh_interface_send_handle_count_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7E;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}

bool mesh_interface_send_adv_int_get(){

	msg_for_mesh.buffer[0] = 1;		// size
	msg_for_mesh.buffer[1] = 0x7F;		// cmd opcode
	
	return hal_aci_tl_send(&msg_for_mesh);
}

void mesh_interface_loop(){
	hal_aci_data_t data;
	hal_aci_tl_event_get(&data);

}

void mesh_interface_hw_init(aci_pins_t* pins){
	
  	hal_aci_tl_init(pins, false);
}
