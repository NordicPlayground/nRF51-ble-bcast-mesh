

bool mesh_interface_send_echo(uint8_t* buffer, int len);

bool mesh_interface_send_init(
	uint8_t* access_addr,
	uint8_t chanNr,
	uint8_t handleCount);

bool mesh_interface_send_value_set(uint8_t handle, uint8_t* buffer, int len);

bool mesh_interface_send_value_enable(uint8_t handle);

bool mesh_interface_send_value_disable(uint8_t handle);

bool mesh_interface_send_value_get(uint8_t handle);

bool mesh_interface_send_build_version_get();

bool mesh_interface_send_adv_addr_get();

bool mesh_interface_send_channel_get();

bool mesh_interface_send_handle_count_get();

bool mesh_interface_send_adv_int_get();

void mesh_interface_loop();

void mesh_interface_hw_init(aci_pins_t* other_pins);
