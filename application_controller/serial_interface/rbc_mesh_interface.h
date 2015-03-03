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

#ifndef MESH_INTERFACE_H__
#define MESH_INTERFACE_H__


/** @brief executes an echo-test
 *  @details
 *  Promts the slave to echo whatever the buffer contains
 *  @param buffer memory containing data to echo
 *  @param len amount of data to send
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_echo(uint8_t* buffer, int len);

/** @brief initialization of rcb_mesh
 *  @details
 *  promts the slave to call rbc_mesh_init
 *  @param accessAddr pointer to 4 bytes containing the address
 *  @param chan Bluetooth channel to use. Must be 37, 38 or 39
 *  @param handleCount amount of handles in the system
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 *  or if chanNr is incorrect
 */
bool rbc_mesh_init(
	uint8_t* accessAddr,
	uint8_t chanNr,
	uint8_t handleCount);

/** @brief alter value of a handle
 *  @details
 *  promts the slave to call the rbc_mesh_init
 *  @param handle handle ID of the variable to be updated
 *  @param buffer pointer to the memory area containing data to send
 *  @param len Amount of bytes to be send
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_value_set(uint8_t handle, uint8_t* buffer, int len);

/** @brief read value of a handle
 *  @details
 *  promts the slave to call rbc_mesh_value_get
 *  @param handle handle ID of the variable to be read
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_value_get(uint8_t handle);

/** @brief start broadcasting value of a handle
 *  @details
 *  promts the slave to call rbc_mesh_value_enable
 *  @param handle handle ID of the variable to be enabled
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_value_enable(uint8_t handle);

/** @brief stop broadcasting value of a handle
 *  @details
 *  promts the slave to call rbc_mesh_value_disable
 *  @param handle handle ID of the variable to be disabled
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_value_disable(uint8_t handle);

/** @brief read the build_version
 *  @details
 *  promts the slave to return its build version
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_build_version_get();

/** @brief read the advertising address
 *  @details
 *  promts the slave to return the advertising address specified in the initialization
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_access_addr_get();

/** @brief read the operational channel
 *  @details
 *  promts the slave to return the operational channel specified in the initialization
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_channel_get();

/** @brief read the amount of handles
 *  @details
 *  promts the slave to return the amount of handles specified in the initialization
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_handle_count_get();

/** @brief read the advertising intervall
 *  @details
 *  promts the slave to return the advertising intervall
 *  it is by default set to 100 ms in this interface
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool rbc_mesh_adv_int_get();

/** @brief checkes if new events arrived
 *  @details
 *  checks for new events and takes them off the queue
 *  needs to be called in the main loop
 *  @return True if there is an event
 */
bool rbc_mesh_evt_get(hal_aci_data_t* p_evt);

/** @brief initialisation of local hardware
 *  @details
 *  Sets the SPI-pins
 *  @param pins struct containing pin configuration
 */
void rbc_mesh_hw_init(aci_pins_t* pins);

#endif //MESH_INTERFACE_H__
