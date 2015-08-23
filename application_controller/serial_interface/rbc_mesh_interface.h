/* Copyright (c) 2014, Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MESH_INTERFACE_H__
#define MESH_INTERFACE_H__

#include "serial_evt.h"

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
 *  @param advInt_ms the lowest possible transmission interval
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 *  or if chanNr is incorrect
 */
bool rbc_mesh_init(
	uint32_t accessAddr,
	uint8_t chanNr,
	uint8_t handleCount,
    uint32_t advInt_ms);

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
bool rbc_mesh_evt_get(serial_evt_t* p_evt);

/** @brief initialisation of local hardware
 *  @details
 *  Sets the SPI-pins
 *  @param pins struct containing pin configuration
 */
void rbc_mesh_hw_init(aci_pins_t* pins);

/** @brief Transmit a reset command to the slave
 *  @details 
 *  The slave will do a software reset, calling NVIC_SystemReset(), effectively 
 *  restarting the device, erasing all configuration and halting operation.
 *  To resume operation, the initialization process has to be redone.
 *  The command does not yield a command response, but the slave will transmit
 *  a DEVICE_STARTED event when it is ready to receive initialization commands.
 */
void rbc_mesh_radio_reset();

#endif //MESH_INTERFACE_H__
