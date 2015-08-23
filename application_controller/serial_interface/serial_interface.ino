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

#include <SPI.h>
#include <stdint.h>
#include "lib_aci.h"

#include "rbc_mesh_interface.h"

#define ACCESS_ADDR     (0xA541A68F)

// defining the state of the system for communication
enum state_t{
    STATE_INIT,           // not yet initialized
    STATE_READY,          // no external command in work
    STATE_WAITING_FOR_EVT // external command in work, waiting for answer from SPI slave
};

int state = STATE_INIT;

// buffer for the last value send by the SPI slave
int lastResponse;

int initState = 0;

// callback function for http command to set a handle value
int set_val(String args){

    if(state != ready){
        return -1;
    }
    state = waitingForEvent;

    uint8_t handle = args[0] - '0';
    uint8_t value  = args[1] - '0';
    
    return rbc_mesh_value_set(handle, &value, (uint8_t) 1);
}

// callback function for http command to get a handle value
int get_val_req(String args){

    if(state != ready){
        return -1;
    }
    state = waitingForEvent;

    uint8_t handle = args[0] - '0';
    
    return rbc_mesh_value_get(handle);
}

aci_pins_t pins;

// arduino conform init function
void setup(void)
{
  Serial.begin(9600);
  
  pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  pins.reqn_pin   = A2;
  pins.rdyn_pin   = D5;
  pins.mosi_pin   = A5;
  pins.miso_pin   = A4;
  pins.sck_pin    = A3;

  pins.spi_clock_divider      = SPI_CLOCK_DIV8;

  pins.reset_pin              = D2;
  pins.active_pin             = NRF_UNUSED;
  pins.optional_chip_sel_pin  = NRF_UNUSED;

  pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  pins.interrupt_number       = 1;

  rbc_mesh_hw_init(&pins);
}

// sending intialization commands to SPI slave
// alternating sending of commands and waiting for response
void initConnectionSlowly(){
    switch(initState) {
        case 0:
            rbc_mesh_init(ACCESS_ADDR, (uint8_t) 38, (uint8_t) 2, (uint32_t) 100);
            initState++;
            Serial.println("Sent init command");
            break;
        case 1:
            rbc_mesh_value_enable((uint8_t) 1);
            initState++;
            Serial.println("Enabled value 1");
            break;
        case 2:
            rbc_mesh_value_enable((uint8_t) 2);
            initState++;
            Serial.println("Enabled value 2");
            break;
        case 3:
            state = ready;
            Serial.println("init done");
    }
}

// arduino conform main loop
void loop() {
    static bool newMessage = false;
    // send next initialization command to SPI slave until we leave init state
    if (state == STATE_INIT && newMessage) {
        initConnectionSlowly();
    }
    
    //Process any ACI commands or events
    serial_evt_t evnt;
    newMessage = rbc_mesh_evt_get(&evnt);

    if(newMessage && evnt.opcode == SERIAL_EVT_OPCODE_CMD_RSP){
        if (evnt.params.cmd_rsp.status != ACI_STATUS_SUCCESS)
        {
            Serial.print("Error response on cmd ");
            Serial.print(evnt.params.cmd_rsp.command_opcode, HEX);
            Serial.print(": ");
            Serial.print(evnt.params.cmd_rsp.status, HEX);
            Serial.println();
        }

        if (state == STATE_WAITING_FOR_EVT) {
            state = STATE_READY;
            // save the response value of SPI slave into buffer
            // can be read via callback function
            lastResponse = evnt.params.cmd_rsp.response.val_get.data[0];
        }
    }
}
