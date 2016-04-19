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
#include <EEPROM.h>
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

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}


// callback function for http command to set a handle value
int set_val(String args){

    if(state != STATE_READY){
        return -1;
    }
    state = STATE_WAITING_FOR_EVT;

    uint8_t handle = args[0] - '0';
    uint8_t value  = args[1] - '0';
    
    return rbc_mesh_value_set(handle, &value, (uint8_t) 1);
}

// callback function for http command to get a handle value
int get_val_req(String args){

    if(state != STATE_READY){
        return -1;
    }
    state = STATE_WAITING_FOR_EVT;

    uint8_t handle = args[0] - '0';
    
    return rbc_mesh_value_get(handle);
}

aci_pins_t pins;

// arduino conform init function
void setup(void)
{
  Serial.begin(9600);
  
  pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  pins.reqn_pin   = 9;
  pins.rdyn_pin   = 8;
  pins.mosi_pin   = MOSI;
  pins.miso_pin   = MISO;
  pins.sck_pin    = SCK;

  pins.spi_clock_divider      = SPI_CLOCK_DIV8;

  pins.reset_pin              = UNUSED;
  pins.active_pin             = UNUSED;
  pins.optional_chip_sel_pin  = UNUSED;

  pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  pins.interrupt_number       = 1;

  rbc_mesh_hw_init(&pins);
}

// sending intialization commands to SPI slave
// alternating sending of commands and waiting for response
void initConnectionSlowly(){
    switch(initState) {
        case 0:
            rbc_mesh_init(ACCESS_ADDR, 38, 100);
            initState++;
            Serial.println("Sent init command");
            break;
        case 1:
            rbc_mesh_value_enable(1);
            initState++;
            Serial.println("Enabled value 1");
            break;
        case 2:
            rbc_mesh_value_enable(2);
            initState++;
            Serial.println("Enabled value 2");
            break;
        case 3:
            state = STATE_READY;
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
