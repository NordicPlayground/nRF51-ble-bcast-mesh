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

static aci_pins_t pins;

static uint8_t         uart_buffer[20];
static uint8_t         dummychar = 0;

// access address in our example mesh application
//uint32_t accAddr = 0x8FA641A5;
uint32_t accAddr = 0xA541A68F;

void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}


void setup(void)
{
  Serial.begin(115200);
  //Wait until the serial port is available (useful only for the Leonardo)
  //As the Leonardo board is not reseted every time you open the Serial Monitor
  #if defined (__AVR_ATmega32U4__)
    while(!Serial)
    {}
    delay(1000);  //5 seconds delay for enabling to see the start up comments on the serial board
  #elif defined(__PIC32MX__)
    delay(1000);
  #endif
  
  pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  pins.reqn_pin   = SS; 
  pins.rdyn_pin   = 8; 
  pins.mosi_pin   = MOSI;
  pins.miso_pin   = MISO;
  pins.sck_pin    = SCK;

  pins.spi_clock_divider      = SPI_CLOCK_DIV8;
  pins.reset_pin              = UNUSED; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  pins.active_pin             = UNUSED;
  pins.optional_chip_sel_pin  = UNUSED;

  pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  pins.interrupt_number       = 1;

  // initialize mesh_interface
  rbc_mesh_hw_init(&pins);

  Serial.println("Local setup done");
  Serial.println("Starting connection setup");
  return;

}

bool stringComplete = false;  // whether the string is complete
uint8_t stringIndex = 0;      //Initialize the index to store incoming chars

enum state_t{
    starting,
    ready,
    waitingForEvent
};

int state = starting;


int initState = 0;
bool newMessage = false;

void initConnectionSlowly(){
    switch(initState) {
        case 0:
            Serial.print(".");
            rbc_mesh_init(accAddr, (uint8_t) 38, (uint8_t) 2, (uint32_t) 100);
            initState++;
            break;
        case 1:
            if(newMessage){
                Serial.print(".");
                rbc_mesh_value_enable((uint8_t) 1);
                initState++;
            }
            break;
        case 2:
            if(newMessage){
                Serial.println(".");
                rbc_mesh_value_enable((uint8_t) 2);
                initState++;
            }
            break;
        case 3:
            if(newMessage){
                state = ready;
                Serial.println("connection setup done");
                Serial.println("possible commands are");
                Serial.println(" 1 - red off");
                Serial.println(" 2 - red on");
                Serial.println(" 3 - green off");
                Serial.println(" 4 - green on");
            }
    }
}

void actions_loop() {
  if (stringComplete){
    Serial.print("");
    Serial.print("running, state is ");
    Serial.println(state);
      
    uint8_t handle = 1;
    uint8_t value = 1;
    if(uart_buffer[0] == '1'){
      Serial.println("got 1: red off");
      handle = 1;
      value = 0;
    }
    if(uart_buffer[0] == '2'){
      Serial.println("got 2: red on");
      handle = 1;
      value = 1;
    }
    if(uart_buffer[0] == '3'){
      Serial.println("got 3: green off");
      handle = 2;
      value = 0;
    }
    if(uart_buffer[0] == '4'){
      Serial.println("got 4: green on");
      handle = 2;
      value = 1;
    }
    if(state == ready){
      Serial.println("sending now");
      state = waitingForEvent;
      rbc_mesh_value_set(handle, &value, (uint8_t) 1);
    }  
    stringIndex = 0;
    stringComplete = false;
  }
  if (Serial.available()) {
      serialEvent();
  }
}

void loop() {
  
    if(state == starting){
        initConnectionSlowly();
    }
    //Process any ACI commands or events
    serial_evt_t evnt;
    newMessage = rbc_mesh_evt_get(&evnt);

    if(state == waitingForEvent && newMessage){
        state = ready;
    } 
    
  // react to command line interaction
  actions_loop();

}

// handle incoming UART messages
void serialEvent() {

  while(Serial.available() > 0){
    // get the new byte:
    dummychar = (uint8_t)Serial.read();
    if(stringComplete){
      continue;
    }
    if (dummychar == '\n'){
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it
      uart_buffer[stringIndex] = 0;
      stringIndex--;
      stringComplete = true;
      continue;
    }
      
    if(stringIndex > 19){
      Serial.println("Serial input truncated");
      stringIndex--;
      stringComplete = true;
      return;
    }
    // add it to the uart_buffer
    uart_buffer[stringIndex] = dummychar;
    stringIndex++;
  }
}
