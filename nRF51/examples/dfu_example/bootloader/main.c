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

#include "rbc_mesh.h"
#include "timeslot_handler.h"

#include "app_uart.h"
#include "app_error.h"
#include "dfu.h"
#include "nrf_gpio.h"
#include "boards.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/** The number of allocated handles */
#define HANDLE_COUNT            (100)

#define _LOG(str, ...) do{\
        char tx_str[128];\
        sprintf(tx_str, str, ##__VA_ARGS__);\
        char* c = tx_str;\
        while (*c)\
            app_uart_put(*(c++));\
    } while(0)

#define _LOG_BUFFER(buf, len) do{\
        for (uint32_t i = 0; i < len; ++i)\
            app_uart_put(buf[i]);\
    } while(0)

/** 
* @brief Handle an incoming command, and act accordingly.
*/
static void cmd_rx(uint8_t* cmd, uint32_t len)
{
    if (len < 2)
        return;
    
    uint32_t length = cmd[0];
    uint8_t opcode = cmd[1];
    uint32_t error;
    switch (opcode)
    {
        case 0x20: /* begin */
        {
            dfu_bootloader_info_t bl_info;
            memcpy(&bl_info.start_addr, &cmd[2], 4);
            memcpy(&bl_info.size, &cmd[6], 4);
            bl_info.dfu_type = (dfu_type_t) cmd[10];
            bl_info.using_crc = 0;
            bl_info.app_id = 0;
            error = dfu_transfer_begin(&bl_info);
        }
            break;
        case 0x21: /* data */
        {
            uint32_t addr;
            memcpy(&addr, &cmd[2], 4);
            error = dfu_transfer_data(addr, length - 7, &cmd[8]);
        }
            break; 
        case 0x22: /* end */
            error = dfu_end();
            break;
        case 0x02: /* echo */
        {
            uint8_t echo_reply[32];
            echo_reply[0] = length;
            echo_reply[1] = 0x82;
            memcpy(&echo_reply[0], &cmd[2], length - 1);
            app_uart_put(length);
            app_uart_put(0x82);
            app_uart_put(cmd[1]);
            for (uint32_t i = 0; i < length - 1; ++i)
            {
                app_uart_put(echo_reply[i]);
            }
        }
            break;
    }
    uint8_t reply[] = {0x05, 0x82, opcode, 0, 0, 0, 0};
    memcpy(&reply[3], &error, 4);
    for (uint32_t i = 0; i < reply[0] + 2; ++i)
    {
        app_uart_put(reply[i]);
    }
    
}

/** 
* @brief Handle incoming character on the control channel (UART or RTT).
*   Holds text until a \r or \n has been received, upon which it calls cmd_rx()
*/
static void char_rx(uint8_t c)
{
    static uint8_t rx_buf[32];
    static uint8_t* pp = rx_buf;
    
    *(pp++) = c;
    
    uint32_t len = (uint32_t)(pp - rx_buf);
    if (len >= sizeof(rx_buf) || (len > 1 && len >= rx_buf[0] + 1)) /* end of command */
    {
        cmd_rx(rx_buf, len);
        pp = rx_buf;
    }
}


/**
* @brief General error handler.
*/
static void error_loop(void)
{
    NVIC_SystemReset(); /* reset the system.  */
}

/**
* @brief Softdevice crash handler, never returns
* 
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
* 
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    error_loop();
}

void HardFault_Handler(void)
{
    error_loop();
}

/**
* @brief Softdevice event handler 
*/
uint32_t sd_evt_handler(void)
{
    rbc_mesh_sd_irq_handler();
    return NRF_SUCCESS;
}

/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Propagates the event to the host via UART or RTT.
*
* @param[in] evt RBC event propagated from framework
*/
void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{ 
    char cmd = 'x';
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL: cmd = 'C'; break;
        case RBC_MESH_EVENT_TYPE_INITIALIZED: cmd = 'I'; break;
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL: cmd = 'U'; break;
        case RBC_MESH_EVENT_TYPE_NEW_VAL: cmd = 'N'; break;
    }
    _LOG("%c[%d] ", cmd, evt->value_handle);
    _LOG_BUFFER(evt->data, evt->data_len);
    _LOG(" \tL:%d [%02X:%02X:%02X:%02X:%02X:%02X]\r\n",
        (int)evt->data_len,
        evt->originator_address.addr[0],
        evt->originator_address.addr[1],
        evt->originator_address.addr[2],
        evt->originator_address.addr[3],
        evt->originator_address.addr[4],
        evt->originator_address.addr[5]
    );
}

/** Handler for incoming UART events */
void uart_event_handler(app_uart_evt_t * p_app_uart_event)
{
    uint8_t c;
    switch (p_app_uart_event->evt_type)
    {
        case APP_UART_DATA_READY:
            while (app_uart_get(&c) == NRF_SUCCESS)
            {
                /* log single character */
                char_rx(c);
            }
            break;
        default:
            break;
    }
}

/** @brief main function */
int main(void)
{   
    uint32_t error_code;
    /* Enable Softdevice (including sd_ble before framework */
    
    nrf_gpio_range_cfg_output(0, 32);
    for (uint32_t i = LED_START; i <= LED_STOP; ++i)
        NRF_GPIO->OUTSET = (1 << i);
    NRF_GPIO->OUTCLR = (1 << LED_2);
    
    app_uart_comm_params_t uart_params;
    uart_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud460800;
    uart_params.cts_pin_no = CTS_PIN_NUMBER;
    uart_params.rts_pin_no = RTS_PIN_NUMBER;
    uart_params.rx_pin_no = RX_PIN_NUMBER;
    uart_params.tx_pin_no = TX_PIN_NUMBER;
    uart_params.flow_control = APP_UART_FLOW_CONTROL_ENABLED;
    uart_params.use_parity = false;
    APP_UART_FIFO_INIT(&uart_params, 8, 256, uart_event_handler, APP_IRQ_PRIORITY_LOW, error_code);
    APP_ERROR_CHECK(error_code);
    
    while (true)
    {
        __WFE();
    }
}

