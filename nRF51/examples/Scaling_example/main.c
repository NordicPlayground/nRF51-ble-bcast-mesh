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
#include "nrf_adv_conn.h"
#include "timeslot_handler.h"

#include "app_uart.h"
#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"
#include "boards.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/** Log transport medium flag. Set to 0 to use UART, 1 to use RTT */
#define LOG_RTT                 (0)

/** The number of allocated handles */
#define HANDLE_COUNT            (100)

/** Logging predefines. Hides UART/RTT functions */
#if LOG_RTT

#define _LOG(str, ...)          SEGGER_RTT_printf(0, str, ##__VA_ARGS__)

#define _LOG_BUFFER(buf, len)   do{ \
        char hdr[] = {0, len};\
        SEGGER_RTT_Write(0, hdr, 2); \
        SEGGER_RTT_Write(0, (char*) buf, len); \
    } while (0)

#else
    
#define _LOG(str, ...) do{\
        char tx_str[128];\
        sprintf(tx_str, str, ##__VA_ARGS__);\
        char* c = tx_str;\
        while (*c)\
            app_uart_put(*(c++));\
    } while(0)

#define _LOG_BUFFER(buf, len) do{\
        uint8_t* c = buf;\
        uint8_t length = len;\
        while (length--)\
            app_uart_put(*(c++));\
    } while(0)

#endif

#define MESH_ACCESS_ADDR        (0xA541A68F)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (38)
#define MESH_HANDLE_COUNT       (2)

typedef enum
{
    SCALING_CMD_TX = 'U',
    SCALING_CMD_DISABLE = 'D'
} scaling_cmd_t;

/** @brief parse a number from a string (similar to std::atoi) */
static uint8_t get_num(uint8_t** buf, uint32_t* len)
{
    uint8_t num = 0;
    uint8_t* p = *buf;
    
    while (*p >= '0' && *p <= '9') 
    {
        num *= 10;
        num += *p - '0';
        p++;
        (*len)--;
    }
    *buf = p;
    return num;
}
    
/** @brief Parse an incoming set command */
static void parse_set(uint8_t* buf, uint32_t* len, uint8_t* handle, uint8_t** payload)
{
    *handle = 0;
    uint8_t* p = buf;
    
    while (*p == ' ') 
    {
        (*len)--;
        p++;
    }
    *handle = get_num(&p, len);
    while (*p == ' ') 
    {
        (*len)--;
        p++;
    }
    (*len)--;
    *payload = p;
}

/** 
* @brief Handle an incoming command, and act accordingly.
*/
static void cmd_rx(uint8_t* cmd, uint32_t len)
{
    if (len < 2)
        return;
    
    uint8_t* payload;
    uint8_t handle = 0;
    len--;
    switch ((scaling_cmd_t) cmd[0])
    {
        case SCALING_CMD_TX:
            /* got a new value */
            parse_set(&cmd[1], &len, &handle, &payload);
            
            if (rbc_mesh_value_set(handle, payload, len) == NRF_SUCCESS)
            {
                _LOG("V[%d] %s\tL:%d\r\n", (int)handle, payload, (int)len);
            }
            
            break;
        case SCALING_CMD_DISABLE:
            rbc_mesh_value_disable(cmd[1]);
            _LOG("D[%d]\r\n", (int)cmd[1]);
            break;
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
    
    if (c != '\r' && c != '\n')
    {
        *(pp++) = c;
    }
    else
    {
        *(pp++) = 0;
    }
    uint32_t len = (uint32_t)(pp - rx_buf);
    if (len >= sizeof(rx_buf) || c == '\r' || c == '\n') /* end of command */
    {
        if (len > 0)
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
        case RBC_MESH_EVENT_TYPE_TX: cmd = 'T'; break;
    }
    _LOG("%c[%d] ", cmd, evt->value_handle);
    _LOG_BUFFER(evt->data, evt->data_len);
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
    /* Enable Softdevice (including sd_ble before framework */
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_evt_handler);
    
    /* Init the rbc_mesh */
    rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.handle_count = MESH_HANDLE_COUNT;
    init_params.packet_format = RBC_MESH_PACKET_FORMAT_ORIGINAL;
    init_params.radio_mode = RBC_MESH_RADIO_MODE_BLE_1MBIT;
   
    uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);
    
    nrf_gpio_range_cfg_output(0, 32);
 
    
#if LOG_RTT
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#else
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
#endif

    _LOG("START\r\n");
    
#if LOG_RTT
    while (true)
    {
        uint8_t c = SEGGER_RTT_WaitKey();
        char_rx(c);
    }
#else      
    while (true)
    {
        sd_app_evt_wait();
    }
#endif
}

