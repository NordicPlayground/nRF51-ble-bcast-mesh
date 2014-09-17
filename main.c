#include "radio_control.h"
#include "trickle.h"
#include "trickle_common.h"


#include "nrf_sdm.h"
#include "app_error.h"
#include "nrf_assert.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "boards.h"
#include "simple_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>




/*****************************************************************************
* Static globals
*****************************************************************************/

/**
* Timeslot request structures
*/
static const nrf_radio_request_t radio_request_normal = 
                {
                    .request_type = NRF_RADIO_REQ_TYPE_NORMAL,
                    .params.normal = 
                    {
                        .hfclk = NRF_RADIO_HFCLK_CFG_DEFAULT,
                        .priority = NRF_RADIO_PRIORITY_NORMAL,
                        .distance_us = TRICKLE_INTERVAL_US,
                        .length_us = TRICKLE_TIMESLOT_LENGTH_US
                    }
                };
                
static const nrf_radio_request_t radio_request_earliest = 
                {
                    .request_type = NRF_RADIO_REQ_TYPE_EARLIEST,
                    .params.earliest = 
                    {
                        .hfclk = NRF_RADIO_HFCLK_CFG_DEFAULT,
                        .priority = NRF_RADIO_PRIORITY_NORMAL,
                        .length_us = TRICKLE_TIMESLOT_LENGTH_US,
                        .timeout_us = 1000000 /* 1 second */
                    }
                };
                        
                
/**
* Timeslot callback return parameters
*/
static const nrf_radio_signal_callback_return_param_t radio_signal_cb_ret_param_none =
                {
                    .callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE
                };
                
static const nrf_radio_signal_callback_return_param_t radio_signal_cb_ret_param_request =
                {
                    .callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
                    .params.request.p_next = (nrf_radio_request_t*) &radio_request_normal
                };

/*****************************************************************************
* System callback functions
*****************************************************************************/

void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    SET_PIN(PIN_ABORTED);
    while (true)
    {
        nrf_delay_ms(500);
        SET_PIN(LED_0);
        CLEAR_PIN(LED_1);
        nrf_delay_ms(500);
        SET_PIN(LED_1);
        CLEAR_PIN(LED_0);
    }
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    SET_PIN(PIN_ABORTED);
    while (true)
    {
        nrf_delay_ms(500);
        SET_PIN(LED_0);
        CLEAR_PIN(LED_1);
        nrf_delay_ms(500);
        SET_PIN(LED_1);
        CLEAR_PIN(LED_0);
    }
}

/**
* Timeslot related events callback
* Called whenever the softdevice tries to change the original course of actions 
* related to the timeslots
*/
void SD_EVT_IRQHandler(void)
{
    uint32_t evt;
    
    while (sd_evt_get(&evt) == NRF_SUCCESS)
    {
        switch (evt)
        {
            case NRF_EVT_RADIO_SESSION_IDLE:
                
                break;
            
            case NRF_EVT_RADIO_SESSION_CLOSED:
                
                break;
            
            case NRF_EVT_RADIO_BLOCKED:
                
                break;
            
            case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                break;
            
            case NRF_EVT_RADIO_CANCELED:
                
                break;
            
            default:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
        }
    }
}

static nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t sig)
{
    /* If the trickle step is not finished, the default action is to continue the timeslot */
    nrf_radio_signal_callback_return_param_t* ret_param = (nrf_radio_signal_callback_return_param_t*) &radio_signal_cb_ret_param_none;
    
    switch (sig)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            /* fire up the radio ASAP, send/receive data */
            radio_init();
            trickle_step();
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            /* send to radio control module */
            radio_event_handler();
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            /* TODO */
            break;
        default:
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
    }
        
    return ret_param;
}

/*****************************************************************************
* Static Functions
*****************************************************************************/



/*****************************************************************************
* Interface Functions
*****************************************************************************/

void timeslot_handler_init(void)
{
    uint32_t error;
    
    error = sd_softdevice_enable((uint32_t)NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_handler);
    APP_ERROR_CHECK(error);
    
    error = sd_nvic_EnableIRQ(SD_EVT_IRQn);
    APP_ERROR_CHECK(error);
    
    sd_radio_session_open(&radio_signal_callback);
    
}