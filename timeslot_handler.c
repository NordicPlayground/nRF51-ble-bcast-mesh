#include "timeslot_handler.h"

#include "radio_control.h"
#include "trickle.h"
#include "trickle_common.h"
#include "rbc_database.h"
#include "timer_control.h"
#include "ll_control.h"

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

#define USE_SWI_FOR_PROCESSING      (0)


#define PACKET_ADV_ADDR_INDEX       (3)
#define PACKET_LENGTH_INDEX         (1)
#define PACKET_TYPE_INDEX           (0)
#define PACKET_ADV_DATA_INDEX       (9)
#define PACKET_MAN_DATA_INDEX       (13)

#define PACKET_TYPE                 (0x02)

#define PACKET_ADV_DATA_MAN_TYPE    (0xFF)
#define PACKET_ADV_DATA_MAN_COMPANY (0x0059) /* Nordic Semiconductor identifier */

#define PACKET_PROPAGATION_TIME_US  (104)
#define PACKET_PRECOMP_TIME_US      (70)
#define PACKET_SAFETY_MARGIN_US     (80)
#define PACKET_RAMP_UP_TIME_US      (140)

#define TIMESLOT_END_SAFETY_MARGIN_US  (150)

/*****************************************************************************
* Local type definitions
*****************************************************************************/



/*****************************************************************************
* Static globals
*****************************************************************************/

/**
* Timeslot request structures
*/


static nrf_radio_request_t radio_request_normal = 
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
                
static nrf_radio_request_t radio_request_earliest = 
                {
                    .request_type = NRF_RADIO_REQ_TYPE_EARLIEST,
                    .params.earliest = 
                    {
                        .hfclk = NRF_RADIO_HFCLK_CFG_DEFAULT,
                        .priority = NRF_RADIO_PRIORITY_NORMAL,
                        .length_us = TRICKLE_TIMESLOT_LENGTH_US,
                        .timeout_us = 1000000 /* 1s */
                    }
                };
                        
#if 0                
/**
* Timeslot callback return parameters
*/
static const nrf_radio_signal_callback_return_param_t radio_signal_cb_ret_param_none =
                {
                    .callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE
                };
                
static nrf_radio_signal_callback_return_param_t radio_signal_cb_ret_param_request =
                {
                    .callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
                    .params.request.p_next = (nrf_radio_request_t*) &radio_request_normal
                };
                
static const nrf_radio_signal_callback_return_param_t radio_signal_cb_ret_param_extend =
                {
                    .callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,
                    .params.extend.length_us = 900000 /* 0.9 seconds */
                };
#endif
                
                
                  
static nrf_radio_signal_callback_return_param_t g_ret_param;
static nrf_radio_signal_callback_return_param_t g_final_ret_param;

static bool g_is_in_callback = true;
                
static uint32_t g_timeslot_length;      
static uint32_t g_timeslot_end_timer;      
static uint32_t g_next_timeslot_length;    
static uint32_t g_total_time = 0;                
                
/*****************************************************************************
* Static Functions
*****************************************************************************/


/*****************************************************************************
* System callback functions
*****************************************************************************/


/**
* Callback for TX completion. After a successful TX, the timeslot should end.
*/
void radio_tx_callback(void)
{
    TICK_PIN(PIN_TRICKLE_TX);
}


void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    
    SET_PIN(PIN_ABORTED);
    uint8_t* name_ptr = (uint8_t*) p_file_name;
    while (*name_ptr != '\0')
    {
        PIN_OUT(name_ptr[0], 8);
        
        ++name_ptr;
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
    }
    while (true)
    {
        PIN_OUT(line_num, 16);
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
        PIN_OUT(error_code, 32);
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
void ts_sd_event_handler(void)
{
    uint32_t evt;
    SET_PIN(6);
    while (sd_evt_get(&evt) == NRF_SUCCESS)
    {
        PIN_OUT(evt, 32);
        switch (evt)
        {
            case NRF_EVT_RADIO_SESSION_IDLE:
                
                timeslot_order_earliest(20000, true);
                break;
            
            case NRF_EVT_RADIO_SESSION_CLOSED:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                
                break;
            
            case NRF_EVT_RADIO_BLOCKED:
                /*TODO*/
                timeslot_order_earliest(20000, true);
                break;
            
            case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                break;
            
            case NRF_EVT_RADIO_CANCELED:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                break;
            default:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
        }
    }
    CLEAR_PIN(6);
}

static void end_timer_handler(void)
{
    timeslot_order_earliest(25000, true);
    //timeslot_extend(10000);
}
    
#if USE_SWI_FOR_PROCESSING
void SWI0_IRQHandler(void)
{
    ll_control_timeslot_begin(g_total_time);
}
#endif
/**
* Radio signal callback handler taking care of all signals in searching mode
*/
static nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t sig)
{
    g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
    g_is_in_callback = true;
    static uint32_t requested_extend_time = 0;
    SET_PIN(PIN_SYNC_TIME);
    
    switch (sig)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            /* default end action is extend */
            timer_init();
            SET_PIN(2);
            g_timeslot_length = g_next_timeslot_length;
        
            g_timeslot_end_timer = 
                timer_order_cb(g_timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US, 
                    end_timer_handler);
            
#if USE_SWI_FOR_PROCESSING
            NVIC_EnableIRQ(SWI0_IRQn);
            NVIC_SetPendingIRQ(SWI0_IRQn);
#else
            ll_control_timeslot_begin(g_total_time);
#endif
        
            break;
        
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            /* send to radio control module */
            DEBUG_PIN_TH(PIN_RADIO_SIGNAL);
            radio_event_handler();
            break;
        
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            /* send to timer control module */
            DEBUG_PIN_TH(PIN_TIMER_SIGNAL);
            timer_event_handler();
            break;
            
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
            TICK_PIN(1);
            g_timeslot_length += requested_extend_time;
            requested_extend_time = 0;
            g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        
            timer_abort(g_timeslot_end_timer);
        
            g_timeslot_end_timer = 
                timer_order_cb(g_timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US, 
                    end_timer_handler);
        
            break;
        
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
            /* trust timeout timer to take care of the timeslot before it runs out */
            g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;           
            break;
        
        default:
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
    }
    
    
    g_is_in_callback = false;
    if (g_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND)
    {
        requested_extend_time = g_ret_param.params.extend.length_us;
    }
    else if (g_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END)
    {
        CLEAR_PIN(2);
        g_total_time += timer_get_timestamp();
    }
    else
    {
        requested_extend_time = 0;
    }
    
    CLEAR_PIN(PIN_SYNC_TIME);
    return &g_ret_param;
}


/*****************************************************************************
* Interface Functions
*****************************************************************************/

void timeslot_handler_init(void)
{
    uint32_t error;
    
    g_is_in_callback = false;
    
    error = sd_softdevice_enable((uint32_t)NRF_CLOCK_LFCLKSRC_XTAL_250_PPM, sd_assert_handler);
    APP_ERROR_CHECK(error); 
    
    error = sd_nvic_EnableIRQ(SD_EVT_IRQn);
    APP_ERROR_CHECK(error);
    
    sd_radio_session_open(&radio_signal_callback);
    g_total_time = 0;
    
    timeslot_order_earliest(10000, true);
}



void timeslot_order_earliest(uint32_t length_us, bool immediately)
{
    if (immediately)
    {
        radio_request_earliest.params.earliest.length_us = length_us;
        g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        g_ret_param.params.request.p_next = &radio_request_earliest;
        
        g_next_timeslot_length = length_us;
        
        if (!g_is_in_callback)
        {
            sd_radio_request(&radio_request_earliest);
        }
    }
    else
    {
        radio_request_earliest.params.earliest.length_us = length_us;
        g_final_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        g_final_ret_param.params.request.p_next = &radio_request_earliest;
        
        g_next_timeslot_length = length_us;
    }
}


void timeslot_order_normal(uint32_t length_us, uint32_t distance_us, bool immediately)
{
    if (immediately)
    {
        radio_request_normal.params.normal.length_us = length_us;
        radio_request_normal.params.normal.distance_us = distance_us;
        g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        g_ret_param.params.request.p_next = &radio_request_normal;
        
        g_next_timeslot_length = length_us;
        
        if (!g_is_in_callback)
        {
            sd_radio_request(&radio_request_normal);
        }
    }
    else
    {
        radio_request_normal.params.normal.length_us = length_us;
        radio_request_normal.params.normal.distance_us = distance_us;
        g_final_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        g_final_ret_param.params.request.p_next = &radio_request_normal;
        
        g_next_timeslot_length = length_us;
    }
}

void timeslot_extend(uint32_t extra_time_us)
{
    if (g_is_in_callback)
    {
        g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
        g_ret_param.params.extend.length_us = extra_time_us;
    }
}

uint32_t timeslot_get_remaining_time(void)
{
    uint32_t timestamp = timer_get_timestamp();
    return (g_timeslot_length - timestamp - TIMESLOT_END_SAFETY_MARGIN_US);
}
