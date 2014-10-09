#include "radio_control.h"
#include "timer_control.h"
#include "trickle_common.h"
#include "trickle.h"
#include "nrf.h"
#include "nrf_sdm.h"

#include <stdbool.h>
#include <string.h>

#define RADIO_FIFO_QUEUE_SIZE 4 /* must be power of two */
#define RADIO_FIFO_QUEUE_MASK (RADIO_FIFO_QUEUE_SIZE - 1)

#define RADIO_RX_TIMEOUT                (15080)


#define RADIO_EVENT(evt)  (NRF_RADIO->evt == 1)



/** 
* Internal enum denoting radio state. 
*/
typedef enum 
{
    RADIO_STATE_RX,
    RADIO_STATE_TX,
    RADIO_STATE_DISABLED
} radio_state_t;



/*****************************************************************************
* Static globals
*****************************************************************************/

/**
* Global radio state
*/
static radio_state_t radio_state = RADIO_STATE_RX;

/**
* RX buffers
*/
static uint8_t rx_data_buf[512];
static uint8_t* rx_data[2] = {&rx_data_buf[0], &rx_data_buf[256]};
static uint8_t current_rx_buf = 0;

/**
* local tx_data pointer
*/
//static uint8_t* tx_data;

static radio_rx_cb g_radio_rx_cb;
static radio_tx_cb g_radio_tx_cb;


static radio_event_t radio_fifo_queue[RADIO_FIFO_QUEUE_SIZE];

static uint8_t radio_fifo_head;
static uint8_t radio_fifo_tail;

static uint8_t rx_abort_timer_index;


/*****************************************************************************
* Static functions
*****************************************************************************/

static void rx_abort_cb(void);

static bool radio_fifo_full(void)
{
    return ((radio_fifo_tail + RADIO_FIFO_QUEUE_SIZE) == radio_fifo_head);
}

static bool radio_fifo_empty(void)
{
    return (radio_fifo_head == radio_fifo_tail);
}

static uint8_t radio_fifo_get_length(void)
{
    return (radio_fifo_head - radio_fifo_tail) & 0xFF;
}

static uint8_t radio_fifo_put(radio_event_t* evt)
{
    if (radio_fifo_full())
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    
    radio_event_t* head = &radio_fifo_queue[radio_fifo_head & RADIO_FIFO_QUEUE_MASK];
    
    memcpy(head, evt, sizeof(radio_event_t));
    
    return ((radio_fifo_head++) & RADIO_FIFO_QUEUE_MASK);
}

static uint32_t radio_fifo_get(radio_event_t* evt)
{
    if (radio_fifo_empty())
    {
        return NRF_ERROR_NULL;
    }
    
    radio_event_t* tail = &radio_fifo_queue[radio_fifo_tail & RADIO_FIFO_QUEUE_MASK];
    
    memcpy(evt, tail, sizeof(radio_event_t));
    
    ++radio_fifo_tail;
    
    return NRF_SUCCESS;
}


static uint32_t radio_fifo_peek_at(radio_event_t* evt, uint8_t offset)
{
    if (radio_fifo_get_length() < offset)
    {
        return NRF_ERROR_NULL;
    }
    
    radio_event_t* tail = &radio_fifo_queue[(radio_fifo_tail + offset) & RADIO_FIFO_QUEUE_MASK];
    
    memcpy(evt, tail, sizeof(radio_event_t));
    
    return NRF_SUCCESS;
}

static uint32_t radio_fifo_peek(radio_event_t* evt)
{
    return radio_fifo_peek_at(evt, 0);
}




static void radio_channel_set(uint8_t ch)
{
    if (ch < 37)
    {
        NRF_RADIO->FREQUENCY = 4 + ch * 2;
    }
    else
    {
        uint32_t adv_freqs[] = {2, 26, 80};
        NRF_RADIO->FREQUENCY = adv_freqs[(ch - 37)];
    }
    
    NRF_RADIO->DATAWHITEIV = ch;
    
}

static bool radio_will_go_to_disabled_state(void)
{
    return (radio_fifo_get_length() == 2 && 
        (NRF_RADIO->EVENTS_READY) && !(NRF_RADIO->EVENTS_END));
}

static void setup_rx_timeout(uint32_t rx_start_time)
{
    rx_abort_timer_index = timer_order_cb_ppi(rx_start_time + RADIO_RX_TIMEOUT, 
        rx_abort_cb, 
        (uint32_t*) &(NRF_RADIO->TASKS_DISABLE));  
}

/**
* One event just finished. Setup next event and propagate an 
* event report to user space
*/
static void radio_transition_end(bool successful_transmission)
{
    /* pop the event that just finished */
    radio_event_t prev_evt;
    radio_fifo_get(&prev_evt);
    bool fly_through_disable = (NRF_RADIO->SHORTS & 
        (RADIO_SHORTS_DISABLED_RXEN_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk));    
    
    current_rx_buf = !current_rx_buf;
    NRF_RADIO->SHORTS = 0;
    
    if (radio_fifo_empty())
    {
        radio_state = RADIO_STATE_DISABLED;
        SET_PIN(PIN_SEARCHING);
    }
    else
    {
        
        /* Take care of the upcoming event */
        
        bool start_manually = false;
        radio_event_t evt;
        radio_fifo_peek(&evt);
        
        uint32_t curr_time = timer_get_timestamp();
        
        if (evt.start_time < curr_time)
        {
            evt.start_time = 0;
        }
        
        if (evt.start_time == 0)
        {
            NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
            
            /* the ready->start shortcut doesn't work when we already are in IDLE */
            if (prev_evt.event_type == evt.event_type)
            {
                start_manually = true;
            }
        }
        else
        {
            timer_order_ppi(evt.start_time, (uint32_t*) &(NRF_RADIO->TASKS_START));
        }
        
        /* setup buffers and addresses */
        if (evt.event_type == RADIO_EVENT_TYPE_RX)
        {
            NRF_RADIO->RXADDRESSES = evt.access_address;
            radio_state = RADIO_STATE_RX;
            
            NRF_RADIO->PACKETPTR = (uint32_t) rx_data[current_rx_buf];
            NRF_RADIO->INTENSET = RADIO_INTENSET_ADDRESS_Msk;
            
            if (evt.start_time != 0)
            {
                setup_rx_timeout(evt.start_time);
            }
            
            /* manually begin ramp up */
            if (!fly_through_disable)
            {
                NRF_RADIO->TASKS_RXEN = 1;
            }
                
        }
        else
        {
            NRF_RADIO->TXADDRESS = evt.access_address;
            radio_state = RADIO_STATE_TX;
            NRF_RADIO->PACKETPTR = (uint32_t) evt.packet_ptr;
            NRF_RADIO->INTENCLR = RADIO_INTENCLR_ADDRESS_Msk;
            
            
            /* manually begin ramp up */
            if (!fly_through_disable)
            {
                NRF_RADIO->TASKS_TXEN = 1;
            }
        }
        
        /* safe to kickstart it now */
        if (start_manually)
        {
            NRF_RADIO->TASKS_START = 1;
        }
            
        
        /* prepare shortcuts for next transmission */
        if (radio_fifo_get_length() == 1)
        {
            NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;
        }
        else
        {
            /* More events after the upcoming one */
            radio_event_t next_evt;
            radio_fifo_peek_at(&next_evt, 1);
            
            if (next_evt.event_type != evt.event_type)
            {
                NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;
                
                /* make shortcut through disabled to accelerate the process */
                if (next_evt.event_type == RADIO_EVENT_TYPE_RX)
                {
                    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
                }
                else /* shortcut to TX */
                {
                    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
                }
            }
        }
    }
    
    if (successful_transmission)
    {
        /* send to super space */
        if (prev_evt.event_type == RADIO_EVENT_TYPE_RX)
        {
            (*g_radio_rx_cb)(rx_data[!current_rx_buf]);
        }
        else
        {
            (*g_radio_tx_cb)();
        }
    }
}

static void rx_abort_cb(void)
{
    DEBUG_PIN_TH(PIN_RADIO_SIGNAL);
    DEBUG_PIN_TH(PIN_RADIO_SIGNAL);
    radio_state = RADIO_STATE_DISABLED;
    
    radio_transition_end(false);
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void radio_init(radio_rx_cb radio_rx_callback, radio_tx_cb radio_tx_callback)
{
	/* Reset all states in the radio peripheral */
	NRF_RADIO->POWER = 1;
	//NRF_RADIO->EVENTS_DISABLED = 0; 
    NVIC_EnableIRQ(RADIO_IRQn);
    
    /* Set radio configuration parameters */
    NRF_RADIO->TXPOWER      = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE 	    = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

    NRF_RADIO->FREQUENCY 	    = 2;					// Frequency bin 2, 2402MHz, channel 37.
    NRF_RADIO->DATAWHITEIV      = 37;					// NOTE: This value needs to correspond to the frequency being used


    /* Configure Access Address to be the BLE standard */
    NRF_RADIO->PREFIX0	    = 0x8e;//0x8e;
    NRF_RADIO->BASE0 		= 0x89bed600; 
    NRF_RADIO->TXADDRESS    = 0x00;			    // Use logical address 0 (prefix0 + base0) = 0x8E89BED6 when transmitting
    NRF_RADIO->RXADDRESSES  = 0x01;				// Enable reception on logical address 0 (PREFIX0 + BASE0)

    /* PCNF-> Packet Configuration. Now we need to configure the sizes S0, S1 and length field to match the datapacket format of the advertisement packets. */
    NRF_RADIO->PCNF0 =  (
                          (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)    // length of S0 field in bytes 0-1.
                        | (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk)    // length of S1 field in bits 0-8.
                        | (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)    // length of length field in bits 0-8.
                      );

    /* Packet configuration */
    NRF_RADIO->PCNF1 =  (
                          (((37UL)                      << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)   // maximum length of payload in bytes [0-255]
                        | (((0UL)                       << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk)	// expand the payload with N bytes in addition to LENGTH [0-255]
                        | (((3UL)                       << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)    // base address length in number of bytes.
                        | (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)   // endianess of the S0, LENGTH, S1 and PAYLOAD fields.
                        | (((1UL)                       << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)	// enable packet whitening
                      );

    /* CRC config */
    NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | 
                            (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); // Skip Address when computing crc     
    NRF_RADIO->CRCINIT = 0x555555;    // Initial value of CRC
    NRF_RADIO->CRCPOLY = 0x00065B;    // CRC polynomial function
 
    /* Lock interframe spacing, so that the radio won't send too soon / start RX too early */
    NRF_RADIO->TIFS = 148;
    
    g_radio_rx_cb = radio_rx_callback;
    g_radio_tx_cb = radio_tx_callback;
    radio_state = RADIO_STATE_DISABLED;
}

void radio_order(radio_event_t* radio_event)
{
    radio_fifo_put(radio_event);
    
    CLEAR_PIN(PIN_SEARCHING);
    
    if (radio_state == RADIO_STATE_DISABLED)
    {
        /* order radio right away */
        
        uint32_t curr_time = timer_get_timestamp();
        
        if (radio_event->start_time < curr_time)
        {
            radio_event->start_time = 0;
        }
        
        if (radio_event->event_type == RADIO_EVENT_TYPE_RX)
        {
            NRF_RADIO->TASKS_RXEN = 1;
            NRF_RADIO->PACKETPTR = (uint32_t) rx_data[current_rx_buf];
            
            /* if the event is not an "as soon as possible", we setup an RX timeout,
                else the user must do it themselves */
            if (radio_event->start_time != 0)
            {
                setup_rx_timeout(radio_event->start_time);
            }
            
            radio_state = RADIO_STATE_RX;
            NRF_RADIO->INTENSET = RADIO_INTENSET_ADDRESS_Msk;
        }
        else
        {
            NRF_RADIO->TASKS_TXEN = 1;
            NRF_RADIO->PACKETPTR = (uint32_t) radio_event->packet_ptr;     
            NRF_RADIO->INTENCLR = RADIO_INTENCLR_ADDRESS_Msk;
            radio_state = RADIO_STATE_TX;
        }
        
        if (radio_event->start_time == 0)
        {
            NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | 
                            RADIO_SHORTS_END_DISABLE_Msk;
        }
        else
        {
            timer_order_ppi(radio_event->start_time, (uint32_t*) &(NRF_RADIO->TASKS_START));
            NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk;
        }
        
        NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
        
    }
    else
    {
        //NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
        /* queue the event */
        
        if (radio_will_go_to_disabled_state())
        {
            /* too late to schedule, wake up upon END, and take the event then. */
            DEBUG_PIN_TH(1);
            DEBUG_PIN_TH(1);
        }
        else
        {
            uint8_t queue_length = radio_fifo_get_length();
            
            if (queue_length == 2)
            {
                /* get current event */
                radio_event_t ev;
                radio_fifo_peek_at(&ev, 1);
                
                /* setup shorts */
                if (ev.event_type == radio_event->event_type)
                {
                    NRF_RADIO->SHORTS &= ~(RADIO_SHORTS_END_DISABLE_Msk);
                }
                else if (radio_event->event_type == RADIO_EVENT_TYPE_RX)
                {
                    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
                }
                else /* going to TX */
                {
                    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
                }
            }
        }   
    }
            
}



void radio_disable(void)
{
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->TASKS_DISABLE = 1;
    radio_state = RADIO_STATE_DISABLED;
}
        
/**
* IRQ handler for radio. Sends the radio around the state machine, ensuring secure radio state changes
*/
void radio_event_handler(void)
{
    switch (radio_state)
    {
        case RADIO_STATE_RX:
            if (RADIO_EVENT(EVENTS_ADDRESS))
            {
                DEBUG_PIN_TH(1);
                timer_abort(rx_abort_timer_index);
            }
            
        case RADIO_STATE_TX:
            if (RADIO_EVENT(EVENTS_END))
            {
                NRF_RADIO->EVENTS_END = 0;
                radio_transition_end(true);
            }
        
            
            break;
        
        case RADIO_STATE_DISABLED:
            
            break;
    }
    
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_PAYLOAD = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    

}
        

