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

#define RADIO_RX_TIMEOUT                (80)


#define RADIO_EVENT(evt)  (NRF_RADIO->#evt == 1)



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
static uint8_t* tx_data;

static radio_rx_cb g_radio_rx_cb;
static radio_tx_cb g_radio_tx_cb;


static radio_event_t radio_fifo_queue[RADIO_FIFO_QUEUE_SIZE];

static uint8_t radio_fifo_head;
static uint8_t radio_fifo_tail;


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
    return (radio_fifo_get_length() == 1 && 
        (NRF_RADIO->STATE == RADIO_STATE_STATE_Rx || 
        NRF_RADIO->STATE == RADIO_STATE_STATE_Tx));
}

static void setup_rx_timeout(uint32_t rx_start_time)
{
    uint8_t timer_index = timer_order_cb_ppi(rx_start_time + RADIO_RX_TIMEOUT, 
        rx_abort_cb, 
        (uint32_t*) &(NRF_RADIO->TASKS_STOP));
    
    /* setup a PPI channel to stop the timeout if the radio's address event fires */
    NRF_PPI->CHG[0] = (1 << (TIMER_PPI_CH_START + timer_index));
    NRF_PPI->TASKS_CHG[0].EN = 1;
    NRF_RADIO->EVENTS_ADDRESS = 0;
    
    /* Setup PPI */
	NRF_PPI->CH[TIMER_PPI_CH_START + 4].EEP   = (uint32_t) &(NRF_RADIO->EVENTS_ADDRESS);
    NRF_PPI->CH[TIMER_PPI_CH_START + 4].TEP   = (uint32_t) &(NRF_PPI->TASKS_CHG[0].DIS);
	NRF_PPI->CHENSET 			             |= (1 << (TIMER_PPI_CH_START + 4));
    
}

static void radio_transition_end(bool successful_transmission)
{
    /* pop the event that just finished */
    radio_event_t prev_evt;
    radio_fifo_get(&prev_evt);
            
    current_rx_buf = !current_rx_buf;
    NRF_RADIO->SHORTS = 0;
    
    if (radio_fifo_empty())
    {
        radio_state = RADIO_STATE_DISABLED;
    }
    else
    {
        /* Take care of the upcoming event */
        radio_event_t evt;
        radio_fifo_peek(&evt);
        
        
        if (evt.start_time == 0)
        {
            NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
        }
        else
        {
            timer_order_ppi(evt.start_time, (uint32_t*) &(NRF_RADIO->TASKS_START));
        }
        
        
        if (evt.event_type == RADIO_EVENT_TYPE_RX)
        {
            NRF_RADIO->RXADDRESSES = evt.access_address;
            radio_state = RADIO_STATE_RX;
            
            NRF_RADIO->PACKETPTR = (uint32_t) rx_data[current_rx_buf];
            
            if (evt.start_time != 0)
            {
                setup_rx_timeout(evt.start_time);
            }
        }
        else
        {
            NRF_RADIO->TXADDRESS = evt.access_address;
            radio_state = RADIO_STATE_TX;
            NRF_RADIO->PACKETPTR = (uint32_t) evt.packet_ptr;
        }
        
        
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
    NRF_RADIO->PREFIX0	    = 0x1e;//0x8e;
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
    NRF_RADIO->TIFS = 145;
    
    g_radio_rx_cb = radio_rx_callback;
    g_radio_tx_cb = radio_tx_callback;
    radio_state = RADIO_STATE_DISABLED;
}

void radio_order(radio_event_t* radio_event)
{
    radio_fifo_put(radio_event);
    
    if (radio_state == RADIO_STATE_DISABLED)
    {
        /* order radio right away */
        if (radio_event->event_type == RADIO_EVENT_TYPE_RX)
        {
            NRF_RADIO->TASKS_RXEN = 1;
            NRF_RADIO->PACKETPTR = (uint32_t) rx_data_buf;
            
            /* if the event is not an "as soon as possible", we setup an RX timeout,
                else the user must do it themselves */
            if (radio_event->start_time != 0)
            {
                setup_rx_timeout(radio_event->start_time);
            }
            
            radio_state = RADIO_STATE_RX;
        }
        else
        {
            NRF_RADIO->TASKS_TXEN = 1;
            NRF_RADIO->PACKETPTR = (uint32_t) radio_event->packet_ptr;       
            radio_state = RADIO_STATE_TX;
        }
        
        if (radio_event->start_time == 0)
        {
            //NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk;
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
        /* queue the event */
        
        if (radio_will_go_to_disabled_state())
        {
            /* too late to schedule, wake up upon END, and take the event then. */
            NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
        }
        else
        {
            uint8_t queue_length = radio_fifo_get_length();
            
            if (queue_length == 2)
            {
                /* get current event */
                radio_event_t ev;
                radio_fifo_peek(&ev);
                
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


#if 0

void radio_tx(uint8_t* data)
{   
    tx_data = data;
    
    switch (global_state)
    {
        case RADIO_STATE_RX:
            global_state = RADIO_STATE_TX; /* set state to TX, in order to catch the radio as it passes through disabled */
            NRF_RADIO->SHORTS = RADIO_SHORTS_DISABLED_TXEN_Msk | 
                                RADIO_SHORTS_READY_START_Msk | 
                                RADIO_SHORTS_END_DISABLE_Msk;
            
            /* wake up at disable state to change packet pointer and shorts */
            NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
            
            /* ready to start state change */
            NRF_RADIO->TASKS_DISABLE = 1; 
        
            NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;

            break;
        
        case RADIO_STATE_DISABLED:
            if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled || NRF_RADIO->STATE == RADIO_STATE_STATE_TxDisable)
            {                    
                NRF_RADIO->EVENTS_DISABLED = 0;
                NRF_RADIO->TASKS_TXEN = 1;
                NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk;
                NRF_RADIO->PACKETPTR = (uint32_t) &tx_data[0];
                NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
                
            }
            else /* In the middle of a TX */
            {
                APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE); /* treat as error, should not happen.. */
            }
            break;
        
        case RADIO_STATE_TX:
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE); /* treat as error, should not happen.. */
            break;
    }
    
    global_state = RADIO_STATE_TX;
}
                
                
            
void radio_rx(uint8_t consecutive_receives)
{
    g_consecutive_receives = consecutive_receives;
    global_state = RADIO_STATE_RX;

    if (true || NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled)
    {
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
        NRF_RADIO->PACKETPTR = (uint32_t) rx_data_buf;
        NRF_RADIO->EVENTS_READY = 0;
        NRF_RADIO->TASKS_RXEN = 1;
        NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
        NRF_RADIO->EVENTS_END = 0;
        NRF_RADIO->EVENTS_ADDRESS = 0;
    }
    else /* send the radio to a disabled state. May abort TX operation */
    {
        radio_aborted = 1;
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
        
        NRF_RADIO->TASKS_DISABLE = 1;
        NRF_RADIO->PACKETPTR = (uint32_t) rx_data_buf;
        NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    }    
   
    DEBUG_PIN_TH(4);
}

#endif

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
    
    
    
#if 0    
    switch (radio_state)
    {
        case RADIO_STATE_RX:
            /* change shortcut and set packet pointer*/
            if (NRF_RADIO->EVENTS_DISABLED)
            {
                NRF_RADIO->EVENTS_DISABLED = 0;
                NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
                NRF_RADIO->PACKETPTR = (uint32_t) &rx_data_buf[0];
                NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
                NRF_RADIO->INTENCLR = RADIO_INTENCLR_DISABLED_Msk;
                NRF_RADIO->EVENTS_READY = 0;
            }
            /* analyze incoming msg */
            if (NRF_RADIO->EVENTS_END)
            {
                DEBUG_PIN_TH(PIN_RX);
                NRF_RADIO->EVENTS_END = 0;
                
                /* propagate receive to RX callback function */
                (*g_radio_rx_cb)(rx_data_buf);   
                
                /* check if the radio is scheduled to receive any more messages, 0 denotes "receive indefinitely" */
                if (g_consecutive_receives == 1)
                {
                    radio_disable();
                }
                else 
                {
                    /* reenable rx */
                    NRF_RADIO->SHORTS = 0;
                    NRF_RADIO->TASKS_START = 1;
                    
                    if (g_consecutive_receives > 1)
                    {
                        --g_consecutive_receives;
                    }
                }
            }
            
            break;
            
        case RADIO_STATE_TX:
            /* change shortcut */
            if (NRF_RADIO->EVENTS_DISABLED)
            {
                NRF_RADIO->EVENTS_DISABLED = 0;
                NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
                NRF_RADIO->PACKETPTR = (uint32_t) &tx_data[0];
                NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk; /* wake when done sending to change the packet pointer back */
                NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Msk;
                global_state = RADIO_STATE_RX;
            }
            if (NRF_RADIO->EVENTS_END)
            {
                NRF_RADIO->EVENTS_END = 0;
                
                /* send ended, notify user. */
                global_state = RADIO_STATE_DISABLED;
                NRF_RADIO->SHORTS = 0; 
                (*g_radio_tx_cb)();
            }
            
            break;
        case RADIO_STATE_DISABLED:
            /* Doesn't have any related interrupts, treat as error. */
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
    }
#endif    
}
        

