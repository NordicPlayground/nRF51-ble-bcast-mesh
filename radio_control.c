#include "radio_control.h"
#include "trickle_common.h"
#include "trickle.h"
#include "nrf.h"
#include "nrf_sdm.h"

/** 
* Internal enum denoting radio state. More states will be added 
* later.
*/
typedef enum 
{
    RADIO_STATE_RX,
    RADIO_STATE_TX,
    RADIO_STATE_DISABLED
} radio_state_t;

/**
* Global radio state
*/
static radio_state_t global_state = RADIO_STATE_RX;

/**
* RX buffer
*/
static uint8_t rx_data_buf[256];

/**
* local tx_data pointer
*/
static uint8_t* tx_data;

/**
* RX counter. 
*/
static uint8_t g_consecutive_receives;

/* flag that is set every time the radio is disabled in the midst of an operation */
uint8_t radio_aborted = 0;

static radio_rx_cb g_radio_rx_cb;
static radio_tx_cb g_radio_tx_cb;


void radio_init(radio_rx_cb radio_rx_callback, radio_tx_cb radio_tx_callback)
{
	/* Reset all states in the radio peripheral */
	NRF_RADIO->POWER = 1;
	//NRF_RADIO->EVENTS_DISABLED = 0; 
    NVIC_EnableIRQ(RADIO_IRQn);
    
    /* Set radio configuration parameters */
    NRF_RADIO->TXPOWER      = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE 	    = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

    NRF_RADIO->FREQUENCY 	    = 2;					// Frequency bin 80, 2480MHz, channel 37.
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
    NRF_RADIO->TIFS = 145;
    
    g_radio_rx_cb = radio_rx_callback;
    g_radio_tx_cb = radio_tx_callback;
}


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
                NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
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
    }
    else /* send the radio to a disabled state. May abort TX operation */
    {
        radio_aborted = 1;
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
        
        NRF_RADIO->TASKS_DISABLE = 1;
        NRF_RADIO->PACKETPTR = (uint32_t) rx_data_buf;
        NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    }    
   
    TICK_PIN(4);
}

void radio_disable(void)
{
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->TASKS_DISABLE = 1;
    global_state = RADIO_STATE_DISABLED;
}
        
/**
* IRQ handler for radio. Sends the radio around the state machine, ensuring secure radio state changes
*/
void radio_event_handler(void)
{
    switch (global_state)
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
                TICK_PIN(PIN_RX);
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
}
        

