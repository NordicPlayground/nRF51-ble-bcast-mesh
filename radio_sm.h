#ifndef _RADIO_SM_H__
#define _RADIO_SM_H__

typedef enum 
{
    RADIO_STATE_RX,
    RADIO_STATE_TX
} radio_state_t;

extern radio_state_t global_state;





#endif /* _RADIO_SM_H__ */
