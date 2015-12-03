#ifndef TRANSPORT_H__
#define TRANSPORT_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_packet.h"

#define TX_REPEATS_EXPONENTIAL_MAX      (12)
#define TX_REPEATS_INF                  (0xFF)

#define DFU_PACKET_ADV_OVERHEAD         (1 /* adv_type */ + 2 /* UUID */) /* overhead inside adv data */
#define DFU_PACKET_OVERHEAD             (MESH_PACKET_BLE_OVERHEAD + 1 + DFU_PACKET_ADV_OVERHEAD) /* dfu packet total overhead */

typedef enum
{
    TX_INTERVAL_TYPE_EXPONENTIAL,
    TX_INTERVAL_TYPE_REGULAR
} tx_interval_type_t;


typedef void(*rx_cb_t)(mesh_packet_t* p_packet);

void transport_init(rx_cb_t rx_cb, uint32_t access_addr);
bool transport_tx(mesh_packet_t* p_packet, uint8_t repeats, tx_interval_type_t type);
void transport_tx_reset(mesh_packet_t* p_packet);
void transport_tx_abort(mesh_packet_t* p_packet);
void transport_rtc_irq_handler(void);

#endif /* TRANSPORT_H__ */
