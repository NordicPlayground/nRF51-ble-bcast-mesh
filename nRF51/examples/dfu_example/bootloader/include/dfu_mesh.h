#ifndef DFU_MESH_H__
#define DFU_MESH_H__

#include <stdint.h>
#include <stdbool.h>

void dfu_init(void);
void dfu_start(uint32_t* p_start_addr, uint32_t* p_bank_addr, uint16_t segment_count, bool final_transfer);
uint32_t dfu_data(uint32_t p_addr, uint8_t* p_data, uint16_t length);
bool dfu_has_entry(uint32_t* p_addr, uint8_t* p_out_buffer, uint16_t len);
void dfu_sha256(uint8_t* p_hash);
void dfu_end(void);

#endif /* DFU_MESH_H__ */
