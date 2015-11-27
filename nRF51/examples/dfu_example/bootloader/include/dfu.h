#ifndef DFU_H__
#define DFU_H__

#include <stdint.h>
#include <stdbool.h>

#define SEGMENT_LENGTH          (16)
#define SIGNCHUNK_LENGTH        (32)

#define DATA_SEQ_END            (0xFFFF)

typedef uint16_t segment_t;
typedef uint16_t seq_t;

typedef enum
{
    DFU_PACKET_TYPE_DATA_RSP    = 0xFFFA,
    DFU_PACKET_TYPE_DATA_REQ    = 0xFFFB,
    DFU_PACKET_TYPE_DATA        = 0xFFFC,
    DFU_PACKET_TYPE_STATE       = 0xFFFD,
    DFU_PACKET_TYPE_FWID        = 0xFFFE,
} dfu_packet_type_t;

typedef enum
{
    DFU_TYPE_SD         = 0x01,
    DFU_TYPE_BOOTLOADER = 0x02,
    DFU_TYPE_APP        = 0x03,
} dfu_type_t;

typedef struct __attribute((packed))
{
    uint32_t company_id;
    uint16_t app_id;
    uint32_t app_version;
} app_id_t;

typedef union __attribute((packed))
{
    app_id_t app;
    uint16_t bootloader;
    uint16_t sd;
} id_t;

typedef struct __attribute((packed))
{
    uint16_t packet_type;
    union __attribute((packed))
    {
        struct __attribute((packed))
        {
            uint16_t sd_version;
            uint16_t bootloader_version;
            app_id_t app_id;
        } fwid;
        struct __attribute((packed))
        {
            uint8_t dfu_type    : 3;
            uint8_t ready       : 1;
            uint8_t authority   : 4;
            union __attribute((packed))
            {
                struct __attribute((packed))
                {
                    uint32_t transaction_id;
                    union __attribute((packed))
                    {
                        struct __attribute((packed))
                        {
                            uint16_t requested;
                            uint16_t original;
                            uint32_t MIC;
                        } sd;
                        struct __attribute((packed))
                        {
                            uint16_t requested;
                            uint16_t original;
                            uint32_t MIC;
                        } bootloader;
                        struct __attribute((packed))
                        {
                            app_id_t requested;
                            uint32_t original_version;
                            uint32_t MIC;
                        } app;
                    } id;
                } ready;
                struct __attribute((packed))
                {
                    union __attribute((packed))
                    {
                        struct __attribute((packed))
                        {
                            uint16_t requested;
                            uint16_t original;
                        } sd;
                        struct __attribute((packed))
                        {
                            uint16_t requested;
                            uint16_t original;
                        } bootloader;
                        struct __attribute((packed))
                        {
                            app_id_t requested;
                            uint32_t original_version;
                        } app;
                    } id;
                } req;
            } params;
        } state;
        struct __attribute((packed))
        {
            uint16_t seq;
            uint32_t transaction_id;
            uint32_t start_address;
            uint32_t end_address;
            uint16_t segment_count;
            uint8_t dfu_type    : 3;
            uint8_t ready       : 1;
            uint8_t diff        : 1;
            uint8_t single_bank : 1;
            uint8_t first       : 1;
            uint8_t last        : 1;
        } start;
        struct __attribute((packed))
        {
            uint16_t seq;
            uint32_t transaction_id;
            uint16_t segment_index;
            uint8_t data[SEGMENT_LENGTH];
        } data;
        struct __attribute((packed))
        {
            uint16_t seq;
            uint32_t transaction_id;
            uint16_t offset;
            uint8_t signchunk[SIGNCHUNK_LENGTH];
        } end;
        struct __attribute((packed))
        {
            uint32_t transaction_id;
            uint16_t missing_seq;
        } req_data;
        struct __attribute((packed))
        {
            uint32_t transaction_id;
            uint16_t segment_index;
            uint8_t data[SEGMENT_LENGTH];
        } rsp_data;
    } payload;
} dfu_packet_t;

void dfu_init(void);
void dfu_start(uint32_t* p_start_addr, uint32_t* p_bank_addr, uint16_t segment_count, bool final_transfer);
uint32_t dfu_data(uint32_t p_addr, uint8_t* p_data, uint16_t length);
void dfu_sha256(uint8_t* p_hash);
void dfu_end(void);

#endif /* DFU_H__ */
