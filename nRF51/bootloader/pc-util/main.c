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
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <malloc.h>
#include <stdlib.h>

#define NRF51_PAGE_SIZE       (1024)
#define NRF51_BL_INFO_PAGE    (0x40000 - NRF51_PAGE_SIZE)
#define NRF52_PAGE_SIZE       (4096)
#define NRF52_BL_INFO_PAGE    (0x80000 - NRF52_PAGE_SIZE)

typedef enum
{
    BL_INFO_TYPE_INVALID            = 0x00,
    BL_INFO_TYPE_ECDSA_PUBLIC_KEY   = 0x01,
    BL_INFO_TYPE_VERSION            = 0x02,
    BL_INFO_TYPE_JOURNAL            = 0x03,
    BL_INFO_TYPE_FLAGS              = 0x04,

    BL_INFO_TYPE_SEGMENT_SD         = 0x10,
    BL_INFO_TYPE_SEGMENT_BL         = 0x11,
    BL_INFO_TYPE_SEGMENT_APP        = 0x12,

    BL_INFO_TYPE_LAST8              = 0x7F,
    BL_INFO_TYPE_LAST16             = 0x7FFF,
} bl_info_type_t;

typedef struct
{
    uint32_t start;
    uint32_t size;
} segment_t;

typedef struct __attribute((packed))
{
    uint32_t company_id;
    uint16_t app_id;
    uint32_t app_version;
} app_id_t;

typedef struct __attribute((packed))
{
    uint8_t id;
    uint8_t version;
} bl_id_t;

typedef struct __attribute((packed))
{
    uint16_t sd;
    bl_id_t bootloader;
    app_id_t app;
} fwid_t;

static uint32_t parse_byte_string(char* string, uint8_t* p_dest, uint32_t max_length)
{
    if (string == NULL)
    {
        return 0;
    }
    uint32_t count = 0;
    while (1)
    {
        p_dest[count] = 0;
        for (uint8_t i = 0; i < 2; ++i)
        {
            char c = string[count * 2 + i];
            if (c >= '0' && c <= '9')
            {
                p_dest[count] |= (c - '0') << (4 - i * 4);
            }
            else if (c >= 'A' && c <= 'F')
            {
                p_dest[count] |= (c - 'A' + 10) << (4 - i * 4);
            }
            else if (c >= 'a' && c <= 'f')
            {
                p_dest[count] |= (c - 'a' + 10) << (4 - i * 4);
            }
            else
            {
                return count;
            }
        }
        if (count++ == max_length)
        {
            return max_length;
        }
    }
}

static void missing_entry(char* entry)
{
    printf("Info file missing entry \"%s\", exiting.\n", entry);
    exit(1);
}

static void missing_segment(char* segment_name)
{
    printf("Info file missing segment %s, exiting.\n", segment_name);
    exit(1);
}

static void print_usage(char* exec_name)
{
    printf("Usage: %s <info file> [--nrf52]\n", exec_name);
}

static uint32_t put_info_entry(uint16_t type, uint32_t length, uint8_t* p_data, uint8_t* p_dest)
{
    if (p_data)
    {
        uint32_t i = 0;
        uint32_t actual_length = length + 4;
        if (actual_length & 0x03) actual_length = (actual_length + 4) & 0xFFFFFFFC;
        memset(&p_dest[i + actual_length - 3], 0xFF, 3);
        p_dest[i++] = (actual_length / 4) & 0xFF;
        p_dest[i++] = (actual_length / 4) >> 8;
        p_dest[i++] = (type & 0xFF);
        p_dest[i++] = (type >> 8);
        memcpy(&p_dest[i], p_data, length);
        return actual_length;
    }
    return 0;
}

static void generate_hex_file(char* file_name, uint8_t* data, uint32_t length, uint32_t start_addr)
{
    char bin_file_name[256];
    sprintf(bin_file_name, "%s.bin", file_name);
    char hex_file_name[256];
    sprintf(hex_file_name, "%s.hex", file_name);
    FILE* p_bin_out = fopen(bin_file_name, "wb");
    if (!p_bin_out)
    {
        printf("Failed to open %s, exiting.\n", bin_file_name);
        exit(1);
    }
    fwrite(data, 1, length, p_bin_out);
    fclose(p_bin_out);

    char tohex_str[256];
    sprintf(tohex_str, "objcopy --change-address %#X -I binary -O ihex %s %s", start_addr, bin_file_name, hex_file_name);
    system(tohex_str);
}

static char* get_file_entry(char** p_file_lines, uint32_t lines, char* name)
{
    for (uint32_t i = 0; i < lines; ++i)
    {
        if (strstr(p_file_lines[i], name) == p_file_lines[i])
        {
            char* pC = &p_file_lines[i][0];
            while (*(pC++) != ':');
            while (*pC == ' ') pC++;
            if (*pC == '\n' || *pC == '\r')
            {
                printf("Illegal info file syntax at entry %s (line %d)\n", name, i);
                exit(1);
            }
            return pC;
        }
    }
    return NULL;
}

static uint32_t put_segment(char** lines, uint32_t line_count, bl_info_type_t type, uint8_t* p_dest)
{
    segment_t segment;
    char* entry;
    char start_str[256];
    char size_str[256];
    switch (type)
    {
        case BL_INFO_TYPE_SEGMENT_APP:
            sprintf(start_str, "APP_START");
            sprintf(size_str, "APP_SIZE");
            break;
        case BL_INFO_TYPE_SEGMENT_SD:
            sprintf(start_str, "SD_START");
            sprintf(size_str, "SD_SIZE");
            break;
        case BL_INFO_TYPE_SEGMENT_BL:
            sprintf(start_str, "BL_START");
            sprintf(size_str, "BL_SIZE");
            break;
        default:
            printf("NOT A SEGMENT: %02X\n", type);
            exit(1);
    }

    entry = get_file_entry(lines, line_count, start_str);
    if (!entry) missing_segment(start_str);
    if (sscanf(entry, "%x", &segment.start) == 0) missing_segment(start_str);

    entry = get_file_entry(lines, line_count, size_str);
    if (!entry) missing_segment(size_str);
    if (sscanf(entry, "%x", &segment.size) == 0) missing_segment(size_str);

    return put_info_entry(type, 8, (uint8_t*) &segment, p_dest);
}

static uint32_t create_info(char* p_file_name, uint8_t* p_data_buf)
{
    FILE* p_in = fopen(p_file_name, "r");
    if (!p_in)
    {
        printf("Can't open file %s\n", p_file_name);
        exit(1);
    }
    fseek(p_in, 0L, SEEK_END);
    uint32_t input_size = ftell(p_in);
    rewind(p_in);
    char* input_string = malloc(input_size + 1);
    char* lines[256];
    uint32_t line_count = 0;

    memset(lines, 0, sizeof(char*) * 256);
    uint32_t current = 0;
    while (1)
    {
        if (!lines[current] ||
                (
                 lines[current][0] != '\n' &&
                 lines[current][0] != '\r' &&
                 lines[current][0] != ' ' &&
                 lines[current][0] != '#'
                )
           )
        {
            current = line_count;
            lines[line_count++] = malloc(2048);
        }
        if (fgets(lines[current], 2048, p_in) == NULL)
        {
            break;
        }

    }

    /* create metadata */
    uint32_t i = 0;
    p_data_buf[i++] = 4;
    p_data_buf[i++] = 1;
    p_data_buf[i++] = 8;
    p_data_buf[i++] = 8;

    char* p_entry;
    /* public key */
    p_entry = get_file_entry(lines, line_count, "PUBLIC_KEY");
    if (p_entry)
    {
        uint8_t public_key[64];
        uint32_t key_len = parse_byte_string(p_entry, public_key, 64);
        i += put_info_entry(BL_INFO_TYPE_ECDSA_PUBLIC_KEY, key_len, public_key, &p_data_buf[i]);
    }
    else
    {
        p_entry = get_file_entry(lines, line_count, "Verification key Qx");
        uint8_t public_key[256];
        uint32_t key_len = 0;
        if (p_entry)
        {
            key_len = parse_byte_string(p_entry, public_key, 256);
            p_entry = get_file_entry(lines, line_count, "Verification key Qy");
            key_len += parse_byte_string(p_entry, &public_key[key_len], 256 - key_len);

            if (p_entry)
            {
                i += put_info_entry(BL_INFO_TYPE_ECDSA_PUBLIC_KEY, key_len, public_key, &p_data_buf[i]);
            }
            else
            {
                missing_entry("Verification key Qy");
            }
        }
    }


    i += put_segment(lines, line_count, BL_INFO_TYPE_SEGMENT_APP, &p_data_buf[i]);
    i += put_segment(lines, line_count, BL_INFO_TYPE_SEGMENT_SD, &p_data_buf[i]);
    i += put_segment(lines, line_count, BL_INFO_TYPE_SEGMENT_BL, &p_data_buf[i]);

    /* versions */
    fwid_t id;

    p_entry = get_file_entry(lines, line_count, "COMPANY_ID");
    uint32_t company_id = 0;
    if (!p_entry || sscanf(p_entry, "%llx", &company_id) == 0) missing_entry("COMPANY_ID");
    p_entry = get_file_entry(lines, line_count, "APP_VERSION");
    if (!p_entry || sscanf(p_entry, "%x", &id.app.app_version) == 0) missing_entry("APP_VERSION");
    p_entry = get_file_entry(lines, line_count, "APP_ID");
    if (!p_entry || sscanf(p_entry, "%hx", &id.app.app_id) == 0) missing_entry("APP_ID");
    p_entry = get_file_entry(lines, line_count, "SD_VERSION");
    if (!p_entry || sscanf(p_entry, "%hx", &id.sd) == 0) missing_entry("SD_VERSION");
    p_entry = get_file_entry(lines, line_count, "BL_ID");
    if (!p_entry || sscanf(p_entry, "%hhx", &id.bootloader.id) == 0) missing_entry("BL_ID");
    p_entry = get_file_entry(lines, line_count, "BL_VERSION");
    if (!p_entry || sscanf(p_entry, "%hhx", &id.bootloader.version) == 0) missing_entry("BL_VERSION");
    id.app.company_id = company_id;

    i += put_info_entry(BL_INFO_TYPE_VERSION, 14, (uint8_t*) &id, &p_data_buf[i]);

    /* flags */
    uint8_t flags[4];
    memset(flags, 0xFF, 4);
    i += put_info_entry(BL_INFO_TYPE_FLAGS, 4, flags, &p_data_buf[i]);

    /* end-of-entries */
    memset(&p_data_buf[i], 0xFF, 4);
    p_data_buf[i + 3] = BL_INFO_TYPE_LAST8;

    free(input_string);
    return i + 4;
}

int main(int argc, char** argv)
{
    if (argc < 2 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
    {
        print_usage(argv[0]);
        exit(1);
    }
    uint32_t page_size  = NRF51_PAGE_SIZE;
    uint32_t start_addr = NRF51_BL_INFO_PAGE;
    for (uint32_t i = 0; i < argc; i++)
    {
        if (strcmp(argv[i], "--nrf52") == 0 || strcmp(argv[i], "--NRF52") == 0)
        {
            page_size  = NRF52_PAGE_SIZE;
            start_addr = NRF52_BL_INFO_PAGE;
            break;
        }
    }

    uint8_t* p_data_buf = malloc(page_size);
    generate_hex_file(argv[1], p_data_buf, create_info(argv[1], p_data_buf), start_addr);
    free(p_data_buf);
}

