/* Copyright (c) Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UTILS_H__
#define UTILS_H__

#include <stdint.h>
#include <string.h>

/**
 * @defgroup UTILS Utility functions
 *
 * This library provides implementations of common utility functions.
 *
 * @{
 */

#define LE2BE24(n) (((n << 16) & 0xff0000) | ((n >> 16) & 0xff) | (n & 0x00ff00))
#define BE2LE24(n) LE2BE24(n)

#ifdef __CC_ARM
#define BE2LE16(n) __REV16(n)
#define LE2BE16(n) __REV16(n)
#else
#define BE2LE16(n) (((n << 8) & 0xff00) | ((n >> 8) & 0x00ff))
#define LE2BE16(n) BE2LE16(n)
#endif

#ifndef DEBUG
/**
 * General debug flag.
 *
 * Values:
 * 0. No debug information
 * 1. Only severe (error) information is printed.
 * 2. Warnings and erros printed.
 * 3. All info printed (info, warnings, errors).
 */
#define DEBUG 3
#endif

#ifndef DEBUG_ENABLE_TRACE
/** Enable tracing of function calls.  */
#define DEBUG_ENABLE_TRACE  0
#endif

#ifndef DEBUG_ENABLE_TIMING
/** Enable timing of function calls.  */
#define DEBUG_ENABLE_TIMING 0
#endif

#if DEBUG > 0
    #define __FILENAME__                                                    \
        (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
    #ifdef NRF51
        /* Compiling for target. */
        #include "SEGGER_RTT.h"
        #include "SEGGER_RTT_Conf.h"
        #define __DEBUG_INIT() SEGGER_RTT_Init()
        #define __LOG(term, ...)                                        \
            do {                                                        \
                SEGGER_RTT_printf(term, __VA_ARGS__);                   \
            } while (0)

    #define DEBUG_COLOR_WARN  "\x1b[2;33m"
    #define DEBUG_COLOR_ERROR "\x1b[2;31m"
    #define DEBUG_COLOR_INFO  "\x1b[2;36m"
    #define DEBUG_COLOR_RESET "\x1b[0m"
#else
    /* Compiling for host. */
    #include <stdio.h>
    #define __DEBUG_INIT()
    #define __LOG(term, ...) printf(__VA_ARGS__)

    #define DEBUG_COLOR_WARN  ""
    #define DEBUG_COLOR_ERROR ""
    #define DEBUG_COLOR_INFO  ""
    #define DEBUG_COLOR_BLACK ""
    #endif
#else
    #define __DEBUG_INIT(...)
    #define __LOG(...)
#endif

/* Verbosity levels */
#if DEBUG > 0
    #define __LOG_ERROR(...)                        \
        do {                                        \
            __LOG(0, DEBUG_COLOR_ERROR);            \
            __LOG(0, "Error <%s:%d>: ", __FILENAME__, __LINE__); \
            __LOG(0, __VA_ARGS__);                  \
            __LOG(0, DEBUG_COLOR_RESET);            \
        } while (0)
#else
    #define __LOG_ERROR(...)
#endif

#if DEBUG > 1
     #define __LOG_WARN(...)                         \
         do {                                        \
             __LOG(0, DEBUG_COLOR_WARN);             \
             __LOG(0, "Warning <%s:%d>: ", __FILENAME__, __LINE__);  \
             __LOG(0, __VA_ARGS__);                  \
             __LOG(0, DEBUG_COLOR_RESET);            \
         } while (0)
#else
    #define __LOG_WARN(...)
#endif

#if DEBUG > 2
    #define __LOG_INFO(...)                         \
        do {                                        \
            __LOG(0, DEBUG_COLOR_INFO);             \
            __LOG(0, "Info <%s:%d>:", __FILENAME__, __LINE__);  \
            __LOG(0, __VA_ARGS__);                  \
            __LOG(0, DEBUG_COLOR_RESET);            \
        } while (0)
#else
    #define __LOG_INFO(...)
#endif


/* Extras */
#if DEBUG > 2
#define __LOG_PACKET(p_packet, msg)                                     \
      do {                                                                \
          __LOG(0, DEBUG_COLOR_INFO);                                     \
          __LOG(0, "Packet <%s:%d>: %s\r\n", __FILENAME__, __LINE__, msg); \
          __LOG(0, "\ttype addr_type len Adv. Address len ad_type   data\r\n"); \
                                                                        \
          __LOG(0, "\t %02x"  , p_packet->header.type);                   \
          __LOG(0, "      %u" , p_packet->header.addr_type);              \
          __LOG(0, "     %03u ", p_packet->header.length);                \
          for (int i=0; i < BLE_GAP_ADDR_LEN; ++i)                        \
            __LOG(0, "%02x", p_packet->addr[i]);                        \
          __LOG(0, " %03u", p_packet->payload[0]);                        \
          __LOG(0, "   0x%02x    ", p_packet->payload[1]);                \
                                                                        \
          for (int i=2; i < p_packet->payload[0]+1; ++i)                  \
            __LOG(0, "%02x", p_packet->payload[i]);                     \
          __LOG(0, "\r\n");                                               \
      } while (0) 


#if DEBUG_ENABLE_TRACE
    

    
    
    #define __LOG_TRACE()                           \
        do {                                        \
            __LOG_INFO(1, "%s\r\n", __FUNCTION__);  \
        } while (0)
#else
    #define __LOG_TRACE()
#endif

#if DEBUG_ENABLE_TIMING
    #define DEBUG_TIMING_INIT()                     \
        do {                                        \
            /* TODO: Which timer to use? */         \
        } while (0)

    #define DEBUG_TICK()
    #define DEBUG_TOCK()
#else
    #define DEBUG_TIMING_INIT()
    #define DEBUG_TICK()
    #define DEBUG_TOCK()
#endif

#define __LOG128(msg, n)                        \
    do {                                        \
        __LOG(0, msg);                          \
        __LOG(0, ":");                          \
        for (int i=0; i<16; ++i)                \
            __LOG(0, " %.02x", n[i]);           \
        __LOG(0, "\n");                         \
    } while (0)
#define __LOGX(msg, n, size)                    \
    do {                                        \
        __LOG(0, msg);                          \
        __LOG(0, ":");                          \
        for (int i=0; i<size; ++i)              \
            __LOG(0, " %.02x", n[i]);           \
        __LOG(0, "\n");                         \
    } while (0)
#else
#define __LOG128(...)
#define __LOGX(...)
#endif

/**
 * Reverse memcpy.
 *
 * Writes size bytes from p_src to p_dst in reverse order.
 *
 * @param p_dst Destination address.
 * @param p_src Source address.
 * @param size  Number of bytes to write.
 */
inline void utils_reverse_memcpy(uint8_t * p_dst, const uint8_t * p_src, uint16_t size)
{
    p_src += size;
    while (size--)
    {
        *((uint8_t *) p_dst++) = *((uint8_t *) --p_src);
    }
}

/**
 * Bytewise XOR for an array.
 *
 * XORs size amount of data from p_src1 and p_src2 and stores it in p_dst.
 *
 * @note p_dst may be equal to one or more of the sources.
 *
 * @param p_dst  Destination address.
 * @param p_src1 First source address.
 * @param p_src2 Secound source address.
 * @param size   Number of bytes to XOR.
 */
inline void utils_xor(uint8_t * p_dst, uint8_t const * p_src1, uint8_t const * p_src2, uint16_t size)
{
    while (0 != size)
    {
        size--;
        p_dst[size] = p_src1[size] ^ p_src2[size];
    }
}

/**
 * Left shift an array of bytes one bit.
 *
 * @warning Cannot be done in place.
 *
 * @param p_dst Destination address.
 * @param p_src Source address.
 * @param size  Size of p_dst and p_src.
 */
inline void utils_lshift(uint8_t * p_dst, uint8_t const * p_src, uint16_t size)
{
    uint8_t overflow;

    overflow = 0;
    while (0 != size)
    {
        size--;
        p_dst[size] = p_src[size] << 1;
        p_dst[size] |= overflow;
        overflow = p_src[size] >> 7;
    }
}

/**
 * Pads an array according to AES-CMAC RFC 4493.
 *
 * For an input string x of r-octets, where 0 <= r < 16, the padding
 * function, padding(x), is defined as follows:
 *
 * @verbatim padding(x) = x || 10^i      where i is 128-8*r-1 @endverbatim
 *
 * That is, padding(x) is the concatenation of x and a single '1',
 * followed by the minimum number of '0's, so that the total length is
 * equal to 128 bits.
 *
 * @param p_dst Destination address.
 * @param p_src Source address.
 * @param size  Size of p_dst and p_src arrays.
 */
inline void utils_pad(uint8_t * p_dst, uint8_t const * p_src, uint16_t size)
{
    memcpy(p_dst, p_src, size);
    p_dst[size] = 0x80;

    for (int i = size+1; i < 16; ++i)
        p_dst[i] = 0x00;
}

/**
 * Swaps the endianness of an array.
 *
 * @param p_arr Array address.
 * @param size  Size of array.
 */
inline void utils_swap_endianess(uint8_t * p_arr, uint16_t size)
{
    uint8_t tmp;
    for (uint_fast16_t i = 0; i < size/2; ++i)
    {
        tmp             = p_arr[i];
        p_arr[i]        = p_arr[size-i-1];
        p_arr[size-i-1] = tmp;
    }
}

/**
 * @}
 */
#endif
