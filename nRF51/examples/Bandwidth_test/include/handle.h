#ifndef _HANDLE_H__
#define _HANDLE_H__

#include <stdint.h>

#if defined (NRF51)
extern uint32_t node_handle __attribute__((at(0x3F000)));
#endif


#endif

