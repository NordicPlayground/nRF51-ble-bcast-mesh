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

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


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
#ifndef _FIFO_H_
#define _FIFO_H_

#include <stdint.h>
#include <stdbool.h>

/* specialized function pointer for copying memory between two instances */
typedef void (*fifo_memcpy)(void* dest, const void* src);

typedef struct
{
  void* elem_array;
  uint32_t elem_size;
  uint32_t array_len;
  uint32_t head;
  uint32_t tail;
  fifo_memcpy memcpy_fptr; /* must be a valid function or NULL */
} fifo_t;

void fifo_init(fifo_t* p_fifo);
uint32_t fifo_push(fifo_t* p_fifo, const void* p_elem);
uint32_t fifo_pop(fifo_t* p_fifo, void* p_elem);
uint32_t fifo_peek_at(fifo_t* p_fifo, void* p_elem, uint32_t elem);
uint32_t fifo_peek(fifo_t* p_fifo, void* p_elem);
void fifo_flush(fifo_t* p_fifo);
uint32_t fifo_get_len(fifo_t* p_fifo);
bool fifo_is_full(fifo_t* p_fifo);
bool fifo_is_empty(fifo_t* p_fifo);



#endif /* _FIFO_H_ */
