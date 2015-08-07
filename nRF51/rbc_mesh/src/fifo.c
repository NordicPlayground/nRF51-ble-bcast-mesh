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
#include "fifo.h"
#include "rbc_mesh_common.h"
#include "nrf_error.h"
#include <string.h>

/*****************************************************************************
* Static functions
*****************************************************************************/
static void* s_fifo_at(fifo_t* p_fifo, uint32_t index)
{
  return ((uint8_t*) p_fifo->elem_array + p_fifo->elem_size * index);
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void fifo_init(fifo_t* p_fifo)
{
    /* round off to nearest(lower) size that can use & operator instead of modulo */
    uint32_t i = 32;
    while (!((p_fifo->array_len >> --i) & 0x01));
    p_fifo->array_len = (1 << i);

    p_fifo->head = 0;
    p_fifo->tail = 0;
}

uint32_t fifo_push(fifo_t* p_fifo, const void* p_elem)
{
    if (fifo_is_full(p_fifo))
    {
        return NRF_ERROR_NO_MEM;
    }

  void* p_dest = s_fifo_at(p_fifo, p_fifo->head & (p_fifo->array_len - 1));

  if (p_fifo->memcpy_fptr)
    p_fifo->memcpy_fptr(p_dest, p_elem);
  else
    memcpy(p_dest, p_elem, p_fifo->elem_size);

  ++p_fifo->head;
  return NRF_SUCCESS;
}

uint32_t fifo_pop(fifo_t* p_fifo, void* p_elem)
{
  if (fifo_is_empty(p_fifo))
  {
    return NRF_ERROR_NULL;
  }

  void* p_src = s_fifo_at(p_fifo, p_fifo->tail & (p_fifo->array_len - 1));

  if (p_fifo->memcpy_fptr)
    p_fifo->memcpy_fptr(p_elem, p_src);
  else
    memcpy(p_elem, p_src, p_fifo->elem_size);

  ++p_fifo->tail;

  return NRF_SUCCESS;
}

uint32_t fifo_peek_at(fifo_t* p_fifo, void* p_elem, uint32_t elem)
{
  if (fifo_get_len(p_fifo) <= elem)
  {
    return NRF_ERROR_NULL;
  }

  void* p_src = s_fifo_at(p_fifo, (p_fifo->tail + elem) & (p_fifo->array_len - 1));

  if (p_fifo->memcpy_fptr)
    p_fifo->memcpy_fptr(p_elem, p_src);
  else
    memcpy(p_elem, p_src, p_fifo->elem_size);

  return NRF_SUCCESS;
}

uint32_t fifo_peek(fifo_t* p_fifo, void* p_elem)
{
  return fifo_peek_at(p_fifo, p_elem, 0);
}

void fifo_flush(fifo_t* p_fifo)
{
  p_fifo->tail = p_fifo->head;
}

uint32_t fifo_get_len(fifo_t* p_fifo)
{
  return (p_fifo->head - p_fifo->tail);
}

bool fifo_is_full(fifo_t* p_fifo)
{
  return (p_fifo->tail + p_fifo->array_len == p_fifo->head);
}

bool fifo_is_empty(fifo_t* p_fifo)
{
  return (p_fifo->tail == p_fifo->head);
}
