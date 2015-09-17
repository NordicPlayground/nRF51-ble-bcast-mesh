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

#include "mesh_packet.h"

/******************************************************************************
* Static globals
******************************************************************************/
static mesh_packet_t g_packet_pool[MESH_PACKET_POOL_SIZE];
static bool g_packet_alloc_array[MESH_PACKET_POOL_SIZE];
/******************************************************************************
* Interface functions
******************************************************************************/
void mesh_packet_init(void)
{
    for (uint32_t i = 0; i < MESH_PACKET_POOL_SIZE; ++i)
    {
        g_packet_alloc_array[i] = false;
    }
}

bool mesh_packet_acquire(mesh_packet_t** pp_packet)
{
    for (uint32_t i = 0; i < MESH_PACKET_POOL_SIZE; ++i)
    {
        if (!g_packet_alloc_array[i])
        {
            g_packet_alloc_array[i] = true;
            *pp_packet = &g_packet_pool[i];
            return true;
        }
    }
    return false;
}

bool mesh_packet_free(mesh_packet_t* p_packet)
{
    uint32_t index = (((uint32_t) p_packet) - ((uint32_t) &g_packet_pool[0])) / sizeof(mesh_packet_t);
    if (index > MESH_PACKET_POOL_SIZE)
        return false;
    
    g_packet_alloc_array[index] = false;
    return true;
}

void mesh_packet_on_ts_begin(void)
{
    /* do nothing */
}
