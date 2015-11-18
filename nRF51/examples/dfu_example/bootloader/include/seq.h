#ifndef SEQ_H__
#define SEQ_H__

#include <stdint.h>
#include <stdbool.h>
#include "dfu.h"

#define SEQ_BACKLOG_SIZE    (32)

void seq_init(void);
bool seq_exists(seq_t seq);
bool seq_finished(void);
void seq_lost(seq_t seq);
void seq_found(seq_t seq);

#endif /* SEQ_H__ */
