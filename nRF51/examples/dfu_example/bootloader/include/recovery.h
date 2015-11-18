#ifndef SEQ_H__
#define SEQ_H__

#include <stdint.h>
#include <stdbool.h>
#include "dfu.h"

#define SEQ_BACKLOG_SIZE    (32)

bool seq_exists(seq_t seg);
bool seq_finished(void);
void seq_found(seq_t seq_num);
void seq_lost(seq_t seq_num);

#endif /* SEQ_H__ */
