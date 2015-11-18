#include "seq.h"

#define SEQ_INVALID     (0xFFFF)

static seq_t m_seq_backlog[SEQ_BACKLOG_SIZE];
static seq_t m_max_seq;


void seq_init(void)
{
    for (uint32_t i = 0; i < SEQ_BACKLOG_SIZE; ++i)
    {
        m_seq_backlog[i] = SEQ_INVALID;
    }
    m_max_seq = 0;
}

bool seq_exists(seq_t seq)
{
    if (seq > m_max_seq)
    {
        return false;
    }
    for (uint32_t i = 0; i < SEQ_BACKLOG_SIZE; ++i)
    {
        if (m_seq_backlog[i] == seq)
        {
            return false;
        }
    }
    return true;
}

bool seq_finished(void)
{
    for (uint32_t i = 0; i < SEQ_BACKLOG_SIZE; ++i)
    {
        if (m_seq_backlog[i] != SEQ_INVALID)
        {
            return false;
        }
    }
    return true;
}

void seq_found(seq_t seq)
{
    if (seq > m_max_seq)
    {
        m_max_seq = seq;
        return;
    }
    for (uint32_t i = 0; i < SEQ_BACKLOG_SIZE; ++i)
    {
        if (m_seq_backlog[i] == seq)
        {
            m_seq_backlog[i] = SEQ_INVALID;
            return;
        }
    }
}

void seq_lost(seq_t seq)
{
    if (seq > m_max_seq)
    {
        m_max_seq = seq;
    }
    for (uint32_t i = 0; i < SEQ_BACKLOG_SIZE; ++i)
    {
        if (m_seq_backlog[i] == SEQ_INVALID)
        {
            m_seq_backlog[i] = seq;
            return;
        }
    }
    /* If we get here, we've run out of space */
    bootloader_abort();
}

