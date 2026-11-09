#ifndef EVENT_JOURNAL_H
#define EVENT_JOURNAL_H

#include <stdbool.h>
#include <stdint.h>

struct rt_mtd_nor_device;

enum {
    JOURNAL_BEGIN = 0x040000,
    JOURNAL_END = 0x100000,
    JOURNAL_SECTOR_BYTES = 4096,
    JOURNAL_SLOT_BYTES = 64,
    JOURNAL_DATA_BYTES = 56
};

typedef struct {
    struct rt_mtd_nor_device *device;
    uint64_t generation;
    uint32_t sector;
    uint32_t next_offset;
} EventJournal;

/* The caller owns the single writer and supplies a version 1 FEVT body. */
bool event_journal_open(EventJournal *journal);
bool event_journal_append(EventJournal *journal, const uint8_t body[JOURNAL_DATA_BYTES]);

#endif
