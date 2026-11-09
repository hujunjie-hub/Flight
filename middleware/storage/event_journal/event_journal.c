#include "event_journal.h"
#include "mavlink/mavlink_frame.h"
#include <drivers/mtd_nor.h>
#include <rtdevice.h>
#include <string.h>

enum { CRC_OFFSET = JOURNAL_DATA_BYTES, COMMIT_OFFSET = 60, COMMIT_WORD = 0x434f4d4d };

static uint32_t crc32c(const uint8_t *data)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < JOURNAL_DATA_BYTES; ++i)
    {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? UINT32_C(0x82f63b78) : 0u);
    }
    return ~crc;
}

static uint64_t read_le(const uint8_t *data, unsigned length)
{
    uint64_t value = 0;
    while (length)
        value = (value << 8) | data[--length];
    return value;
}

static bool erased(const uint8_t *data, unsigned length)
{
    for (unsigned i = 0; i < length; ++i)
        if (data[i] != 0xffu)
            return false;
    return true;
}

static bool read_exact(EventJournal *journal, uint32_t address, uint8_t *data, unsigned length)
{
    return rt_mtd_nor_read(journal->device, address, data, length) == (rt_ssize_t)length;
}

static uint64_t sector_generation(const uint8_t *data)
{
    if (memcmp(data, "FSEC\1\0\1\0", 8) != 0 ||
        read_le(data + COMMIT_OFFSET, 4) != COMMIT_WORD ||
        read_le(data + CRC_OFFSET, 4) != crc32c(data))
        return 0;
    return read_le(data + 8, 8);
}

static bool commit_slot(EventJournal *journal, uint32_t address, uint8_t *data)
{
    uint8_t check[JOURNAL_SLOT_BYTES];
    wire_u32(data + CRC_OFFSET, crc32c(data));
    wire_u32(data + COMMIT_OFFSET, COMMIT_WORD);
    /* Read back even on a reported write error: the device may have finished. */
    (void)rt_mtd_nor_write(journal->device, address, data, COMMIT_OFFSET);
    if (!read_exact(journal, address, check, sizeof(check)) ||
        memcmp(check, data, COMMIT_OFFSET) != 0 ||
        !erased(check + COMMIT_OFFSET, sizeof(check) - COMMIT_OFFSET))
        return false;
    (void)rt_mtd_nor_write(journal->device, address + COMMIT_OFFSET,
                           data + COMMIT_OFFSET, sizeof(check) - COMMIT_OFFSET);
    return read_exact(journal, address, check, sizeof(check)) &&
           memcmp(check, data, sizeof(check)) == 0;
}

bool event_journal_open(EventJournal *journal)
{
    uint8_t slot[JOURNAL_SLOT_BYTES];
    rt_device_t native = rt_device_find("nor0");
    if (!native || native->type != RT_Device_Class_MTD)
        return false;
    struct rt_mtd_nor_device *device = RT_MTD_NOR_DEVICE(native);
    if (device->block_size != JOURNAL_SECTOR_BYTES ||
        device->block_start > JOURNAL_BEGIN / JOURNAL_SECTOR_BYTES ||
        device->block_end < JOURNAL_END / JOURNAL_SECTOR_BYTES)
        return false;

    *journal = (EventJournal){ .device = device, .next_offset = JOURNAL_SECTOR_BYTES };
    bool ambiguous = false;
    for (uint32_t address = JOURNAL_BEGIN; address < JOURNAL_END; address += JOURNAL_SECTOR_BYTES)
    {
        if (!read_exact(journal, address, slot, sizeof(slot)))
            return false;
        const uint64_t generation = sector_generation(slot);
        if (generation > journal->generation)
        {
            journal->generation = generation;
            journal->sector = address;
            ambiguous = false;
        }
        else if (generation && generation == journal->generation)
            ambiguous = true;
        if ((address / JOURNAL_SECTOR_BYTES) % 8u == 7u)
            rt_thread_mdelay(1);
    }
    if (ambiguous)
        return false;
    if (!journal->generation)
        return true;

    journal->next_offset = JOURNAL_SLOT_BYTES;
    for (uint32_t offset = JOURNAL_SLOT_BYTES; offset < JOURNAL_SECTOR_BYTES;
         offset += JOURNAL_SLOT_BYTES)
    {
        if (!read_exact(journal, journal->sector + offset, slot, sizeof(slot)))
            return false;
        if (!erased(slot, sizeof(slot)))
            journal->next_offset = offset + JOURNAL_SLOT_BYTES;
    }
    return true;
}

static bool check_erased(EventJournal *journal, uint32_t address, bool *blank)
{
    uint8_t page[256];
    *blank = false;
    for (unsigned offset = 0; offset < JOURNAL_SECTOR_BYTES; offset += sizeof(page))
    {
        if (!read_exact(journal, address + offset, page, sizeof(page)))
            return false;
        if (!erased(page, sizeof(page)))
            return true;
    }
    *blank = true;
    return true;
}

static bool next_sector(EventJournal *journal)
{
    uint8_t slot[JOURNAL_SLOT_BYTES];
    uint32_t address = journal->generation ? journal->sector + JOURNAL_SECTOR_BYTES : JOURNAL_BEGIN;
    uint32_t oldest_address = 0;
    uint64_t oldest = journal->generation;
    uint32_t target = 0;
    if (journal->generation == UINT64_MAX)
        return false;
    for (unsigned count = 0; count < (JOURNAL_END - JOURNAL_BEGIN) / JOURNAL_SECTOR_BYTES; ++count)
    {
        if (address == JOURNAL_END)
            address = JOURNAL_BEGIN;
        if (!read_exact(journal, address, slot, sizeof(slot)))
            return false;
        if (erased(slot, sizeof(slot)))
        {
            bool blank;
            if (!check_erased(journal, address, &blank))
                return false;
            if (blank)
            {
                target = address;
                break;
            }
        }
        const uint64_t generation = sector_generation(slot);
        if (generation && generation < oldest)
        {
            oldest = generation;
            oldest_address = address;
        }
        if (count % 8u == 7u)
            rt_thread_mdelay(1);
        address += JOURNAL_SECTOR_BYTES;
    }
    if (!target)
    {
        bool blank;
        target = oldest_address;
        if (!target || rt_mtd_nor_erase_block(journal->device, target, JOURNAL_SECTOR_BYTES) != RT_EOK ||
            !check_erased(journal, target, &blank) || !blank)
            return false;
    }
    memset(slot, 0xff, sizeof(slot));
    memcpy(slot, "FSEC\1\0\1\0", 8);
    wire_u64(slot + 8, journal->generation + 1u);
    if (!commit_slot(journal, target, slot))
        return false;
    journal->sector = target;
    journal->generation++;
    journal->next_offset = JOURNAL_SLOT_BYTES;
    return true;
}

bool event_journal_append(EventJournal *journal, const uint8_t body[JOURNAL_DATA_BYTES])
{
    uint8_t slot[JOURNAL_SLOT_BYTES];
    if (journal->next_offset == JOURNAL_SECTOR_BYTES && !next_sector(journal))
        return false;
    memcpy(slot, body, JOURNAL_DATA_BYTES);
    if (!commit_slot(journal, journal->sector + journal->next_offset, slot))
        return false;
    journal->next_offset += JOURNAL_SLOT_BYTES;
    return true;
}
