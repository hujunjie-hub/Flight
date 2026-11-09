#include "task_config.h"
#include "runtime_internal.h"
#include "board_identity.h"
#include "event_journal/event_journal.h"
#include "mavlink/mavlink_frame.h"
#include <string.h>

static bool store_event(EventJournal *journal, const RuntimeEvent *event, bool boot)
{
    uint8_t body[JOURNAL_DATA_BYTES] = {0};
    memcpy(body, "FEVT", 4);
    wire_u16(body + 4, 1u);
    body[6] = boot ? 1u : 2u;
    body[7] = event->severity;
    wire_u64(body + 8, event->boot_id);
    wire_u64(body + 16, event->time_us);
    wire_u32(body + 24, event->time_epoch);
    wire_u32(body + 28, event->event_id);
    wire_u16(body + 32, event->source_id);
    wire_u16(body + 34, event->reason_namespace);
    wire_u32(body + 36, event->reason_code);
    wire_u32(body + 40, event->authority_epoch);
    wire_u32(body + 44, event->directive_epoch);
    wire_u32(body + 48, event->permit_epoch);
    wire_u32(body + 52, boot ? board_reset_flags() : 0u);
    return event_journal_append(journal, body);
}

static void task_flash_entry(void *parameter)
{
    EventJournal journal;
    ClockInstant clock;
    (void)parameter;
    if (!runtime_nor_writes_allowed() || clock_read(&clock) != STATUS_OK)
        return;
    const RuntimeEvent boot_event = {
        .boot_id = clock.boot_id, .time_us = clock.monotonic_us,
        .time_epoch = clock.time_epoch, .source_id = SOURCE_PLATFORM, .severity = 1u
    };
    bool pending_boot = true;
    while (runtime_nor_writes_allowed())
    {
        bool ready = event_journal_open(&journal);
        if (ready && pending_boot)
        {
            ready = store_event(&journal, &boot_event, true);
            pending_boot = false;
            if (!ready)
            {
                rt_base_t level = rt_hw_interrupt_disable();
                runtime_stats.critical_event_drops++;
                rt_hw_interrupt_enable(level);
            }
        }
        runtime_stats.nor_recording = ready;
        while (ready && runtime_nor_writes_allowed())
        {
            RuntimeEvent event;
            if (runtime_data_take_critical_event(&event) && !store_event(&journal, &event, false))
            {
                rt_base_t level = rt_hw_interrupt_disable();
                runtime_stats.critical_event_drops++;
                rt_hw_interrupt_enable(level);
                ready = false;
            }
            else
                rt_thread_mdelay(TASK_FLASH_PERIOD_MS);
        }
        runtime_stats.nor_recording = false;
        if (!ready)
            runtime_stats.nor_io_errors++;
        rt_thread_mdelay(2000);
    }
}

static uint8_t task_flash_stack[2048] __attribute__((section(".flash_work"), aligned(8)));
const TaskDescriptor task_flash = {
    "flash", task_flash_entry, task_flash_stack, sizeof(task_flash_stack), TASK_FLASH_PRIORITY
};
