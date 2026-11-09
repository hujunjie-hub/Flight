#include "systime.h"
#include <stdio.h>
#include <string.h>
static unsigned depth,maximum,locks,unlocks;
static void lock(void *context) { (void)context; locks++; depth++; if(depth>maximum) maximum=depth; }
static void unlock(void *context) { (void)context; unlocks++; depth--; }
static Status tick(uint64_t *value,uint32_t *epoch) { *value=1000u; *epoch=1u; return STATUS_OK; }
int main(void)
{
    TimeConfig config; uint64_t now;
    memset(&config,0,sizeof(config)); config.boot_id=1u; config.time_epoch=1u; config.tick_hz=10000000u;
    systime_set_access_hooks(lock,unlock,0);
    if(systime_init(&config,tick)!=STATUS_OK || clock_now_us(&now)!=STATUS_OK) return 1;
    if(clock_read(0)!=STATUS_INVALID_ARGUMENT) return 2;
    if(time_submit_pps(0)!=STATUS_NOT_READY) return 3;
    if(depth || locks!=unlocks || maximum<2u) return 4;
    /* Invalid unpaired hooks must leave the functioning pair installed. */
    systime_set_access_hooks(lock,0,0);
    if(clock_read(0)!=STATUS_INVALID_ARGUMENT || depth || locks!=unlocks) return 5;
    systime_set_access_hooks(0,0,0);
    if(systime_stop()!=STATUS_OK || depth) return 6;
    puts("time access hooks: recursive calls and error paths balanced");
    return 0;
}
