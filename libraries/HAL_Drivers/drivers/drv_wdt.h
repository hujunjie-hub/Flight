#ifndef DRV_WDT_H
#define DRV_WDT_H

/* Call from the supervisor while the application heartbeats are fresh. */
void watchdog_keepalive(void);

#endif
