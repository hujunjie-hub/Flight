#ifndef FMT_SIM_HOST_H
#define FMT_SIM_HOST_H

#include "mavlink/mavlink_hil.h"

/* The Windows Host fixture owns one loopback connection. */
int fmt_sim_host_open(uint16_t port);
void fmt_sim_host_close(void);
unsigned fmt_sim_host_receive(mavlink_fmt_state_t *state);
int fmt_sim_host_send(uint64_t now_us, const float controls[4], int armed);
uint64_t fmt_sim_host_now_us(void);
void fmt_sim_host_sleep(void);

#endif
