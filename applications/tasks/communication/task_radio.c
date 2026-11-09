#include "task_comm.h"
#include "telemetry_link/telemetry_link.h"

void comm_radio_receive(void)
{
    telemetry_link_poll_receive(comm_gcs_receive);
}

void comm_radio_transmit(void)
{
    telemetry_link_poll_transmit();
}
