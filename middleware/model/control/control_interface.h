#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H
#include "ports/port_api.h"
EXTERN_C_BEGIN
/* Run on the 50 Hz worker. Inputs are immutable for this call. It never runs
 * from controller_update; serialize lifecycle with both worker and fast loop.
 * Snapshot publication is nonblocking; calls on the worker itself serialize. */
Status control_outer_update(const ControllerInput *input);
/* Acceptance accounting: the MPC task reports each worker tick so a scenario
 * that never reaches the solver can explain itself. Diagnostics only. */
void mpc_note_worker(uint64_t now_us, int ready);
/* Apply session tuning while the vehicle is disarmed; the runtime gates this. */
Status controller_apply_payload(const ProductControllerConfigPayload *payload);
/* SIM_ONLY defaults, explicitly supplied by a Host/SIL integrator. */
void control_sim_config(ProductControllerConfig *config);
EXTERN_C_END
#endif
