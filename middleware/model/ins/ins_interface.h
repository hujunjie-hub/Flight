#ifndef INS_INTERFACE_H
#define INS_INTERFACE_H
#include "ports/port_api.h"
EXTERN_C_BEGIN
/* Optional event drain; reset events never replace the new snapshot epochs. */
Status ins_take_reset_event(StateResetEvent *event);
void ins_sim_config(ProductEstimatorConfig *config);
/* Enable before the owner starts. Runtime meters shadow work in a 400 us /
 * 5 ms server; each indivisible numerical step still needs target WCET proof. */
Status ins_set_replay_budget(uint32_t budget_us, uint32_t period_us);
/* Serialized M3-owner query. A late first position fix is staged, not fused
 * into an unanchored local frame. M4 must authorize the origin rebuild. */
Status ins_get_pending_origin(ContractHeader *candidate,uint32_t *next_origin_id);
EXTERN_C_END
#endif
