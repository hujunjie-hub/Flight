#ifndef ACTUATOR_GUARD_H
#define ACTUATOR_GUARD_H
#include "ports/port_api.h"
EXTERN_C_BEGIN
enum { ACTUATOR_GUARD_DISARMED, ACTUATOR_GUARD_ARMING_IDLE, ACTUATOR_GUARD_ACTIVE, ACTUATOR_GUARD_HOLD_GRACE,
       ACTUATOR_GUARD_SAFE_ACTION, ACTUATOR_GUARD_FAULT_LATCHED };
/* All calls have one serialized owner, except permit publication. The lock
 * protects permit updates and the final check with nonblocking submission.
 * The inhibit and fault counter callbacks must also be nonblocking. */
typedef struct {
  void (*lock)(void);
  void (*unlock)(void);
  void (*inhibit)(void);
  /* Submit four command values in output channel order at a frame boundary. */
  Status (*commit)(const uint16_t commands[4], uint64_t *actual_apply_us);
  uint32_t (*fault_count)(void);
} ActuatorDriver;
Status actuator_guard_set_driver(const ActuatorDriver *driver);
/* Call at the 400 Hz frame boundary, independently of incoming demands. */
Status actuator_guard_frame_boundary(uint64_t now_us);
uint32_t actuator_guard_state(void);
/* HIL adapter support: configured per-motor thrust range in newtons. Read-only;
 * it grants no output capability and never stages or commits a demand. */
Status actuator_guard_get_thrust_limits(float min_n[4], float max_n[4]);
EXTERN_C_END
#endif
