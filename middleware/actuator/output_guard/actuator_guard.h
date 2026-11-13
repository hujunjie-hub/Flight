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
  /* Nonblocking submission under the permit lock; never wait for DMA/ISR. */
  Status (*commit)(const uint16_t commands[4], uint64_t *actual_apply_us);
  /* Nonblocking completion poll. STATUS_NOT_READY means the submitted frame is in flight. */
  Status (*poll_apply)(uint64_t *actual_apply_us);
  uint32_t (*fault_count)(void);
} ActuatorDriver;
Status actuator_guard_set_driver(const ActuatorDriver *driver);
/* Call at the 400 Hz frame boundary, independently of incoming demands. */
Status actuator_guard_frame_boundary(uint64_t now_us);
uint32_t actuator_guard_state(void);
/* Ground stop: accept a certificate that authorises a bounded ramp to zero
 * thrust, then apply it at the following frame boundaries and DISARM once zero
 * is confirmed. Returns STATUS_OK when the transaction is accepted, not when the
 * ramp has finished; poll actuator_get_commit_receipt() for
 * ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED. */
Status actuator_guard_ground_stop(const GroundStopCertificate *certificate,
                                  uint64_t now_us);
/* Evidence for the acceptance gate: when the transaction started and how long
 * the ramp is allowed to take. Zero when no ground stop has been accepted. */
uint64_t actuator_guard_ground_stop_start_us(void);
uint64_t actuator_guard_ground_stop_predicted_us(void);
/* HIL adapter support: configured per-motor thrust range in newtons. Read-only;
 * it grants no output capability and never stages or commits a demand. */
Status actuator_guard_get_thrust_limits(float min_n[4], float max_n[4]);
EXTERN_C_END
#endif
