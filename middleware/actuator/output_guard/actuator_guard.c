#include "output_guard/actuator_guard.h"
#include "contract_checks/algorithm_checks.h"
#include <string.h>

static struct {
  ProductActuatorConfig config;
  ActuationPermit permit;
  ActuatorDemand latest;
  ActuatorCommitReceipt receipt;
  ActuatorHealth health;
  ClockInstant clock;
  float last_thrust[4];
  uint64_t last_commit, last_submit_us, last_frame, last_demand;
  uint32_t state, sequence, watermark, demand_sequence, faults;
  uint8_t initialized, has_demand, has_commit, has_frame;
  uint8_t commit_pending, pending_ground_stop, pending_zero;
  uint32_t pending_limit_flags, pending_control_sequence, pending_output_policy_id;
  uint64_t pending_commit_time_us, pending_apply_time_us;
  ActuationPermit pending_permit;
  float pending_thrust[4];
} a;
static ActuatorDriver io;
/* Ground stop transaction. `latched` survives until actuator_init(): once the
 * ramp has confirmed zero thrust the output stays at zero and DISARMED, so a
 * later demand cannot resurrect it without a fresh initialisation. */
static struct {
  uint8_t active, latched, overrun;
  uint32_t stop_code;
  float ramp_nps;
  uint64_t start_us, predicted_us;
} gs;
static void lock(void) { if (io.lock) io.lock(); }
static void unlock(void) { if (io.unlock) io.unlock(); }
static void inhibit(void) { if (io.inhibit) io.inhibit(); }
static int newer(uint32_t x, uint32_t y) { return x != y && (uint32_t)(x-y) < 0x80000000u; }
static int clock_at_or_after(uint64_t now, ClockInstant *clock) {
  return !clock_read(clock) && clock->boot_id == a.clock.boot_id &&
         clock->time_epoch == a.clock.time_epoch && now <= clock->monotonic_us;
}
static void header(ContractHeader *h, uint16_t type, uint64_t now) {
  memset(h, 0, sizeof(*h));
  h->schema_major=CONTRACT_SCHEMA_MAJOR; h->schema_minor=CONTRACT_SCHEMA_MINOR;
  h->message_type=type; h->source_id=SOURCE_M7; h->profile_id=PRODUCT_PROFILE_ID;
  h->boot_id=a.clock.boot_id; h->time_epoch=a.clock.time_epoch;
  h->frame_id=FRAME_BODY_FRD; h->sequence=++a.sequence;
  h->timestamp_sample_us=now; h->timestamp_publish_us=now; h->valid_until_us=now+5000;
  h->timestamp_quality=TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
  h->timestamp_flags=TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED; h->validity_flags=3;
}
static int bound(const ActuationPermit *p, uint64_t now) {
  return header_valid(&p->header,now) && p->header.source_id==SOURCE_M4 &&
    p->header.boot_id==a.clock.boot_id && p->header.time_epoch==a.clock.time_epoch &&
    p->permit_epoch && p->permit_epoch==a.watermark &&
    p->output_policy_id==a.config.payload.output_policy_id &&
    p->permit_expires_at_us>now && p->permit_flags==PERMIT_SCOPE_CONTROL_ACTIVE &&
    p->authority_epoch && p->directive_epoch && p->state_epoch && p->reference_epoch;
}
static Status safe(Status s, uint32_t state) {
  inhibit(); a.has_demand=0; a.state=state;
  a.health.last_error_code=(uint32_t)s;
  a.health.health_state=state==ACTUATOR_GUARD_FAULT_LATCHED ? MODULE_HEALTH_FAILED : MODULE_HEALTH_DEGRADED;
  return s;
}

/* Complete a submitted frame outside the permit critical section. The DShot
 * adapter returns STATUS_NOT_READY until the ISR has observed frame completion;
 * only that completion may update thrust history or publish a commit receipt. */
static Status complete_pending_commit(uint64_t now) {
  uint64_t actual = a.pending_apply_time_us;
  Status r = STATUS_OK;
  uint8_t ground_stop, zero, latch_zero = 0u;

  lock();
  if (!a.commit_pending) {
    unlock();
    return STATUS_OK;
  }
  unlock();

  /* Poll the physical frame outside the permit lock. This callback is nonblocking. */
  if (io.poll_apply) {
    r = io.poll_apply(&actual);
    if (r == STATUS_NOT_READY) return STATUS_NOT_READY;
    if (r != STATUS_OK) {
      lock();
      a.commit_pending = 0u;
      unlock();
      return safe(r, ACTUATOR_GUARD_FAULT_LATCHED);
    }
  }
  if (actual < a.pending_commit_time_us) {
    lock();
    a.commit_pending = 0u;
    unlock();
    return safe(STATUS_STALE_DATA, ACTUATOR_GUARD_FAULT_LATCHED);
  }

  ground_stop = a.pending_ground_stop;
  zero = a.pending_zero;
  lock();
  /* actuator_stop() may have cancelled the pending owner while polling. */
  if (!a.commit_pending) {
    unlock();
    return STATUS_NOT_READY;
  }
  if (ground_stop &&
      (actual < gs.start_us ||
       actual - gs.start_us > ACTUATOR_GROUND_STOP_MAX_APPLY_US))
    gs.overrun = 1u;

  memset(&a.receipt, 0, sizeof(a.receipt));
  header(&a.receipt.header, MSG_ACTUATOR_COMMIT_RECEIPT, now);
  a.receipt.applied_control_sequence = ground_stop ? 0u : a.pending_control_sequence;
  a.receipt.permit_epoch = a.pending_permit.permit_epoch;
  a.receipt.directive_epoch = a.pending_permit.directive_epoch;
  a.receipt.authority_epoch = a.pending_permit.authority_epoch;
  a.receipt.state_epoch = a.pending_permit.state_epoch;
  a.receipt.reference_epoch = a.pending_permit.reference_epoch;
  a.receipt.commit_time_us = a.pending_commit_time_us;
  a.receipt.estimated_apply_time_us = actual;
  a.receipt.receipt_expires_at_us = now + 5000u;
  a.receipt.output_policy_id = (uint16_t)a.pending_output_policy_id;
  a.receipt.limit_flags = a.pending_limit_flags;
  if (ground_stop && gs.overrun)
    a.receipt.limit_flags |= ACTUATOR_GROUND_STOP_OVERRUN_FLAG;
  a.receipt.result_code = ground_stop && zero ?
      ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED : ACTUATOR_RECEIPT_COMMITTED;
  a.receipt.committed_demand.valid_mask = 15u;
  memcpy(a.receipt.committed_demand.motor_thrust_n, a.pending_thrust,
         sizeof(a.pending_thrust));

  memcpy(a.last_thrust, a.pending_thrust, sizeof(a.last_thrust));
  a.last_commit = actual;
  /* Slew is bounded per commanded frame interval. The separate actual apply
   * timestamp remains authoritative for receipts and ground-stop deadlines. */
  a.last_submit_us = a.pending_commit_time_us;
  a.has_commit = 1u;
  a.commit_pending = 0u;
  a.pending_ground_stop = 0u;
  a.pending_zero = 0u;

  if (ground_stop && zero) {
    gs.active = 0u;
    gs.latched = 1u;
    a.has_demand = 0u;
    a.state = ACTUATOR_GUARD_DISARMED;
    a.health.health_state = MODULE_HEALTH_DEGRADED;
    latch_zero = 1u;
  } else if (ground_stop) {
    a.state = ACTUATOR_GUARD_SAFE_ACTION;
    a.health.health_state = MODULE_HEALTH_DEGRADED;
  } else {
    a.state = ACTUATOR_GUARD_ACTIVE;
    a.health.health_state = MODULE_HEALTH_HEALTHY;
  }
  unlock();

  if (latch_zero)
    inhibit();
  return STATUS_OK;
}
Status actuator_guard_set_driver(const ActuatorDriver *d) {
  if (a.initialized || !d || (!!d->lock != !!d->unlock)) return STATUS_INVALID_ARGUMENT;
  io=*d; return STATUS_OK;
}
Status actuator_validate(const ProductProfile *p) {
  return p && p->product_profile_id==PRODUCT_PROFILE_ID &&
    p->actuator_layout_id==PRODUCT_ACTUATOR_LAYOUT_ID ? STATUS_OK : STATUS_PROFILE_MISMATCH;
}
Status actuator_init(const ProductActuatorConfig *c) {
  unsigned m,j,seen=0;
  if (!c || !c->profile_hash || !c->config_version) return STATUS_INVALID_ARGUMENT;
  inhibit(); memset(&a,0,sizeof(a)); memset(&gs,0,sizeof(gs));
  if (clock_read(&a.clock)) return STATUS_NOT_READY;
  /* Registry: H723 supports only BENCH_INHIBIT. SIM_CONTROLLED_STOP is built
   * solely into tests, never selectable by a runtime command on the board. */
  if (c->payload.output_policy_id!=1u
#ifdef FLIGHT_HOST_TEST
      && c->payload.output_policy_id!=2u
#endif
     ) return STATUS_UNSUPPORTED;
  if (c->payload.output_policy_id!=1u) {
    if (!c->calibration_hash || !c->calibration_set_id || !c->payload.calibration_version ||
        (c->payload.esc_bit_rate!=300000 && c->payload.esc_bit_rate!=600000)) return STATUS_INVALID_ARGUMENT;
    for(m=0;m<4;++m) {
      unsigned n=c->payload.thrust_curve_count[m],ch=c->payload.motor_to_timer_channel[m];
      if(n<2 || n>16 || ch<1 || ch>4 || (seen & (1u<<ch))) return STATUS_INVALID_ARGUMENT;
      seen|=1u<<ch;
      if (!isfinite(c->payload.min_thrust_n[m]) || !isfinite(c->payload.max_thrust_n[m]) ||
          !isfinite(c->payload.max_slew_nps[m]) || c->payload.min_thrust_n[m]<0 ||
          c->payload.max_thrust_n[m]<=c->payload.min_thrust_n[m] || c->payload.max_slew_nps[m]<=0)
        return STATUS_INVALID_ARGUMENT;
      for(j=0;j<n;++j) if(!isfinite(c->payload.thrust_curve_n[m][j]) ||
          c->payload.dshot_curve_code[m][j]<48 || c->payload.dshot_curve_code[m][j]>2047 ||
          (j && (c->payload.thrust_curve_n[m][j]<=c->payload.thrust_curve_n[m][j-1] ||
                 c->payload.dshot_curve_code[m][j]<=c->payload.dshot_curve_code[m][j-1])))
        return STATUS_INVALID_ARGUMENT;
      if(c->payload.thrust_curve_n[m][0]>c->payload.min_thrust_n[m] ||
         c->payload.thrust_curve_n[m][n-1]<c->payload.max_thrust_n[m]) return STATUS_INVALID_ARGUMENT;
    }
    if(!io.commit || !io.inhibit) return STATUS_NOT_READY;
  }
  a.config=*c; a.initialized=1; a.state=ACTUATOR_GUARD_DISARMED;
  a.health.health_state=MODULE_HEALTH_DEGRADED; return STATUS_OK;
}
Status actuator_stop(void) {
  inhibit();
  lock();
  a.initialized=0; a.has_demand=0; a.commit_pending=0u;
  a.state=ACTUATOR_GUARD_DISARMED;
  memset(&gs,0,sizeof(gs));
  unlock();
  return STATUS_OK;
}
Status actuator_update_permit(const ActuationPermit *p, uint64_t now) {
  ClockInstant clock;
  if(!a.initialized) return STATUS_NOT_READY;
  if(!p || !clock_at_or_after(now,&clock) || !header_valid(&p->header,now) ||
     p->header.source_id!=SOURCE_M4 || p->header.boot_id!=a.clock.boot_id ||
     p->header.time_epoch!=a.clock.time_epoch || !p->permit_epoch) return STATUS_NOT_PERMITTED;
  lock();
  if(!a.initialized) { unlock(); return STATUS_NOT_READY; }
  /* A ground-stop transaction owns the output until zero is committed and
   * latched. A newer permit must not be able to replace that terminal action. */
  if(gs.active || gs.latched) { unlock(); return STATUS_NOT_PERMITTED; }
  if(a.watermark && p->permit_epoch!=a.watermark && !newer(p->permit_epoch,a.watermark)) {
    unlock(); return STATUS_STALE_DATA;
  }
  /* A revoked generation cannot be resurrected by a late renewal. */
  if(p->permit_epoch==a.watermark && (a.permit.permit_flags & PERMIT_FLAG_REVOKED) &&
     !(p->permit_flags & PERMIT_FLAG_REVOKED)) { unlock(); return STATUS_NOT_PERMITTED; }
  if(p->permit_epoch==a.watermark && !(p->permit_flags & PERMIT_FLAG_REVOKED) &&
     (p->authority_epoch!=a.permit.authority_epoch || p->directive_epoch!=a.permit.directive_epoch ||
      p->state_epoch!=a.permit.state_epoch || p->reference_epoch!=a.permit.reference_epoch ||
      p->output_policy_id!=a.permit.output_policy_id)) { unlock(); return STATUS_NOT_PERMITTED; }
  if(p->permit_epoch==a.watermark &&
     (p->header.sequence<a.permit.header.sequence ||
      p->header.timestamp_publish_us<a.permit.header.timestamp_publish_us)) {
    unlock(); return STATUS_STALE_DATA;
  }
  a.permit=*p; a.watermark=p->permit_epoch;
  if(!bound(p,now)) inhibit();
  unlock();
  return STATUS_OK;
}
static int demand_valid(const ActuatorDemand *d,const ActuationPermit *p,uint64_t now) {
  return bound(p,now) && header_valid(&d->header,now) &&
    same_clock(&d->header,&p->header) && d->header.source_id==SOURCE_M6 &&
    d->header.frame_id==FRAME_BODY_FRD && d->actuator_layout_id==PRODUCT_ACTUATOR_LAYOUT_ID &&
    d->valid_channel_mask==15 && d->demand.valid_mask==15 && d->control_sequence &&
    d->permit_epoch==p->permit_epoch && d->directive_epoch==p->directive_epoch &&
    d->authority_epoch==p->authority_epoch && d->state_epoch==p->state_epoch &&
    d->reference_epoch==p->reference_epoch && d->demand_expires_at_us>now &&
    now-d->header.timestamp_publish_us<=5000 && finite_array(d->demand.motor_thrust_n,4);
}
Status actuator_apply(const ActuatorDemand *d,const ActuationPermit *p,ActuatorStatus *s) {
  ClockInstant c; ActuationPermit current; Status r;
  if(!s || !d || !p) return STATUS_INVALID_ARGUMENT;
  /* The ramp owns every frame once accepted; do not acknowledge new demands
   * while it is in progress, and keep the completed zero latch terminal. */
  if(gs.active) return STATUS_NOT_PERMITTED;
  if(gs.latched) { a.has_demand=0; inhibit(); return STATUS_NOT_PERMITTED; }
  if(!a.initialized || a.state==ACTUATOR_GUARD_FAULT_LATCHED || clock_read(&c)) return STATUS_NOT_READY;
  r=actuator_update_permit(p,c.monotonic_us); if(r) return r;
  lock(); current=a.permit; unlock();
  if(!demand_valid(d,&current,c.monotonic_us)) return safe(STATUS_NOT_PERMITTED,ACTUATOR_GUARD_SAFE_ACTION);
  if(a.demand_sequence && !newer(d->control_sequence,a.demand_sequence)) return STATUS_STALE_DATA;
  a.latest=*d; a.demand_sequence=d->control_sequence; a.last_demand=c.monotonic_us; a.has_demand=1;
  /* Publishing a demand is deliberately not a commit receipt. */
  return actuator_poll(c.monotonic_us,s);
}
/* Map one motor's thrust to its frame code through the configured curve. Shared
 * by the demand path and the ground-stop ramp so there is exactly one mapping. */
static uint16_t thrust_to_code(unsigned m,float thrust_n) {
  const ProductActuatorConfigPayload *c=&a.config.payload;
  unsigned j;
  for(j=1;j<c->thrust_curve_count[m]-1u && thrust_n>c->thrust_curve_n[m][j];++j) {}
  return (uint16_t)lroundf(c->dshot_curve_code[m][j-1]+(thrust_n-c->thrust_curve_n[m][j-1])/
    (c->thrust_curve_n[m][j]-c->thrust_curve_n[m][j-1])*
    (c->dshot_curve_code[m][j]-c->dshot_curve_code[m][j-1]));
}

/* One frame of the ground-stop ramp. Never jumps: every channel moves at most
 * ramp_nps*elapsed toward zero, so the commanded thrust is monotone and the last
 * frame is exactly zero. DISARMED is entered only after that frame commits. */
static Status ground_stop_frame(uint64_t now) {
  uint16_t commands[4]={0,0,0,0}; float applied[4]; uint64_t elapsed,actual; Status r;
  unsigned m; int zero=1;
  if (a.commit_pending) return STATUS_NOT_READY;
  elapsed = (a.has_commit && now>=a.last_submit_us) ? now-a.last_submit_us : 0u;
  {
    float step = gs.ramp_nps*(float)elapsed*1e-6f;
    for(m=0;m<4;++m) {
      float prev = a.has_commit ? a.last_thrust[m] : 0.0f;
      float next = prev-step;
      if(next<0.0f) next=0.0f;
      if(next>prev) next=prev;
      applied[m]=next; if(next>0.0f) zero=0;
      commands[a.config.payload.motor_to_timer_channel[m]-1]=thrust_to_code(m,next);
    }
  }
  if(now < gs.start_us) return STATUS_STALE_DATA;
  if(now-gs.start_us>ACTUATOR_GROUND_STOP_MAX_APPLY_US) gs.overrun=1u;
  lock();
  if (a.commit_pending) { unlock(); return STATUS_NOT_READY; }
  actual=now;
  r=io.commit ? io.commit(commands,&actual) : STATUS_NOT_READY;
  if (r == STATUS_OK) {
    a.commit_pending=1u;
    a.pending_ground_stop=1u;
    a.pending_zero=(uint8_t)zero;
    a.pending_limit_flags=gs.overrun?ACTUATOR_GROUND_STOP_OVERRUN_FLAG:0u;
    a.pending_control_sequence=0u;
    a.pending_output_policy_id=a.config.payload.output_policy_id;
    a.pending_commit_time_us=now;
    a.pending_apply_time_us=actual;
    a.pending_permit=a.permit;
    memcpy(a.pending_thrust,applied,sizeof(applied));
    memset(&a.receipt,0,sizeof(a.receipt));
  }
  unlock();
  if(r) return safe(r,ACTUATOR_GUARD_FAULT_LATCHED);
  r=complete_pending_commit(now);
  return r==STATUS_NOT_READY ? STATUS_OK : r;
}

Status actuator_guard_ground_stop(const GroundStopCertificate *c,uint64_t now) {
  ClockInstant clock; ActuationPermit current;
  float max_thrust=0.0f,worst_us; unsigned m;
  if(!c) return STATUS_INVALID_ARGUMENT;
  if(!a.initialized) return STATUS_NOT_READY;
  if(!clock_at_or_after(now,&clock)) return STATUS_STALE_DATA;
  if(!header_valid(&c->header,now) || c->header.message_type!=MSG_GROUND_STOP_CERTIFICATE ||
     c->header.source_id!=SOURCE_M4 || c->header.boot_id!=a.clock.boot_id ||
     c->header.time_epoch!=a.clock.time_epoch) return STATUS_NOT_PERMITTED;
  /* The certificate is only meaningful while it is still current. */
  if(c->certificate_expires_at_us<=now) return STATUS_STALE_DATA;
  if(c->stop_code!=ACTUATOR_GROUND_STOP_CODE_ZERO) return STATUS_OUT_OF_RANGE;
  if(!(c->ramp_nps>0.0f) || c->ramp_nps>ACTUATOR_GROUND_STOP_MAX_RAMP_NPS) return STATUS_OUT_OF_RANGE;
  /* A stop that is not backed by a landed or disarming lifecycle is refused. */
  if(c->lifecycle!=ACTUATOR_GROUND_STOP_LIFECYCLE_LANDED &&
     c->lifecycle!=ACTUATOR_GROUND_STOP_LIFECYCLE_DISARMING) return STATUS_NOT_PERMITTED;
  /* BENCH_INHIBIT has no driver, so there is nothing to ramp. */
  if(a.config.payload.output_policy_id!=2u) return STATUS_NOT_PERMITTED;
  for(m=0;m<4;++m) if(a.config.payload.max_thrust_n[m]>max_thrust) max_thrust=a.config.payload.max_thrust_n[m];
  if(!(max_thrust>0.0f)) return STATUS_OUT_OF_RANGE;
  worst_us=max_thrust/c->ramp_nps*1e6f+(float)ACTUATOR_GROUND_STOP_FRAME_US+
           (float)ACTUATOR_GROUND_STOP_APPLY_LATENCY_US;
  /* Refuse a rate that cannot meet the apply-time budget from the configured
   * maximum thrust instead of silently overrunning it later. */
  if(worst_us>(float)ACTUATOR_GROUND_STOP_MAX_APPLY_US) return STATUS_OUT_OF_RANGE;

  /* Permit publication may run concurrently with the serialized actuator
   * owner. Read the bound epochs and claim the transaction under the same lock
   * used by permit updates, so no newer permit can slip between the check and
   * the ground-stop latch. */
  lock();
  if(!a.initialized) { unlock(); return STATUS_NOT_READY; }
  current=a.permit;
  if(a.commit_pending) { unlock(); return STATUS_NOT_READY; }
  if(gs.latched) { unlock(); return STATUS_NOT_READY; }
  if(!current.permit_epoch ||
     (current.permit_flags & PERMIT_FLAG_REVOKED)==0u ||
     !c->authority_epoch || !c->directive_epoch ||
     c->authority_epoch!=current.authority_epoch ||
     c->directive_epoch!=current.directive_epoch) {
    unlock();
    return STATUS_NOT_PERMITTED;
  }
  /* Publishers repeat their message every cycle. A repeat of the running
   * transaction must not restart it: moving start_us forward would push the
   * apply-time budget with it and hide an overrun. A different stop code or rate
   * mid-ramp is a new transaction and is refused rather than silently applied. */
  if(gs.active) {
    int same = c->stop_code==gs.stop_code && c->ramp_nps==gs.ramp_nps;
    unlock();
    return same ? STATUS_OK : STATUS_NOT_PERMITTED;
  }
  gs.active=1; gs.latched=0; gs.overrun=0;
  gs.stop_code=c->stop_code; gs.ramp_nps=c->ramp_nps;
  gs.start_us=now; gs.predicted_us=(uint64_t)worst_us;
  a.state=ACTUATOR_GUARD_SAFE_ACTION;
  unlock();
  return STATUS_OK;
}

uint64_t actuator_guard_ground_stop_start_us(void) {
  uint64_t value;
  lock();
  value=gs.active||gs.latched ? gs.start_us : 0u;
  unlock();
  return value;
}
uint64_t actuator_guard_ground_stop_predicted_us(void) {
  uint64_t value;
  lock();
  value=gs.predicted_us;
  unlock();
  return value;
}

Status actuator_guard_frame_boundary(uint64_t now) {
  ActuationPermit p; float applied[4]; uint16_t commands[4]; unsigned m; uint32_t limits=0;
  ClockInstant clock;
  uint64_t actual; Status r;
  if(!a.initialized || a.state==ACTUATOR_GUARD_FAULT_LATCHED) return STATUS_NOT_READY;
  if(!clock_at_or_after(now,&clock)) return safe(STATUS_STALE_DATA,ACTUATOR_GUARD_FAULT_LATCHED);
  r=complete_pending_commit(now);
  if(r==STATUS_NOT_READY) return STATUS_NOT_READY;
  if(r!=STATUS_OK) return r;
  if(a.has_frame && (now<a.last_frame || now-a.last_frame<2500)) return STATUS_DEADLINE_MISSED;
  a.last_frame=now; a.has_frame=1;
  /* A completed ground stop owns the output: nothing else may commit. */
  if(gs.latched) return safe(STATUS_NOT_PERMITTED,ACTUATOR_GUARD_DISARMED);
  if(gs.active) return ground_stop_frame(now);
  if(a.config.payload.output_policy_id==1) return safe(STATUS_NOT_PERMITTED,ACTUATOR_GUARD_DISARMED);
  if(io.fault_count && io.fault_count()!=a.faults) return safe(STATUS_NOT_READY,ACTUATOR_GUARD_FAULT_LATCHED);
  lock(); p=a.permit; unlock();
  if(!a.has_demand || !demand_valid(&a.latest,&p,now) || now-a.last_demand>5000)
    return safe(STATUS_STALE_DATA,ACTUATOR_GUARD_SAFE_ACTION);
  for(m=0;m<4;++m) {
    const ProductActuatorConfigPayload *c=&a.config.payload;
    float lo=c->min_thrust_n[m],hi=c->max_thrust_n[m],x=a.latest.demand.motor_thrust_n[m];
    if(a.has_commit) {
      const uint64_t elapsed=now>=a.last_submit_us ? now-a.last_submit_us : 0;
      float delta=c->max_slew_nps[m]*(float)elapsed*1e-6f;
      lo=fmaxf(lo,a.last_thrust[m]-delta); hi=fminf(hi,a.last_thrust[m]+delta);
    }
    applied[m]=clip(x,lo,hi); if(applied[m]!=x) limits|=1u<<m;
    commands[c->motor_to_timer_channel[m]-1]=thrust_to_code(m,applied[m]);
  }
  /* Recheck against the clock at submission, since frame preparation can
   * be preempted. Permit revocation and the nonblocking driver call serialize. */
  lock();
  if(!clock_at_or_after(now,&clock)) { unlock(); return safe(STATUS_STALE_DATA,ACTUATOR_GUARD_FAULT_LATCHED); }
  if(io.fault_count && io.fault_count()!=a.faults) { unlock(); return safe(STATUS_NOT_READY,ACTUATOR_GUARD_FAULT_LATCHED); }
  now=clock.monotonic_us;
  if(!demand_valid(&a.latest,&a.permit,now) || now-a.last_demand>5000) {
    unlock(); return safe(STATUS_NOT_PERMITTED,ACTUATOR_GUARD_SAFE_ACTION);
  }
  if(a.commit_pending) { unlock(); return STATUS_NOT_READY; }
  actual=now;
  r=io.commit(commands,&actual);
  if(r==STATUS_OK) {
    a.commit_pending=1u;
    a.pending_ground_stop=0u;
    a.pending_zero=0u;
    a.pending_limit_flags=limits;
    a.pending_control_sequence=a.latest.control_sequence;
    a.pending_output_policy_id=a.config.payload.output_policy_id;
    a.pending_commit_time_us=now;
    a.pending_apply_time_us=actual;
    a.pending_permit=p;
    memcpy(a.pending_thrust,applied,sizeof(applied));
    memset(&a.receipt,0,sizeof(a.receipt));
  }
  unlock();
  if(r) return safe(r,ACTUATOR_GUARD_FAULT_LATCHED);
  r=complete_pending_commit(now);
  return r==STATUS_NOT_READY ? STATUS_OK : r;
}
Status actuator_poll(uint64_t now,ActuatorStatus *s) {
  ActuationPermit p;
  ClockInstant clock;
  if(!s) return STATUS_INVALID_ARGUMENT;
  memset(s,0,sizeof(*s)); if(!a.initialized) return STATUS_NOT_READY;
  header(&s->header,MSG_ACTUATOR_STATUS,now); s->actuator_layout_id=PRODUCT_ACTUATOR_LAYOUT_ID;
  if(a.state==ACTUATOR_GUARD_FAULT_LATCHED) { s->fault_mask=15; return STATUS_NOT_READY; }
  if(!clock_at_or_after(now,&clock)) { s->fault_mask=15; return safe(STATUS_STALE_DATA,ACTUATOR_GUARD_FAULT_LATCHED); }
  {
    Status pending_status = complete_pending_commit(now);
    if (pending_status != STATUS_OK && pending_status != STATUS_NOT_READY) {
      s->fault_mask=15;
      return pending_status;
    }
  }
  if(io.fault_count && io.fault_count()!=a.faults) { s->fault_mask=15; return safe(STATUS_NOT_READY,ACTUATOR_GUARD_FAULT_LATCHED); }
  lock(); p=a.permit; unlock();
  if(a.config.payload.output_policy_id==1) { inhibit(); a.state=ACTUATOR_GUARD_DISARMED; }
  else {
    s->available_mask=15;
    if(!bound(&p,now) || (a.has_demand && !demand_valid(&a.latest,&p,now)))
      (void)safe(STATUS_STALE_DATA,ACTUATOR_GUARD_SAFE_ACTION);
  }
  /* ESC_FEEDBACK_CAPABILITY=NONE: both feedback validity masks stay zero. */
  return STATUS_OK;
}
Status actuator_get_capability(ActuatorCapability *c) {
  ClockInstant now; if(!c) return STATUS_INVALID_ARGUMENT;
  memset(c,0,sizeof(*c)); if(!a.initialized || clock_read(&now)) return STATUS_NOT_READY;
  header(&c->header,MSG_ACTUATOR_CAPABILITY,now.monotonic_us); c->actuator_layout_id=PRODUCT_ACTUATOR_LAYOUT_ID;
  if(a.config.payload.output_policy_id!=1 && a.state!=ACTUATOR_GUARD_FAULT_LATCHED) {
    c->available_mask=15; c->controllable_dof_mask=15;
    memcpy(&c->vehicle_extension.min_thrust_n,a.config.payload.min_thrust_n,sizeof(float)*4);
    memcpy(&c->vehicle_extension.max_thrust_n,a.config.payload.max_thrust_n,sizeof(float)*4);
    memcpy(&c->vehicle_extension.max_slew_nps,a.config.payload.max_slew_nps,sizeof(float)*4);
  }
  return STATUS_OK;
}
Status actuator_get_health(ActuatorHealth *h) { if(!h) return STATUS_INVALID_ARGUMENT; *h=a.health; return a.initialized?STATUS_OK:STATUS_NOT_READY; }
/* HIL adapter support. The thrust-curve range is configuration, not output
 * capability: this call grants nothing, stages nothing and commits nothing.
 * It exists because the BENCH_INHIBIT capability intentionally reports a zero
 * available mask, while a HIL run still has to normalize motor[0..3] demand. */
Status actuator_guard_get_thrust_limits(float min_n[4],float max_n[4]) {
  unsigned m;
  if(!min_n || !max_n) return STATUS_INVALID_ARGUMENT;
  if(!a.initialized) return STATUS_NOT_READY;
  for(m=0;m<4;++m) { min_n[m]=a.config.payload.min_thrust_n[m]; max_n[m]=a.config.payload.max_thrust_n[m]; }
  return STATUS_OK;
}
Status actuator_get_commit_receipt(ActuatorCommitReceipt *r) {
  if(!r) return STATUS_INVALID_ARGUMENT;
  *r=a.receipt;
  return a.receipt.header.sequence ? STATUS_OK : STATUS_NOT_READY;
}
uint32_t actuator_guard_state(void) { return a.state; }
