/* Navigation task: 1 kHz INS owner.
 * The IMU publish event or 1 ms timeout triggers one navigation and
 * fast-control step. */
#include <rtthread.h>
#include "task_config.h"
#include "runtime_internal.h"
#include "calibration/sensor_calibration_service.h"
#include "sample_acquisition/sensor_input.h"
#include "source_health/sensor_integrity.h"
#include "ins_interface.h"
#include "messages/message_ids.h"
#include <string.h>

static const sensor_imu_config_t *imu_config;
static const ProductEstimatorConfigPayload *estimator_config;
static ControlBundle fast_bundle;
static uint32_t fast_bundle_revision;
static NavigationCapability fast_nav_cap;
static ControllerOutput controller_output;
static si_manager_t integrity;
static SensorUsePolicy policies[4];
static uint32_t consistency_sequence[4];
static uint16_t consistency_count[4], consistency_rejected[4], consistency_accepted[4];
static uint32_t sensor_rejects[4], sensor_gaps[4];
static sensor_context_t imu_context;
static uint64_t policy_refresh;

static void local_header(ContractHeader *h,unsigned source,unsigned type,uint64_t now) {
  ClockInstant c; memset(h,0,sizeof(*h));
  if(clock_read(&c)) return;
  h->schema_major=CONTRACT_SCHEMA_MAJOR; h->schema_minor=CONTRACT_SCHEMA_MINOR;
  h->profile_id=PRODUCT_PROFILE_ID; h->boot_id=c.boot_id; h->time_epoch=c.time_epoch;
  h->source_id=(uint16_t)source; h->message_type=(uint16_t)type;
  h->timestamp_sample_us=now; h->timestamp_publish_us=now; h->valid_until_us=now+200000;
  h->frame_id=FRAME_LOCAL_NED; h->validity_flags=3;
  h->timestamp_quality=TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
  h->timestamp_flags=TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
}
static void refresh_policies(uint64_t now) {
  unsigned i; si_state_t previous[4];
  for(i=0;i<4;++i) previous[i]=si_state(&integrity,i+1);
  (void)si_tick(&integrity,now);
  for(i=0;i<4;++i) {
    SensorUsePolicy *p=&policies[i];
    si_policy_t state=si_policy(&integrity,i+1);
    if(previous[i]!=SI_SOURCE_ISOLATED && si_state(&integrity,i+1)==SI_SOURCE_ISOLATED)
      sensor_gaps[i]++;
    unsigned next=state==SI_POLICY_USABLE ? SENSOR_USE_NORMAL :
                  state==SI_POLICY_DEGRADED ? SENSOR_USE_DEGRADED : SENSOR_USE_REJECT;
    if(p->use_state!=next) {
      p->use_state=(uint16_t)next;p->policy_epoch++;p->effective_time_us=now;
      consistency_count[i]=consistency_rejected[i]=consistency_accepted[i]=0;
    }
    local_header(&p->header,SOURCE_HEALTH,MSG_SENSOR_USE_POLICY,now);
    p->degraded_noise_profile_id=next==SENSOR_USE_DEGRADED ? 1 : 0;
    p->evaluation_flags=(i!=0 && next==SENSOR_USE_REJECT) ? SENSOR_USE_EVAL_ALLOW_SHADOW : 0;
    p->policy_expires_at_us=p->header.valid_until_us;
    (void)estimator_set_sensor_policy(p,now);
  }
}
static void publish_sensor_health(uint64_t now) {
  static const unsigned logical_source[RUNTIME_SENSOR_COUNT]={SOURCE_IMU,SOURCE_MAG,SOURCE_BARO,SOURCE_GNSS};
  RuntimeSensorHealth snapshot[RUNTIME_SENSOR_COUNT];
  unsigned changed=0u;
  for(unsigned i=0;i<RUNTIME_SENSOR_COUNT;++i) {
    const si_source_t *s=&integrity.sources[i]; RuntimeSensorHealth h={0};
    h.sensor_instance_id=i+1;h.policy_epoch=policies[i].policy_epoch;
    h.use_state=policies[i].use_state;h.reject_count=sensor_rejects[i];h.gap_count=sensor_gaps[i];
    h.health_state=s->state==SI_SOURCE_ISOLATED ? 5 : s->state==SI_SOURCE_PROBATION ? 7 :
      s->state==SI_SOURCE_SUSPECT ? (s->recovery_degraded ? 4 : 3) : s->last_valid_us ? 2 : 1;
    h.reason_code=h.health_state==1 ? 2 : h.health_state==2 ? 0 :
      s->last_valid_us && now>=s->last_valid_us && now-s->last_valid_us>integrity.config.timeout_us ? 17 : 11;
    if(i==1 && !(estimator_config->aiding_mask&PRODUCT_AIDING_MAG)) {
      h.health_state=4;h.reason_code=19;h.use_state=SENSOR_USE_REJECT;
    }
    snapshot[i]=h;
  }
  if(!runtime_data_lock(0)) return;
  for(unsigned i=0;i<RUNTIME_SENSOR_COUNT;++i) {
    const RuntimeSensorHealth *old=&runtime_data.sensors[i];
    if(old->health_state!=snapshot[i].health_state || old->use_state!=snapshot[i].use_state)
      changed|=1u<<i;
    runtime_data.sensors[i]=snapshot[i];
  }
  runtime_data_unlock();
  for(unsigned i=0;i<RUNTIME_SENSOR_COUNT;++i) {
    const RuntimeSensorHealth *h=&snapshot[i];
    if(changed&(1u<<i))
      runtime_data_record_event(logical_source[i],0,h->reason_code,h->health_state>=5 ? 2 : h->reason_code ? 1 : 0,now);
  }
}
static void consume_consistency(uint64_t now) {
  unsigned i;
  for(i=1;i<4;++i) {
    SensorConsistencyEvidence e;
    uint16_t old_rejected=consistency_rejected[i],old_accepted=consistency_accepted[i];
    if(estimator_get_sensor_consistency(i+1,&e) || e.header.sequence==consistency_sequence[i] ||
       !runtime_data_header_is_current(&e.header,now) || e.policy_epoch!=policies[i].policy_epoch ||
       e.state_epoch!=fast_bundle.nav.state_epoch ||
       e.calibration_version!=estimator_config->calibration_version ||
       e.evidence_expires_at_us<=now || (e.evidence_flags&15u)!=15u) continue;
    if(e.evaluated_count<consistency_count[i]) old_rejected=old_accepted=0;
    consistency_sequence[i]=e.header.sequence; consistency_count[i]=e.evaluated_count;
    consistency_rejected[i]=e.rejected_count; consistency_accepted[i]=e.accepted_count;
    if(e.rejected_count>=old_rejected) sensor_rejects[i]+=e.rejected_count-old_rejected;
    if(e.evaluation_mode==2 && policies[i].use_state==SENSOR_USE_REJECT) {
      (void)si_report_shadow(&integrity,i+1,e.rejected_count==old_rejected &&
          e.accepted_count>old_accepted,e.header.sequence,now);
    } else if(e.evaluation_mode==1 && policies[i].use_state!=SENSOR_USE_REJECT) {
      if(e.rejected_count>old_rejected) (void)si_report(&integrity,i+1,SI_EVIDENCE_SAMPLE_REJECTED,now);
      else if(e.accepted_count>old_accepted) (void)si_report(&integrity,i+1,SI_EVIDENCE_SAMPLE_ACCEPTED,now);
    }
  }
}

void task_sensor_prepare(const sensor_imu_config_t *imu, const ProductEstimatorConfigPayload *estimator,
                             const ClockInstant *clock)
{
    static const si_config_t integrity_config = {3, 10, 20, 2000000, 200000, 2000000};
    imu_config = imu;
    estimator_config = estimator;
    memset(&fast_bundle, 0, sizeof(fast_bundle));
    fast_bundle_revision = 0u;
    memset(&fast_nav_cap, 0, sizeof(fast_nav_cap));
    memset(&controller_output, 0, sizeof(controller_output));
    memset(policies, 0, sizeof(policies));
    memset(consistency_sequence, 0, sizeof(consistency_sequence));
    memset(consistency_count, 0, sizeof(consistency_count));
    memset(consistency_rejected, 0, sizeof(consistency_rejected));
    memset(consistency_accepted, 0, sizeof(consistency_accepted));
    memset(sensor_rejects, 0, sizeof(sensor_rejects));
    memset(sensor_gaps, 0, sizeof(sensor_gaps));
    memset(&imu_context, 0, sizeof(imu_context));
    (void)si_init(&integrity, &integrity_config);
    for (unsigned i = 0; i < 4; ++i)
    {
        (void)si_register(&integrity, i + 1u, i == 0u);
        policies[i].sensor_instance_id = i + 1u;
        policies[i].policy_epoch = 1u;
        policies[i].effective_time_us = clock->monotonic_us;
    }
    imu_context.boot_id = clock->boot_id;
    imu_context.time_epoch = clock->time_epoch;
    imu_context.profile_id = PRODUCT_PROFILE_ID;
    imu_context.sensor_instance_id = 1u;
    imu_context.sequence = 1u;
    refresh_policies(clock->monotonic_us);
    policy_refresh = 0u;
    (void)ins_set_replay_budget(400, 5000);
}

static void task_sensor_step(void) {
  unsigned budget; uint64_t now;

  now=runtime_now_us();
  runtime_data_mark_heartbeat(RUNTIME_OWNER_SENSOR,now);
  {
    EstimatorResetRequest reset; int have=0;
    if(runtime_data_lock(0)) {
      if(runtime_data.have_origin_reset) { reset=runtime_data.origin_reset; runtime_data.have_origin_reset=0; have=1; }
      runtime_data_unlock();
    }
    if(have) {
      runtime_stats.last_error=estimator_reset(&reset);
      runtime_data_record_event(SOURCE_M3,0,runtime_stats.last_error ? (unsigned)-runtime_stats.last_error : 13,
          runtime_stats.last_error ? 1 : 0,now);
    }
  }
  if(now-policy_refresh>=10000) { refresh_policies(now); policy_refresh=now; }
  for(budget=0;budget<RUNTIME_MEASUREMENT_CAPACITY;++budget) {
    SensorMeasurement m;
    if(!runtime_data_take_measurement(&m)) break;
    if(estimator_push_measurement(&m)) runtime_stats.measurement_rejected++;
    /* Queue admission is not NIS/shadow recovery evidence. An isolated
     * source cannot be promoted by decoding a packet successfully. */
  }
  for(budget=0;budget<4;++budget) {
    ImuSample sample;
    Status r=sensor_imu_read(imu_config,&imu_context,&sample,&now);
    int decoded=r==STATUS_OK;
    if(r==STATUS_NOT_READY) break;
    runtime_stats.imu_batches++;
    if(decoded) {
      runtime_stats.last_imu_us=sample.header.timestamp_sample_us;
      sensor_calibration_feed_imu(&sample,runtime_now_us());
      r=estimator_push_imu(&sample);
    }
    /* Physical transport evidence never promotes a quarantined source. */
    if(si_state(&integrity,1)==SI_SOURCE_OK || si_state(&integrity,1)==SI_SOURCE_SUSPECT)
      (void)si_report(&integrity,1,decoded ? SI_EVIDENCE_SAMPLE_ACCEPTED : SI_EVIDENCE_SAMPLE_REJECTED,now);
    if(r==STATUS_NOT_READY) {
      EstimatorHealth health;
      if(!estimator_get_health(&health) && health.common.health_state==MODULE_HEALTH_INITIALIZING)
        r=STATUS_OK; /* A valid sample in the static alignment window. */
    }
    if(r) { runtime_stats.imu_rejected++; sensor_rejects[0]++; runtime_stats.navigation_status=r; }
    else {
      ControllerInput in;
      runtime_stats.navigation_status=estimator_update(runtime_now_us());
      (void)estimator_get_snapshot(&fast_bundle.nav);
      (void)estimator_get_capability(&fast_nav_cap);
      consume_consistency(runtime_now_us());
      runtime_stats.nav_updates++; runtime_stats.last_navigation_us=fast_bundle.nav.header.timestamp_sample_us;
      if(runtime_data_lock(0)) {
#ifdef FLIGHT_LOGGER_ENABLE
        runtime_data.log_imu = sample;
#endif
        runtime_data.nav = fast_bundle.nav;
        runtime_data.nav_cap = fast_nav_cap;
        if(runtime_data.bundle_ready && fast_bundle_revision != runtime_data.bundle_revision) {
          /* Preserve the just-propagated state when copying the slower
           * decision/reference bundle; never extrapolate M3 at M6. */
          fast_bundle.guidance=runtime_data.bundle.guidance;
          fast_bundle.directive=runtime_data.bundle.directive; fast_bundle.envelope=runtime_data.bundle.envelope;
          fast_bundle.authority=runtime_data.bundle.authority; fast_bundle.permit=runtime_data.bundle.permit;
          fast_bundle.model=runtime_data.bundle.model;
          fast_bundle_revision=runtime_data.bundle_revision;
        }
        fast_bundle.actuator=runtime_data.actuator; fast_bundle.actuator_cap=runtime_data.actuator_cap;
        fast_bundle.receipt=runtime_data.receipt; runtime_data_unlock();
      }
      runtime_data_controller_input(&fast_bundle,&in,runtime_now_us());
      runtime_stats.control_status=controller_update(&in,&controller_output);
      if(runtime_stats.control_status) runtime_stats.control_rejected++; else runtime_stats.control_updates++;
      if(runtime_data_lock(0)) {
        runtime_data.control_cap=controller_output.capability;
        runtime_data.control_health=controller_output.health;
        if(!runtime_stats.control_status) {
          runtime_data.demand=controller_output.actuator_demand;runtime_data.have_demand=1;
          runtime_data.control_reference_id=fast_bundle.guidance.guidance_status.reference_id;
        }
        runtime_data_unlock();
      }
    }
  }
  {
    ContractHeader candidate; uint32_t next=0; int signal=0;
    Status result=ins_get_pending_origin(&candidate,&next);
    if(runtime_data_lock(0)) {
      signal=!result && (candidate.sequence!=runtime_data.origin_candidate.sequence ||
        candidate.timestamp_sample_us!=runtime_data.origin_candidate.timestamp_sample_us);
      runtime_data.origin_candidate=candidate; runtime_data.next_origin_id=next; runtime_data_unlock();
    }
    if(signal) (void)rt_sem_release(&runtime_data.plan_event);
  }
  sensor_calibration_tick(runtime_now_us());
  publish_sensor_health(runtime_now_us());
}

static uint8_t task_sensor_stack[8192] __attribute__((aligned(8)));

static void task_sensor_entry(void *parameter)
{
    (void)parameter;
    while (runtime_running()) {
        (void)rt_sem_take(&runtime_data.imu_event,
                          rt_tick_from_millisecond(TASK_SENSOR_PERIOD_MS));
        task_sensor_step();
    }
}

const TaskDescriptor task_sensor = {
    "sensor",
    task_sensor_entry,
    task_sensor_stack,
    sizeof(task_sensor_stack),
    TASK_SENSOR_PRIORITY,
};
