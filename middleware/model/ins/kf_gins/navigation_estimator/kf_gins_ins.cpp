/* Serialized M3 owner. push_imu only propagates. update performs synchronous
 * Host/reference or metered resumable replay at an IMU boundary. Numerical
 * operation WCET and server RTA still need target evidence. */
#include "contract_checks/algorithm_checks.h"
#include "observation_replay/ins_replay.hpp"
#include "backend_adapter/kf_gins_adapter.hpp"
#include "ins_model.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

namespace {
struct Source {
  bool present = false;
  uint16_t logical_source = 0;
  SensorUsePolicy policy{};
  SensorConsistencyEvidence evidence{};
  uint64_t window_start=0, last_evaluated_time=0;
  uint32_t last_evaluated_sequence=0, evaluated_kind_mask=0;
  uint32_t mapping_domain=0,mapping_epoch=0;
  bool gap_seen=false;
  uint32_t gap_first_sequence=0,gap_last_sequence=0,gap_reason=0;
  uint64_t gap_last_good_time=0,gap_next_sample_time=0;
};
struct EstimatorCores {
  kfg::QuadCore active, shadow;
};
#ifdef FLIGHT_NAV_DTCM
static_assert(sizeof(kfg::QuadCore) <= 15360, "M3 core exceeds its DTCM budget");
__attribute__((section(".estimator_cores")))
#endif
alignas(EstimatorCores) unsigned char core_storage[sizeof(EstimatorCores)];

struct Estimator {
  explicit Estimator(EstimatorCores &cores) : active(cores.active), shadow(cores.shadow) {}

  ProductEstimatorConfig config{};
  kfg::QuadCore &active, &shadow;
  kfg::ImuIncrementBuilder builder;
  ins::InsHistory history;
  ins::ReplayEngine replay;
  ins::ReplayCursor cursor;
  ins::ReplayCursor probe_cursor;
  struct {
    ins::ObservationRecord records[2]{};
    uint64_t expires=0,target_time=0;
    uint32_t sensor_id=0,policy_epoch=0,state_epoch=0,target_sequence=0,watermark=0;
    unsigned count=0;
    ContractHeader header{};
    bool queued=false;
  } probe;
  ins::ReplayCandidate candidate{};
  Source source[8];
  ClockInstant clock{};
  NavigationSnapshot snapshot{};
  NavigationOriginSnapshot origin{};
  NavigationCapability capability{};
  EstimatorHealth health{};
  StateResetEvent reset_event{};
  ImuSample last_imu{};
  GnssSample initial_gnss{};
  MagSample initial_mag{};
  double sum[6]{}, sum_sq[6]{};
  double baro_zero = 0;
  unsigned alignment_count = 0;
  uint64_t alignment_start = 0, state_time = 0, last_checkpoint = 0,
           pending_start = UINT64_MAX;
  uint64_t last_gnss = 0, last_baro = 0, last_mag = 0;
  uint32_t sequence = 0, state_epoch = 1, origin_id = 1, fusion_revision = 1;
  bool initialized = false, ready = false, lost = false, baro_reference = false,
       pending = false, event_pending = false;
  bool position_observed = false, velocity_observed = false,
       heading_observed = false;
  double baro_sum = 0;
  unsigned baro_count = 0;
};
alignas(Estimator) unsigned char nav_storage[sizeof(Estimator)];
Estimator *nav_ptr = nullptr;
uint32_t replay_budget_us=0, replay_period_us=5000, replay_spent_us=0;
uint64_t replay_period_start=0;
#define nav (*nav_ptr)
constexpr double kD2R = 0.017453292519943295;
template <std::size_t N> bool finite_double_array(const double (&values)[N]) {
  for (double value : values)
    if (!std::isfinite(value))
      return false;
  return true;
}
bool clock_now(ClockInstant &clock) {
  return clock_read(&clock) == STATUS_OK &&
         clock.boot_id == nav.clock.boot_id &&
         clock.time_epoch == nav.clock.time_epoch;
}
bool mapping_current(uint32_t domain,uint32_t epoch,uint64_t now) {
  if(!domain) return epoch==0;
  ClockDomainMappingStatus m{};
  return !time_get_mapping_status(domain,&m) && m.mapping_epoch==epoch &&
    m.local_time_epoch==nav.clock.time_epoch && m.valid_until_local_us>now &&
    m.uncertainty_ns<=100000 &&
    (m.mapping_state==CLOCK_MAPPING_LOCKED || m.mapping_state==CLOCK_MAPPING_HOLDOVER);
}
ContractHeader base_header(uint64_t sample, uint64_t now) {
  ContractHeader h{};
  h.schema_major = CONTRACT_SCHEMA_MAJOR;
  h.schema_minor = CONTRACT_SCHEMA_MINOR;
  h.profile_id = PRODUCT_PROFILE_ID;
  h.boot_id = nav.clock.boot_id;
  h.time_epoch = nav.clock.time_epoch;
  h.frame_id = FRAME_LOCAL_NED;
  h.source_id = SOURCE_M3;
  h.sequence = ++nav.sequence;
  h.timestamp_sample_us = sample;
  h.timestamp_publish_us = now;
  h.valid_until_us = sample + 5000;
  h.timestamp_quality = TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED;
  h.timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
  h.event_time_uncertainty_us = nav.last_imu.header.event_time_uncertainty_us;
  h.validity_flags = 3;
  return h;
}
Source *find_source(uint32_t id) {
  for (auto &s : nav.source)
    if (s.present && s.policy.sensor_instance_id == id)
      return &s;
  return nullptr;
}
static void clear_evidence(Source &s) {
  s.evidence={}; s.window_start=0; s.last_evaluated_time=0;
  s.last_evaluated_sequence=0; s.evaluated_kind_mask=0;
  s.evidence.sensor_instance_id=s.policy.sensor_instance_id;
}
static void publish_evaluations(const ins::ReplayCandidate &candidate,unsigned mode,uint64_t now) {
  for(unsigned kind=0;kind<4;++kind) {
    const auto &evaluation=candidate.latest_evaluation[kind];
    if(!evaluation.sample_time_us) continue;
    Source *source=nullptr;
    for(auto &s:nav.source)
      if(s.present && s.logical_source==evaluation.source_id) { source=&s; break; }
    if(!source || source->policy.policy_epoch!=evaluation.policy_epoch ||
       source->policy.policy_expires_at_us<=now) continue;
    if(mode==2 && (source->policy.sensor_instance_id!=nav.probe.sensor_id ||
                  evaluation.source_sequence!=nav.probe.records[0].id.source_sequence)) continue;
    if(mode==1 && source->policy.use_state==SENSOR_USE_REJECT) continue;
    auto &s=*source;
    if(s.evidence.state_epoch && s.evidence.state_epoch!=nav.state_epoch) clear_evidence(s);
    if(evaluation.sample_time_us<s.last_evaluated_time ||
       (evaluation.sample_time_us==s.last_evaluated_time &&
        (evaluation.source_sequence!=s.last_evaluated_sequence || (s.evaluated_kind_mask&(1u<<kind))))) continue;
    if(evaluation.sample_time_us!=s.last_evaluated_time) {
      s.evaluated_kind_mask=0; s.last_evaluated_time=evaluation.sample_time_us;
      s.last_evaluated_sequence=evaluation.source_sequence;
    }
    s.evaluated_kind_mask|=1u<<kind;
    auto &e=s.evidence;
    if(!s.window_start) s.window_start=now;
    if(now-s.window_start>5000000 || e.evaluated_count==UINT16_MAX) {
      e.evaluated_count=e.accepted_count=e.rejected_count=e.probation_pass_count=0;
      s.window_start=now;
    }
    if(e.evaluated_count<UINT16_MAX) ++e.evaluated_count;
    if(evaluation.accepted) {
      if(e.accepted_count<UINT16_MAX) ++e.accepted_count;
      if(mode==2 && e.probation_pass_count<UINT16_MAX) ++e.probation_pass_count;
    } else if(e.rejected_count<UINT16_MAX) ++e.rejected_count;
    e.header=base_header(now,now); e.header.message_type=MSG_SENSOR_CONSISTENCY_EVIDENCE;
    e.header.valid_until_us=std::min(now+200000,s.policy.policy_expires_at_us);
    e.header.event_time_uncertainty_us=0; e.header.timestamp_quality=TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
    e.policy_epoch=s.policy.policy_epoch; e.state_epoch=nav.state_epoch;
    e.calibration_version=nav.config.payload.calibration_version;
    e.window_duration_us=static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX,now-s.window_start));
    e.evidence_flags=SENSOR_EVIDENCE_TIME_READY|SENSOR_EVIDENCE_CALIBRATION_READY|
      SENSOR_EVIDENCE_NUMERIC_READY|SENSOR_EVIDENCE_CONSISTENCY_READY;
    e.evaluation_mode=static_cast<uint16_t>(mode);
    e.reason_code=evaluation.accepted ? SENSOR_CONSISTENCY_REASON_NONE : SENSOR_CONSISTENCY_REASON_INCONSISTENT;
    e.last_evaluated_sample_time_us=evaluation.sample_time_us;
    e.evidence_expires_at_us=e.header.valid_until_us;
  }
}
bool permitted(uint32_t id, const ContractHeader &h, uint64_t now,
               Source **source = nullptr) {
  Source *s = find_source(id);
  if (source)
    *source = s;
  return s && same_clock(&s->policy.header, &h) &&
         s->policy.effective_time_us <= h.timestamp_sample_us &&
         s->policy.policy_expires_at_us > now &&
         s->policy.header.valid_until_us > now &&
         s->policy.use_state != SENSOR_USE_REJECT;
}
void reject(Status status) {
  ++nav.health.rejected_measurement_count;
  nav.health.common.last_error_code = static_cast<uint32_t>(status);
  ++nav.health.common.consecutive_error_count;
}
void reset_alignment() {
  nav.alignment_count = 0;
  nav.alignment_start = 0;
  std::memset(nav.sum, 0, sizeof(nav.sum));
  std::memset(nav.sum_sq, 0, sizeof(nav.sum_sq));
  nav.baro_sum = 0;
  nav.baro_count = 0;
  nav.baro_reference = false;
}
kfg::QuadConfig core_config() {
  const auto &p = nav.config.payload;
  kfg::QuadConfig c{};
  for (int i = 0; i < 3; ++i) {
    c.imunoise.gyr_arw(i, 0) = p.gyro_noise_density_radps_per_sqrt_hz[i];
    c.imunoise.acc_vrw(i, 0) = p.accel_noise_density_mps2_per_sqrt_hz[i];
    c.imunoise.gyrbias_std(i, 0) = p.gyro_bias_std_radps[i];
    c.imunoise.accbias_std(i, 0) = p.accel_bias_std_mps2[i];
    c.imunoise.gyrscale_std(i, 0) = p.gyro_scale_std[i];
    c.imunoise.accscale_std(i, 0) = p.accel_scale_std[i];
    c.antlever(i, 0) = p.antenna_lever_frd_m[i];
  }
  c.imunoise.corr_time = p.bias_correlation_time_s[0];
  c.imunoise.correlation_time_s[0] = p.bias_correlation_time_s[0];
  c.imunoise.correlation_time_s[1] = p.bias_correlation_time_s[1];
  c.imunoise.correlation_time_s[2] = p.scale_correlation_time_s[0];
  c.imunoise.correlation_time_s[3] = p.scale_correlation_time_s[1];
  c.imunoise.baro_bias_walk = p.baro_bias_rw_psd_m2ps;
  c.nis_gate_enable = true;
  c.baro_origin_height_m = nav.origin.ellipsoid_height_m;
  return c;
}
Status initialize_core(const NavStateCore &state, bool provided) {
  const auto &p = nav.config.payload;
  const auto &upper = p.initial_covariance_upper;
  const double heading_floor = (!provided && !nav.heading_observed) ? kfg::kPi * kfg::kPi : 0.0;
  const auto std_at = [&](unsigned index) {
    const double variance = upper[kfg::covarianceUpperDiagonalIndex(index)];
    return std::sqrt(index == 8u ? std::max(variance, heading_floor) : variance);
  };
  kfg::QuadInitState init{};
  init.nominal.pos =
      kfg::vec3(nav.origin.latitude_deg * kD2R, nav.origin.longitude_deg * kD2R,
                nav.origin.ellipsoid_height_m);
  init.nominal.qbn = kfg::Quat{state.q_nb_wxyz[0], state.q_nb_wxyz[1],
                               state.q_nb_wxyz[2], state.q_nb_wxyz[3]};
  init.nominal.cbn = kfg::quatToMatrix(init.nominal.qbn);
  if (provided) {
    const auto offset = kfg::matvec(
        init.nominal.cbn, kfg::vec3(p.imu_to_vehicle_reference_frd_m[0],
                                    p.imu_to_vehicle_reference_frd_m[1],
                                    p.imu_to_vehicle_reference_frd_m[2]));
    const auto local =
        kfg::vec3(state.position_ned_m[0], state.position_ned_m[1],
                  state.position_ned_m[2]) -
        offset;
    init.nominal.pos =
        init.nominal.pos + kfg::matvec(kfg::earthDRi(init.nominal.pos), local);
  }
  if (!provided && nav.position_observed) {
    const auto lever =
        kfg::matvec(init.nominal.cbn, kfg::vec3(p.antenna_lever_frd_m[0],
                                                p.antenna_lever_frd_m[1],
                                                p.antenna_lever_frd_m[2]));
    init.nominal.pos =
        init.nominal.pos - kfg::matvec(kfg::earthDRi(init.nominal.pos), lever);
  }
  for (int i = 0; i < 3; ++i) {
    init.nominal.vel(i, 0) = state.velocity_ned_mps[i];
    init.pos_std(i, 0) = std_at(i);
    init.vel_std(i, 0) = std_at(i + 3);
    init.euler_std(i, 0) = std_at(i + 6);
    init.gyrbias_std(i, 0) = std_at(i + 9);
    init.accbias_std(i, 0) = std_at(i + 12);
    init.gyrscale_std(i, 0) = std_at(i + 15);
    init.accscale_std(i, 0) = std_at(i + 18);
    if (!provided)
      init.nominal.gyrbias(i, 0) = nav.sum[i] / nav.alignment_count;
  }
  if(provided){
    const auto lever=kfg::vec3(p.imu_to_vehicle_reference_frd_m[0],p.imu_to_vehicle_reference_frd_m[1],p.imu_to_vehicle_reference_frd_m[2]);
    const auto omega=kfg::vec3(state.body_rate_frd_radps[0],state.body_rate_frd_radps[1],state.body_rate_frd_radps[2]);
    init.nominal.vel=init.nominal.vel-kfg::matvec(init.nominal.cbn,kfg::matvec(kfg::skew(omega),lever));
  }
  /* Earth rotation is physical angular rate, not stationary gyro bias. */
  if (!provided) {
    const auto earth_body = kfg::matvec(
        kfg::transpose3(init.nominal.cbn),
        kfg::vec3(kfg::WGS84_WIE * std::cos(init.nominal.pos(0, 0)), 0,
                  -kfg::WGS84_WIE * std::sin(init.nominal.pos(0, 0))));
    init.nominal.gyrbias = init.nominal.gyrbias - earth_body;
  }
  init.baro_bias_std = std_at(21);
  const auto c = core_config();
  if (nav.shadow.initialize(c, init) != kfg::kOk ||
      nav.shadow.setInitialCovariance(upper, heading_floor) != kfg::kOk)
    return STATUS_NUMERIC_ERROR;
  if (nav.active.initialize(c, init) != kfg::kOk ||
      nav.active.setInitialCovariance(upper, heading_floor) != kfg::kOk)
    return STATUS_NUMERIC_ERROR;
  nav.ready = true;
  nav.lost = false;
  nav.history.reset();
  nav.builder.reset();
  nav.cursor.cancel();
  nav.probe_cursor.cancel(); nav.probe.queued=false;
  nav.pending = false;
  nav.pending_start = UINT64_MAX;
  nav.last_checkpoint = 0;
  return STATUS_OK;
}
void checkpoint() {
  if (nav.history.pushCheckpoint(nav.active, nav.fusion_revision, nav.origin_id,
                                 nav.state_epoch, nav.state_time, nav.last_imu.sample_sequence)) {
    nav.last_checkpoint = nav.state_time;
    if(!nav.cursor.active() && !nav.probe_cursor.active()) nav.history.retireJournal();
  }
}
void publish(uint64_t now) {
  auto h = base_header(nav.state_time, now);
  nav.snapshot.header = h;
  nav.snapshot.header.message_type = MSG_NAVIGATION_SNAPSHOT;
  nav.snapshot.state_epoch = nav.state_epoch;
  nav.snapshot.origin_id = nav.origin_id;
  auto &out = nav.snapshot.core;
  out.valid_mask = 0;
  nav.origin.header = h;
  nav.origin.header.message_type = MSG_NAVIGATION_ORIGIN_SNAPSHOT;
  nav.origin.origin_id = nav.origin_id;
  nav.origin.state_epoch = nav.state_epoch;
  if (nav.ready) {
    const auto &n = nav.active.nominal();
    const auto &cov = nav.active.covariance();
    const auto origin = kfg::vec3(nav.origin.latitude_deg * kD2R,
                                  nav.origin.longitude_deg * kD2R,
                                  nav.origin.ellipsoid_height_m);
    auto geodetic_delta = n.pos - origin;
    geodetic_delta(1, 0) = std::remainder(geodetic_delta(1, 0), 2 * kfg::kPi);
    const auto dp = kfg::matvec(kfg::earthDR(origin), geodetic_delta);
    const auto lever =
        kfg::vec3(nav.config.payload.imu_to_vehicle_reference_frd_m[0],
                  nav.config.payload.imu_to_vehicle_reference_frd_m[1],
                  nav.config.payload.imu_to_vehicle_reference_frd_m[2]);
    const auto offset = kfg::matvec(n.cbn, lever);
    kfg::Vec3 omega, force;
    for (int i = 0; i < 3; ++i) {
      omega(i, 0) = (nav.last_imu.gyro_frd_radps[i] - n.gyrbias(i, 0)) /
                    (1 + n.gyrscale(i, 0));
      force(i, 0) = (nav.last_imu.accel_frd_mps2[i] - n.accbias(i, 0)) /
                    (1 + n.accscale(i, 0));
    }
    const auto lever_velocity =
        kfg::matvec(n.cbn, kfg::matvec(kfg::skew(omega), lever));
    const auto acceleration = kfg::matvec(n.cbn, force);
    for (int i = 0; i < 3; ++i) {
      out.position_ned_m[i] = static_cast<float>(dp(i, 0) + offset(i, 0));
      out.velocity_ned_mps[i] =
          static_cast<float>(n.vel(i, 0) + lever_velocity(i, 0));
      out.acceleration_ned_mps2[i] = static_cast<float>(
          acceleration(i, 0) + (i == 2 ? kfg::gravity(n.pos) : 0));
      out.body_rate_frd_radps[i] = static_cast<float>(omega(i, 0));
      /* Transform the full error covariance to the published CG reference.
         Keep position/attitude and velocity/bias/scale cross terms. */
      double jp[22] = {}, jv[22] = {};
      jp[i] = 1;
      jv[i + 3] = 1;
      const auto sp = kfg::skew(offset), sv = kfg::skew(lever_velocity);
      const auto bg = kfg::matmul3(n.cbn, kfg::skew(lever));
      for (int j = 0; j < 3; ++j) {
        jp[j + 6] = sp(i, j);
        jv[j + 6] = sv(i, j);
        jv[j + 9] = -bg(i, j) / (1 + n.gyrscale(j, 0));
        jv[j + 15] = -bg(i, j) * omega(j, 0) / (1 + n.gyrscale(j, 0));
      }
      double pv = 0, vv = 0;
      for (int j = 0; j < 22; ++j)
        for (int k = 0; k < 22; ++k) {
          pv += jp[j] * cov(j, k) * jp[k];
          vv += jv[j] * cov(j, k) * jv[k];
        }
      out.position_std_m[i] = static_cast<float>(std::sqrt(std::max(0.0, pv)));
      out.velocity_std_mps[i] =
          static_cast<float>(std::sqrt(std::max(0.0, vv)));
      out.attitude_std_rad[i] =
          static_cast<float>(std::sqrt(cov(i + 6, i + 6)));
    }
    out.q_nb_wxyz[0] = static_cast<float>(n.qbn.w);
    out.q_nb_wxyz[1] = static_cast<float>(n.qbn.x);
    out.q_nb_wxyz[2] = static_cast<float>(n.qbn.y);
    out.q_nb_wxyz[3] = static_cast<float>(n.qbn.z);
    const bool fresh =
        now >= nav.state_time &&
        now - nav.state_time + h.event_time_uncertainty_us <= 5000;
    if (!nav.lost && fresh && finite_array(out.position_ned_m, 25) &&
        std::hypot(out.position_ned_m[0], out.position_ned_m[1]) <=
            nav.origin.local_radius_limit_m) {
      out.valid_mask =
          NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE | NAV_VALID_ACCELERATION;
      if (nav.heading_observed && out.attitude_std_rad[2] < .5f)
        out.valid_mask |= NAV_VALID_HEADING;
      if (nav.position_observed)
        out.valid_mask |=
            NAV_VALID_HORIZONTAL_POSITION | NAV_VALID_VERTICAL_POSITION;
      if (nav.velocity_observed)
        out.valid_mask |=
            NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY;
      if (nav.baro_reference && nav.last_baro && now - nav.last_baro < 1000000)
        out.valid_mask |=
            NAV_VALID_VERTICAL_POSITION | NAV_VALID_VERTICAL_VELOCITY;
    }
  }
  nav.capability = {};
  nav.capability.header = h;
  nav.capability.header.message_type = MSG_NAVIGATION_CAPABILITY;
  nav.capability.state_epoch = nav.state_epoch;
  nav.capability.origin_id = nav.origin_id;
  nav.capability.capability_mask = out.valid_mask;
  if (out.valid_mask & NAV_VALID_BODY_RATE)
    nav.capability.active_source_mask |= 1u << SOURCE_IMU;
  if (nav.last_gnss && now >= nav.last_gnss && now - nav.last_gnss < 1000000)
    nav.capability.active_source_mask |= 1u << SOURCE_GNSS;
  if (nav.last_baro && now >= nav.last_baro && now - nav.last_baro < 100000)
    nav.capability.active_source_mask |= 1u << SOURCE_BARO;
  if (nav.last_mag && now >= nav.last_mag && now - nav.last_mag < 100000)
    nav.capability.active_source_mask |= 1u << SOURCE_MAG;
  for (const auto &source : nav.source)
    if (source.present && source.policy.use_state == SENSOR_USE_REJECT &&
        source.logical_source > 0 && source.logical_source < 32) {
      nav.capability.rejected_source_mask |= 1u << source.logical_source;
      nav.capability.active_source_mask &= ~(1u << source.logical_source);
    }
  nav.capability.degraded_mask =
      (NAV_VALID_HEADING | NAV_VALID_HORIZONTAL_POSITION |
       NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_POSITION |
       NAV_VALID_VERTICAL_VELOCITY) &
      ~out.valid_mask;
  nav.capability.solution_state = !nav.ready        ? NAV_SOLUTION_INITIALIZING
                                  : !out.valid_mask ? NAV_SOLUTION_INVALID
                                  : nav.capability.degraded_mask
                                      ? NAV_SOLUTION_DEGRADED
                                      : NAV_SOLUTION_VALID;
  nav.capability.horizontal_position_std_m =
      std::hypot(out.position_std_m[0], out.position_std_m[1]);
  nav.capability.vertical_position_std_m = out.position_std_m[2];
  nav.capability.heading_std_rad = out.attitude_std_rad[2];
  nav.capability.velocity_std_mps =
      std::sqrt(out.velocity_std_mps[0] * out.velocity_std_mps[0] +
                out.velocity_std_mps[1] * out.velocity_std_mps[1] +
                out.velocity_std_mps[2] * out.velocity_std_mps[2]);
  nav.health.common.header = h;
  nav.health.common.header.message_type = MSG_ESTIMATOR_HEALTH;
  nav.health.last_imu_sample_us = nav.state_time;
  nav.health.propagation_age_us =
      now >= nav.state_time ? static_cast<uint32_t>(std::min<uint64_t>(
                                  UINT32_MAX, now - nav.state_time))
                            : UINT32_MAX;
  nav.health.common.last_heartbeat_time_us = now;
  nav.health.common.health_state = !nav.ready ? MODULE_HEALTH_INITIALIZING
                                   : !out.valid_mask ? MODULE_HEALTH_FAILED
                                   : nav.capability.degraded_mask
                                       ? MODULE_HEALTH_DEGRADED
                                       : MODULE_HEALTH_HEALTHY;
}

__attribute__((noinline)) Status align_sample(const kfg::ImuSample &in,
                                              const ClockInstant &now) {
  kfg::ImuIncrement increment{};
  const double g = kfg::gravity(kfg::vec3(nav.origin.latitude_deg * kD2R,
                                          nav.origin.longitude_deg * kD2R,
                                          nav.origin.ellipsoid_height_m));
  double an = 0, gn = 0;
  for (int i = 0; i < 3; ++i) {
    an += in.accel_mps2(i, 0) * in.accel_mps2(i, 0);
    gn += in.gyro_radps(i, 0) * in.gyro_radps(i, 0);
  }
  if (std::fabs(std::sqrt(an) - g) > .3 || std::sqrt(gn) > .05) {
    reset_alignment();
    return STATUS_NOT_READY;
  }
  if (!nav.alignment_count)
    nav.alignment_start = in.sample_time_us;
  for (int i = 0; i < 6; ++i) {
    double v = i < 3 ? in.gyro_radps(i, 0) : in.accel_mps2(i - 3, 0);
    nav.sum[i] += v;
    nav.sum_sq[i] += v * v;
  }
  ++nav.alignment_count;
  if (nav.alignment_count < 1000 ||
      in.sample_time_us - nav.alignment_start < 999000) {
    publish(now.monotonic_us);
    return STATUS_NOT_READY;
  }
  for (int i = 0; i < 6; ++i) {
    const double mean = nav.sum[i] / nav.alignment_count;
    const double variance = nav.sum_sq[i] / nav.alignment_count - mean * mean;
    if (variance > (i < 3 ? 1e-5 : .01)) {
      reset_alignment();
      return STATUS_NOT_READY;
    }
  }
  NavStateCore initial{};
  double ax = nav.sum[3] / nav.alignment_count,
         ay = nav.sum[4] / nav.alignment_count,
         az = nav.sum[5] / nav.alignment_count;
  double roll = std::atan2(-ay, -az),
         pitch = std::atan2(ax, std::hypot(ay, az)), yaw = 0;
  if (nav.initial_gnss.fix_type >= GNSS_FIX_SINGLE &&
      now.monotonic_us - nav.initial_gnss.header.timestamp_sample_us <= 20000) {
    if ((nav.config.payload.aiding_mask & PRODUCT_AIDING_GNSS_POSITION) &&
        (nav.initial_gnss.quality_flags &
         (GNSS_Q_POSITION_VALID | GNSS_Q_POSITION_COV_VALID)) ==
            (GNSS_Q_POSITION_VALID | GNSS_Q_POSITION_COV_VALID)) {
      nav.origin.latitude_deg = nav.initial_gnss.latitude_deg;
      nav.origin.longitude_deg = nav.initial_gnss.longitude_deg;
      nav.origin.ellipsoid_height_m = nav.initial_gnss.altitude_m;
      nav.origin.valid_mask |= 2u;
      nav.position_observed = true;
      nav.last_gnss = nav.initial_gnss.header.timestamp_sample_us;
    }
    if ((nav.config.payload.aiding_mask & PRODUCT_AIDING_GNSS_VELOCITY) &&
        (nav.initial_gnss.quality_flags &
         (GNSS_Q_VELOCITY_VALID | GNSS_Q_VELOCITY_COV_VALID)) ==
            (GNSS_Q_VELOCITY_VALID | GNSS_Q_VELOCITY_COV_VALID)) {
      for (int i = 0; i < 3; ++i)
        initial.velocity_ned_mps[i] = nav.initial_gnss.velocity_ned_mps[i];
      nav.velocity_observed = true;
    }
  }
  if (nav.initial_mag.header.timestamp_sample_us &&
      now.monotonic_us - nav.initial_mag.header.timestamp_sample_us <= 20000) {
    const auto field =
        kfg::matvec(kfg::eulerToMatrix(kfg::vec3(roll, pitch, 0)),
                    kfg::vec3(nav.initial_mag.field_frd_ut[0],
                              nav.initial_mag.field_frd_ut[1],
                              nav.initial_mag.field_frd_ut[2]));
    double variance = 0;
    for (float v : nav.initial_mag.variance_ut2)
      variance = std::max<double>(variance, v);
    const double horizontal = std::hypot(field(0, 0), field(1, 0));
    if (horizontal >= nav.config.payload.min_horizontal_field_ut &&
        variance / (horizontal * horizontal) <= .25) {
      yaw = nav.config.payload.magnetic_declination_rad -
            std::atan2(field(1, 0), field(0, 0));
      nav.heading_observed = true;
      nav.last_mag = nav.initial_mag.header.timestamp_sample_us;
    }
  }
  if (nav.baro_count) {
    nav.baro_zero = nav.baro_sum / nav.baro_count;
    nav.baro_reference = true;
  }
  const auto quat = kfg::eulerToQuat(kfg::vec3(roll, pitch, yaw));
  initial.q_nb_wxyz[0] = static_cast<float>(quat.w);
  initial.q_nb_wxyz[1] = static_cast<float>(quat.x);
  initial.q_nb_wxyz[2] = static_cast<float>(quat.y);
  initial.q_nb_wxyz[3] = static_cast<float>(quat.z);
  const Status status = initialize_core(initial, false);
  if (status)
    return status;
  /* Seed the next increment at this exact alignment boundary. */
  nav.builder.push(in, increment);
  checkpoint();
  publish(now.monotonic_us);
  return STATUS_OK;
}
} // namespace

extern "C" Status ins_model_validate(const ProductProfile *p) {
  if (!p)
    return STATUS_INVALID_ARGUMENT;
  return p->product_profile_id == PRODUCT_PROFILE_ID &&
                 p->nav_layout_id == PRODUCT_NAV_LAYOUT_ID
             ? STATUS_OK
             : STATUS_PROFILE_MISMATCH;
}
extern "C" Status ins_model_init(const ProductEstimatorConfig *config) {
  if (!nav_ptr) {
#ifdef FLIGHT_NAV_DTCM
    /* The startup code clears AXI .bss only. Initialize the NOLOAD arena here. */
    std::memset(core_storage, 0, sizeof(core_storage));
#endif
    auto *cores = new (core_storage) EstimatorCores;
    nav_ptr = new (nav_storage) Estimator(*cores);
  }
  if (!config || !config->profile_hash || !config->parameter_hash ||
      !config->config_version)
    return STATUS_INVALID_ARGUMENT;
  const auto &p = config->payload;
  const double scalar_fields[] = {
      p.baro_bias_rw_psd_m2ps, p.baro_floor_m, p.mag_heading_floor_rad,
      p.magnetic_declination_rad, p.magnetic_declination_variance_rad2,
      p.min_horizontal_field_ut, p.initial_latitude_deg, p.initial_longitude_deg,
      p.initial_ellipsoid_height_m,
  };
  if (!finite_double_array(scalar_fields) ||
      !finite_double_array(p.bias_correlation_time_s) ||
      !finite_double_array(p.scale_correlation_time_s) ||
      !(p.local_radius_limit_m > 0) || !std::isfinite(p.local_radius_limit_m) ||
      (p.initial_origin_valid_mask & 1u) == 0 ||
      (p.initial_origin_valid_mask & ~3u) ||
      std::fabs(p.initial_latitude_deg) >= 89 ||
      std::fabs(p.initial_longitude_deg) > 180 || p.baro_bias_rw_psd_m2ps < 0 ||
      p.baro_floor_m <= 0 || p.mag_heading_floor_rad <= 0 ||
      p.magnetic_declination_variance_rad2 < 0 ||
      p.min_horizontal_field_ut <= 0 || p.aiding_mask & ~15u)
    return STATUS_INVALID_ARGUMENT;
  for (int i = 0; i < 3; ++i) {
    const double axis_fields[] = {
        p.gyro_noise_density_radps_per_sqrt_hz[i], p.accel_noise_density_mps2_per_sqrt_hz[i],
        p.gyro_bias_std_radps[i], p.accel_bias_std_mps2[i], p.gyro_scale_std[i],
        p.accel_scale_std[i], p.gnss_position_floor_m[i], p.gnss_velocity_floor_mps[i],
        p.antenna_lever_frd_m[i], p.imu_to_vehicle_reference_frd_m[i],
    };
    if (!finite_double_array(axis_fields) || p.gyro_noise_density_radps_per_sqrt_hz[i] < 0 ||
        p.accel_noise_density_mps2_per_sqrt_hz[i] < 0 ||
        p.gyro_bias_std_radps[i] < 0 || p.accel_bias_std_mps2[i] < 0 ||
        p.gyro_scale_std[i] < 0 || p.accel_scale_std[i] < 0 ||
        p.gnss_position_floor_m[i] <= 0 || p.gnss_velocity_floor_mps[i] <= 0)
      return STATUS_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 2; ++i)
    if (p.bias_correlation_time_s[i] <= 0 || p.scale_correlation_time_s[i] <= 0)
      return STATUS_INVALID_ARGUMENT;
  if (!nav.shadow.validateInitialCovariance(p.initial_covariance_upper))
    return STATUS_INVALID_ARGUMENT;
  ClockInstant clock{};
  if (clock_read(&clock) || !clock.boot_id)
    return STATUS_NOT_READY;
  /* Explicit reset, avoiding a huge temporary Estimator on the thread stack. */
  nav.config = *config;
  nav.clock = clock;
  nav.initialized = true;
  nav.ready = false;
  nav.lost = false;
  nav.state_epoch = 1;
  nav.origin_id = 1;
  nav.fusion_revision = 1;
  nav.sequence = 0;
  nav.state_time = 0;
  nav.origin = {};
  nav.origin.latitude_deg = p.initial_latitude_deg;
  nav.origin.longitude_deg = p.initial_longitude_deg;
  nav.origin.ellipsoid_height_m = p.initial_ellipsoid_height_m;
  nav.origin.local_radius_limit_m = p.local_radius_limit_m;
  nav.origin.valid_mask = p.initial_origin_valid_mask;
  nav.snapshot = {};
  nav.capability = {};
  nav.health = {};
  nav.initial_gnss = {};
  nav.initial_mag = {};
  nav.last_imu = {};
  for (auto &source : nav.source)
    source = Source{};
  nav.position_observed = nav.velocity_observed = nav.heading_observed = false;
  nav.baro_reference = false;
  nav.last_gnss = nav.last_baro = nav.last_mag = 0;
  nav.pending = false;
  nav.event_pending = false;
  nav.pending_start = UINT64_MAX;
  nav.history.reset();
  nav.builder.reset();
  nav.cursor.cancel();
  nav.probe_cursor.cancel(); nav.probe.queued=false;
  reset_alignment();
  publish(clock.monotonic_us);
  return STATUS_OK;
}
extern "C" Status ins_model_set_sensor_policy(const SensorUsePolicy *policy,
                                              uint64_t now) {
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  if (!policy || !header_valid(&policy->header, now) ||
      !same_clock(&policy->header, &nav.snapshot.header) ||
      policy->header.source_id != SOURCE_HEALTH ||
      policy->use_state > SENSOR_USE_REJECT ||
      (policy->evaluation_flags & ~SENSOR_USE_EVAL_ALLOW_SHADOW) ||
      policy->policy_expires_at_us <= now || !policy->policy_epoch ||
      (policy->use_state == SENSOR_USE_DEGRADED &&
       policy->degraded_noise_profile_id != 1))
    return STATUS_INVALID_ARGUMENT;
  Source *s = find_source(policy->sensor_instance_id);
  if (!s)
    for (auto &slot : nav.source)
      if (!slot.present) {
        s = &slot;
        break;
      }
  if (!s)
    return STATUS_BUFFER_FULL;
  if (s->present && policy->policy_epoch < s->policy.policy_epoch)
    return STATUS_STALE_DATA;
  if (s->present && policy->policy_epoch == s->policy.policy_epoch &&
      (policy->use_state != s->policy.use_state ||
       policy->degraded_noise_profile_id !=
           s->policy.degraded_noise_profile_id))
    return STATUS_INVALID_ARGUMENT;
  const bool changed=!s->present || policy->policy_epoch!=s->policy.policy_epoch;
  s->present = true;
  s->policy = *policy;
  if(changed) clear_evidence(*s);
  if(changed && nav.probe.queued && nav.probe.sensor_id==policy->sensor_instance_id) {
    nav.probe_cursor.cancel(); nav.probe.queued=false;
  }
  return STATUS_OK;
}
extern "C" Status ins_model_push_imu(const ImuSample *sample) {
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  ClockInstant now{};
  if (!sample || !clock_now(now) ||
      !header_valid(&sample->header, now.monotonic_us) ||
      !same_clock(&sample->header, &nav.snapshot.header) ||
      sample->header.frame_id != FRAME_BODY_FRD ||
      sample->header.source_id != SOURCE_IMU ||
      sample->calibration_version != nav.config.payload.calibration_version ||
      (sample->quality_flags & (IMU_Q_GYRO_VALID | IMU_Q_ACCEL_VALID |
                                IMU_Q_CALIBRATED | IMU_Q_SATURATED)) !=
          (IMU_Q_GYRO_VALID | IMU_Q_ACCEL_VALID | IMU_Q_CALIBRATED) ||
      !finite_array(sample->gyro_frd_radps, 7) ||
      !permitted(sample->sensor_instance_id, sample->header, now.monotonic_us))
    return STATUS_INVALID_ARGUMENT;
  if (nav.lost)
    return STATUS_NOT_READY;
  kfg::ImuSample in{};
  in.sample_time_us = sample->header.timestamp_sample_us;
  in.sequence = sample->sample_sequence;
  for (int i = 0; i < 3; ++i) {
    in.gyro_radps(i, 0) = sample->gyro_frd_radps[i];
    in.accel_mps2(i, 0) = sample->accel_frd_mps2[i];
  }
  kfg::ImuIncrement increment{};
  const auto result = nav.builder.push(in, increment);
  if (result != kfg::kAdapterSeed && result != kfg::kAdapterIncrement) {
    if (nav.ready)
      nav.lost = true;
    else {
      reset_alignment();
      nav.builder.reset();
    }
    reject(STATUS_STALE_DATA);
    publish(now.monotonic_us);
    return STATUS_STALE_DATA;
  }
  nav.last_imu = *sample;
  find_source(sample->sensor_instance_id)->logical_source =
      sample->header.source_id;
  nav.state_time = in.sample_time_us;
  if (!nav.ready)
    return align_sample(in, now);
  if (result == kfg::kAdapterIncrement) {
    if (nav.active.propagate(increment) != kfg::kOk) {
      ++nav.health.covariance_fault_count;
      nav.lost = true;
      publish(now.monotonic_us);
      return STATUS_NUMERIC_ERROR;
    }
    nav.history.pushImu(in.sample_time_us, increment, in.sequence);
    if (nav.state_time - nav.last_checkpoint >= 8000)
      checkpoint();
    nav.health.common.last_success_time_us = now.monotonic_us;
    nav.health.common.consecutive_error_count = 0;
  }
  publish(now.monotonic_us);
  return STATUS_OK;
}

extern "C" Status
ins_model_push_measurement(const SensorMeasurement *measurement) {
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  ClockInstant now{};
  if (!measurement || !clock_now(now) ||
      !header_valid(&measurement->header, now.monotonic_us) ||
      !same_clock(&measurement->header, &nav.snapshot.header))
    return STATUS_INVALID_ARGUMENT;
  const ContractHeader *h = nullptr;
  uint32_t calibration = 0;
  Source *source = nullptr;
  ins::ObservationRecord records[2]{};
  unsigned count = 1;
  const auto &p = nav.config.payload;
  switch (measurement->measurement_kind) {
  case PRODUCT_MEASUREMENT_GNSS: {
    const auto &s = measurement->payload.gnss;
    if (s.sensor_instance_id != measurement->sensor_instance_id)
      return STATUS_INVALID_ARGUMENT;
    h = &s.header;
    calibration = s.calibration_version;
    if ((p.aiding_mask & 3u) == 0)
      return STATUS_UNSUPPORTED;
    const bool position =
        (p.aiding_mask & PRODUCT_AIDING_GNSS_POSITION) &&
        (s.quality_flags &
         (GNSS_Q_POSITION_VALID | GNSS_Q_POSITION_COV_VALID)) ==
            (GNSS_Q_POSITION_VALID | GNSS_Q_POSITION_COV_VALID);
    const bool velocity =
        (p.aiding_mask & PRODUCT_AIDING_GNSS_VELOCITY) &&
        (s.quality_flags &
         (GNSS_Q_VELOCITY_VALID | GNSS_Q_VELOCITY_COV_VALID)) ==
            (GNSS_Q_VELOCITY_VALID | GNSS_Q_VELOCITY_COV_VALID);
    if (s.fix_type < GNSS_FIX_SINGLE || s.fix_type > GNSS_FIX_RTK_FIXED ||
        (!position && !velocity) ||
        (position &&
         (!std::isfinite(s.latitude_deg) || !std::isfinite(s.longitude_deg) ||
          std::fabs(s.latitude_deg) >= 89 || std::fabs(s.longitude_deg) > 180 ||
          !std::isfinite(s.altitude_m) ||
          !finite_array(s.position_variance_m2, 3))) ||
        (velocity && (!finite_array(s.velocity_ned_mps, 3) ||
                      !finite_array(s.velocity_variance_m2ps2, 3))) ||
        h->source_id != SOURCE_GNSS ||
        h->frame_id != FRAME_WGS84_GEODETIC)
      return STATUS_INVALID_ARGUMENT;
    count = 0;
    if (position) {
      auto &r = records[count++];
      r.id.kind = ins::ObservationKind::kGnssPosition;
      r.a[0] = s.latitude_deg * kD2R;
      r.a[1] = s.longitude_deg * kD2R;
      r.a[2] = s.altitude_m;
      for (int i = 0; i < 3; ++i) {
        if (s.position_variance_m2[i] <= 0)
          return STATUS_INVALID_ARGUMENT;
        r.b[i] = std::sqrt(std::max<double>(s.position_variance_m2[i],
                                            p.gnss_position_floor_m[i] *
                                                p.gnss_position_floor_m[i]) +
                           .0165 * .0165);
      }
    }
    if (velocity) {
      auto &r = records[count++];
      r.id.kind = ins::ObservationKind::kGnssVelocity;
      for (int i = 0; i < 3; ++i) {
        if (s.velocity_variance_m2ps2[i] <= 0)
          return STATUS_INVALID_ARGUMENT;
        r.a[i] = s.velocity_ned_mps[i];
        r.b[i] = std::sqrt(std::max<double>(s.velocity_variance_m2ps2[i],
                                            p.gnss_velocity_floor_mps[i] *
                                                p.gnss_velocity_floor_mps[i]) +
                           .055 * .055);
      }
    }
    break;
  }
  case PRODUCT_MEASUREMENT_BARO: {
    const auto &s = measurement->payload.baro;
    if (s.sensor_instance_id != measurement->sensor_instance_id)
      return STATUS_INVALID_ARGUMENT;
    h = &s.header;
    calibration = s.calibration_version;
    if (!(p.aiding_mask & PRODUCT_AIDING_BARO))
      return STATUS_UNSUPPORTED;
    if ((s.quality_flags & 7u) != 7u || !std::isfinite(s.altitude_m) ||
        !std::isfinite(s.variance_m2) || s.variance_m2 <= 0 ||
        h->source_id != SOURCE_BARO)
      return STATUS_INVALID_ARGUMENT;
    records[0].id.kind = ins::ObservationKind::kBaroHeight;
    records[0].a[0] = s.altitude_m - nav.baro_zero;
    records[0].b[0] = std::sqrt(
        std::max<double>(s.variance_m2, p.baro_floor_m * p.baro_floor_m) +
        .0165 * .0165);
    break;
  }
  case PRODUCT_MEASUREMENT_MAG: {
    const auto &s = measurement->payload.mag;
    if (s.sensor_instance_id != measurement->sensor_instance_id)
      return STATUS_INVALID_ARGUMENT;
    h = &s.header;
    calibration = s.calibration_version;
    if (!(p.aiding_mask & PRODUCT_AIDING_MAG))
      return STATUS_UNSUPPORTED;
    if ((s.quality_flags & 7u) != (MAG_Q_FIELD_VALID | MAG_Q_CALIBRATED) ||
        !finite_array(s.field_frd_ut, 6) || h->source_id != SOURCE_MAG ||
        h->frame_id != FRAME_BODY_FRD)
      return STATUS_INVALID_ARGUMENT;
    double norm = 0, variance = 0;
    for (int i = 0; i < 3; ++i) {
      norm += s.field_frd_ut[i] * s.field_frd_ut[i];
      if (s.variance_ut2[i] <= 0)
        return STATUS_INVALID_ARGUMENT;
      variance = std::max<double>(variance, s.variance_ut2[i]);
      records[0].a[i] = s.field_frd_ut[i];
    }
    if (norm < 100 || norm > 10000)
      return STATUS_OUT_OF_RANGE;
    records[0].id.kind = ins::ObservationKind::kMagHeading;
    records[0].b[0] =
        std::max(variance /
                     (p.min_horizontal_field_ut * p.min_horizontal_field_ut),
                 p.mag_heading_floor_rad * p.mag_heading_floor_rad) +
        p.magnetic_declination_variance_rad2 + .011 * .011;
    records[0].omega[0] = p.magnetic_declination_rad;
    records[0].omega[1] = p.min_horizontal_field_ut;
    break;
  }
  default:
    return STATUS_UNSUPPORTED;
  }
  source=find_source(measurement->sensor_instance_id);
  const bool normal_allowed=permitted(measurement->sensor_instance_id,*h,now.monotonic_us);
  const bool probe_allowed=source && source->policy.use_state==SENSOR_USE_REJECT &&
    (source->policy.evaluation_flags & SENSOR_USE_EVAL_ALLOW_SHADOW) &&
    same_clock(&source->policy.header,h) &&
    source->policy.effective_time_us<=h->timestamp_sample_us &&
    source->policy.header.valid_until_us>now.monotonic_us && source->policy.policy_expires_at_us>now.monotonic_us;
  if (!header_valid(h, now.monotonic_us) ||
      !same_clock(h, &measurement->header) ||
      h->timestamp_sample_us != measurement->header.timestamp_sample_us ||
      now.monotonic_us - h->timestamp_sample_us > 20000 ||
      calibration != p.calibration_version ||
      (!normal_allowed && !probe_allowed))
    return STATUS_NOT_PERMITTED;
  source->logical_source = h->source_id;
  if (h->source_clock_domain_id) {
    if (!mapping_current(h->source_clock_domain_id,h->source_mapping_epoch,now.monotonic_us))
      return STATUS_STALE_DATA;
  } else if (h->source_mapping_epoch)
    return STATUS_INVALID_ARGUMENT;
  if(source->mapping_domain!=h->source_clock_domain_id || source->mapping_epoch!=h->source_mapping_epoch)
    clear_evidence(*source);
  source->mapping_domain=h->source_clock_domain_id;source->mapping_epoch=h->source_mapping_epoch;
  if(measurement->measurement_kind==PRODUCT_MEASUREMENT_GNSS && nav.ready &&
     !(nav.origin.valid_mask&2u)) {
    if(normal_allowed && (measurement->payload.gnss.quality_flags&
       (GNSS_Q_POSITION_VALID|GNSS_Q_POSITION_COV_VALID))==
       (GNSS_Q_POSITION_VALID|GNSS_Q_POSITION_COV_VALID)) nav.initial_gnss=measurement->payload.gnss;
    return STATUS_NOT_READY;
  }
  if(probe_allowed && (!nav.ready || nav.lost)) return STATUS_NOT_READY;
  if (!nav.ready) {
    if (measurement->measurement_kind == PRODUCT_MEASUREMENT_GNSS)
      nav.initial_gnss = measurement->payload.gnss;
    if (measurement->measurement_kind == PRODUCT_MEASUREMENT_MAG)
      nav.initial_mag = measurement->payload.mag;
    if (measurement->measurement_kind == PRODUCT_MEASUREMENT_BARO &&
        nav.alignment_count) {
      nav.baro_sum += measurement->payload.baro.altitude_m;
      ++nav.baro_count;
      nav.last_baro = h->timestamp_sample_us;
    }
    return STATUS_OK;
  }
  if (nav.lost)
    return STATUS_NOT_READY;
  if (measurement->measurement_kind == PRODUCT_MEASUREMENT_BARO &&
      !nav.baro_reference)
    return STATUS_NOT_READY;
  if (!probe_allowed && nav.history.journalCount() + count > ins::kJournalSlots) {
    reject(STATUS_BUFFER_FULL);
    return STATUS_BUFFER_FULL;
  }
  /* Conservative declared numerical operating domain used for timestamp
     quantization noise: |v|<=30 m/s, |a|<=100 m/s², |omega|<=20 rad/s.
     Do not claim this bound from elapsed arrival time or from PlantTruth. */
  for (int i = 0; i < 3; ++i)
    if (std::fabs(nav.snapshot.core.velocity_ned_mps[i]) > 30 ||
        std::fabs(nav.snapshot.core.acceleration_ned_mps2[i]) > 100 ||
        std::fabs(nav.snapshot.core.body_rate_frd_radps[i]) > 20)
      return STATUS_MODEL_OUT_OF_DOMAIN;
  for (unsigned k = 0; k < count; ++k) {
    auto &r = records[k];
    r.id.boot_id = h->boot_id;
    r.id.source_id = h->source_id;
    r.id.source_sequence = h->sequence;
    r.sample_time_us = h->timestamp_sample_us;
    r.policy_epoch = source->policy.policy_epoch;
    r.noise_profile = source->policy.degraded_noise_profile_id;
    r.calibration_version = calibration;
    r.mapping_epoch = h->source_mapping_epoch;
    if (source->policy.use_state == SENSOR_USE_DEGRADED) {
      if (r.id.kind == ins::ObservationKind::kMagHeading)
        r.b[0] *= 4;
      else
        for (auto &sigma : r.b)
          sigma *= 2;
    }
  }
  if(probe_allowed) {
    if(nav.probe.queued) return STATUS_BUFFER_FULL;
    if(h->timestamp_sample_us<=source->last_evaluated_time) return STATUS_STALE_DATA;
    nav.probe.count=count; nav.probe.sensor_id=measurement->sensor_instance_id;
    nav.probe.header=*h;
    nav.probe.policy_epoch=source->policy.policy_epoch; nav.probe.state_epoch=nav.state_epoch;
    nav.probe.expires=std::min(h->valid_until_us,h->timestamp_sample_us+20000);
    for(unsigned k=0;k<count;++k) nav.probe.records[k]=records[k];
    nav.probe_cursor.cancel(); nav.probe.queued=true;
    return STATUS_OK;
  }
  for(unsigned k=0;k<count;++k) {
    auto &r=records[k];
    const auto accepted = nav.history.acceptObservation(
        r, now.monotonic_us, r.policy_epoch, h->source_mapping_epoch);
    if (accepted != ins::AcceptStatus::kAccepted) {
      reject(STATUS_STALE_DATA);
      return STATUS_STALE_DATA;
    }
  }
  nav.pending = true;
  nav.pending_start = std::min(nav.pending_start, h->timestamp_sample_us);
  return STATUS_OK;
}
static void run_probe(uint64_t now) {
  if(!nav.probe.queued || nav.pending || nav.cursor.active()) return;
  Source *source=find_source(nav.probe.sensor_id);
  if(!source || nav.lost || !nav.ready || now>=nav.probe.expires ||
     source->policy.policy_epoch!=nav.probe.policy_epoch || nav.state_epoch!=nav.probe.state_epoch ||
     source->policy.use_state!=SENSOR_USE_REJECT || source->policy.policy_expires_at_us<=now ||
     !mapping_current(nav.probe.header.source_clock_domain_id,nav.probe.header.source_mapping_epoch,now) ||
     !(source->policy.evaluation_flags&SENSOR_USE_EVAL_ALLOW_SHADOW)) {
    nav.probe_cursor.cancel(); nav.probe.queued=false; return;
  }
  if(nav.probe.records[0].sample_time_us>nav.state_time) return;
  if(nav.probe_cursor.active() && nav.probe.watermark!=nav.history.journalHighWatermark())
    nav.probe_cursor.cancel();
  auto charge=[](uint64_t before,uint64_t after) {
    replay_spent_us+=static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX-replay_spent_us,
        std::max<uint64_t>(1,after-before)));
  };
  const uint32_t limit=replay_budget_us ? replay_budget_us : UINT32_MAX;
  if(!nav.probe_cursor.active() && replay_spent_us<limit) {
    uint32_t index=0; uint64_t endpoint=0;
    const uint64_t start=nav.probe.records[0].sample_time_us>551 ? nav.probe.records[0].sample_time_us-551 : 0;
    if(!nav.history.findCheckpoint(start,&index) ||
       !nav.history.nearestImuTime(nav.probe.records[0].sample_time_us,&endpoint)) return;
    bool found=false;
    for(uint32_t i=0;i<nav.history.imuCount();++i)
      if(nav.history.imuSampleTime(i)==endpoint) { nav.probe.target_sequence=nav.history.imuSequence(i); found=true; break; }
    if(!found) return;
    nav.probe.target_time=endpoint; nav.probe.watermark=nav.history.journalHighWatermark();
    ClockInstant before{},after{};
    if(!clock_now(before)) return;
    auto result=nav.probe_cursor.begin(nav.history,index,nav.fusion_revision,nav.shadow,
        nav.probe.records,nav.probe.count,endpoint);
    if(!clock_now(after) || after.monotonic_us<before.monotonic_us) return;
    charge(before.monotonic_us,after.monotonic_us);
    if(result!=kfg::kOk) { nav.probe.queued=false; return; }
  }
  for(unsigned steps=0;nav.probe_cursor.active() && steps<16 && replay_spent_us<limit;++steps) {
    bool done=false; ClockInstant before{},after{};
    if(!clock_now(before)) return;
    auto result=nav.probe_cursor.step(nav.history,nav.probe.target_time,nav.probe.target_sequence,
        nav.shadow,&nav.candidate,&done);
    if(!clock_now(after) || after.monotonic_us<before.monotonic_us) return;
    charge(before.monotonic_us,after.monotonic_us);
    if(result!=kfg::kOk) { nav.probe_cursor.cancel(); nav.probe.queued=false; return; }
    if(done) {
      /* Deliberately no restore/commit to active. Even ReplayEngine::commit
       * rejects a shadow-only candidate if a caller tries it accidentally. */
      if(after.monotonic_us<nav.probe.expires && nav.candidate.shadow_only &&
         mapping_current(nav.probe.header.source_clock_domain_id,nav.probe.header.source_mapping_epoch,after.monotonic_us))
        publish_evaluations(nav.candidate,2,after.monotonic_us);
      nav.probe.queued=false; return;
    }
  }
}
extern "C" Status ins_model_update(uint64_t now) {
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  ClockInstant clock{};
  if (!clock_now(clock) || now > clock.monotonic_us ||
      now < nav.clock.monotonic_us)
    return STATUS_STALE_DATA;
  Status result = STATUS_OK;
  if(replay_budget_us && now-replay_period_start>=replay_period_us) {
    replay_period_start=now; replay_spent_us=0;
  }
  if (nav.ready && !nav.lost && nav.pending &&
      nav.pending_start <= nav.state_time) {
    nav.probe_cursor.cancel(); /* Active navigation always owns the budget first. */
    /* A queued observation can be newer than the last consumed IMU. Wait
     * for its real boundary; never quantize it backwards to the ring tail. */
    for(uint32_t i=0;i<nav.history.journalCount();++i)
      if(nav.history.journal(i).sample_time_us>nav.state_time) {
        publish(now); return STATUS_NOT_READY;
      }
    uint32_t index = 0;
    const uint64_t start =
        nav.pending_start > 550 ? nav.pending_start - 550 : 0;
    if (!nav.history.findCheckpoint(start, &index))
      result = STATUS_NOT_READY;
    else {
      const auto base = nav.cursor.active() ? nav.cursor.baseTime() : nav.history.checkpoint(index).sample_time_us;
      kfg::Status status=kfg::kOk;
      bool done=true;
      if(replay_budget_us) {
        done=false;
        if(now-replay_period_start>=replay_period_us) {
          replay_period_start=now; replay_spent_us=0;
        }
        if(nav.cursor.active() && nav.candidate.journal_high_watermark!=nav.history.journalHighWatermark())
          nav.cursor.cancel();
        if(!nav.cursor.active() && replay_spent_us<replay_budget_us) {
          ClockInstant before{},after{};
          if(!clock_now(before)) return STATUS_STALE_DATA;
          status=nav.cursor.begin(nav.history,index,nav.fusion_revision,nav.shadow);
          nav.candidate.journal_high_watermark=nav.history.journalHighWatermark();
          if(!clock_now(after) || after.monotonic_us<before.monotonic_us) return STATUS_STALE_DATA;
          replay_spent_us+=static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX-replay_spent_us,
              std::max<uint64_t>(1,after.monotonic_us-before.monotonic_us)));
        }
        /* A hard operation count also bounds work with a broken/frozen clock. */
        for(unsigned steps=0;status==kfg::kOk && !done && steps<16 &&
            replay_spent_us<replay_budget_us;++steps) {
          ClockInstant before{},after{};
          if(!clock_now(before)) { status=kfg::kErrTimeDiscontinuity; break; }
          status=nav.cursor.step(nav.history,nav.state_time,nav.last_imu.sample_sequence,
                                 nav.shadow,&nav.candidate,&done);
          if(!clock_now(after) || after.monotonic_us<before.monotonic_us) {
            status=kfg::kErrTimeDiscontinuity; break;
          }
          replay_spent_us+=static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX-replay_spent_us,
              std::max<uint64_t>(1,after.monotonic_us-before.monotonic_us)));
        }
      } else {
        status=nav.replay.replay(nav.history,index,nav.state_time,nav.last_imu.sample_sequence,
                                 nav.fusion_revision,nav.shadow,&nav.candidate);
      }
      if (status != kfg::kOk)
        result = STATUS_NUMERIC_ERROR;
      else if(!done)
        result = STATUS_NOT_READY;
      else if (nav.candidate.journal_high_watermark !=
               nav.history.journalHighWatermark())
        result = STATUS_STALE_DATA;
      else if (nav.replay.commit(nav.active, nav.candidate, nav.fusion_revision,
                                 nav.last_imu.sample_sequence) != kfg::kOk)
        result = STATUS_NUMERIC_ERROR;
      else {
        publish_evaluations(nav.candidate,1,now);
        ++nav.fusion_revision;
        nav.history.invalidateAfter(base, nav.fusion_revision);
        checkpoint();
        /* Only report source observability after a successful numerical
           update; a queued or NIS-rejected sample is not aiding evidence. */
        const auto *accepted = nav.candidate.accepted_sample_time_us;
        if (accepted[0]) {
          nav.position_observed = true;
          nav.last_gnss = accepted[0];
        }
        if (accepted[1]) {
          nav.velocity_observed = true;
          nav.last_gnss = std::max(nav.last_gnss, accepted[1]);
        }
        if (accepted[2])
          nav.last_baro = accepted[2];
        if (accepted[3]) {
          nav.heading_observed = true;
          nav.last_mag = accepted[3];
        }
      }
    }
    if(result!=STATUS_NOT_READY) {
      nav.pending = false;
      nav.pending_start = UINT64_MAX;
      nav.cursor.cancel();
    }
    if (result && result!=STATUS_NOT_READY)
      reject(result);
  }
  run_probe(now);
  ClockInstant publication{};
  publish(clock_now(publication) ? publication.monotonic_us : now);
  return result;
}
extern "C" Status ins_model_set_replay_budget(uint32_t budget,uint32_t period) {
  if(!period || budget>period) return STATUS_INVALID_ARGUMENT;
  replay_budget_us=budget; replay_period_us=period; replay_spent_us=0; replay_period_start=0;
  if(nav_ptr) nav.cursor.cancel();
  if(nav_ptr) nav.probe_cursor.cancel();
  return STATUS_OK;
}
extern "C" Status ins_model_get_snapshot(NavigationSnapshot *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  ClockInstant clock{};
  if (!clock_now(clock))
    return STATUS_STALE_DATA;
  publish(clock.monotonic_us);
  *out = nav.snapshot;
  return nav.ready ? (out->core.valid_mask ? STATUS_OK : STATUS_STALE_DATA)
                   : STATUS_NOT_READY;
}
extern "C" Status ins_model_get_origin(NavigationOriginSnapshot *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  *out = nav.origin;
  return STATUS_OK;
}
extern "C" Status ins_model_get_capability(NavigationCapability *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  ClockInstant clock{};
  if (!clock_now(clock))
    return STATUS_STALE_DATA;
  publish(clock.monotonic_us);
  *out = nav.capability;
  return STATUS_OK;
}
extern "C" Status ins_model_get_health(EstimatorHealth *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  ClockInstant clock{};
  if (clock_now(clock))
    publish(clock.monotonic_us);
  *out = nav.health;
  return STATUS_OK;
}
extern "C" Status ins_model_push_gap(const SensorDataGapEvent *event) {
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  ClockInstant now{};
  if (!event || !clock_now(now) ||
      !header_valid(&event->header, now.monotonic_us) ||
      !same_clock(&event->header, &nav.snapshot.header) ||
      event->header.message_type != MSG_SENSOR_DATA_GAP_EVENT ||
      !event->sensor_instance_id ||
      (event->last_missing_sequence - event->first_missing_sequence) > 0x7fffffffu ||
      (event->last_good_sample_time_us && event->next_sample_time_us &&
       event->last_good_sample_time_us > event->next_sample_time_us))
    return STATUS_INVALID_ARGUMENT;

  Source *source = find_source(event->sensor_instance_id);
  if (!source)
    return STATUS_NOT_PERMITTED;
  if (source->logical_source && source->logical_source != event->header.source_id)
    return STATUS_INVALID_ARGUMENT;
  if (event->header.source_clock_domain_id) {
    if (!mapping_current(event->header.source_clock_domain_id,
                         event->header.source_mapping_epoch, now.monotonic_us))
      return STATUS_STALE_DATA;
  } else if (event->header.source_mapping_epoch) {
    return STATUS_INVALID_ARGUMENT;
  }

  /* M2 may retry delivery. A gap is identified by sensor + missing range +
     reason + sample-time endpoints, not by the transport header sequence. */
  if (source->gap_seen &&
      source->gap_first_sequence == event->first_missing_sequence &&
      source->gap_last_sequence == event->last_missing_sequence &&
      source->gap_reason == event->reason_code &&
      source->gap_last_good_time == event->last_good_sample_time_us &&
      source->gap_next_sample_time == event->next_sample_time_us)
    return STATUS_OK;

  source->logical_source = event->header.source_id;
  source->gap_seen = true;
  source->gap_first_sequence = event->first_missing_sequence;
  source->gap_last_sequence = event->last_missing_sequence;
  source->gap_reason = event->reason_code;
  source->gap_last_good_time = event->last_good_sample_time_us;
  source->gap_next_sample_time = event->next_sample_time_us;
  clear_evidence(*source);

  if (event->sensor_instance_id == nav.last_imu.sensor_instance_id) {
    if (nav.ready)
      nav.lost = true;
    else {
      reset_alignment();
      nav.builder.reset();
    }
    nav.pending = false;
    nav.pending_start = UINT64_MAX;
    nav.cursor.cancel();
    nav.probe_cursor.cancel();
    nav.probe.queued = false;
  }
  reject(STATUS_STALE_DATA);
  /* Publish the loss immediately; do not wait for the next periodic update. */
  publish(now.monotonic_us);
  return STATUS_OK;
}
extern "C" Status
ins_model_get_sensor_consistency(uint32_t id, SensorConsistencyEvidence *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  Source *s = find_source(id);
  if (!s)
    return STATUS_NOT_READY;
  *out = s->evidence;
  ClockInstant now{};
  if(!clock_now(now) || !header_valid(&out->header,now.monotonic_us) ||
     out->policy_epoch!=s->policy.policy_epoch || out->state_epoch!=nav.state_epoch ||
     out->evidence_expires_at_us<=now.monotonic_us || !out->evaluated_count) {
    out->reason_code=SENSOR_CONSISTENCY_REASON_INSUFFICIENT_SAMPLES;
    out->evaluation_mode=0;
    return STATUS_NOT_READY;
  }
  if(!mapping_current(s->mapping_domain,s->mapping_epoch,now.monotonic_us)) {
    out->reason_code=SENSOR_CONSISTENCY_REASON_TIME_NOT_READY;out->evaluation_mode=0;
    return STATUS_NOT_READY;
  }
  return STATUS_OK;
}
extern "C" Status ins_model_reset(const EstimatorResetRequest *request) {
  if (!nav_ptr || !nav.initialized)
    return STATUS_NOT_READY;
  ClockInstant now{};
  if (!request || !clock_now(now) ||
      !header_valid(&request->header, now.monotonic_us) ||
      !same_clock(&request->header, &nav.snapshot.header) ||
      request->header.source_id != SOURCE_M4 ||
      request->reset_type < RESET_TYPE_INITIALIZE ||
      request->reset_type > RESET_TYPE_RECOVER_AFTER_GAP)
    return STATUS_INVALID_ARGUMENT;
  const bool gnss_assisted=request->initial_source==RESET_SOURCE_GNSS_ASSISTED;
  if (request->initial_source != RESET_SOURCE_STATIC_IMU_LOCAL && !gnss_assisted &&
      request->initial_source != RESET_SOURCE_PROVIDED_STATE)
    return STATUS_UNSUPPORTED;
  const GnssSample seed_gnss=nav.initial_gnss;
  if(gnss_assisted) {
    ContractHeader candidate{}; uint32_t next_origin=0;
    if(request->reset_type!=RESET_TYPE_ORIGIN_REBUILD ||
       ins_model_get_pending_origin(&candidate,&next_origin) ||
       request->requested_origin_id!=next_origin ||
       request->header.timestamp_sample_us!=candidate.timestamp_sample_us) return STATUS_NOT_PERMITTED;
  }
  if (request->initial_source == RESET_SOURCE_PROVIDED_STATE) {
    constexpr uint32_t needed =
        NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE | NAV_VALID_HORIZONTAL_POSITION |
        NAV_VALID_VERTICAL_POSITION | NAV_VALID_HORIZONTAL_VELOCITY |
        NAV_VALID_VERTICAL_VELOCITY;
    const auto &s = request->initial_state;
    double norm = 0;
    for (float q : s.q_nb_wxyz)
      norm += q * q;
    if ((request->valid_initial_state_mask & needed) != needed ||
        !finite_array(s.position_ned_m, 25) || std::fabs(norm - 1) > 1e-3 ||
        nav.last_imu.header.timestamp_sample_us !=
            request->header.timestamp_sample_us ||
        now.monotonic_us - nav.last_imu.header.timestamp_sample_us > 5000 ||
        std::hypot(std::hypot(s.position_ned_m[0], s.position_ned_m[1]),s.position_ned_m[2]) >
            nav.origin.local_radius_limit_m)
      return STATUS_INVALID_ARGUMENT;
  }
  /* Explicit ground/static re-alignment only. No fabricated airborne
     recovery or silently accepted incomplete provided state. */
  nav.reset_event = {};
  nav.reset_event.header = request->header;
  nav.reset_event.header.message_type = MSG_STATE_RESET_EVENT;
  nav.reset_event.header.source_id = SOURCE_M3;
  nav.reset_event.old_state_epoch = nav.state_epoch;
  nav.reset_event.old_origin_id = nav.origin_id;
  nav.reset_event.new_state_epoch = ++nav.state_epoch;
  if (request->reset_type == RESET_TYPE_ORIGIN_REBUILD) {
    nav.origin_id = request->requested_origin_id ? request->requested_origin_id
                                                 : nav.origin_id + 1;
    nav.origin.valid_mask = nav.config.payload.initial_origin_valid_mask;
    nav.origin.latitude_deg = nav.config.payload.initial_latitude_deg;
    nav.origin.longitude_deg = nav.config.payload.initial_longitude_deg;
    nav.origin.ellipsoid_height_m =
        nav.config.payload.initial_ellipsoid_height_m;
    if(gnss_assisted) {
      nav.origin.valid_mask=3;
      nav.origin.latitude_deg=seed_gnss.latitude_deg;
      nav.origin.longitude_deg=seed_gnss.longitude_deg;
      nav.origin.ellipsoid_height_m=seed_gnss.altitude_m;
    }
  }
  nav.reset_event.new_origin_id = nav.origin_id;
  nav.reset_event.reset_type = request->reset_type;
  nav.event_pending = true;
  nav.ready = false;
  nav.lost = false;
  nav.builder.reset();
  nav.history.reset();
  nav.cursor.cancel(); nav.probe_cursor.cancel(); nav.probe.queued=false;
  reset_alignment();
  nav.position_observed = nav.velocity_observed = nav.heading_observed = false;
  nav.initial_gnss = {};
  if(gnss_assisted) nav.initial_gnss=seed_gnss;
  nav.initial_mag = {};
  nav.last_gnss = nav.last_mag = nav.last_baro = 0;
  nav.pending = false;
  nav.pending_start = UINT64_MAX;
  if (request->initial_source == RESET_SOURCE_PROVIDED_STATE) {
    Status status = initialize_core(request->initial_state, true);
    if (status)
      return status;
    nav.position_observed = nav.velocity_observed = true;
    nav.heading_observed =
        (request->valid_initial_state_mask & NAV_VALID_HEADING) != 0;
    kfg::ImuSample seed{};
    seed.sample_time_us = nav.state_time;
    seed.sequence = nav.last_imu.sample_sequence;
    for (int i = 0; i < 3; ++i) {
      seed.gyro_radps(i, 0) = nav.last_imu.gyro_frd_radps[i];
      seed.accel_mps2(i, 0) = nav.last_imu.accel_frd_mps2[i];
    }
    kfg::ImuIncrement unused{};
    nav.builder.push(seed, unused);
    checkpoint();
  }
  publish(now.monotonic_us);
  return STATUS_OK;
}
extern "C" Status ins_model_get_pending_origin(ContractHeader *candidate,uint32_t *next_origin) {
  if(!candidate || !next_origin) return STATUS_INVALID_ARGUMENT;
  *candidate={}; *next_origin=0;
  if(!nav_ptr || !nav.initialized || !nav.ready || nav.lost || (nav.origin.valid_mask&2u) ||
     nav.origin_id==UINT32_MAX) return STATUS_NOT_READY;
  ClockInstant now{}; const auto &s=nav.initial_gnss;
  if(!clock_now(now) || !header_valid(&s.header,now.monotonic_us) ||
     !permitted(s.sensor_instance_id,s.header,now.monotonic_us) ||
     (s.quality_flags&(GNSS_Q_POSITION_VALID|GNSS_Q_POSITION_COV_VALID))!=
       (GNSS_Q_POSITION_VALID|GNSS_Q_POSITION_COV_VALID)) return STATUS_NOT_READY;
  if(s.header.source_clock_domain_id) {
    if(!mapping_current(s.header.source_clock_domain_id,s.header.source_mapping_epoch,now.monotonic_us))
      return STATUS_NOT_READY;
  }
  *candidate=s.header; *next_origin=nav.origin_id+1;
  return STATUS_OK;
}
extern "C" Status ins_model_take_reset_event(StateResetEvent *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  if (!nav_ptr || !nav.event_pending)
    return STATUS_NOT_READY;
  *out = nav.reset_event;
  nav.event_pending = false;
  return STATUS_OK;
}
extern "C" Status ins_model_stop(void) {
  if (!nav_ptr)
    return STATUS_NOT_READY;
  nav.initialized = false;
  nav.ready = false;
  nav.cursor.cancel();
  nav.probe_cursor.cancel(); nav.probe.queued=false;
  nav.builder.reset();
  nav.health.common.health_state = MODULE_HEALTH_STOPPED;
  return STATUS_OK;
}
