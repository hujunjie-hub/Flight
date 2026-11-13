#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "board.h"
#include "board_identity.h"
#include "systime.h"
#include "adis16505.h"
#include "dshot.h"
#include "unicore_ascii/unicore_ascii.h"
#include "mavlink/mavlink_frame.h"
#include "sim/fmt_sim_host.h"
#include "ins_model.h"
#ifdef FLIGHT_GROUND_STATION
#include <windows.h>
#include <signal.h>
#include "ground_station.h"
static volatile sig_atomic_t ground_stop;
static int ground_online;
static int ground_calibration_fixture;
static uint32_t ground_takeoff_request;
static uint64_t ground_rebind_until_us;
static void stop_ground(int signal_number)
{
  (void)signal_number;
  ground_stop = 1;
}
void flight_sil_configure_controller(ProductControllerConfig *controller)
{
  if (ground_online)
    ground_station_apply_config(controller);
}
#endif
/* Execute one real owner turn at a time on a deterministic Host scheduler.
 * No Port is mocked. Only the RTOS/board/time boundary is replaced. */
#include "../../applications/runtime/runtime_data.c"
#include "../../applications/runtime/runtime.c"
#include "../../applications/tasks/task_manager.c"
#include "../../applications/tasks/sensor/task_sensor.c"
#include "../../applications/tasks/mission/task_mission.c"
#include "../../applications/tasks/mpc/task_mpc.c"
#include "../../applications/tasks/actuator/task_actuator.c"
#include "../../applications/tasks/communication/task_comm.c"
#include "../../applications/tasks/communication/task_rc.c"
#include "../../applications/tasks/communication/task_radio.c"
#include "../../applications/tasks/communication/task_gcs.c"
#include "../../applications/tasks/supervisor/task_supervisor.c"
#include "../../middleware/communication/telemetry_link/telemetry_link.c"
static ClockInstant test_clock={0x12345678,1000000,1,0};
static struct imu_raw_sample test_imu;
static int test_imu_pending;
static unsigned checks,failures,sem_calls,commits,inhibits;
static struct dshot_stats sil_dshot_stats;
static uint16_t simulated_rc_channels[16];
static int imu_token, dshot_token, timebase_token, gnss_token_dev, rc_token_dev, radio_token_dev, wdt_token;
static jmp_buf scheduler;
static unsigned seen_packets[14], gnss_log_requests;
static uint64_t last_fmt_us;
static uint64_t fmt_wall_timeout_us = 350000u, fmt_last_state_wall_us, fmt_longest_gap_us;
static unsigned fmt_frames, fmt_active_frames;
static int fmt_online;
static float fmt_hover;
static mavlink_fmt_state_t fmt_state, fmt_origin;
static rt_err_t (*test_tx_complete)(rt_device_t,void *);
#define CHECK(x) do { ++checks; if(!(x)) { ++failures; printf("FAIL %u: %s\n",__LINE__,#x); } } while(0)
rt_base_t rt_hw_interrupt_disable(void) { return 0; }
void rt_hw_interrupt_enable(rt_base_t level) { (void)level; }
rt_base_t rt_enter_critical(void) { return 0; }
void rt_exit_critical(void) {}
int rt_mutex_init(struct rt_mutex *m,const char *s,int f) { (void)m;(void)s;(void)f;return 0; }
int rt_mutex_detach(struct rt_mutex *m) { (void)m;return 0; }
int rt_mutex_take(struct rt_mutex *m,rt_int32_t w) { (void)m;(void)w;return 0; }
int rt_mutex_release(struct rt_mutex *m) { (void)m;return 0; }
int rt_sem_init(struct rt_semaphore *s,const char *n,int v,int f) { (void)s;(void)n;(void)v;(void)f;return 0; }
int rt_sem_detach(struct rt_semaphore *s) { (void)s;return 0; }
int rt_sem_release(struct rt_semaphore *s) { (void)s;return 0; }
int rt_sem_take(struct rt_semaphore *s,rt_tick_t t) { (void)s;(void)t;if(sem_calls++) longjmp(scheduler,1);return 0; }
rt_tick_t rt_tick_get(void) { return (rt_tick_t)(test_clock.monotonic_us/1000); }
rt_tick_t rt_tick_from_millisecond(unsigned ms) { return ms; }
void rt_thread_delay_until(rt_tick_t *l,rt_tick_t t) { (void)l;(void)t;longjmp(scheduler,1); }
rt_err_t rt_thread_mdelay(rt_int32_t ms) { (void)ms;longjmp(scheduler,1);return RT_EOK; }
int rt_thread_init(struct rt_thread *t,const char *n,void (*e)(void *),void *p,void *s,unsigned z,rt_uint8_t pri,unsigned tick) {
  (void)t;(void)n;(void)e;(void)p;(void)s;(void)z;(void)pri;(void)tick;return 0;
}
int rt_thread_startup(struct rt_thread *t) { (void)t;return 0; }
int rt_thread_detach(struct rt_thread *t) { (void)t;return 0; }
rt_device_t rt_device_find(const char *name) {
  if(!strcmp(name,"imu")) return (rt_device_t)&imu_token;
  if(!strcmp(name,"dshot")) return (rt_device_t)&dshot_token;
  if(!strcmp(name,"timebase")) return (rt_device_t)&timebase_token;
  if(!strcmp(name,"uart4")) return (rt_device_t)&gnss_token_dev;
  if(!strcmp(name,"uart3")) return (rt_device_t)&rc_token_dev;
  if(!strcmp(name,"uart2")) return (rt_device_t)&radio_token_dev;
  if(!strcmp(name,"wdt")) return (rt_device_t)&wdt_token;
  return 0;
}
rt_err_t rt_device_open(rt_device_t d,rt_uint16_t f) {
  if(d==(rt_device_t)&radio_token_dev)
    CHECK(f==(RT_DEVICE_OFLAG_RDWR | RT_SERIAL_RX_NON_BLOCKING | RT_SERIAL_TX_NON_BLOCKING));
  if(d==(rt_device_t)&gnss_token_dev || d==(rt_device_t)&rc_token_dev)
    CHECK(f==(RT_DEVICE_OFLAG_RDWR | RT_SERIAL_RX_NON_BLOCKING | RT_SERIAL_TX_BLOCKING));
  return RT_EOK;
}
rt_err_t rt_device_close(rt_device_t d) { (void)d;return RT_EOK; }
rt_err_t rt_device_control(rt_device_t d,int c,void *p) {
  if(d==(rt_device_t)&dshot_token) {
    if(c==DSHOT_CTRL_GET_STATS && p) *(struct dshot_stats *)p=sil_dshot_stats;
    if(c==DSHOT_CTRL_STOP) {
      ++inhibits;
      sil_dshot_stats.active=0u;
      sil_dshot_stats.pending=0u;
    }
    return RT_EOK;
  }
  if(c==RT_SERIAL_CTRL_GET_CONFIG) ((struct serial_configure *)p)->baud_rate=115200u;
  if(d==(rt_device_t)&gnss_token_dev && c==RT_DEVICE_CTRL_CONFIG)
    CHECK(((const struct serial_configure *)p)->baud_rate==460800u);
  return RT_EOK;
}
rt_err_t rt_device_set_rx_indicate(rt_device_t d,rt_err_t (*cb)(rt_device_t,rt_size_t)) {
  (void)d;(void)cb;return RT_EOK;
}
/* Flight's own SIL scenario keeps these deterministic stubs. The Simulink
 * adapter defines FLIGHT_HOST_REAL_TIME_SERVICE and links the firmware time
 * service instead, so the MIL exercises systime.c rather than a stub. */
#ifndef FLIGHT_HOST_REAL_TIME_SERVICE
Status clock_read(ClockInstant *c) { if(!c)return STATUS_INVALID_ARGUMENT;*c=test_clock;return 0; }
#endif
uint64_t board_boot_id(void) { return test_clock.boot_id; }
#ifndef FLIGHT_HOST_REAL_TIME_SERVICE
Status clock_now_us(uint64_t *now) { *now=test_clock.monotonic_us;return 0; }
Status time_get_mapping_status(uint32_t id,ClockDomainMappingStatus *s) { (void)id;memset(s,0,sizeof(*s));return STATUS_NOT_READY; }
Status time_get_latest_pps(PpsCaptureEvent *out) { (void)out;return STATUS_NOT_READY; }
Status time_submit_gnss_time(const GnssTimeObservation *o) { (void)o;return STATUS_NOT_READY; }
Status clock_ticks_to_mono_us(uint64_t ticks,uint32_t epoch,uint64_t *out) {
  if(epoch!=test_clock.time_epoch) return STATUS_STALE_DATA;
  *out=ticks;return STATUS_OK;
}
Status time_get_status(TimeServiceStatus *out) {
  memset(out,0,sizeof(*out));out->time_epoch=test_clock.time_epoch;out->local_clock_valid=1;return STATUS_OK;
}
Status time_get_local_discipline_status(LocalTimebaseDisciplineStatus *out) {
  memset(out,0,sizeof(*out));return STATUS_OK;
}
#endif
rt_err_t rt_device_set_tx_complete(rt_device_t d,rt_err_t (*callback)(rt_device_t,void *)) {
  (void)d;test_tx_complete=callback;return RT_EOK;
}
rt_ssize_t rt_device_read(rt_device_t d,int pos,void *data,size_t length) {
  (void)pos;
  if(d==(rt_device_t)&imu_token) {
    if(!test_imu_pending || length<sizeof(test_imu)) return 0;
    memcpy(data,&test_imu,sizeof(test_imu));test_imu_pending=0;
    return (rt_ssize_t)sizeof(test_imu);
  }
  (void)data;(void)length;return 0;
}
rt_ssize_t rt_device_write(rt_device_t d,int pos,const void *data,size_t length) {
  const uint8_t *p=(const uint8_t *)data;unsigned id,slot,j;
  (void)pos;
  if(d==(rt_device_t)&gnss_token_dev) {
    static const char expected[]="BESTNAVA 0.1\r\nRECTIMEA 1\r\n";
    CHECK(length==sizeof(expected)-1 && !memcmp(data,expected,sizeof(expected)-1));
    gnss_log_requests++;
  }
  if(d==(rt_device_t)&rc_token_dev) {
    static const uint8_t bind[] = {0xc8, 7, 0x32, 0xec, 0xc8, 0x10, 1, 0x9e, 0xe8};
    CHECK(length == sizeof(bind) && memcmp(data, bind, sizeof(bind)) == 0);
    return length == sizeof(bind) && memcmp(data, bind, sizeof(bind)) == 0 ? (rt_ssize_t)length : 0;
  }
  if(d==(rt_device_t)&dshot_token) {
    if(length==8) {
      ++commits;
      sil_dshot_stats.frames++;
      sil_dshot_stats.completed_frames=sil_dshot_stats.frames;
      sil_dshot_stats.active=0u;
      sil_dshot_stats.pending=0u;
    }
    return (rt_ssize_t)length;
  }
  if(d==(rt_device_t)&radio_token_dev) {
    CHECK(length>=12 && p[0]==0xfd);CHECK(p==tx_frame && length<=sizeof(tx_frame));
    id=p[7]|((unsigned)p[8]<<8)|((unsigned)p[9]<<16);
    slot=id==0 ? 13 : id>=43000 && id<=43012 ? id-43000 : 14;
    CHECK(slot<14 || id==1 || id==PARAM_VALUE || id==24 || id==31 || id==32 ||
          id==74 /* VFR_HUD */ || id==MAVLINK_ALTITUDE_ID ||
          id==EXTENDED_SYS_STATE || id==253 /* STATUSTEXT */ || id==65 /* RC_CHANNELS */);
    if(slot<14 && !seen_packets[slot]++) {
      printf("PACKET=");for(j=0;j<length;++j) printf("%02x",p[j]);printf("\n");
    }
    if(test_tx_complete) test_tx_complete(d,RT_NULL);
  }
  return (rt_ssize_t)length;
}
static void test_inhibit(void) { inhibits++; }
static void turn(void (*owner)(void *)) { sem_calls=0;if(!setjmp(scheduler)) owner(0); }
static ContractHeader h(unsigned source,uint64_t expiry) {
  ContractHeader header;local_header(&header,source,0,test_clock.monotonic_us);
  header.valid_until_us=expiry;header.sequence=1;return header;
}
static void imu_frame(unsigned sequence) {
  uint8_t b[32]={0}; unsigned i,checksum=0;
  unsigned axis = 2;
  double acceleration = -9.80665;
#ifdef FLIGHT_GROUND_STATION
  if (ground_calibration_fixture && !fmt_online) {
    SensorCalibrationStatus status;
    sensor_calibration_get_status(&status);
    if (status.kind == SENSOR_CAL_ACCEL && status.state == SENSOR_CAL_RUNNING) {
      unsigned face = 0;
      while (face < 5 && (status.face_mask & (1u << face)))
        ++face;
      const unsigned axes[] = {2, 2, 0, 0, 1, 1};
      axis = axes[face];
      if (face == 1 || face == 2 || face == 4)
        acceleration = -acceleration;
    }
  }
#endif
  const uint32_t v = (uint32_t)(int32_t)lround(acceleration * 26756268.0);
  const unsigned offset = 14u + axis * 4u;
  b[offset]=(uint8_t)(v>>8);b[offset+1]=(uint8_t)v;b[offset+2]=(uint8_t)(v>>24);b[offset+3]=(uint8_t)(v>>16);
  b[26]=0x09;b[27]=0xc4;b[28]=(uint8_t)(sequence>>8);b[29]=(uint8_t)sequence;
  for(i=0;i<30;++i) checksum+=b[i];
  b[30]=(uint8_t)(checksum>>8);b[31]=(uint8_t)checksum;
  memset(&test_imu,0,sizeof(test_imu));
  memcpy(test_imu.data,b,sizeof(b));
  test_imu.length=32;test_imu.sequence=sequence;
  test_imu.capture_tick=test_clock.monotonic_us;
  test_imu.flags=IMU_FLAG_VALID;
  test_imu_pending=1;
}
static void feed_gnss(unsigned sequence) {
  SensorMeasurement m; unsigned axis;
  memset(&m,0,sizeof(m));m.header=h(SOURCE_GNSS,test_clock.monotonic_us+20000);
  m.header.frame_id=FRAME_WGS84_GEODETIC;m.header.sequence=sequence;
  m.measurement_kind=PRODUCT_MEASUREMENT_GNSS;m.sensor_instance_id=4;
  m.payload.gnss.header=m.header;m.payload.gnss.sensor_instance_id=4;
  m.payload.gnss.calibration_version=1;m.payload.gnss.quality_flags=15;
  m.payload.gnss.fix_type=GNSS_FIX_RTK_FIXED;
  m.payload.gnss.latitude_deg=0.0;m.payload.gnss.longitude_deg=0.0;m.payload.gnss.altitude_m=0;
  if (fmt_online) {
    /* Position and velocity use the same local plant frame. Invert the WGS84
     * local mapping at the fixture's equatorial origin. */
    const double radians_to_degrees = 57.29577951308232;
    const double meridian_radius_m = 6335439.32729282;
    const double prime_vertical_radius_m = 6378137.0;
    m.payload.gnss.latitude_deg = ((double)fmt_state.position_ned_m[0] - fmt_origin.position_ned_m[0]) *
                                  radians_to_degrees / meridian_radius_m;
    m.payload.gnss.longitude_deg = ((double)fmt_state.position_ned_m[1] - fmt_origin.position_ned_m[1]) *
                                   radians_to_degrees / prime_vertical_radius_m;
    m.payload.gnss.altitude_m = fmt_origin.position_ned_m[2] - fmt_state.position_ned_m[2];
    memcpy(m.payload.gnss.velocity_ned_mps, fmt_state.velocity_ned_mps,
           sizeof(fmt_state.velocity_ned_mps));
  }
  for(axis=0;axis<3;++axis) {
    m.payload.gnss.position_variance_m2[axis]=.04f;
    m.payload.gnss.velocity_variance_m2ps2[axis]=.01f;
  }
  CHECK(runtime_submit_measurement(&m)==STATUS_OK);
}

/* ======================================================================
 * SIL flight-logic scenario
 *
 * Runs the production runtime + owners (no Port is mocked) against a
 * deterministic clock and a synthesized sensor set, then submits the same
 * OperationRequest/OperatorInput that a protocol adapter would deliver. It
 * answers one question with evidence: can the composed firmware reach
 * ARM and drive the control chain?
 * ==================================================================== */

static unsigned sil_requests;
static const float mission_waypoints[2][3] = {{1.3f, 0.0f, -2.0f}, {1.3f, 1.3f, -2.0f}};
enum { SIL_MISSION_ID = 17, SIL_MISSION_TRANSACTION = 1, SIL_COMMAND_SESSION = 0x5151 };
static uint32_t sil_mission_transaction = SIL_MISSION_TRANSACTION;

static int fmt_fresh(void)
{
  return last_fmt_us != 0 && test_clock.monotonic_us - last_fmt_us < 350000u;
}

static unsigned fmt_wait_state(void)
{
  while (fmt_sim_host_now_us() - fmt_last_state_wall_us < fmt_wall_timeout_us) {
    unsigned frames = fmt_sim_host_receive(&fmt_state);
    if (frames) {
      const uint64_t received_us = fmt_sim_host_now_us();
      const uint64_t gap_us = received_us - fmt_last_state_wall_us;
      if (gap_us >= fmt_wall_timeout_us)
        break;
      fmt_last_state_wall_us = received_us;
      if (gap_us > fmt_longest_gap_us)
        fmt_longest_gap_us = gap_us;
      return frames;
    }
    fmt_sim_host_sleep();
  }
  return 0;
}

static float fmt_height(void)
{
  return fmt_origin.position_ned_m[2] - fmt_state.position_ned_m[2];
}

uint8_t flight_sil_ground_contact(void)
{
  if (!fmt_online)
    return GROUND_CONTACT_CONTACT;
  if (!fmt_fresh())
    return GROUND_CONTACT_UNKNOWN;
  return fabsf(fmt_height()) < .06f && fabsf(fmt_state.velocity_ned_mps[2]) < .2f ?
      GROUND_CONTACT_CONTACT : GROUND_CONTACT_NO_CONTACT;
}

static void earth_to_body(const float earth[3], float body[3])
{
  const float *q = fmt_state.quaternion;
  const float cross[3] = {
    2.0f * (q[3] * earth[1] - q[2] * earth[2]),
    2.0f * (q[1] * earth[2] - q[3] * earth[0]),
    2.0f * (q[2] * earth[0] - q[1] * earth[1]),
  };
  body[0] = earth[0] + q[0] * cross[0] - q[2] * cross[2] + q[3] * cross[1];
  body[1] = earth[1] + q[0] * cross[1] - q[3] * cross[0] + q[1] * cross[2];
  body[2] = earth[2] + q[0] * cross[2] - q[1] * cross[1] + q[2] * cross[0];
}

static void fmt_imu(unsigned sequence)
{
  static float accel[3], gyro[3];
  static int initialized;
  ImuSample sample = {0};
  sample.header = h(SOURCE_IMU, test_clock.monotonic_us + 5000u);
  sample.header.message_type = MSG_IMU_SAMPLE;
  sample.header.frame_id = FRAME_BODY_FRD;
  sample.header.sequence = sequence;
  sample.header.timestamp_quality = TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED;
  sample.sensor_instance_id = 1;
  sample.sample_sequence = sequence;
  sample.calibration_version = 1;
  sample.quality_flags = IMU_Q_GYRO_VALID | IMU_Q_ACCEL_VALID | IMU_Q_CALIBRATED;
  earth_to_body(fmt_state.specific_force_ned_mps2, sample.accel_frd_mps2);
  /* FMT publishes state at the frame rate. Smooth the zero-order hold before
   * presenting a synthetic 1 kHz sensor to the rate controller. */
  const float alpha = initialized ? .05f : 1.0f;
  for (unsigned axis = 0; axis < 3; ++axis) {
    accel[axis] += alpha * (sample.accel_frd_mps2[axis] - accel[axis]);
    gyro[axis] += alpha * (fmt_state.gyro_radps[axis] - gyro[axis]);
    sample.accel_frd_mps2[axis] = accel[axis];
    sample.gyro_frd_radps[axis] = gyro[axis];
  }
  initialized = 1;
  CHECK(sensor_imu_inject(&sample, test_clock.monotonic_us) == STATUS_OK);
}

static int current_commit(void)
{
  const ActuatorCommitReceipt *receipt = &runtime_data.receipt;
  const ActuationPermit *permit = &runtime_data.bundle.permit;
  return receipt->result_code == ACTUATOR_RECEIPT_COMMITTED &&
         receipt->receipt_expires_at_us > test_clock.monotonic_us &&
         permit->permit_flags == PERMIT_SCOPE_CONTROL_ACTIVE &&
         permit->permit_epoch == receipt->permit_epoch &&
         permit->permit_expires_at_us > test_clock.monotonic_us;
}

static void fmt_send_committed(int motion_requested)
{
  const ActuatorCommitReceipt *receipt = &runtime_data.receipt;
  float controls[4] = {0};
  int armed = motion_requested && fmt_fresh() && current_commit();
  if (armed) {
    /* FMT Plant X_Frame_CM orders the channels FR, RL, FL, RR. */
    static const unsigned fmt_to_flight_motor[4] = {1, 3, 0, 2};
    const float hover_thrust_n = config.controller.payload.mass_kg * 9.80665f / 4.0f;
    for (unsigned channel = 0; channel < 4; ++channel) {
      unsigned motor = fmt_to_flight_motor[channel];
      float fraction = fmt_hover * sqrtf(fmaxf(0.0f, receipt->committed_demand.motor_thrust_n[motor]) /
                                         hover_thrust_n);
      controls[channel] = 2.0f * fminf(1.0f, fraction) - 1.0f;
    }
  }
  if (fmt_sim_host_send(test_clock.monotonic_us, controls, armed) && armed)
    ++fmt_active_frames;
}

/* The runtime owns the guard driver (actuator_device_*); the dshot device stub
 * records every committed frame, so commits are observed without replacing it. */

static void feed_baro(unsigned sequence, float altitude_m)
{
  SensorMeasurement m;
  memset(&m, 0, sizeof(m));
  m.header = h(SOURCE_BARO, test_clock.monotonic_us + 20000);
  m.header.sequence = sequence;
  m.measurement_kind = PRODUCT_MEASUREMENT_BARO;
  m.sensor_instance_id = 3;
  m.payload.baro.header = m.header;
  m.payload.baro.sensor_instance_id = 3;
  m.payload.baro.calibration_version = 1;
  m.payload.baro.altitude_m = altitude_m;
  m.payload.baro.variance_m2 = .25f;
  m.payload.baro.quality_flags = 7;
  CHECK(runtime_submit_measurement(&m) == STATUS_OK);
#ifdef FLIGHT_GROUND_STATION
  if (ground_online) {
    const float pressure_pa = 101325.0f * powf(1.0f - altitude_m / 44330.0f, 5.255f);
    sensor_calibration_feed_baro(pressure_pa, test_clock.monotonic_us);
  }
#endif
}

static void feed_operator(uint16_t mode_bits, float throttle)
{
  OperatorInput in;
  memset(&in, 0, sizeof(in));
  in.header = h(SOURCE_RC, test_clock.monotonic_us + 50000);
  in.header.message_type = MSG_OPERATOR_INPUT;
  in.header.sequence = ++sil_requests;
  in.input_layout_id = PRODUCT_OPERATOR_INPUT_LAYOUT_ID;
  in.valid_axis_mask = 15;
  in.payload.throttle_norm = throttle;
  for (unsigned i = 0; i < 16; ++i)
    simulated_rc_channels[i] = CRSF_RAW_CENTER;
  simulated_rc_channels[2] = CRSF_RAW_MIN +
      (uint16_t)(throttle * (CRSF_RAW_MAX - CRSF_RAW_MIN));
  in.payload.mode_switch_bits = mode_bits;
#ifdef FLIGHT_GROUND_STATION
  if (ground_online) {
    if (!mode_bits)
      in.payload.mode_switch_bits = comm_rc_mapped_mode(simulated_rc_channels);
    if (ground_calibration_fixture) {
      SensorCalibrationStatus cal;
      sensor_calibration_get_status(&cal);
      if (cal.kind == SENSOR_CAL_RC && cal.state == SENSOR_CAL_RUNNING) {
        /* Drive full stick travel then centre, so the firmware can finish. */
        static const uint16_t travel[7][4] = {
          { CRSF_RAW_MAX,    CRSF_RAW_CENTER, CRSF_RAW_MIN, CRSF_RAW_CENTER },
          { CRSF_RAW_MIN,    CRSF_RAW_CENTER, CRSF_RAW_MIN, CRSF_RAW_CENTER },
          { CRSF_RAW_CENTER, CRSF_RAW_MAX,    CRSF_RAW_MIN, CRSF_RAW_CENTER },
          { CRSF_RAW_CENTER, CRSF_RAW_MIN,    CRSF_RAW_MIN, CRSF_RAW_CENTER },
          { CRSF_RAW_CENTER, CRSF_RAW_CENTER, CRSF_RAW_MAX, CRSF_RAW_MAX },
          { CRSF_RAW_CENTER, CRSF_RAW_CENTER, CRSF_RAW_MIN, CRSF_RAW_MIN },
          { CRSF_RAW_CENTER, CRSF_RAW_CENTER, CRSF_RAW_MIN, CRSF_RAW_CENTER }
        };
        const uint64_t since_us = test_clock.monotonic_us - (uint64_t)cal.started_us;
        /* Hold centre for the first 3 s so the firmware captures the trim, then
         * sweep the four channels, and settle back to centre + low throttle so
         * the save snapshot satisfies the firmware's final-position check. */
        const unsigned phase = since_us < 3000000u ? 5u :
            (since_us > 8000000u ? 6u : (unsigned)(((since_us - 3000000u) / 150000u) % 7u));
        for (unsigned i = 0; i < 4; ++i)
          simulated_rc_channels[i] = travel[phase][i];
      }
    }
    sensor_calibration_feed_rc(simulated_rc_channels, test_clock.monotonic_us);
  }
#endif
  CHECK(runtime_submit_operator(&in) == STATUS_OK);
}

static void feed_mag(unsigned sequence)
{
  SensorMeasurement m;
  memset(&m, 0, sizeof(m));
  m.header = h(SOURCE_MAG, test_clock.monotonic_us + 100000);
  m.header.frame_id = FRAME_BODY_FRD;
  m.header.sequence = sequence;
  m.measurement_kind = PRODUCT_MEASUREMENT_MAG;
  m.sensor_instance_id = 2;
  m.payload.mag.header = m.header;
  m.payload.mag.sensor_instance_id = 2;
  m.payload.mag.calibration_version = 1;
  m.payload.mag.quality_flags = MAG_Q_FIELD_VALID | MAG_Q_CALIBRATED;
  m.payload.mag.field_frd_ut[0] = 20.0f;
  m.payload.mag.field_frd_ut[1] = 5.0f;
  m.payload.mag.field_frd_ut[2] = 42.0f;
  if (fmt_online) {
    /* The virtual magnetic field must have the declination used by the INS.
     * An arbitrary east component creates a heading bias even on the ground. */
    const double declination = config.estimator.payload.magnetic_declination_rad;
    const float earth_field_ut[3] = {
        (float)(20.0 * cos(declination)), (float)(20.0 * sin(declination)), 42.0f};
    earth_to_body(earth_field_ut, m.payload.mag.field_frd_ut);
  }
  m.payload.mag.variance_ut2[0] = 1.0f;
  m.payload.mag.variance_ut2[1] = 1.0f;
  m.payload.mag.variance_ut2[2] = 1.0f;
  CHECK(runtime_submit_measurement(&m) == STATUS_OK);
#ifdef FLIGHT_GROUND_STATION
  if (ground_online) {
    static const float directions[6][3] = {
      {100,0,0}, {-100,0,0}, {0,100,0}, {0,-100,0}, {0,0,100}, {0,0,-100}
    };
    const float *field = ground_calibration_fixture && !fmt_online ?
                         directions[(sequence / 10u) % 6u] : m.payload.mag.field_frd_ut;
    sensor_calibration_feed_mag(field, test_clock.monotonic_us);
  }
#endif
}

static void stage_mission(void)
{
  MissionUploadCandidate candidate = {0};
  candidate.header = h(SOURCE_GCS, test_clock.monotonic_us + 200000u);
  candidate.header.message_type = MSG_MISSION_UPLOAD_CANDIDATE;
  candidate.header.sequence = ++sil_requests;
  candidate.transaction_id = SIL_MISSION_TRANSACTION;
  candidate.source_session_id = SIL_COMMAND_SESSION;
  candidate.origin_id = runtime_data.nav.origin_id;
  candidate.state_epoch = runtime_data.nav.state_epoch;
  candidate.mission_hash = 0x9e42u;
  candidate.mission.mission_id = SIL_MISSION_ID;
  candidate.mission.mission_revision = 1u;
  candidate.mission.mission_hash = candidate.mission_hash;
  candidate.mission.waypoint_count = 2u;
  for (unsigned i = 0; i < candidate.mission.waypoint_count; ++i) {
    ProductMissionWaypoint *waypoint = &candidate.mission.waypoints[i];
    waypoint->waypoint_id = i + 1u;
    memcpy(waypoint->position_ned_m, mission_waypoints[i], sizeof(waypoint->position_ned_m));
    waypoint->acceptance_radius_m = .35f;
    waypoint->speed_limit_mps = 2.0f;
  }
  CHECK(runtime_submit_mission(&candidate) == STATUS_OK);
}

static uint32_t submit_command_with_target(uint16_t command, uint32_t prearm_epoch,
                                           float altitude_m, const float *target_ned)
{
  OperationRequest req;
  memset(&req, 0, sizeof(req));
  req.header = h(SOURCE_GCS, test_clock.monotonic_us + 200000);
  req.header.message_type = MSG_OPERATION_REQUEST;
  req.header.sequence = ++sil_requests;
  req.request_id = 1000u + sil_requests;
  req.source_session_id = SIL_COMMAND_SESSION;
  req.command_code = command;
  req.authority_class = 0;
  req.execution_deadline_us = test_clock.monotonic_us + 150000u;
  if (command == PRODUCT_CMD_ARM) {
    req.payload.valid_mask = PRODUCT_OPERATION_VALID_PREARM_EPOCH;
    req.payload.prearm_check_epoch = prearm_epoch;
  } else if (command == PRODUCT_CMD_TAKEOFF) {
    req.payload.valid_mask = PRODUCT_OPERATION_VALID_ALTITUDE_ABOVE_HOME;
    req.payload.requested_altitude_m = altitude_m;
  } else if (command == PRODUCT_CMD_SET_HOME) {
    req.payload.valid_mask = PRODUCT_OPERATION_VALID_TARGET_NED;
    memcpy(req.payload.target_ned_m, target_ned ? target_ned : runtime_data.nav.core.position_ned_m,
           sizeof(req.payload.target_ned_m));
  } else if (command == PRODUCT_CMD_MISSION_COMMIT) {
    req.payload.valid_mask = PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_TRANSACTION;
    req.payload.mission_id = SIL_MISSION_ID;
    req.payload.mission_transaction_id = sil_mission_transaction;
  } else if (command >= PRODUCT_CMD_MISSION_START && command <= PRODUCT_CMD_MISSION_CLEAR &&
             !(command == PRODUCT_CMD_MISSION_CLEAR &&
               fms_output.mission_status.mission_state == MISSION_STATE_IDLE)) {
    req.payload.valid_mask = PRODUCT_OPERATION_VALID_MISSION_ID | PRODUCT_OPERATION_VALID_MISSION_EPOCH;
    req.payload.mission_id = SIL_MISSION_ID;
    req.payload.mission_epoch = fms_output.mission_status.mission_epoch;
  }
  const Status status = runtime_submit_request(&req);
  if (status != STATUS_OK)
    return 0;
  printf("[command] code=%u request=%u\n", command, req.request_id);
  return req.request_id;
}

static uint32_t submit_command(uint16_t command, uint32_t prearm_epoch, float altitude_m)
{
  return submit_command_with_target(command, prearm_epoch, altitude_m, NULL);
}

#ifdef FLIGHT_GROUND_STATION
static uint32_t ground_set_home(const float target_ned[3])
{
  return submit_command_with_target(PRODUCT_CMD_SET_HOME, 0u, 0.0f, target_ned);
}

static uint32_t ground_request(uint16_t command, float altitude_m)
{
  static uint64_t mission_transition_until_us;
  const int transition = command >= PRODUCT_CMD_MISSION_START &&
                         command <= PRODUCT_CMD_MISSION_ABORT;
  if (transition && (test_clock.monotonic_us < mission_transition_until_us ||
                     test_clock.monotonic_us < ground_rebind_until_us || !current_commit()))
    return 0; /* A fresh M7 receipt must back the mode being left. */
  if (command == PRODUCT_CMD_MISSION_RESUME) {
    const NavStateCore *nav = &runtime_data.nav.core;
    const float *velocity = nav->velocity_ned_mps;
    const float *acceleration = nav->acceleration_ned_mps2;
    if (hypotf(velocity[0], velocity[1]) > 0.25f || fabsf(velocity[2]) > 0.15f ||
        hypotf(acceleration[0], acceleration[1]) > 1.0f || fabsf(acceleration[2]) > 0.5f)
      return 0; /* Wait for a stable hover before requesting a new trajectory. */
  }
  if (command == PRODUCT_CMD_LAND &&
      (fms_output.mission_status.mission_state == MISSION_STATE_COMPLETED ||
       fms_output.mission_status.mission_state == MISSION_STATE_ABORTED) &&
      runtime_data.fms.failsafe_level < FAILSAFE_ACTION_REQUIRED &&
      test_clock.monotonic_us < ground_rebind_until_us)
    return 0; /* Allow the post-mission hold to regain a continuous reference. */
  const uint32_t id = submit_command(command, runtime_data.prearm.check_epoch, altitude_m);
  if (transition && id)
    mission_transition_until_us = test_clock.monotonic_us + 100000u;
  if (command == PRODUCT_CMD_TAKEOFF)
    ground_takeoff_request = id;
  return id;
}

static uint32_t ground_commit_mission(const ProductMissionPayload *mission)
{
  MissionUploadCandidate candidate = {0};
  candidate.header = h(SOURCE_GCS, test_clock.monotonic_us + 200000u);
  candidate.header.message_type = MSG_MISSION_UPLOAD_CANDIDATE;
  candidate.header.sequence = ++sil_requests;
  candidate.transaction_id = ++sil_mission_transaction;
  candidate.source_session_id = SIL_COMMAND_SESSION;
  candidate.origin_id = runtime_data.nav.origin_id;
  candidate.state_epoch = runtime_data.nav.state_epoch;
  candidate.mission_hash = mission->mission_hash;
  candidate.mission = *mission;
  candidate.mission.mission_id = SIL_MISSION_ID;
  if (runtime_submit_mission(&candidate) != STATUS_OK)
    return 0;
  return ground_request(PRODUCT_CMD_MISSION_COMMIT, 0.0f);
}

static void ground_write_rc_channels(uint8_t *payload, uint32_t now_ms)
{
  write_standard_rc_channels(payload, now_ms, simulated_rc_channels, 16u, UINT8_MAX);
}

static void ground_write_sensor_status(uint8_t *payload, int prearm_ready)
{
  write_standard_sensor_status(payload, prearm_ready, 1);
}

static const GroundStationActions ground_actions = {
  .command = ground_request,
  .set_home = ground_set_home,
  .commit_mission = ground_commit_mission,
  .replace_geofences = fms_sil_replace_geofences,
  .write_sensor_status = ground_write_sensor_status,
  .send_crsf_bind = elrs_send_bind,
  .write_rc_channels = ground_write_rc_channels
};

typedef struct {
  unsigned ms, mode, mission, item, permit, permit_reason;
  unsigned guidance, guidance_reason, constraint;
  int control;
  float velocity[3], acceleration[3];
} GroundTrace;
static GroundTrace ground_trace[8];
static unsigned ground_trace_index;

static void trace_ground_transition(unsigned ms)
{
  GroundTrace *trace = &ground_trace[ground_trace_index++ % 8u];
  *trace = (GroundTrace){
    .ms = ms, .mode = runtime_data.fms.flight_mode, .mission = fms_output.mission_status.mission_state,
    .item = fms_output.mission_status.current_item_id, .permit = fms_output.actuation_permit.permit_flags,
    .permit_reason = fms_output.actuation_permit.reason_code,
    .guidance = plan_bundle.guidance.guidance_status.guidance_state,
    .guidance_reason = plan_bundle.guidance.guidance_status.reason_code,
    .constraint = plan_bundle.guidance.feasibility.violated_constraint_id,
    .control = runtime_stats.control_status
  };
  memcpy(trace->velocity, runtime_data.nav.core.velocity_ned_mps, sizeof(trace->velocity));
  memcpy(trace->acceleration, runtime_data.nav.core.acceleration_ned_mps2, sizeof(trace->acceleration));
}

static void dump_ground_transition(void)
{
  const unsigned first = ground_trace_index > 8u ? ground_trace_index - 8u : 0u;
  for (unsigned i = first; i < ground_trace_index; ++i) {
    const GroundTrace *t = &ground_trace[i % 8u];
    printf("[ground] t=%u mode=%u mission=%u item=%u permit=%u/%u guidance=%u/%u/%u control=%d "
           "v=(%.3f,%.3f,%.3f) a=(%.3f,%.3f,%.3f)\n", t->ms, t->mode, t->mission, t->item,
           t->permit, t->permit_reason, t->guidance, t->guidance_reason, t->constraint, t->control,
           (double)t->velocity[0], (double)t->velocity[1], (double)t->velocity[2],
           (double)t->acceleration[0], (double)t->acceleration[1], (double)t->acceleration[2]);
  }
}

static void trace_ground_navigation(unsigned ms)
{
  const NavStateCore *nav = &runtime_data.nav.core;
  NavigationCapability cap = {0};
  SensorConsistencyEvidence gnss = {0};
  const float *q = runtime_data.nav.core.q_nb_wxyz;
  const float *truth_q = fmt_state.quaternion;
  const float dot = fabsf(q[0]*truth_q[0] + q[1]*truth_q[1] + q[2]*truth_q[2] + q[3]*truth_q[3]);
  (void)ins_model_get_capability(&cap);
  (void)ins_model_get_sensor_consistency(4u, &gnss);
  printf("[navigation] t=%u truth_NE=(%.3f,%.3f) estimate_NE=(%.3f,%.3f) "
         "truth_V_NE=(%.3f,%.3f) estimate_V_NE=(%.3f,%.3f) pos_std=%.4f "
         "attitude_error_deg=%.2f gnss=%u/%u reason=%u sample_ms=%llu\n", ms,
         (double)(fmt_state.position_ned_m[0] - fmt_origin.position_ned_m[0]),
         (double)(fmt_state.position_ned_m[1] - fmt_origin.position_ned_m[1]),
         (double)nav->position_ned_m[0], (double)nav->position_ned_m[1],
         (double)fmt_state.velocity_ned_mps[0], (double)fmt_state.velocity_ned_mps[1],
         (double)nav->velocity_ned_mps[0], (double)nav->velocity_ned_mps[1],
         (double)cap.horizontal_position_std_m, (double)(114.59156f * acosf(fminf(dot, 1.0f))),
         gnss.accepted_count, gnss.rejected_count,
         gnss.reason_code, (unsigned long long)(gnss.last_evaluated_sample_time_us / 1000u));
}
#endif

static void sil_print_state(const char *tag)
{
  RuntimeStats s;
  FmsStateSnapshot fms;
  PreArmCheckResult pre;
  ActuatorCapability acap;
  ActuatorStatus status;
  NavigationSnapshot nav;
  NavigationCapability ncap;
  runtime_get_stats(&s);
  runtime_get_fms(&fms, &pre);
  (void)runtime_get_navigation(&nav, &ncap);
  (void)actuator_get_capability(&acap);
  (void)actuator_poll(test_clock.monotonic_us, &status);
  printf("[%s] t=%.3fs arm=%u mode=%u permit_epoch=%u\n", tag,
         (double)test_clock.monotonic_us / 1e6, (unsigned)fms.arm_state,
         (unsigned)fms.flight_mode, (unsigned)fms.permit_epoch);
  printf("    prearm state=%u failed=0x%04x reason=%u epoch=%u | nav=%d ctl=%d plan=%d\n",
         (unsigned)pre.state, (unsigned)pre.failed_checks_mask,
         (unsigned)pre.primary_reason_code, (unsigned)pre.check_epoch,
         (int)s.navigation_status, (int)s.control_status, (int)s.planning_status);
  printf("    nav cap=0x%04x pos_std=%.2f heading_std=%.2f\n",
         (unsigned)ncap.capability_mask, (double)ncap.horizontal_position_std_m,
         (double)ncap.heading_std_rad);
  printf("    landed=%u guidance=%u reason=%u feasible=%u\n",
         (unsigned)fms_output.landed_state.state,
         (unsigned)plan_bundle.guidance.guidance_status.guidance_state,
         (unsigned)plan_bundle.guidance.guidance_status.reason_code,
         (unsigned)plan_bundle.guidance.feasibility.feasible);
  if (fmt_online)
    printf("    contact=%u nav_vh=%.3f nav_vd=%.3f nav_ad=%.3f\n",
           (unsigned)flight_sil_ground_contact(),
           (double)hypotf(nav.core.velocity_ned_mps[0], nav.core.velocity_ned_mps[1]),
           (double)nav.core.velocity_ned_mps[2], (double)nav.core.acceleration_ned_mps2[2]);
  printf("    failsafe=%u authority=%u scope=%u ref=%u/%u dir=%u/%u op=%u kind=%u cause=%u\n",
         fms.failsafe_level, fms_output.authority_status.authority_owner,
         fms_output.actuation_permit.permit_flags, fms_output.actuation_permit.reference_epoch,
         plan_bundle.guidance.guidance_status.reference_epoch, fms_output.directive.directive_epoch,
         plan_bundle.guidance.guidance_status.directive_epoch, fms_output.directive.operation_code,
         plan_bundle.guidance.guidance_status.reference_kind, fms_output.actuation_permit.reason_code);
  printf("    inputs: imu=%u rej=%u nav_up=%u ctl_up=%u ctl_rej=%u plan_up=%u | "
         "act cap=0x%x faults=0x%x commits=%u meas_rej=%u\n",
         (unsigned)s.imu_batches, (unsigned)s.imu_rejected, (unsigned)s.nav_updates,
         (unsigned)s.control_updates, (unsigned)s.control_rejected,
         (unsigned)s.planning_updates, (unsigned)acap.available_mask,
         (unsigned)status.fault_mask, commits, (unsigned)s.measurement_rejected);
}

static int start_fmt(const char *port_text, const char *hover_text)
{
  char *end;
  const unsigned long port = strtoul(port_text, &end, 10);
  if (*end || port == 0 || port > 65535)
    return 0;
  fmt_hover = strtof(hover_text, &end);
  if (*end || !isfinite(fmt_hover) || fmt_hover < .1f || fmt_hover > .95f ||
      !fmt_sim_host_open((uint16_t)port))
    return 0;

  uint64_t start = fmt_sim_host_now_us(), now = start, next_send = start;
  while (now - start < 4000000u) {
    if (now >= next_send) {
      (void)fmt_sim_host_send(now - start, NULL, 0);
      next_send = now + 20000u;
    }
    fmt_frames += fmt_sim_host_receive(&fmt_state);
    if (fmt_frames >= 5u && fabsf(fmt_state.velocity_ned_mps[2]) < .15f) {
      fmt_origin = fmt_state;
      last_fmt_us = test_clock.monotonic_us;
      fmt_last_state_wall_us = fmt_sim_host_now_us();
      printf("[fmt] loopback HIL port=%lu hover=%.3f initial down=%.3f m\n",
             port, (double)fmt_hover, (double)fmt_state.position_ned_m[2]);
      return 1;
    }
    fmt_sim_host_sleep();
    now = fmt_sim_host_now_us();
  }
  fmt_sim_host_close();
  return 0;
}


static void sil_seed_live_fms_inputs(void)
{
  if (!runtime_data_lock(RT_WAITING_FOREVER)) {
    CHECK(0);
    return;
  }
  memset(&runtime_data.nav, 0, sizeof(runtime_data.nav));
  memset(&runtime_data.nav_cap, 0, sizeof(runtime_data.nav_cap));
  memset(&runtime_data.control_cap, 0, sizeof(runtime_data.control_cap));
  memset(&runtime_data.actuator, 0, sizeof(runtime_data.actuator));
  memset(&runtime_data.actuator_cap, 0, sizeof(runtime_data.actuator_cap));
  runtime_data.nav.state_epoch = 7u;
  runtime_data.nav.origin_id = 42u;
  runtime_data.nav.core.valid_mask = 0xFFu;
  runtime_data.nav.core.q_nb_wxyz[0] = 1.0f;
  runtime_data.nav.core.position_std_m[0] = 0.2f;
  runtime_data.nav.core.position_std_m[1] = 0.2f;
  runtime_data.nav.core.position_std_m[2] = 0.3f;
  runtime_data.nav.core.velocity_std_mps[0] = 0.05f;
  runtime_data.nav.core.velocity_std_mps[1] = 0.05f;
  runtime_data.nav.core.velocity_std_mps[2] = 0.05f;
  runtime_data.nav.core.attitude_std_rad[0] = 0.01f;
  runtime_data.nav.core.attitude_std_rad[1] = 0.01f;
  runtime_data.nav.core.attitude_std_rad[2] = 0.02f;

  runtime_data.nav_cap.state_epoch = 7u;
  runtime_data.nav_cap.origin_id = 42u;
  runtime_data.nav_cap.capability_mask = 0xFFu;
  runtime_data.nav_cap.solution_state = 1u;
  runtime_data.nav_cap.horizontal_position_std_m = 0.3f;
  runtime_data.nav_cap.vertical_position_std_m = 0.4f;
  runtime_data.nav_cap.velocity_std_mps = 0.1f;
  runtime_data.nav_cap.heading_std_rad = 0.05f;

  runtime_data.control_cap.control_layout_id = 1u;
  runtime_data.control_cap.controllable_dof_mask = 0x0Fu;
  runtime_data.control_cap.health_state = 2u;

  runtime_data.actuator_cap.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
  runtime_data.actuator_cap.available_mask = 0x0Fu;
  runtime_data.actuator_cap.controllable_dof_mask = 0x0Fu;
  runtime_data.actuator.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
  runtime_data.actuator.available_mask = 0x0Fu;
  runtime_data.actuator.fault_mask = 0u;

  runtime_data.bundle_ready = 0u;
  runtime_data.have_demand = 0u;
  runtime_data.have_mission = 0u;
  runtime_data.have_operator = 0u;
  runtime_data_unlock();
}

static void sil_refresh_live_fms_inputs(void)
{
  const uint64_t now = test_clock.monotonic_us;
  if (!runtime_data_lock(RT_WAITING_FOREVER)) {
    CHECK(0);
    return;
  }
  runtime_data.nav.header = h(20u, now + 50000u);
  runtime_data.nav.header.message_type = MSG_NAVIGATION_SNAPSHOT;
  runtime_data.nav.header.sequence = ++sil_requests;
  runtime_data.nav_cap.header = h(20u, now + 50000u);
  runtime_data.nav_cap.header.message_type = MSG_NAVIGATION_CAPABILITY;
  runtime_data.nav_cap.header.sequence = ++sil_requests;
  runtime_data.control_cap.header = h(23u, now + 100000u);
  runtime_data.control_cap.header.message_type = MSG_CONTROL_CAPABILITY;
  runtime_data.control_cap.header.sequence = ++sil_requests;
  runtime_data.actuator.header = h(24u, now + 50000u);
  runtime_data.actuator.header.message_type = MSG_ACTUATOR_STATUS;
  runtime_data.actuator.header.sequence = ++sil_requests;
  runtime_data.actuator_cap.header = h(24u, now + 100000u);
  runtime_data.actuator_cap.header.message_type = MSG_ACTUATOR_CAPABILITY;
  runtime_data.actuator_cap.header.sequence = ++sil_requests;
  runtime_data_unlock();
  feed_operator(0u, 0.0f);
}

static int sil_command_has_ack(uint32_t request_id, uint16_t expected_state)
{
  for (uint16_t i = 0u; i < fms_output.command_result_count; ++i) {
    if (fms_output.command_results[i].request_id == request_id)
      return fms_output.command_results[i].ack_state == expected_state;
  }
  return 0;
}

/* Exercise the real request queue, mission owner, FMS policy and actuator owner
 * in one process. Unlike the ramp-only smoke, this does not manufacture an
 * FmsOutputView: the certificate must come from the live DISARM command path. */
static void run_live_fms_ground_stop_task_smoke(void)
{
  uint32_t arm_request, disarm_request;
  unsigned tick;
  const unsigned commits_before = commits;

  CHECK(!runtime_bench_inhibit_active());
  sil_seed_live_fms_inputs();

  /* The real LandDetector requires its acceleration filter warm-up and stable
   * ground-contact dwell before PreArm can become READY. */
  for (tick = 0u; tick < 430u; ++tick) {
    test_clock.monotonic_us += 10000u;
    sil_refresh_live_fms_inputs();
    task_mission_step();
  }
  printf("live_fms_prearm: landed=%u prearm=%u\n",
         fms_output.landed_state.state, fms_output.prearm_result.state);
  CHECK(fms_output.landed_state.state == LANDED_STATE_LANDED);
  CHECK(fms_output.prearm_result.state == PREARM_READY);
  if (fms_output.prearm_result.state != PREARM_READY)
    return;

  sil_refresh_live_fms_inputs();
  arm_request = submit_command_with_target(PRODUCT_CMD_ARM,
      fms_output.prearm_result.check_epoch, 0.0f, NULL);
  CHECK(arm_request != 0u);
  test_clock.monotonic_us += 10000u;
  sil_refresh_live_fms_inputs();
  task_mission_step();
  CHECK(sil_command_has_ack(arm_request, COMMAND_ACK_COMPLETED));
  CHECK(fms_output.state_snapshot.arm_state == ARM_STATE_ARMED);
  if (fms_output.state_snapshot.arm_state != ARM_STATE_ARMED)
    return;

  /* Give the production guidance update one owner cycle, then request a
   * ground disarm while the LandDetector still reports LANDED. */
  test_clock.monotonic_us += 10000u;
  sil_refresh_live_fms_inputs();
  task_mission_step();
  test_clock.monotonic_us += 10000u;
  sil_refresh_live_fms_inputs();
  disarm_request = submit_command_with_target(PRODUCT_CMD_DISARM, 0u, 0.0f, NULL);
  CHECK(disarm_request != 0u);
  task_mission_step();

  CHECK(sil_command_has_ack(disarm_request, COMMAND_ACK_COMPLETED));
  CHECK(fms_output.state_snapshot.arm_state == ARM_STATE_DISARMED);
  CHECK(fms_output.landed_state.state == LANDED_STATE_LANDED);
  CHECK(fms_output.actuation_permit.permit_flags == PERMIT_FLAG_REVOKED);
  CHECK(fms_output.actuation_permit.reason_code == FMS_PERMIT_REVOKE_REASON_COMMAND_DISARM);
  CHECK(runtime_data.bundle.ground_stop_certificate.header.message_type ==
        MSG_GROUND_STOP_CERTIFICATE);
  CHECK(runtime_data.bundle.ground_stop_certificate.authority_epoch ==
        fms_output.actuation_permit.authority_epoch);
  CHECK(runtime_data.bundle.ground_stop_certificate.directive_epoch ==
        fms_output.actuation_permit.directive_epoch);

  /* The real actuator task consumes the certificate published by the real
   * mission task and must produce a terminal zero-target receipt. */
  for (tick = 0u; tick < 180u &&
       runtime_data.receipt.result_code != ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED; ++tick) {
    test_clock.monotonic_us += 1000u;
    turn(task_actuator_entry);
  }
  CHECK(commits >= commits_before + 1u);
  CHECK(actuator_guard_state() == ACTUATOR_GUARD_DISARMED);
  CHECK(runtime_data.receipt.result_code == ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED);
  CHECK(runtime_data.receipt.header.message_type == MSG_ACTUATOR_COMMIT_RECEIPT);
  CHECK(runtime_data.receipt.committed_demand.motor_thrust_n[0] == 0.0f);
  CHECK((runtime_data.receipt.limit_flags & ACTUATOR_GROUND_STOP_OVERRUN_FLAG) == 0u);
  printf("live_fms_ground_stop: landed/disarm certificate -> actuator zero receipt\n");
}



static void sil_publish_late_first_reference(void)
{
  const uint64_t now = test_clock.monotonic_us;
  GuidanceStatus *guidance = &plan_bundle.guidance.guidance_status;
  GuidanceFeasibility *feasibility = &plan_bundle.guidance.feasibility;
  memset(&plan_bundle.guidance, 0, sizeof(plan_bundle.guidance));
  guidance->header = h(22u, now + 100000u);
  guidance->header.message_type = MSG_GUIDANCE_STATUS;
  guidance->header.sequence = ++sil_requests;
  guidance->reference_id = 772u;
  guidance->reference_epoch = 72u;
  guidance->state_epoch = runtime_data.nav.state_epoch;
  guidance->origin_id = runtime_data.nav.origin_id;
  guidance->authority_epoch = fms_output.state_snapshot.authority_epoch;
  guidance->directive_epoch = fms_output.state_snapshot.directive_epoch;
  guidance->mission_epoch = fms_output.mission_status.mission_epoch;
  guidance->current_item_id = fms_output.mission_status.current_item_id;
  guidance->execution_token = fms_output.mission_status.execution_token;
  guidance->guidance_state = GUIDANCE_STATE_READY;
  guidance->reference_kind = GUIDANCE_REFERENCE_TRAJECTORY;
  guidance->reference_expires_at_us = now + 100000u;

  feasibility->header = h(22u, now + 100000u);
  feasibility->header.message_type = MSG_GUIDANCE_FEASIBILITY;
  feasibility->header.sequence = ++sil_requests;
  feasibility->reference_id = guidance->reference_id;
  feasibility->reference_epoch = guidance->reference_epoch;
  feasibility->feasible = 1u;
  feasibility->reason_code = 0u;
}

/* Mirror the M4 first-reference timeout case through the actual mission owner:
 * keep the vehicle on the ground, withhold every guidance reference for the
 * absolute 100 ms window, then deliver a valid reference exactly at expiry.
 * The late reference must not resurrect the permit; M4 must publish the
 * ground-disarm certificate and the real M7 owner must consume it. */
static void run_live_fms_first_reference_task_smoke(void)
{
  uint32_t arm_request;
  unsigned tick;
  const unsigned commits_before = commits;

  CHECK(!runtime_bench_inhibit_active());
  sil_seed_live_fms_inputs();
  for (tick = 0u; tick < 430u; ++tick) {
    test_clock.monotonic_us += 10000u;
    sil_refresh_live_fms_inputs();
    task_mission_step();
  }
  printf("live_first_reference_prearm: landed=%u prearm=%u\n",
         fms_output.landed_state.state, fms_output.prearm_result.state);
  CHECK(fms_output.landed_state.state == LANDED_STATE_LANDED);
  CHECK(fms_output.prearm_result.state == PREARM_READY);
  if (fms_output.prearm_result.state != PREARM_READY)
    return;

  sil_refresh_live_fms_inputs();
  arm_request = submit_command_with_target(PRODUCT_CMD_ARM,
      fms_output.prearm_result.check_epoch, 0.0f, NULL);
  CHECK(arm_request != 0u);
  test_clock.monotonic_us += 10000u;
  sil_refresh_live_fms_inputs();
  task_mission_step();
  CHECK(sil_command_has_ack(arm_request, COMMAND_ACK_COMPLETED));
  CHECK(fms_output.state_snapshot.arm_state == ARM_STATE_ARMED);
  if (fms_output.state_snapshot.arm_state != ARM_STATE_ARMED)
    return;

  /* Nine owner turns keep the first-reference window open. Discard any
   * generated guidance after each turn to model a guidance publisher that has
   * not yet delivered a usable reference to FMS. */
  for (tick = 0u; tick < 9u; ++tick) {
    test_clock.monotonic_us += 10000u;
    sil_refresh_live_fms_inputs();
    memset(&plan_bundle.guidance, 0, sizeof(plan_bundle.guidance));
    task_mission_step();
    memset(&plan_bundle.guidance, 0, sizeof(plan_bundle.guidance));
  }
  CHECK(fms_output.state_snapshot.arm_state == ARM_STATE_ARMED);
  CHECK((fms_output.actuation_permit.permit_flags & PERMIT_SCOPE_CONTROL_ACTIVE) == 0u);

  /* The first valid reference is deliberately one full 100 ms window late. */
  test_clock.monotonic_us += 10000u;
  sil_refresh_live_fms_inputs();
  sil_publish_late_first_reference();
  task_mission_step();

  CHECK(fms_output.state_snapshot.arm_state == ARM_STATE_DISARMED);
  CHECK(fms_output.landed_state.state == LANDED_STATE_LANDED);
  CHECK(fms_output.actuation_permit.permit_flags == PERMIT_FLAG_REVOKED);
  CHECK(fms_output.actuation_permit.reason_code ==
        FMS_PERMIT_REVOKE_REASON_FIRST_REFERENCE_GROUND_DISARM);
  CHECK(fms_output.decision_event.decision_code == DECISION_FORCE_SAFE_ACTION);
  CHECK(fms_output.decision_event.reason_code ==
        FMS_DECISION_REASON_FIRST_REFERENCE_GROUND_DISARM);
  CHECK(runtime_data.bundle.ground_stop_certificate.header.message_type ==
        MSG_GROUND_STOP_CERTIFICATE);
  CHECK(runtime_data.bundle.ground_stop_certificate.lifecycle ==
        ACTUATOR_GROUND_STOP_LIFECYCLE_LANDED);
  CHECK(runtime_data.bundle.ground_stop_certificate.authority_epoch ==
        fms_output.actuation_permit.authority_epoch);
  CHECK(runtime_data.bundle.ground_stop_certificate.directive_epoch ==
        fms_output.actuation_permit.directive_epoch);

  for (tick = 0u; tick < 180u &&
       runtime_data.receipt.result_code != ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED; ++tick) {
    test_clock.monotonic_us += 1000u;
    turn(task_actuator_entry);
  }
  CHECK(commits >= commits_before + 1u);
  CHECK(actuator_guard_state() == ACTUATOR_GUARD_DISARMED);
  CHECK(runtime_data.receipt.result_code == ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED);
  CHECK(runtime_data.receipt.header.message_type == MSG_ACTUATOR_COMMIT_RECEIPT);
  CHECK(runtime_data.receipt.committed_demand.motor_thrust_n[0] == 0.0f);
  CHECK((runtime_data.receipt.limit_flags & ACTUATOR_GROUND_STOP_OVERRUN_FLAG) == 0u);
  printf("live_fms_first_reference: late reference -> ground-disarm certificate -> actuator zero receipt\n");
}

static void run_ground_stop_task_smoke(void)
{
  FmsOutputView output;
  ActuationPermit active_permit, permit;
  GroundStopCertificate certificate, invalid_certificate;
  ActuatorDemand demand;
  uint64_t now = test_clock.monotonic_us;
  uint64_t stop_start = 0u;
  unsigned commits_before = commits;
  unsigned tick;

  CHECK(!runtime_bench_inhibit_active());

  /* First prove that an invalid ground-stop candidate cannot starve a valid
   * M6 demand. The guard must reject the candidate and let this 5 N demand
   * reach the normal frame boundary. */
  memset(&active_permit, 0, sizeof(active_permit));
  active_permit.header = h(SOURCE_M4, now + 5000u);
  active_permit.header.message_type = MSG_ACTUATION_PERMIT;
  active_permit.permit_epoch = 1u;
  active_permit.authority_epoch = 5u;
  active_permit.directive_epoch = 3u;
  active_permit.state_epoch = 1u;
  active_permit.reference_epoch = 1u;
  active_permit.permit_flags = PERMIT_SCOPE_CONTROL_ACTIVE;
  active_permit.output_policy_id = 2u;
  active_permit.permit_expires_at_us = now + 200000u;

  memset(&demand, 0, sizeof(demand));
  demand.header = h(SOURCE_M6, now + 5000u);
  demand.header.message_type = MSG_ACTUATOR_DEMAND;
  demand.header.frame_id = FRAME_BODY_FRD;
  demand.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
  demand.valid_channel_mask = 15u;
  demand.permit_epoch = active_permit.permit_epoch;
  demand.directive_epoch = active_permit.directive_epoch;
  demand.authority_epoch = active_permit.authority_epoch;
  demand.state_epoch = active_permit.state_epoch;
  demand.reference_epoch = active_permit.reference_epoch;
  demand.control_sequence = 1u;
  demand.demand_expires_at_us = now + 5000u;
  demand.demand.valid_mask = 15u;
  for (unsigned motor = 0; motor < 4u; ++motor)
    demand.demand.motor_thrust_n[motor] = 5.0f;

  memset(&invalid_certificate, 0, sizeof(invalid_certificate));
  invalid_certificate.header = h(SOURCE_M4, now + 5000u);
  invalid_certificate.header.message_type = MSG_GROUND_STOP_CERTIFICATE;
  invalid_certificate.stop_code = ACTUATOR_GROUND_STOP_CODE_ZERO;
  invalid_certificate.lifecycle = LANDED_STATE_IN_AIR;
  invalid_certificate.ramp_nps = 100.0f;
  invalid_certificate.authority_epoch = active_permit.authority_epoch;
  invalid_certificate.directive_epoch = active_permit.directive_epoch;
  invalid_certificate.certificate_expires_at_us = now + 200000u;

  CHECK(runtime_data_lock(0));
  runtime_data.bundle.permit = active_permit;
  runtime_data.bundle.ground_stop_certificate = invalid_certificate;
  runtime_data.bundle_ready = 1u;
  runtime_data.demand = demand;
  runtime_data.have_demand = 1u;
  runtime_data_unlock();

  turn(task_actuator_entry);
  CHECK(commits == commits_before + 1u);
  CHECK(runtime_data.receipt.result_code == ACTUATOR_RECEIPT_COMMITTED);
  CHECK(fabsf(runtime_data.receipt.committed_demand.motor_thrust_n[0] - 5.0f) < 1e-3f);
  CHECK(actuator_guard_state() == ACTUATOR_GUARD_ACTIVE);

  /* Now revoke the active permit and publish a valid landed/disarmed
   * certificate through the same helper used by task_mission_step(). */
  memset(&output, 0, sizeof(output));
  memset(&permit, 0, sizeof(permit));
  permit.header = h(SOURCE_M4, now + 5000u);
  permit.header.message_type = MSG_ACTUATION_PERMIT;
  permit.permit_epoch = 2u;
  permit.authority_epoch = 5u;
  permit.directive_epoch = 3u;
  permit.state_epoch = 1u;
  permit.reference_epoch = 1u;
  permit.permit_flags = PERMIT_FLAG_REVOKED;
  permit.reason_code = FMS_PERMIT_REVOKE_REASON_COMMAND_DISARM;
  permit.output_policy_id = 2u;
  permit.permit_expires_at_us = now + 200000u;

  output.actuation_permit = permit;
  output.decision_event.header = h(SOURCE_M4, now + 5000u);
  output.decision_event.header.message_type = MSG_DECISION_EVENT;
  output.decision_event.decision_code = DECISION_ACCEPT;
  output.decision_event.reason_code = 0u;
  output.landed_state.header = h(SOURCE_M4, now + 5000u);
  output.landed_state.header.message_type = MSG_LANDED_STATE_SNAPSHOT;
  output.landed_state.state = LANDED_STATE_LANDED;
  output.state_snapshot.header = h(SOURCE_M4, now + 5000u);
  output.state_snapshot.header.message_type = MSG_FMS_STATE_SNAPSHOT;
  output.state_snapshot.arm_state = ARM_STATE_DISARMED;

  CHECK(task_mission_update_ground_stop_certificate(&output, now, 1) == STATUS_OK);
  certificate = plan_bundle.ground_stop_certificate;
  {
    FmsOutputView invalid_output = output;
    const uint64_t certificate_expiry = certificate.certificate_expires_at_us;
    CHECK(task_mission_update_ground_stop_certificate(NULL, now + 1000u, 0) ==
              STATUS_NOT_READY);
    CHECK(plan_bundle.ground_stop_certificate.header.sequence == certificate.header.sequence &&
          plan_bundle.ground_stop_certificate.certificate_expires_at_us == certificate_expiry);
    invalid_output.landed_state.state = LANDED_STATE_IN_AIR;
    CHECK(task_mission_update_ground_stop_certificate(&invalid_output, now + 2000u, 1) ==
              STATUS_NOT_PERMITTED);
    CHECK(plan_bundle.ground_stop_certificate.header.message_type == 0u &&
          plan_bundle.ground_stop_certificate.certificate_expires_at_us == 0u);
    CHECK(task_mission_update_ground_stop_certificate(&output, now, 1) == STATUS_OK);
    certificate = plan_bundle.ground_stop_certificate;
    CHECK(task_mission_update_ground_stop_certificate(NULL, certificate_expiry + 1u, 0) ==
              STATUS_NOT_READY);
    CHECK(plan_bundle.ground_stop_certificate.header.message_type == 0u &&
          plan_bundle.ground_stop_certificate.certificate_expires_at_us == 0u);
    CHECK(task_mission_update_ground_stop_certificate(&output, now, 1) == STATUS_OK);
    certificate = plan_bundle.ground_stop_certificate;
  }
  CHECK(actuator_update_permit(&permit, now) == STATUS_OK);
  CHECK(certificate.header.message_type == MSG_GROUND_STOP_CERTIFICATE);

  CHECK(runtime_data_lock(0));
  runtime_data.bundle.permit = permit;
  runtime_data.bundle.ground_stop_certificate = certificate;
  runtime_data.have_demand = 0u;
  runtime_data_unlock();

  /* Exercise the real task owner from a non-zero 5 N frame through the
   * bounded 100 N/s ramp to the confirmed zero-target receipt. */
  turn(task_actuator_entry);
  stop_start = actuator_guard_ground_stop_start_us();
  for (tick = 0u; tick < 180u &&
       runtime_data.receipt.result_code != ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED; ++tick) {
    test_clock.monotonic_us += 1000u;
    turn(task_actuator_entry);
  }

  CHECK(commits >= commits_before + 3u);
  CHECK(stop_start != 0u);
  CHECK(actuator_guard_state() == ACTUATOR_GUARD_DISARMED);
  CHECK(runtime_data.receipt.result_code == ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED);
  CHECK(runtime_data.receipt.header.message_type == MSG_ACTUATOR_COMMIT_RECEIPT);
  CHECK(runtime_data.receipt.committed_demand.motor_thrust_n[0] == 0.0f);
  CHECK(runtime_data.receipt.estimated_apply_time_us >=
        runtime_data.receipt.commit_time_us);
  CHECK(runtime_data.receipt.estimated_apply_time_us - stop_start <=
        ACTUATOR_GROUND_STOP_MAX_APPLY_US);
  CHECK((runtime_data.receipt.limit_flags & ACTUATOR_GROUND_STOP_OVERRUN_FLAG) == 0u);
}

int main(int argc, char **argv)
{
  unsigned takeoff_commits = 0, takeoff_completed_ms = 0;
  uint32_t takeoff_request = 0, terminal_request = 0, disarm_request = 0;
  uint32_t mission_commit_request = 0, mission_start_request = 0;
  uint16_t takeoff_ack = COMMAND_ACK_REJECTED, terminal_ack = COMMAND_ACK_REJECTED;
  uint16_t disarm_ack = COMMAND_ACK_REJECTED;
  uint16_t terminal_command = PRODUCT_CMD_LAND;
  int mission_scenario = 0;
  int return_accepted_in_hold = 0;
  float peak_height = 0;
  float hold_error = 0;
  float near_ground_max_down = 0;
  float waypoint_error[2] = {INFINITY, INFINITY};
  int logged_control_rejection = 0;
  int fmt_timeout = 0;
  int motion_request_failed = 0;
  RuntimeStats stats;

  const int ground_stop_smoke = argc == 2 && !strcmp(argv[1], "--ground-stop-smoke");
  const int live_fms_ground_stop_smoke =
      argc == 2 && !strcmp(argv[1], "--live-fms-ground-stop-smoke");
  const int live_fms_first_reference_ground_stop_smoke =
      argc == 2 && !strcmp(argv[1], "--live-fms-first-reference-ground-stop-smoke");
  if (argc != 1 && !ground_stop_smoke && !live_fms_ground_stop_smoke &&
      !live_fms_first_reference_ground_stop_smoke) {
#ifdef FLIGHT_GROUND_STATION
    if (argc == 6 && !strcmp(argv[3], "Ground")) {
      ground_online = 1;
      ground_calibration_fixture = getenv("FLIGHT_SIL_CAL_FIXTURE") != NULL;
    } else
#endif
    if ((argc < 3 || argc > 5) ||
        (argc >= 4 && strcmp(argv[3], "Return") && strcmp(argv[3], "Land") &&
         strcmp(argv[3], "Mission"))) {
      fprintf(stderr, "Usage: sil_flight [port hover [Land|Return|Mission [wall_timeout_ms]]]\n");
      return 1;
    }
    if (argc >= 5) {
      char *end;
      unsigned long timeout_ms = strtoul(argv[4], &end, 10);
      if (*end || timeout_ms < 350 || timeout_ms > 1000) {
        fprintf(stderr, "Host wall timeout must be 350 through 1000 ms\n");
        return 1;
      }
      fmt_wall_timeout_us = 1000u * timeout_ms;
    }
    fmt_online = 1;
#ifdef FLIGHT_GROUND_STATION
    if (ground_online && !strcmp(argv[1], "0"))
      fmt_online = 0;
#endif
    if (fmt_online && !start_fmt(argv[1], argv[2])) {
      fprintf(stderr, "No stable FMT_EXTERNAL_STATE stream on the requested loopback HIL port\n");
      return 1;
    }
    mission_scenario = argc >= 4 && !strcmp(argv[3], "Mission");
    if (mission_scenario || (argc >= 4 && !strcmp(argv[3], "Return")))
      terminal_command = PRODUCT_CMD_RETURN;
  }
#ifdef FLIGHT_GROUND_STATION
  if (ground_online) {
    char *end;
    const unsigned long port = strtoul(argv[5], &end, 10);
    if (*end || !port || port > 65535 || !ground_station_open((uint16_t)port)) {
      fmt_sim_host_close();
      return 1;
    }
    signal(SIGINT, stop_ground);
    signal(SIGTERM, stop_ground);
  }
#endif
  if (runtime_start() != RT_EOK) {
#ifdef FLIGHT_GROUND_STATION
    ground_station_close();
#endif
    fmt_sim_host_close();
    return 1;
  }
  if (live_fms_first_reference_ground_stop_smoke) {
    run_live_fms_first_reference_task_smoke();
    runtime_shutdown();
    printf("sil_live_fms_first_reference: %u checks, %u failures, commits=%u\\n",
           checks, failures, commits);
    return failures ? 1 : 0;
  }
  if (live_fms_ground_stop_smoke) {
    run_live_fms_ground_stop_task_smoke();
    runtime_shutdown();
    printf("sil_live_fms_ground_stop: %u checks, %u failures, commits=%u\\n",
           checks, failures, commits);
    return failures ? 1 : 0;
  }
  if (ground_stop_smoke) {
    run_ground_stop_task_smoke();
    runtime_shutdown();
    printf("sil_ground_stop_smoke: %u checks, %u failures, commits=%u inhibits=%u\\n",
           checks, failures, commits, inhibits);
    return failures ? 1 : 0;
  }
  if (fmt_online)
    sensor_imu_use_external_source(true);

  unsigned duration_ms = mission_scenario ? 36000u : fmt_online ? 24000u : 8000u;
#ifdef FLIGHT_GROUND_STATION
  if (ground_online)
    duration_ms = 24u * 60u * 60u * 1000u;
  int ground_motion = 0;
#endif
  /* The installed FMT version sends timestamp=0. Measurements of position
   * against velocity show about 10 ms of plant time per external-state frame.
   * Run the stationary alignment on the same frame cadence as flight. */
  const unsigned alignment_ticks = 8000u, ticks_per_fmt_frame = 20u;
  for (unsigned tick = 1; tick <= 2u * duration_ms; ++tick) {
#ifdef FLIGHT_GROUND_STATION
    if (ground_online && ground_stop)
      break;
#endif
    unsigned ms = tick / 2u;
    if (fmt_online && (tick - 1u) % ticks_per_fmt_frame == 0u) {
      unsigned frames = fmt_wait_state();
      if (!frames) {
        printf("[fmt] state timeout during %s at t=%u ms, received=%u\n",
               tick <= alignment_ticks ? "alignment" : "flight", ms, fmt_frames);
        fmt_timeout = 1;
        break;
      }
      last_fmt_us = test_clock.monotonic_us + 500u;
      fmt_frames += frames;
      peak_height = fmaxf(peak_height, fmt_height());
      if (terminal_request != 0u && fmt_height() >= 0.0f && fmt_height() <= 0.5f)
        near_ground_max_down = fmaxf(near_ground_max_down, fmt_state.velocity_ned_mps[2]);
      if (fms_output.mission_status.mission_state == MISSION_STATE_RUNNING) {
        for (unsigned i = 0; i < 2; ++i) {
          const float north = fmt_state.position_ned_m[0] - fmt_origin.position_ned_m[0];
          const float east = fmt_state.position_ned_m[1] - fmt_origin.position_ned_m[1];
          waypoint_error[i] = fminf(waypoint_error[i],
              hypotf(hypotf(north - mission_waypoints[i][0], east - mission_waypoints[i][1]),
                     fmt_height() + mission_waypoints[i][2]));
        }
      }
    }
    test_clock.monotonic_us += 500u;
    if (tick % 2u == 0u) {
      if (ms % 20u == 0u)
        feed_operator(0, 0.0f);
      if (!fmt_online || fmt_fresh()) {
        if (ms % 20u == 0u)
          feed_baro(ms, fmt_online ? fmt_state.altitude_m - fmt_origin.altitude_m : 0.0f);
        const unsigned gnss_period_ms = fmt_online ? 100u : 200u;
        if (ms % gnss_period_ms == gnss_period_ms - 10u)
          feed_gnss(ms);
        if (ms % 10u == 0u)
          feed_mag(ms);
        if (fmt_online)
          fmt_imu(ms);
        else
          imu_frame(ms);
      }
      turn(task_sensor_entry);
      if (fmt_online && !logged_control_rejection && ms > 4140u &&
          fms_output.actuation_permit.permit_flags == PERMIT_SCOPE_CONTROL_ACTIVE &&
          runtime_stats.control_status != STATUS_OK) {
        logged_control_rejection = 1;
        printf("[fmt] control rejected t=%u status=%d reason=%u nav_mask=0x%x "
               "velocity_ned=%.3f,%.3f,%.3f rates=%.3f,%.3f,%.3f\n",
               ms, (int)runtime_stats.control_status, controller_output.capability.reason_code,
               (unsigned)fast_bundle.nav.core.valid_mask,
               (double)fast_bundle.nav.core.velocity_ned_mps[0],
               (double)fast_bundle.nav.core.velocity_ned_mps[1],
               (double)fast_bundle.nav.core.velocity_ned_mps[2],
               (double)fmt_state.gyro_radps[0], (double)fmt_state.gyro_radps[1],
               (double)fmt_state.gyro_radps[2]);
      }
      if (ms % 4u == 0u)
        turn(task_communication_entry);
      turn(task_actuator_entry);
    }
    if (tick % 5u == 0u)
      (void)actuator_guard_frame_boundary(test_clock.monotonic_us);
    if (tick % 40u != 0u)
      continue;
    turn(task_mission_entry);
    turn(task_mpc_entry);
    if (ms % 100u == 0u)
      task_supervisor_step();
    for (unsigned i = 0; i < fms_output.command_result_count; ++i) {
      const CommandAck *ack = &fms_output.command_results[i];
#ifdef FLIGHT_GROUND_STATION
      if (ground_online) {
        ground_station_ack(ack);
        if (ground_takeoff_request && ack->request_id == ground_takeoff_request &&
            (ack->ack_state == COMMAND_ACK_ACCEPTED || ack->ack_state == COMMAND_ACK_IN_PROGRESS ||
             ack->ack_state == COMMAND_ACK_COMPLETED))
          ground_motion = 1;
      }
#endif
      printf("[ack] t=%u request=%u state=%u reason=%u\n",
             ms, ack->request_id, ack->ack_state, ack->reason_code);
      if (ack->request_id == takeoff_request)
        takeoff_ack = ack->ack_state;
      if (ack->request_id == terminal_request) {
        terminal_ack = ack->ack_state;
        if (mission_scenario && ack->ack_state == COMMAND_ACK_ACCEPTED)
          return_accepted_in_hold = runtime_data.fms.flight_mode == FLIGHT_MODE_POS_HOLD;
      }
      if (ack->request_id == disarm_request)
        disarm_ack = ack->ack_state;
      if ((ack->request_id == takeoff_request || ack->request_id == terminal_request ||
           ack->request_id == disarm_request || ack->request_id == mission_commit_request ||
           ack->request_id == mission_start_request) &&
          (ack->ack_state == COMMAND_ACK_REJECTED || ack->ack_state == COMMAND_ACK_FAILED ||
           ack->ack_state == COMMAND_ACK_CANCELLED))
        motion_request_failed = 1;
    }
#ifdef FLIGHT_GROUND_STATION
    if (ground_online) {
      static uint8_t last_ground_failsafe;
      trace_ground_transition(ms);
      if (fmt_online && ms % 2000u == 0u)
        trace_ground_navigation(ms);
      if (runtime_data.fms.failsafe_level == FAILSAFE_ACTION_REQUIRED &&
          last_ground_failsafe < FAILSAFE_ACTION_REQUIRED) {
        dump_ground_transition();
        sil_print_state("ground failsafe action");
        if (fmt_online) {
          printf("    plant NED offset=(%.3f,%.3f,%.3f)\n",
                 (double)(fmt_state.position_ned_m[0] - fmt_origin.position_ned_m[0]),
                 (double)(fmt_state.position_ned_m[1] - fmt_origin.position_ned_m[1]),
                 (double)(fmt_state.position_ned_m[2] - fmt_origin.position_ned_m[2]));
        }
        fflush(stdout);
      }
      last_ground_failsafe = runtime_data.fms.failsafe_level;
      if (!current_commit())
        ground_rebind_until_us = test_clock.monotonic_us + 200000u;
      if (runtime_data.fms.arm_state != ARM_STATE_ARMED)
        ground_motion = 0;
      LandedStateSnapshot reported_landed = fms_output.landed_state;
      if (ground_calibration_fixture && !fmt_online &&
          runtime_data.fms.arm_state == ARM_STATE_DISARMED) {
        /* The fixture rotates an IMU on a virtual workbench. It has ground
         * contact throughout, independent of the estimator's tilt inference. */
        reported_landed.state = LANDED_STATE_LANDED;
      }
      ground_station_step(test_clock.monotonic_us, &runtime_data.nav, &runtime_data.fms,
                          &reported_landed, &fms_output.directive.payload.home,
                          &fms_output.mission_status, &fms_output.prearm_result,
                          /* Keep the GCS command surface available for bench
                           * arm/disarm even when the FMT plant is offline. */
                          &ground_actions);
      if (fmt_online)
        fmt_send_committed(ground_motion);
      else
        Sleep(20); /* Static-sensor bench. Give the UDP peer time to respond. */
      if (ms % 1000u == 0u)
        fflush(stdout);
      if (runtime_data.fms.failsafe_level == FAILSAFE_TERMINAL) {
        dump_ground_transition();
        printf("[ground] terminal t=%u decision=%u reason=%u control=%d/%u dof=0x%x "
               "permit=%u nav=0x%x guidance=%u/%u actuators=0x%x faults=0x%x\n",
               ms, fms_output.decision_event.decision_code, fms_output.decision_event.reason_code,
               (int)runtime_stats.control_status, runtime_data.control_cap.reason_code,
               (unsigned)runtime_data.control_cap.controllable_dof_mask,
               (unsigned)fms_output.actuation_permit.permit_flags,
               (unsigned)runtime_data.nav.core.valid_mask,
               plan_bundle.guidance.guidance_status.guidance_state,
               plan_bundle.guidance.guidance_status.reason_code,
               (unsigned)runtime_data.actuator_cap.available_mask,
               (unsigned)runtime_data.actuator.fault_mask);
        break;
      }
      continue;
    }
#endif
    if (fmt_online) {
      if (terminal_request != 0u && terminal_ack != COMMAND_ACK_COMPLETED) {
        const NavStateCore *nav = &runtime_data.nav.core;
        const ControlCapability *control = &runtime_data.control_cap;
        const ActuationPermit *permit = &fms_output.actuation_permit;
        printf("[settle] t=%u mode=%u contact=%u landed=%u height=%.4f "
               "v=%.4f,%.4f,%.4f a=%.4f,%.4f,%.4f truth_v=%.4f,%.4f,%.4f "
               "scope=%u control_sample=%llu permit_sample=%llu\n", ms,
               runtime_data.fms.flight_mode, flight_sil_ground_contact(), fms_output.landed_state.state,
               (double)fmt_height(), (double)nav->velocity_ned_mps[0],
               (double)nav->velocity_ned_mps[1], (double)nav->velocity_ned_mps[2],
               (double)nav->acceleration_ned_mps2[0], (double)nav->acceleration_ned_mps2[1],
               (double)nav->acceleration_ned_mps2[2], (double)fmt_state.velocity_ned_mps[0],
               (double)fmt_state.velocity_ned_mps[1], (double)fmt_state.velocity_ned_mps[2],
               permit->permit_flags, (unsigned long long)control->header.timestamp_sample_us,
               (unsigned long long)permit->header.timestamp_sample_us);
      }
      /* Keep the vehicle parked through alignment and the first control handshake. */
      fmt_send_committed(takeoff_ack == COMMAND_ACK_ACCEPTED ||
                         takeoff_ack == COMMAND_ACK_IN_PROGRESS ||
                         takeoff_ack == COMMAND_ACK_COMPLETED);
      if (ms % 1000u == 0u) {
        printf("[fmt] t=%u state=%u armed_frames=%u height=%.3f vn=%.3f ve=%.3f vd=%.3f "
               "nav_height=%.3f north=%.3f east=%.3f nav_north=%.3f nav_east=%.3f "
               "mode=%u failsafe=%u mission=%u item=%u\n",
               ms, fmt_frames, fmt_active_frames, (double)fmt_height(),
               (double)fmt_state.velocity_ned_mps[0], (double)fmt_state.velocity_ned_mps[1],
               (double)fmt_state.velocity_ned_mps[2], (double)-runtime_data.nav.core.position_ned_m[2],
               (double)(fmt_state.position_ned_m[0] - fmt_origin.position_ned_m[0]),
               (double)(fmt_state.position_ned_m[1] - fmt_origin.position_ned_m[1]),
               (double)(runtime_data.nav.core.position_ned_m[0] -
                        fms_output.directive.payload.home.position_ned_m[0]),
               (double)(runtime_data.nav.core.position_ned_m[1] -
                        fms_output.directive.payload.home.position_ned_m[1]),
               runtime_data.fms.flight_mode, runtime_data.fms.failsafe_level,
               fms_output.mission_status.mission_state, fms_output.mission_status.current_item_id);
        fflush(stdout);
      }
    }
    if (mission_scenario && ms == 3980u) {
      stage_mission();
      mission_commit_request = submit_command(PRODUCT_CMD_MISSION_COMMIT, 0u, 0.0f);
    }
    if (ms == 4000u) {
      sil_print_state("before-arm");
      CHECK(runtime_data.prearm.state == PREARM_READY);
      if (runtime_data.prearm.state != PREARM_READY)
        break;
      (void)submit_command(PRODUCT_CMD_ARM, runtime_data.prearm.check_epoch, 0.0f);
    }
    if (ms == 4100u) {
      sil_print_state("before-takeoff");
      CHECK(runtime_data.fms.arm_state == ARM_STATE_ARMED);
      CHECK(fms_output.actuation_permit.permit_flags == PERMIT_SCOPE_CONTROL_ACTIVE);
      takeoff_commits = commits;
      takeoff_request = submit_command(PRODUCT_CMD_TAKEOFF, 0u, 2.0f);
    }
    if (fmt_online && takeoff_ack == COMMAND_ACK_COMPLETED) {
      if (takeoff_completed_ms == 0u)
        takeoff_completed_ms = ms;
      if (ms - takeoff_completed_ms <= 2000u)
        hold_error = fmaxf(hold_error, fabsf(fmt_height() - 2.0f));
      if (terminal_request == 0u && ms - takeoff_completed_ms >= 2000u) {
        if (mission_scenario && mission_start_request == 0u)
          mission_start_request = submit_command(PRODUCT_CMD_MISSION_START, 0u, 0.0f);
        else if (!mission_scenario || fms_output.mission_status.result_code == MISSION_RESULT_SUCCESS)
          terminal_request = submit_command(terminal_command, 0u, 0.0f);
      }
    }
    if (fmt_online && terminal_ack == COMMAND_ACK_COMPLETED && disarm_request == 0u)
      disarm_request = submit_command(PRODUCT_CMD_DISARM, 0u, 0.0f);
    if (disarm_ack == COMMAND_ACK_COMPLETED)
      break;
    if (fmt_online && (motion_request_failed || runtime_data.fms.failsafe_level == FAILSAFE_TERMINAL))
      break;
  }
#ifdef FLIGHT_GROUND_STATION
  if (ground_online) {
    for (unsigned i = 0; fmt_online && i < 5; ++i)
      (void)fmt_sim_host_send(test_clock.monotonic_us + 20000u * i, NULL, 0);
    ground_station_close();
    fmt_sim_host_close();
    printf("[ground] stopped, state_frames=%u virtual_output_frames=%u timeout=%d\n",
           fmt_frames, fmt_active_frames, fmt_timeout);
    return fmt_timeout || failures || runtime_data.fms.failsafe_level == FAILSAFE_TERMINAL ? 1 : 0;
  }
#endif
  sil_print_state("final");
  runtime_get_stats(&stats);
  CHECK(seen_packets[FCU_STORAGE_STATUS_ID - FCU_OPERATION_STATUS_ID] > 0);
  CHECK((seen_packets[FCU_BUILD_INFO_ID - FCU_OPERATION_STATUS_ID] > 0) ==
        (FLIGHT_TELEMETRY_ABI_LOCK != 0));
  CHECK(stats.control_updates > 100u);
  CHECK(commits > takeoff_commits + 40u);
  CHECK(runtime_data.fms.failsafe_level != FAILSAFE_TERMINAL);
  CHECK(takeoff_ack == COMMAND_ACK_ACCEPTED || takeoff_ack == COMMAND_ACK_IN_PROGRESS ||
        takeoff_ack == COMMAND_ACK_COMPLETED);
  if (fmt_online) {
    for (unsigned i = 0; i < 5; ++i) {
      (void)fmt_sim_host_send(test_clock.monotonic_us + 20000u * i, NULL, 0);
      fmt_sim_host_sleep();
    }
    fmt_sim_host_close();
    printf("[fmt] state=%u armed_frames=%u peak_height=%.3f m hold_error=%.3f m\n",
           fmt_frames, fmt_active_frames, (double)peak_height, (double)hold_error);
    printf("[fmt] measured maximum descent in the last 0.5 m: %.3f m/s\n",
           (double)near_ground_max_down);
    printf("[fmt] wall timeout %.0f ms, longest successful state gap %.1f ms\n",
           (double)fmt_wall_timeout_us / 1000.0, (double)fmt_longest_gap_us / 1000.0);
    CHECK(fmt_active_frames > 0u);
    CHECK(!fmt_timeout);
    CHECK(peak_height >= 1.8f);
    CHECK(takeoff_ack == COMMAND_ACK_COMPLETED);
    CHECK(hold_error <= .30f);
    CHECK(near_ground_max_down > 0.0f && near_ground_max_down <= 0.40f);
    CHECK(terminal_ack == COMMAND_ACK_COMPLETED);
    CHECK(disarm_ack == COMMAND_ACK_COMPLETED);
    CHECK(runtime_data.fms.arm_state == ARM_STATE_DISARMED);
    CHECK(flight_sil_ground_contact() == GROUND_CONTACT_CONTACT);
    if (terminal_command == PRODUCT_CMD_RETURN) {
      const float return_error = hypotf(fmt_state.position_ned_m[0] - fmt_origin.position_ned_m[0],
                                       fmt_state.position_ned_m[1] - fmt_origin.position_ned_m[1]);
      printf("[fmt] return touchdown distance %.3f m\n", (double)return_error);
      CHECK(return_error <= .5f);
    }
    if (mission_scenario) {
      printf("[mission] closest waypoint distances %.3f m, %.3f m\n",
             (double)waypoint_error[0], (double)waypoint_error[1]);
      CHECK(fms_output.mission_status.result_code == MISSION_RESULT_SUCCESS);
      CHECK(return_accepted_in_hold);
      CHECK(waypoint_error[0] <= .45f && waypoint_error[1] <= .45f);
    }
  }
  printf("sil_flight: %u checks, %u failures, commits=%u inhibits=%u\n",
         checks, failures, commits, inhibits);
  return failures ? 1 : 0;
}


