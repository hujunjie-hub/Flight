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
#include "attitude_rate_control/controller_internal.h"
#include "sim/hil_link.h"
#include <drivers/mtd_nor.h>
#define FLIGHT_BENCH_TUNING
#define FLIGHT_NOR_LOG_ENABLE
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
#include "../../applications/tasks/logger/task_logger.c"
#include "../../applications/tasks/flash/task_flash.c"
#include "../../middleware/communication/telemetry_link/telemetry_link.c"
static ClockInstant test_clock={0x12345678,1000000,1,0};
static struct imu_raw_sample test_imu;
static int test_imu_pending;
static unsigned checks,failures,sem_calls,commits,inhibits;
static bool preempt_supervisor;
static bool inline_delays;
static struct rt_mtd_nor_device *test_nor_device;
static int imu_token, dshot_token, timebase_token, gnss_token_dev, rc_token_dev, radio_token_dev, wdt_token;
static jmp_buf scheduler;
static unsigned seen_packets[21], gnss_log_requests;
static unsigned rc_reads, radio_reads, hil_rx_frames;
static bool test_hil_active;
static uint8_t test_hil_input[64];
static size_t test_hil_input_size;
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
int rt_sem_init(struct rt_semaphore *s,const char *n,int v,int f) {
  (void)n;(void)f;s->value=(unsigned)v;s->max_value=UINT16_MAX;return RT_EOK;
}
int rt_sem_detach(struct rt_semaphore *s) { (void)s;return 0; }
int rt_sem_release(struct rt_semaphore *s) {
  if(s->value==s->max_value) return -RT_EFULL;
  ++s->value;return RT_EOK;
}
int rt_sem_take(struct rt_semaphore *s,rt_tick_t t) {
  (void)t;if(sem_calls++) longjmp(scheduler,1);
  if(!s->value) return -RT_ETIMEOUT;
  --s->value;return RT_EOK;
}
rt_tick_t rt_tick_get(void) { return (rt_tick_t)(test_clock.monotonic_us/1000); }
rt_tick_t rt_tick_from_millisecond(unsigned ms) { return ms; }
void rt_thread_delay_until(rt_tick_t *l,rt_tick_t t) { (void)l;(void)t;longjmp(scheduler,1); }
rt_err_t rt_thread_mdelay(rt_int32_t ms) { (void)ms;if(!inline_delays) longjmp(scheduler,1);return RT_EOK; }
uint32_t board_reset_flags(void) { return 0x04000000u; }
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
  if(!strcmp(name,"nor0") && test_nor_device) return &test_nor_device->parent;
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
  if(c==RT_SERIAL_CTRL_GET_CONFIG) ((struct serial_configure *)p)->baud_rate=115200u;
  if(d==(rt_device_t)&gnss_token_dev && c==RT_DEVICE_CTRL_CONFIG)
    CHECK(((const struct serial_configure *)p)->baud_rate==460800u);
  if(d==(rt_device_t)&radio_token_dev && c==RT_DEVICE_CTRL_CONFIG)
    CHECK(test_hil_active && ((const struct serial_configure *)p)->baud_rate==921600u);
  return RT_EOK;
}
rt_err_t rt_device_set_rx_indicate(rt_device_t d,rt_err_t (*cb)(rt_device_t,rt_size_t)) {
  (void)d;(void)cb;return RT_EOK;
}
Status clock_read(ClockInstant *c) { if(!c)return STATUS_INVALID_ARGUMENT;*c=test_clock;return 0; }
uint64_t board_boot_id(void) { return test_clock.boot_id; }
Status clock_now_us(uint64_t *now) {
  *now=test_clock.monotonic_us;
  if(preempt_supervisor) {
    preempt_supervisor=false;
    test_clock.monotonic_us+=1000;
    runtime_data_mark_heartbeat(RUNTIME_OWNER_SENSOR,test_clock.monotonic_us);
  }
  return STATUS_OK;
}
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
  if(d==(rt_device_t)&rc_token_dev) ++rc_reads;
  if(d==(rt_device_t)&radio_token_dev) {
    ++radio_reads;
    if(test_hil_input_size && length>=test_hil_input_size) {
      const size_t count=test_hil_input_size;
      memcpy(data,test_hil_input,count);test_hil_input_size=0;
      return (rt_ssize_t)count;
    }
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
  if(d==(rt_device_t)&radio_token_dev) {
    CHECK(length>=12 && p[0]==0xfd);
    id=p[7]|((unsigned)p[8]<<8)|((unsigned)p[9]<<16);
    if(test_hil_active) {
      CHECK(id==MAVLINK_HIL_MSG_HEARTBEAT && test_tx_complete==NULL);
      return (rt_ssize_t)length;
    }
    CHECK(p==tx_frame && length<=sizeof(tx_frame));
    switch (id) {
      case 0: slot=13; break;
      case 1: slot=14; break;
      case 24: slot=15; break;
      case 31: slot=16; break;
      case 32: slot=17; break;
      case EXTENDED_SYS_STATE: slot=18; break;
      case 74: slot=19; break;
      case MAVLINK_ALTITUDE_ID: slot=20; break;
      default: slot=id>=43000 && id<=43012 ? id-43000 : 21; break;
    }
    CHECK(slot<21 || id==PARAM_VALUE);
    if(slot<21 && !seen_packets[slot]++) {
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
  int32_t az=(int32_t)lround(-9.80665*26756268.0);uint32_t v=(uint32_t)az;
  b[22]=(uint8_t)(v>>8);b[23]=(uint8_t)v;b[24]=(uint8_t)(v>>24);b[25]=(uint8_t)(v>>16);
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
static void late_gnss(unsigned sequence) {
  SensorMeasurement m; unsigned axis;
  memset(&m,0,sizeof(m));m.header=h(SOURCE_GNSS,test_clock.monotonic_us+20000);
  m.header.frame_id=FRAME_WGS84_GEODETIC;m.header.sequence=sequence;
  m.measurement_kind=PRODUCT_MEASUREMENT_GNSS;m.sensor_instance_id=4;
  m.payload.gnss.header=m.header;m.payload.gnss.sensor_instance_id=4;
  m.payload.gnss.calibration_version=1;m.payload.gnss.quality_flags=15;
  m.payload.gnss.fix_type=GNSS_FIX_RTK_FIXED;
  m.payload.gnss.latitude_deg=30;m.payload.gnss.longitude_deg=114;m.payload.gnss.altitude_m=30;
  for(axis=0;axis<3;++axis) {
    m.payload.gnss.position_variance_m2[axis]=.04f;
    m.payload.gnss.velocity_variance_m2ps2[axis]=.01f;
  }
  CHECK(runtime_submit_measurement(&m)==STATUS_OK);
}
static void test_readonly_ground_link(void);

static void test_pipeline(void) {
  unsigned i; RuntimeStats s; NavigationSnapshot n; NavigationCapability c;
  { int result=runtime_start(); CHECK(result==RT_EOK); if(result) return; }
  {
    const unsigned kp = (unsigned)parameter_store_find("CTL_RATE_KP_X");
    const unsigned mass = (unsigned)parameter_store_find("VEH_MASS");
    const float original = config.controller.payload.rate_kp_nm_per_radps[0];
    CHECK(parameter_store_value(kp) == original);
    CHECK(!set_bench_parameter(kp, NAN) && !set_bench_parameter(kp, -1.0f));
    CHECK(!set_bench_parameter(mass, 2.0f));
    CHECK(set_bench_parameter(kp, 0.15f) && parameter_store_value(kp) == 0.15f);
    ProductControllerConfigPayload candidate = {0};
    parameter_store_read_payload(&candidate);
    CHECK(candidate.rate_kp_nm_per_radps[0] == 0.15f);
    CHECK(m6.config.payload.rate_kp_nm_per_radps[0] == 0.15f);
    runtime_data.fms.arm_state = ARM_STATE_ARMED;
    CHECK(!set_bench_parameter(kp, 0.16f) && parameter_store_value(kp) == 0.15f);
    CHECK(m6.config.payload.rate_kp_nm_per_radps[0] == 0.15f);
    runtime_data.fms.arm_state = ARM_STATE_DISARMED;
    CHECK(set_bench_parameter(kp, original));
  }
  signal_imu_ready();signal_imu_ready();
  CHECK(runtime_data.imu_event.value==1u);
  (void)rt_sem_release(&runtime_data.plan_event);
  (void)rt_sem_release(&runtime_data.plan_event);
  CHECK(runtime_data.plan_event.value==1u);
  for(i=1;i<=1250;++i) {
    test_clock.monotonic_us+=1000;imu_frame(i);turn(task_sensor_entry);turn(task_actuator_entry);
    if(i==990) {
      SensorMeasurement m;memset(&m,0,sizeof(m));m.header=h(SOURCE_BARO,test_clock.monotonic_us+20000);
      m.measurement_kind=PRODUCT_MEASUREMENT_BARO;m.sensor_instance_id=3;
      m.payload.baro.header=m.header;m.payload.baro.sensor_instance_id=3;m.payload.baro.calibration_version=1;
      m.payload.baro.altitude_m=100;m.payload.baro.variance_m2=.25f;m.payload.baro.quality_flags=7;
      CHECK(runtime_submit_measurement(&m)==STATUS_OK);
    }
    if(i%20==0) { turn(task_mission_entry);turn(task_mpc_entry); }
    if(i%4==0) turn(task_communication_entry);
    if(i%100==0) task_supervisor_step();
  }
  runtime_get_stats(&s);CHECK(s.imu_batches==1250);CHECK(s.imu_rejected==0);
  CHECK(fast_bundle_revision==runtime_data.bundle_revision);
  CHECK(gnss_log_requests==1);
  CHECK(s.nav_updates==1250);CHECK(s.planning_updates==62);CHECK(s.mpc_updates==62);
  CHECK(runtime_get_navigation(&n,&c)==0);CHECK(n.core.valid_mask & NAV_VALID_ATTITUDE);
  CHECK(s.control_updates==0);CHECK(s.control_rejected==1250);CHECK(commits==0);
  CHECK(runtime_data.actuator.available_mask==0);CHECK(runtime_data.actuator.feedback_valid_mask==0);
  CHECK(runtime_data.prearm.state!=PREARM_READY);
  CHECK(s.deadline_misses==0);
  /* Drive a real isolation -> shadow probation -> degraded -> normal cycle.
   * Only the transport/time boundary is mocked; M3/Health policies and NIS
   * evaluation run through the production owner turns. */
  for(i=0;i<10;++i) CHECK(si_report(&integrity,3,SI_EVIDENCE_SAMPLE_REJECTED,test_clock.monotonic_us)==SI_OK);
  CHECK(si_state(&integrity,3)==SI_SOURCE_ISOLATED);refresh_policies(test_clock.monotonic_us);
  {
    int saw_probation=0,saw_degraded=0;
    for(i=1251;i<=5550;++i) {
      test_clock.monotonic_us+=1000;imu_frame(i);turn(task_sensor_entry);turn(task_actuator_entry);
      if(i%100==50) {
        SensorMeasurement m;memset(&m,0,sizeof(m));m.header=h(SOURCE_BARO,test_clock.monotonic_us+20000);
        m.header.sequence=i;m.measurement_kind=PRODUCT_MEASUREMENT_BARO;m.sensor_instance_id=3;
        m.payload.baro.header=m.header;m.payload.baro.sensor_instance_id=3;m.payload.baro.calibration_version=1;
        m.payload.baro.altitude_m=100;m.payload.baro.variance_m2=.25f;m.payload.baro.quality_flags=7;
        CHECK(runtime_submit_measurement(&m)==STATUS_OK);
      }
      if(i%20==0) { turn(task_mission_entry);turn(task_mpc_entry); }
      if(i%4==0) turn(task_communication_entry);
      if(i%100==0) task_supervisor_step();
      if(si_state(&integrity,3)==SI_SOURCE_PROBATION) saw_probation=1;
      if(si_state(&integrity,3)==SI_SOURCE_SUSPECT && policies[2].use_state==SENSOR_USE_DEGRADED) saw_degraded=1;
      if(i<3200) CHECK(si_state(&integrity,3)!=SI_SOURCE_OK);
    }
    CHECK(saw_probation);CHECK(saw_degraded);CHECK(si_state(&integrity,3)==SI_SOURCE_OK);
    CHECK(commits==0);
  }
  /* GNSS/PPS becomes available after local IMU alignment. M3 may stage a
   * candidate but only M4's fresh, disarmed authorization changes origin. */
  {
    NavigationSnapshot before,staged,after; ContractHeader candidate,bad;
    EstimatorResetRequest request; uint32_t next=0;
    CHECK(estimator_get_snapshot(&before)==0);
    CHECK(!(before.core.valid_mask&NAV_VALID_HORIZONTAL_POSITION));
    late_gnss(5550);turn(task_sensor_entry);
    CHECK(ins_get_pending_origin(&candidate,&next)==0);
    CHECK(estimator_get_snapshot(&staged)==0);
    CHECK(staged.origin_id==before.origin_id && staged.state_epoch==before.state_epoch);
    CHECK(memcmp(&before.core,&staged.core,sizeof(before.core))==0);
    CHECK(fms_authorize_origin_reset(&candidate,next+1,&request)==STATUS_NOT_PERMITTED);
    bad=candidate;bad.boot_id++;
    CHECK(fms_authorize_origin_reset(&bad,next,&request)!=0);
    bad=candidate;bad.timestamp_quality=TIMESTAMP_QUALITY_RECEIVE_TIME_ESTIMATED;
    CHECK(fms_authorize_origin_reset(&bad,next,&request)!=0);
    bad=candidate;bad.timestamp_quality=UINT8_MAX;
    CHECK(fms_authorize_origin_reset(&bad,next,&request)==STATUS_STALE_DATA);
    bad=candidate;bad.clock_domain_id=CLOCK_DOMAIN_GNSS_GPS;
    CHECK(fms_authorize_origin_reset(&bad,next,&request)==STATUS_STALE_DATA);
    /* task_mission_entry refreshes M4's clock before signing the bounded request. */
    turn(task_mission_entry);CHECK(runtime_data.have_origin_reset);
    request=runtime_data.origin_reset;request.header.timestamp_sample_us--;
    CHECK(estimator_reset(&request)==STATUS_NOT_PERMITTED);
    turn(task_sensor_entry);CHECK(!runtime_data.have_origin_reset);
    CHECK(estimator_get_snapshot(&after)==STATUS_NOT_READY);
    CHECK(after.origin_id==next && after.state_epoch==before.state_epoch+1);
    CHECK(!(after.core.valid_mask&NAV_VALID_ATTITUDE));
    CHECK(ins_get_pending_origin(&candidate,&next)==STATUS_NOT_READY);
    for(i=5551;i<=6700;++i) {
      test_clock.monotonic_us+=1000;
      if(i%100==40) late_gnss(i);
      imu_frame(i);turn(task_sensor_entry);turn(task_actuator_entry);
      if(i%20==0) { turn(task_mission_entry);turn(task_mpc_entry); }
      if(i%4==0) turn(task_communication_entry);
      if(i%100==0) task_supervisor_step();
    }
    CHECK(estimator_get_snapshot(&after)==0);
    CHECK(after.core.valid_mask&NAV_VALID_ATTITUDE);
    CHECK(after.core.valid_mask&NAV_VALID_HORIZONTAL_POSITION);
    CHECK(after.origin_id==before.origin_id+1);
    CHECK(commits==0 && runtime_data.prearm.state!=PREARM_READY);
  }
  preempt_supervisor=true;task_supervisor_step();
  CHECK(!preempt_supervisor && runtime_stats.deadline_misses==0);
  test_clock.monotonic_us+=501000;task_supervisor_step();
  runtime_get_stats(&s);CHECK(s.deadline_misses==1);CHECK(s.last_error==STATUS_DEADLINE_MISSED);
  for(i=0;i<17;++i) CHECK(seen_packets[i]>0);
  CHECK(seen_packets[19]>0 && seen_packets[20]>0);
  {
    RuntimeSensorHealth health={0}; RuntimeEvent event={0}; uint32_t previous=0;
    CHECK(runtime_get_sensor_health(1,&health)==0);
    CHECK(health.health_state==4 && health.use_state==SENSOR_USE_REJECT && health.reason_code==19);
    CHECK(runtime_get_sensor_health(4,&health)==STATUS_INVALID_ARGUMENT);
    /* UART delay retains the first 32 events, reports overflow, and cannot
     * corrupt a snapshot or wrap the bounded event queue. */
    while(runtime_take_event(&event)==0) {}
    for(i=0;i<40;++i) runtime_data_record_event(SOURCE_PLATFORM,0,6,3,test_clock.monotonic_us);
    for(i=0;i<32;++i) {
      CHECK(runtime_take_event(&event)==0);CHECK(event.event_id>previous);
      CHECK(event.reason_code==6 && event.severity==3 && event.boot_id==test_clock.boot_id);
      previous=event.event_id;
    }
    CHECK(runtime_take_event(&event)==STATUS_NOT_READY);
    runtime_get_stats(&s);CHECK(s.event_drops==8);
  }
  /* A stale/foreign sample cannot enter the bounded measurement queue. */
  { SensorMeasurement m;memset(&m,0,sizeof(m));m.header=h(SOURCE_GNSS,test_clock.monotonic_us-1);
    CHECK(runtime_submit_measurement(&m)==STATUS_INVALID_ARGUMENT);
    m.header=h(SOURCE_GNSS,test_clock.monotonic_us+1000);m.header.boot_id++;
    CHECK(runtime_submit_measurement(&m)==STATUS_INVALID_ARGUMENT);
    m.header=h(SOURCE_GNSS,test_clock.monotonic_us+1000);m.header.timestamp_quality=UINT8_MAX;
    CHECK(runtime_submit_measurement(&m)==STATUS_INVALID_ARGUMENT); }
  test_readonly_ground_link();
  running=0;CHECK(actuator_stop()==0);CHECK(controller_stop()==0);CHECK(estimator_stop()==0);
  CHECK(fms_policy_stop()==0);CHECK(guidance_stop()==0);
}
static Status commit(const uint16_t commands[4], uint64_t *actual) {
  for (unsigned channel = 0; channel < 4; ++channel) {
    CHECK(commands[channel] >= 48 && commands[channel] <= 2047);
  }
  commits++;
  *actual = test_clock.monotonic_us;
  return STATUS_OK;
}
static unsigned guard_locks_until_delay;
static uint32_t guard_faults;
static void test_guard_lock(void) {
  if(guard_locks_until_delay && !--guard_locks_until_delay)
    test_clock.monotonic_us+=5000;
}
static void test_guard_unlock(void) {}
static uint32_t test_guard_fault_count(void) { return guard_faults; }
static void test_actuator(void) {
  ProductActuatorConfig cfg; ActuationPermit p; ActuatorDemand d; ActuatorStatus s;
  ActuatorCommitReceipt receipt; ActuatorDriver driver;unsigned i;
  memset(&driver,0,sizeof(driver));driver.inhibit=test_inhibit;driver.commit=commit;
  driver.lock=test_guard_lock;driver.unlock=test_guard_unlock;driver.fault_count=test_guard_fault_count;
  CHECK(actuator_guard_set_driver(&driver)==0);
  memset(&cfg,0,sizeof(cfg));cfg.profile_hash=1;cfg.config_version=1;
  cfg.calibration_hash=1;cfg.calibration_set_id=1;cfg.payload.calibration_version=1;
  cfg.payload.output_policy_id=2;cfg.payload.esc_bit_rate=300000;
  for(i=0;i<4;++i) {
    cfg.payload.motor_to_timer_channel[i]=(uint8_t)(4-i);cfg.payload.thrust_curve_count[i]=2;
    cfg.payload.thrust_curve_n[i][1]=10;cfg.payload.dshot_curve_code[i][0]=48;cfg.payload.dshot_curve_code[i][1]=2047;
    cfg.payload.max_thrust_n[i]=10;cfg.payload.max_slew_nps[i]=100;
  }
  CHECK(actuator_init(&cfg)==0); memset(&p,0,sizeof(p));memset(&d,0,sizeof(d));
  p.header=h(SOURCE_M4,test_clock.monotonic_us+100000);p.permit_epoch=1;
  p.directive_epoch=p.authority_epoch=p.state_epoch=p.reference_epoch=1;
  p.permit_flags=PERMIT_SCOPE_CONTROL_ACTIVE;p.output_policy_id=2;p.permit_expires_at_us=p.header.valid_until_us;
  d.header=h(SOURCE_M6,test_clock.monotonic_us+5000);d.header.frame_id=FRAME_BODY_FRD;
  d.actuator_layout_id=PRODUCT_ACTUATOR_LAYOUT_ID;d.valid_channel_mask=d.demand.valid_mask=15;
  d.permit_epoch=d.directive_epoch=d.authority_epoch=d.state_epoch=d.reference_epoch=1;
  d.control_sequence=1;d.demand_expires_at_us=d.header.valid_until_us;
  for(i=0;i<4;++i) d.demand.motor_thrust_n[i]=5;
  CHECK(actuator_apply(&d,&p,&s)==0);CHECK(commits==0);
  CHECK(actuator_get_commit_receipt(&receipt)==STATUS_NOT_READY);
  CHECK(actuator_guard_frame_boundary(test_clock.monotonic_us)==0);CHECK(commits==1);
  CHECK(actuator_get_commit_receipt(&receipt)==0);CHECK(receipt.applied_control_sequence==1);
  CHECK(receipt.result_code==ACTUATOR_RECEIPT_COMMITTED);CHECK(s.feedback_valid_mask==0);
  CHECK(actuator_apply(&d,&p,&s)==STATUS_STALE_DATA);
  test_clock.monotonic_us+=2500;d.control_sequence=2;d.header=h(SOURCE_M6,test_clock.monotonic_us+5000);
  d.header.frame_id=FRAME_BODY_FRD;d.demand_expires_at_us=d.header.valid_until_us;
  for(i=0;i<4;++i) d.demand.motor_thrust_n[i]=8;
  CHECK(actuator_apply(&d,&p,&s)==0);CHECK(actuator_guard_frame_boundary(test_clock.monotonic_us)==0);
  CHECK(actuator_get_commit_receipt(&receipt)==0);CHECK(receipt.limit_flags==15);
  CHECK(fabsf(receipt.committed_demand.motor_thrust_n[0]-5.25f)<1e-5f);
  p.header=h(SOURCE_M4,test_clock.monotonic_us+100000);p.header.sequence=2;p.permit_flags=PERMIT_FLAG_REVOKED;
  CHECK(actuator_update_permit(&p,test_clock.monotonic_us)==0);
  p.header.sequence=3;p.permit_flags=PERMIT_SCOPE_CONTROL_ACTIVE;
  CHECK(actuator_update_permit(&p,test_clock.monotonic_us)==STATUS_NOT_PERMITTED);
  test_clock.monotonic_us+=2500;CHECK(actuator_guard_frame_boundary(test_clock.monotonic_us)!=0);CHECK(commits==2);

  test_clock.monotonic_us+=2500;
  p.permit_epoch=d.permit_epoch=2;
  d.control_sequence=3;
  d.header=h(SOURCE_M6,test_clock.monotonic_us+5000);d.header.frame_id=FRAME_BODY_FRD;
  d.demand_expires_at_us=d.header.valid_until_us;
  CHECK(actuator_apply(&d,&p,&s)==STATUS_OK);
  /* The demand is valid at wake and expired by the time the driver lock is taken. */
  guard_locks_until_delay=2;
  CHECK(actuator_guard_frame_boundary(test_clock.monotonic_us)==STATUS_NOT_PERMITTED);
  CHECK(!guard_locks_until_delay && commits==2);
  CHECK(actuator_get_commit_receipt(&receipt)==STATUS_OK && receipt.applied_control_sequence==2);

  guard_faults=1;
  CHECK(actuator_poll(test_clock.monotonic_us,&s)==STATUS_NOT_READY && s.fault_mask==15);
  guard_faults=0;
  CHECK(actuator_apply(&d,&p,&s)==STATUS_NOT_READY);
  CHECK(actuator_poll(test_clock.monotonic_us,&s)==STATUS_NOT_READY && s.available_mask==0);
  CHECK(actuator_guard_state()==ACTUATOR_GUARD_FAULT_LATCHED);
  CHECK(actuator_guard_frame_boundary(test_clock.monotonic_us)==STATUS_NOT_READY && commits==2);
  CHECK(actuator_stop()==0);cfg.payload.motor_to_timer_channel[1]=cfg.payload.motor_to_timer_channel[0];
  CHECK(actuator_init(&cfg)==STATUS_INVALID_ARGUMENT);
}
static void test_native(void) {
  const char *golden="#BESTNAVA,97,GPS,FINE,2294,472312000,0,0,18,16;SOL_COMPUTED,SINGLE,40.07895888272,116.23651029820,65.8312,-8.4925,WGS84,1.2221,1.1053,2.1970,\"0\",0.000,0.000,50,28,28,0,1,12,12,41,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0046,335.592288,0.0045,0.0194,0.0123*c1b4f7fe\r\n";
  UnicoreAsciiParser parser={0};UnicoreAsciiMessage msg;unsigned i;int result=0;
  for(i=0;golden[i];++i) { int r=unicore_ascii_feed(&parser,(uint8_t)golden[i],&msg);if(r) result=r; }
  CHECK(result==1);CHECK(parser.valid_frames==1);CHECK(msg.pvt.satellites==28);
  CHECK(msg.pvt.valid_flags==15);CHECK(fabs(msg.pvt.altitude_m-57.3387)<1e-4);
  CHECK(fabs(msg.pvt.altitude_msl_m-65.8312)<1e-4);
  CHECK(msg.pvt.velocity_ned_mps[2]<0);CHECK(msg.pvt.fix_type==GNSS_FIX_SINGLE);
  for(i=0;golden[i];++i) unicore_ascii_feed(&parser,(uint8_t)(i==70 ? '9' : golden[i]),&msg);
  CHECK(parser.crc_errors==1);
  {
    const char *rectime="#RECTIMEA,97,GPS,FINE,2190,365121000,0,0,18,12;VALID,3.580410506e-04,0.000000000e+00,-18.00000000000,2021,12,30,5,25,3000,VALID*7e364e74\r\n";
    result=0;
    for(i=0;rectime[i];++i) { int r=unicore_ascii_feed(&parser,(uint8_t)rectime[i],&msg);if(r)result=r; }
    CHECK(result==1);CHECK(msg.kind==UNICORE_ASCII_RECTIME);CHECK(msg.receiver_accuracy_ns>=358041);
    CHECK(msg.utc_time_ns==1640841903000000000ULL);
  }
  unicore_ascii_feed(&parser,'#',&msg);for(i=0;i<1000;++i)unicore_ascii_feed(&parser,'x',&msg);CHECK(parser.overflows==1);
  /* An arbitrary byte stream cannot write past the bounded parser state. */
  for(i=0;i<65536;++i)unicore_ascii_feed(&parser,(uint8_t)(i*37),&msg);
}
static void test_wire(void) {
  uint8_t p[FCU_OPERATION_STATUS_LEN]={0},out[128];size_t n,i;
  wire_u64(p,0x0102030405060708ULL);wire_u64(p+8,0x1122334455667788ULL);
  wire_u32(p+16,PRODUCT_PROFILE_ID);wire_u32(p+20,9);wire_u32(p+24,10);wire_u32(p+28,11);
  p[FCU_OPERATION_STATUS_FLIGHT_MODE]=4;p[FCU_OPERATION_STATUS_AUTHORITY]=1;
  n=wire_encode(FCU_OPERATION_STATUS_ID,FCU_OPERATION_STATUS_CRC,p,sizeof(p),7,out,sizeof(out));
  CHECK(n==61);CHECK(out[0]==0xfd);CHECK(out[1]==49);CHECK(out[4]==7);
  printf("WIRE=");for(i=0;i<n;++i)printf("%02x",out[i]);printf("\n");
  CHECK(wire_encode(43000,213,p,sizeof(p),0,out,10)==0);
  CHECK(wire_encode(43000,213,p,0,0,out,sizeof(out))==0);
}
static void test_hil_frame(const mavlink_hil_frame_t *frame)
{
  CHECK(frame->id==MAVLINK_HIL_MSG_HEARTBEAT && frame->sequence==42u);
  ++hil_rx_frames;
}

static void inject_gcs_request(uint8_t id, uint8_t extra, const uint8_t *payload,
                               size_t size, bool valid_crc)
{
  uint8_t frame[64];
  const size_t length=wire_encode(id, extra, payload, size, 0, frame, sizeof(frame));
  CHECK(length>0);
  if(!length) return;
  frame[5]=255; frame[6]=190;
  uint16_t crc=0xffffu;
  for(size_t i=1;i<length-2;++i) crc=mavlink_crc_accumulate(crc,frame[i]);
  wire_u16(frame+length-2,mavlink_crc_accumulate(crc,extra) ^ (valid_crc ? 0u : 1u));
  for(size_t i=0;i<length;++i) receive_byte(frame[i], comm_gcs_receive);
}

static void check_ground_ack(const uint8_t *command, uint8_t id, uint8_t extra,
                             size_t size, uint8_t result, uint8_t reason)
{
  uint8_t frame[TX_FRAME_SIZE];
  inject_gcs_request(id, extra, command, size, true);
  CHECK(rt_mq_recv(&tx_queue,frame,sizeof(frame),0)==22);
  CHECK(frame[7]==COMMAND_ACK && read_u16(frame+10)==read_u16(command+28));
  CHECK(frame[12]==result && frame[14]==reason && frame[18]==255 && frame[19]==190);
  --tx_pending;
}

static void test_calibration_service(void)
{
  sensor_calibration_service_init();
  sensor_calibration_service_set_complete(calibration_complete);
  SensorCalibrationStatus status;
  char error[40];
  uint64_t now=test_clock.monotonic_us;

  /* Gyro: a known 0.1 rad/s bias must become about 5.73 dps. */
  CHECK(sensor_calibration_start(SENSOR_CAL_GYRO,now,error,sizeof(error)));
  for(unsigned i=0;i<1600;++i) {
    ImuSample s; memset(&s,0,sizeof(s));
    s.gyro_frd_radps[0]=0.1f;
    sensor_calibration_feed_imu(&s,now+i*1000ull);
    sensor_calibration_tick(now+i*1000ull);
  }
  sensor_calibration_get_status(&status);
  CHECK(status.state==SENSOR_CAL_DONE);
  CHECK(fabsf(parameter_store_value((unsigned)parameter_store_find("CAL_GYRO_X"))-5.7296f)<0.01f);
  CHECK(parameter_store_value((unsigned)parameter_store_find("CAL_GYRO_OK"))==1.0f);

  /* Accelerometer six-face: identity sensor, zero bias. */
  now+=2000000;
  CHECK(sensor_calibration_start(SENSOR_CAL_ACCEL,now,error,sizeof(error)));
  static const float faces[6][3]={{0,0,-9.8f},{0,0,9.8f},{9.8f,0,0},{-9.8f,0,0},{0,9.8f,0},{0,-9.8f,0}};
  for(unsigned face=0;face<6;++face) {
    for(unsigned i=0;i<200;++i) {
      ImuSample s; memset(&s,0,sizeof(s));
      memcpy(s.accel_frd_mps2,faces[face],sizeof(faces[face]));
      sensor_calibration_feed_imu(&s,now+i*1000ull);
    }
    if (face == 0)
      CHECK(!sensor_calibration_capture_accel_face(1,now+200000,error,sizeof(error)));
    CHECK(sensor_calibration_capture_accel_face((uint8_t)face,now+200000,error,sizeof(error)));
    sensor_calibration_tick(now+200000);
    now+=200000;
  }
  sensor_calibration_get_status(&status);
  CHECK(status.state==SENSOR_CAL_DONE);
  CHECK(fabsf(parameter_store_value((unsigned)parameter_store_find("CAL_ACC_X")))<0.01f);
  CHECK(fabsf(parameter_store_value((unsigned)parameter_store_find("CAL_ACC_SX"))-1.0f)<0.01f);
  CHECK(parameter_store_value((unsigned)parameter_store_find("CAL_ACC_OK"))==1.0f);

  /* Magnetometer: min/max sphere gives zero offset and unit scale. */
  now+=2000000;
  CHECK(sensor_calibration_start(SENSOR_CAL_MAG,now,error,sizeof(error)));
  for(unsigned i=0;i<1001;++i) {
    static const float directions[6][3]={{100,0,0},{-100,0,0},{0,100,0},{0,-100,0},{0,0,100},{0,0,-100}};
    const float *m=directions[i%6];
    sensor_calibration_feed_mag(m,now+i*20000ull);
  }
  sensor_calibration_tick(now+1001*20000ull);
  sensor_calibration_get_status(&status);
  CHECK(status.state==SENSOR_CAL_DONE);
  CHECK(fabsf(parameter_store_value((unsigned)parameter_store_find("CAL_MAG_X")))<0.1f);
  CHECK(fabsf(parameter_store_value((unsigned)parameter_store_find("CAL_MAG_SX"))-1.0f)<0.05f);
  CHECK(parameter_store_value((unsigned)parameter_store_find("CAL_MAG_OK"))==1.0f);

  /* Level and barometric reference. */
  now+=2000000;
  CHECK(sensor_calibration_start(SENSOR_CAL_LEVEL,now,error,sizeof(error)));
  for(unsigned i=0;i<1200;++i) {
    ImuSample s; memset(&s,0,sizeof(s)); s.accel_frd_mps2[2]=-9.8f;
    sensor_calibration_feed_imu(&s,now+i*1000ull);
    sensor_calibration_tick(now+i*1000ull);
  }
  CHECK(parameter_store_value((unsigned)parameter_store_find("CAL_LVL_OK"))==1.0f);
  now+=2000000;
  CHECK(sensor_calibration_start(SENSOR_CAL_BARO,now,error,sizeof(error)));
  for(unsigned i=0;i<120;++i) {
    sensor_calibration_feed_baro(101000.0f,now+i*10000ull);
    sensor_calibration_tick(now+i*10000ull);
  }
  CHECK(fabsf(parameter_store_value((unsigned)parameter_store_find("CAL_BARO_P0"))-101000.0f)<1.0f);

  /* RC: full stick travel on four channels. */
  now+=2000000;
  CHECK(sensor_calibration_start(SENSOR_CAL_RC,now,error,sizeof(error)));
  for(unsigned i=0;i<200;++i) {
    uint16_t ch[16]={0};
    ch[0]=(uint16_t)(172+i*8); ch[1]=(uint16_t)(1811-i*8);
    ch[2]=(uint16_t)(172+i*8); ch[3]=(uint16_t)(172+i*8);
    sensor_calibration_feed_rc(ch,now+i*20000ull);
  }
  CHECK(!sensor_calibration_finish_rc(now+200*20000ull,error,sizeof(error)));
  const uint16_t centered[16]={990,1000,172,1010};
  sensor_calibration_feed_rc(centered,now+200*20000ull);
  CHECK(sensor_calibration_finish_rc(now+200*20000ull,error,sizeof(error)));
  sensor_calibration_tick(now+200*20000ull);
  sensor_calibration_get_status(&status);
  CHECK(status.state==SENSOR_CAL_DONE);
  CHECK(parameter_store_value((unsigned)parameter_store_find("RC_CAL_OK"))==1.0f);
  CHECK(parameter_store_value((unsigned)parameter_store_find("RC_MIN_1"))==172.0f);
  CHECK(parameter_store_value((unsigned)parameter_store_find("RC_MAX_1"))==1764.0f);
  const uint64_t trim_us = now + 4010000u;
  const uint16_t updated_trim[16] = {1000, 1010, 172, 1020};
  const uint16_t unsafe_trim[16] = {1750, 1010, 172, 1020};
  sensor_calibration_feed_rc(unsafe_trim, trim_us);
  CHECK(!sensor_calibration_copy_rc_trims(trim_us,error,sizeof(error)));
  sensor_calibration_feed_rc(updated_trim, trim_us);
  CHECK(!sensor_calibration_copy_rc_trims(trim_us+500000u,error,sizeof(error)));
  CHECK(sensor_calibration_copy_rc_trims(trim_us,error,sizeof(error)));
  sensor_calibration_tick(trim_us);
  CHECK(parameter_store_value((unsigned)parameter_store_find("RC_TRIM_1"))==1000.0f);
  CHECK(parameter_store_value((unsigned)parameter_store_find("RC_TRIM_3"))==172.0f);
  CHECK(parameter_store_value((unsigned)parameter_store_find("RC_MAX_1"))==1764.0f);
  const unsigned gyro_x=(unsigned)parameter_store_find("CAL_GYRO_X");
  const float saved_gyro=parameter_store_value(gyro_x);
  CHECK(sensor_calibration_start(SENSOR_CAL_GYRO,now+5000000,error,sizeof(error)));
  sensor_calibration_cancel(now+5001000);
  sensor_calibration_tick(now+5001000);
  CHECK(parameter_store_value(gyro_x)==saved_gyro);

  uint8_t command[28]={0};
  wire_float(command+16,2.0f);
  CHECK(mavlink_calibration_kind(command)==SENSOR_CAL_LEVEL);
  wire_float(command,1.0f);
  CHECK(mavlink_calibration_kind(command)==SENSOR_CAL_NONE);
  wire_float(command,0.0f);
  wire_float(command+16,0.0f);
  wire_float(command+20,1.0f);
  CHECK(mavlink_calibration_kind(command)==SENSOR_CAL_NONE);

}

static void test_readonly_ground_link(void)
{
  uint8_t frame[TX_FRAME_SIZE];
  runtime_data.fms.header = h(SOURCE_M4,test_clock.monotonic_us+100000);
  runtime_data.fms.header.clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
  runtime_data.fms.arm_state = ARM_STATE_DISARMED;
  const unsigned mode_ch = (unsigned)parameter_store_find_exposed("RC_MODE_CH");
  const unsigned mode_4 = (unsigned)parameter_store_find_exposed("RC_MODE_4");
  uint16_t rc[16] = {0};
  CHECK(comm_rc_mapped_mode(rc)==0);
  CHECK(!parameter_store_set(mode_ch,4.0f));
  CHECK(parameter_store_set(mode_ch,5.0f));
  CHECK(parameter_store_set(mode_4,4.0f));
  rc[4] = CRSF_RAW_CENTER;
  CHECK(comm_rc_mapped_mode(rc)==(PRODUCT_MODE_SWITCH_MAPPED | FLIGHT_MODE_POS_HOLD));
  rc[4] = CRSF_RAW_MIN;
  CHECK(comm_rc_mapped_mode(rc)==PRODUCT_MODE_SWITCH_MAPPED);
  CHECK(parameter_store_set(mode_4,0.0f));
  CHECK(parameter_store_set(mode_ch,0.0f));
  const unsigned aux6 = (unsigned)parameter_store_find_exposed("RC_AUX_6");
  const unsigned aux7 = (unsigned)parameter_store_find_exposed("RC_AUX_7");
  CHECK(parameter_store_set(aux6,3.0f));
  CHECK(parameter_store_set(aux7,4.0f));
  rc[5]=rc[6]=CRSF_RAW_MAX;
  CHECK(comm_rc_mapped_mode(rc)==(PRODUCT_MODE_SWITCH_MAPPED | FLIGHT_MODE_ALT_HOLD));
  rc[5]=CRSF_RAW_MIN;
  CHECK(comm_rc_mapped_mode(rc)==(PRODUCT_MODE_SWITCH_MAPPED | FLIGHT_MODE_POS_HOLD));
  CHECK(parameter_store_set(aux6,0.0f));
  CHECK(parameter_store_set(aux7,0.0f));
  const uint8_t bind_expected[] = {0xc8,7,0x32,0xec,0xc8,0x10,1,0x9e,0xe8};
  uint8_t bind_frame[ELRS_BIND_FRAME_SIZE];
  elrs_encode_bind(bind_frame);
  CHECK(memcmp(bind_frame,bind_expected,sizeof(bind_expected))==0);
  while(rt_mq_recv(&tx_queue,frame,sizeof(frame),0)>0) --tx_pending;
  const uint8_t parameter_list[2]={1,0};
  const unsigned parameter_count=parameter_store_exposed_count();
  CHECK(parameter_count>0);
  CHECK(next_parameter==UINT16_MAX);
  inject_gcs_request(21,159,parameter_list,sizeof(parameter_list),false);
  CHECK(next_parameter==UINT16_MAX);
  inject_gcs_request(21,159,parameter_list,sizeof(parameter_list),true);
  CHECK(next_parameter==0);
  for(unsigned i=0;i<parameter_count;++i) {
    send_pending_parameter();
    CHECK(rt_mq_recv(&tx_queue,frame,sizeof(frame),0)==37);
    CHECK(frame[7]==22 && read_u16(frame+14)==parameter_count && read_u16(frame+16)==i);
    CHECK(memcmp(frame+18,parameter_store_name(parameter_store_exposed_at(i)),16)==0);
    --tx_pending;
  }
  CHECK(next_parameter==parameter_count);
  const unsigned kp=(unsigned)parameter_store_find("CTL_RATE_KP_X");
  const float kp_value=parameter_store_value(kp);
  uint8_t read[20]={0};
  wire_u16(read,UINT16_MAX);
  read[2]=1;
  memcpy(read+4,"CTL_RATE_KP_X",13);
  inject_gcs_request(20,214,read,sizeof(read),true);
  CHECK(rt_mq_recv(&tx_queue,frame,sizeof(frame),0)==37);
  uint8_t expected[4];
  wire_float(expected,kp_value);
  CHECK(frame[7]==22 && memcmp(frame+10,expected,sizeof(expected))==0);
  --tx_pending;

  uint8_t set[23]={0};
  set[4]=1;
  set[22]=9;
  memcpy(set+6,"CTL_RATE_KP_X",13);
  wire_float(set,0.15f);
  inject_gcs_request(23,168,set,sizeof(set),true);
  CHECK(rt_mq_recv(&tx_queue,frame,sizeof(frame),0)==37);
  wire_float(expected,0.15f);
  CHECK(frame[7]==22 && memcmp(frame+10,expected,sizeof(expected))==0);
  CHECK(parameter_store_value(kp)==0.15f);
  --tx_pending;
  memcpy(set+6,"VEH_MASS",9);
  wire_float(set,2.0f);
  const unsigned mass=(unsigned)parameter_store_find("VEH_MASS");
  const float mass_original=parameter_store_value(mass);
  inject_gcs_request(23,168,set,sizeof(set),true);
  CHECK(rt_mq_recv(&tx_queue,frame,sizeof(frame),0)==37);
  wire_float(expected,2.0f);
  CHECK(frame[7]==22 && memcmp(frame+10,expected,sizeof(expected))==0);
  --tx_pending;
  wire_float(set,mass_original);
  inject_gcs_request(23,168,set,sizeof(set),true);
  CHECK(rt_mq_recv(&tx_queue,frame,sizeof(frame),0)==37);
  --tx_pending;
  CHECK(parameter_store_value(kp)==0.15f);
  memcpy(set+6,"CTL_RATE_KP_X",13);
  wire_float(set,kp_value);
  inject_gcs_request(23,168,set,sizeof(set),true);
  CHECK(rt_mq_recv(&tx_queue,frame,sizeof(frame),0)==37);
  --tx_pending;
  uint8_t command[35]={0};
  const size_t requests_before=request_queue.count;
  wire_float(command,1.0f);
  wire_u16(command+28,400);
  command[30]=1;
  command[31]=1;
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_DENIED,BENCH_POLICY_REASON);
  wire_float(command,0.0f);
  wire_float(command+4,NAN); /* MAVSDK may leave the force parameter unset. */
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_DENIED,BENCH_POLICY_REASON);
  wire_float(command+4,21196.0f);
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_DENIED,1);
  wire_u16(command+28,22);
  wire_float(command+24,2.0f);
  command[32]=6;
  check_ground_ack(command,COMMAND_INT,158,sizeof(command),MAV_DENIED,BENCH_POLICY_REASON);
  wire_u16(command+28,21);
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_DENIED,BENCH_POLICY_REASON);
  wire_u16(command+28,262);
  wire_float(command,5.0f);
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_DENIED,BENCH_POLICY_REASON);
  wire_u16(command+28,176);
  wire_float(command,129.0f);
  wire_float(command+4,4.0f);
  wire_float(command+8,5.0f);
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_DENIED,BENCH_POLICY_REASON);
  wire_float(command+8,6.0f);
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_UNSUPPORTED,0);
  command[30]=2;
  inject_gcs_request(COMMAND_LONG,152,command,33,true);
  CHECK(tx_pending==0 && request_queue.count==requests_before);

  command[30]=1;
  wire_u16(command+28,MAV_CMD_SET_MESSAGE_INTERVAL);
  wire_float(command,EXTENDED_SYS_STATE);
  wire_float(command+4,200000.0f);
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_ACCEPTED,0);
  CHECK(extended_state_period_us==200000u);
  wire_float(command+4,199999.0f);
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_DENIED,1);
  CHECK(extended_state_period_us==200000u);
  wire_float(command+4,0.0f);
  check_ground_ack(command,COMMAND_LONG,152,33,MAV_ACCEPTED,0);
  CHECK(extended_state_period_us==EXTENDED_STATE_DEFAULT_US);
}

static void test_telemetry_hil_handoff(void)
{
  const unsigned before_rc=rc_reads, before_radio=radio_reads;
  CHECK(before_rc>0 && before_radio>0 && test_tx_complete!=NULL);
  telemetry_link_release_radio();
  CHECK(test_tx_complete==NULL && !telemetry_link_tx_idle());
  comm_rc_poll();
  comm_radio_receive();
  CHECK(rc_reads==before_rc+1u && radio_reads==before_radio);

  test_hil_active=true;
  CHECK(hil_link_open()==STATUS_OK && hil_link_is_open());
  test_hil_input_size=mavlink_hil_build_heartbeat(42u,test_hil_input,sizeof(test_hil_input));
  hil_link_poll_receive(test_hil_frame);
  hil_link_send_heartbeat();
  const HilLinkStats stats=hil_link_stats();
  CHECK(hil_rx_frames==1u && stats.rx_frames==1u && stats.rx_bytes>0u && stats.discarded_bytes==0u);
  CHECK(stats.tx_frames==1u && stats.tx_dropped==0u);
}

/* Memory-backed DFS boundary. The production recorder and RT message queues
 * remain in use while mount, name collisions and short writes are injected. */
static struct {
  bool present, mounted, short_write;
  unsigned open_files, flushes;
  char directory[32];
  char csv[2][2048];
  size_t size[2];
} log_disk;

void fd_init(struct dfs_file *file) { file->stream = 0; }

int dfs_mount(const char *device, const char *path, const char *fs,
              unsigned long flags, const void *data)
{
  CHECK(!strcmp(device, "sd") && !strcmp(path, "/") && !strcmp(fs, "elm"));
  CHECK(flags == 0 && data == NULL && !log_disk.mounted);
  if (!log_disk.present) return -ENODEV;
  log_disk.mounted = true;
  return 0;
}

int dfs_unmount(const char *path)
{
  CHECK(!strcmp(path, "/") && log_disk.mounted && log_disk.open_files == 0);
  log_disk.mounted = false;
  return 0;
}

int dfs_file_open(struct dfs_file *file, const char *path, int flags)
{
  CHECK(log_disk.mounted);
  if (flags & O_DIRECTORY) {
    if (!strcmp(path, "/LOG") || !strcmp(path, "/LOG/F000abcd"))
      return -EEXIST;
    (void)snprintf(log_disk.directory, sizeof(log_disk.directory), "%s", path);
  } else {
    const size_t prefix = strlen(log_disk.directory);
    CHECK(!strncmp(path, log_disk.directory, prefix));
    CHECK((flags & (O_EXCL | O_CREAT | O_WRONLY)) == (O_EXCL | O_CREAT | O_WRONLY));
    file->stream = !strcmp(path + prefix, "/STATE.CSV") ? 1u : 2u;
    CHECK(file->stream == 1u || !strcmp(path + prefix, "/EVENTS.CSV"));
    log_disk.size[file->stream - 1] = 0;
  }
  ++log_disk.open_files;
  return 0;
}

int dfs_file_close(struct dfs_file *file)
{
  CHECK(log_disk.open_files > 0);
  --log_disk.open_files;
  file->stream = 0;
  return 0;
}

ssize_t dfs_file_write(struct dfs_file *file, const void *data, size_t length)
{
  if (log_disk.short_write)
    return (ssize_t)length - 1;
  const unsigned stream = file->stream - 1;
  CHECK(stream < 2);
  if (stream >= 2 || length >= sizeof(log_disk.csv[stream]) - log_disk.size[stream])
    return -ENOSPC;
  memcpy(log_disk.csv[stream] + log_disk.size[stream], data, length);
  log_disk.size[stream] += length;
  log_disk.csv[stream][log_disk.size[stream]] = '\0';
  return (ssize_t)length;
}

int dfs_file_flush(struct dfs_file *file)
{
  CHECK(file->stream == 1 || file->stream == 2);
  ++log_disk.flushes;
  return 0;
}

static unsigned csv_columns(const char *line)
{
  unsigned columns = 1;
  for (; *line && *line != '\n'; ++line)
    columns += *line == ',';
  return columns;
}

static void test_flight_logger(void)
{
  LogSession session = {.next_directory = 0xabcd};
  test_clock.monotonic_us = 1000000;
  CHECK(runtime_data_init(test_clock.monotonic_us) == RT_EOK);
  CHECK(!log_open(&session) && !session.mounted);
  CHECK(log_scaled(NAN, 1000) == INT32_MIN);

  log_disk.present = true;
  CHECK(log_open(&session));
  CHECK(!strcmp(log_disk.directory, "/LOG/F000abce"));
  CHECK(log_disk.flushes == 2 && log_disk.open_files == 2);
  runtime_data.log_imu.header.timestamp_sample_us = 999000;
  runtime_data.log_imu.sample_sequence = 7;
  runtime_data.log_imu.quality_flags = IMU_Q_GYRO_VALID;
  runtime_data.log_imu.gyro_frd_radps[0] = -1.5f;
  runtime_data_record_event(SOURCE_M4, 0, 19, 2, test_clock.monotonic_us);
  CHECK(log_state(&session, &test_clock) && log_events(&session));
  const char *row = strchr(log_disk.csv[0], '\n');
  CHECK(row && strstr(row + 1, "0000000012345678,1.000000,0.999000,7,1,-1500,"));
  CHECK(row && csv_columns(log_disk.csv[0]) == csv_columns(row + 1));
  row = strchr(log_disk.csv[1], '\n');
  CHECK(row && strstr(row + 1, "0000000012345678,1.000000,1,"));
  CHECK(row && csv_columns(log_disk.csv[1]) == csv_columns(row + 1));

  RuntimeEvent telemetry_event;
  CHECK(rt_mq_recv(&event_queue, &telemetry_event, sizeof(telemetry_event), 0) ==
        (rt_ssize_t)sizeof(telemetry_event));
  CHECK(telemetry_event.event_id == 1);
  for (unsigned i = 0; i < LOG_EVENT_CAPACITY + 1; ++i)
    runtime_data_record_event(SOURCE_M4, 0, 19, 2, test_clock.monotonic_us);
  CHECK(runtime_stats.log_event_drops == 1 && runtime_stats.event_drops == 0);

  log_disk.short_write = true;
  CHECK(!log_state(&session, &test_clock));
  CHECK(!log_events(&session) && runtime_stats.log_event_drops == 2);
  log_close(&session);
  CHECK(!session.mounted && !session.state_open && !session.events_open);
  log_disk.short_write = false;
  CHECK(log_open(&session) && !strcmp(log_disk.directory, "/LOG/F000abcf"));
  log_close(&session);
  runtime_data_deinit();
}

static uint8_t nor_image[JOURNAL_END - JOURNAL_BEGIN];
static unsigned nor_erases;
static bool nor_torn_commit;

rt_ssize_t rt_mtd_nor_read(struct rt_mtd_nor_device *device, rt_off_t offset, uint8_t *data,
                          rt_size_t length)
{
  (void)device;
  if (offset < JOURNAL_BEGIN || (uint32_t)offset > JOURNAL_END ||
      length > JOURNAL_END - (uint32_t)offset)
    return -RT_EINVAL;
  memcpy(data, nor_image + offset - JOURNAL_BEGIN, length);
  return length;
}

rt_ssize_t rt_mtd_nor_write(struct rt_mtd_nor_device *device, rt_off_t offset, const uint8_t *data,
                           rt_size_t length)
{
  (void)device;
  if (offset < JOURNAL_BEGIN || (uint32_t)offset > JOURNAL_END ||
      length > JOURNAL_END - (uint32_t)offset || length > 256u - ((uint32_t)offset % 256u))
    return -RT_EINVAL;
  const bool tear = nor_torn_commit && length == 4;
  uint8_t *dst = nor_image + offset - JOURNAL_BEGIN;
  for (rt_size_t i = 0; i < length - (tear ? 2u : 0u); ++i)
    dst[i] &= data[i];
  return tear ? -RT_ERROR : (rt_ssize_t)length;
}

rt_err_t rt_mtd_nor_erase_block(struct rt_mtd_nor_device *device, rt_off_t offset, rt_size_t length)
{
  (void)device;
  if (offset < JOURNAL_BEGIN || offset > JOURNAL_END - JOURNAL_SECTOR_BYTES ||
      offset % JOURNAL_SECTOR_BYTES || length != JOURNAL_SECTOR_BYTES)
    return -RT_EINVAL;
  memset(nor_image + offset - JOURNAL_BEGIN, 0xff, length);
  ++nor_erases;
  return RT_EOK;
}

static void test_event_journal(void)
{
  struct rt_mtd_nor_device device = {
    .parent.type = RT_Device_Class_MTD,
    .block_size = JOURNAL_SECTOR_BYTES,
    .block_end = JOURNAL_END / JOURNAL_SECTOR_BYTES,
  };
  RuntimeEvent event = {.boot_id = 0x1234, .time_epoch = 1, .source_id = SOURCE_PLATFORM};
  EventJournal journal;
  memset(nor_image, 0xff, sizeof(nor_image));
  /* A sector with an erased header and dirty payload belongs to no known journal. */
  nor_image[JOURNAL_SLOT_BYTES] = 0x12;
  inline_delays = true;
  CHECK(!event_journal_open(&journal));
  test_nor_device = &device;
  CHECK(event_journal_open(&journal) && journal.generation == 0);
  CHECK(store_event(&journal, &event, true));
  CHECK(journal.sector == JOURNAL_BEGIN + JOURNAL_SECTOR_BYTES && nor_erases == 0);
  const uint8_t *boot = nor_image + JOURNAL_SECTOR_BYTES + JOURNAL_SLOT_BYTES;
  CHECK(!memcmp(boot, "FEVT\1\0\1", 7) && boot[8] == 0x34 && boot[55] == 4);
  CHECK(!memcmp(boot + 60, "MMOC", 4));

  event.event_id = 1;
  nor_torn_commit = true;
  CHECK(!store_event(&journal, &event, false));
  nor_torn_commit = false;
  CHECK(event_journal_open(&journal) && journal.next_offset == 3 * JOURNAL_SLOT_BYTES);
  CHECK(boot[2 * JOURNAL_SLOT_BYTES - 1] == 0xff);
  const unsigned sectors = sizeof(nor_image) / JOURNAL_SECTOR_BYTES;
  const unsigned records = JOURNAL_SECTOR_BYTES / JOURNAL_SLOT_BYTES - 1;
  bool filled = true;
  for (unsigned i = 0; i < (sectors - 1) * records - 2; ++i)
    filled = filled && store_event(&journal, &event, false);
  CHECK(filled && journal.generation == sectors - 1 && nor_erases == 0);
  CHECK(store_event(&journal, &event, false));
  CHECK(journal.generation == sectors && nor_erases == 1);
  CHECK(journal.sector == JOURNAL_BEGIN + JOURNAL_SECTOR_BYTES);
  CHECK(nor_image[JOURNAL_SLOT_BYTES] == 0x12);
  CHECK(event_journal_open(&journal) && journal.generation == sectors &&
        journal.next_offset == 2 * JOURNAL_SLOT_BYTES);
  test_nor_device = NULL;
  inline_delays = false;
}

int main(void) {
  test_pipeline();test_calibration_service();test_actuator();test_native();test_wire();
  test_telemetry_hil_handoff();test_flight_logger();test_event_journal();
  printf("firmware_runtime: %u checks, %u failures\n",checks,failures);return failures?1:0;
}
