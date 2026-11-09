/* UM982 remains fitted: exercise runtime PVT/PPS/UTC loss and recovery without
 * deinitializing the transport or resetting local time. No HAL/RTOS required. */
#include "systime.h"
#include "sample_normalization/sensor_sample.h"
#include <stdio.h>
#include <string.h>

#define HZ 10000000u
#define UTC_BASE UINT64_C(1700000000000000000)
static unsigned checks, failures;
static uint64_t raw;
#define CHECK(x) do { checks++; if (!(x)) { failures++; printf("FAIL %u: %s\n",(unsigned)__LINE__,#x); } } while(0)
static Status tick(uint64_t *value,uint32_t *epoch) { *value=raw; *epoch=1u; return STATUS_OK; }
static Status edge(uint64_t value,uint32_t seq,uint16_t flags)
{
    PpsCaptureEvent p={0};
    raw=value; p.capture_tick64=value; p.time_epoch=1u; p.edge_sequence=seq; p.capture_flags=flags;
    return time_submit_pps(&p);
}
static Status utc(unsigned seconds,unsigned sequence)
{
    GnssTimeObservation o={0};
    o.utc_time_ns=UTC_BASE+(uint64_t)seconds*1000000000u;
    o.paired_pps_tick64=(uint64_t)seconds*HZ; o.pps_edge_sequence=sequence;
    o.time_epoch=1u; o.receiver_accuracy_ns=50000u; o.time_scale=TIME_SCALE_UTC;
    o.validity_flags=GNSS_TIME_UTC_VALID;
    return time_submit_gnss_time(&o);
}
static TimeServiceStatus status(void)
{
    TimeServiceStatus s;
    CHECK(time_get_status(&s)==STATUS_OK);
    CHECK(s.local_clock_valid && s.time_epoch==1u);
    return s;
}
static LocalTimebaseDisciplineStatus discipline(void)
{
    LocalTimebaseDisciplineStatus s;
    CHECK(time_get_local_discipline_status(&s)==STATUS_OK);
    CHECK(s.valid && s.time_epoch==1u);
    return s;
}
int main(void)
{
    TimeConfig config={0};
    ClockInstant instant;
    TimeServiceStatus s;
    LocalTimebaseDisciplineStatus d;
    sensor_gnss_config_t gc={0}; sensor_context_t context={0};
    sensor_gnss_state_t state={0}; sensor_gnss_pvt_t pvt={0}; GnssSample sample;
    unsigned i; uint64_t previous, last_valid; uint32_t mapping;
    config.boot_id=0x1234u; config.time_epoch=1u; config.tick_hz=HZ;
    config.gnss_pps_installed=1u; config.holdover_rate_bound_ppb=10000u;
    CHECK(systime_init(&config,tick)==STATUS_OK);
    raw=HZ/10u; CHECK(clock_read(&instant)==STATUS_OK);
    CHECK(instant.boot_id==config.boot_id && instant.monotonic_us>0u);
    s=status(); CHECK(!s.utc_valid && s.state==TIME_QUALITY_LOCAL_ONLY);

    for(i=1;i<=9;i++) CHECK(edge((uint64_t)i*HZ,i,TIMER_CAPTURE_VALID)==STATUS_OK);
    CHECK(utc(9,9)==STATUS_OK); s=status(); CHECK(s.state==TIME_QUALITY_LOCKED);
    mapping=s.utc_mapping_epoch;

    /* Receiving a packet/no Fix is not hardware removal and is not a valid
     * horizontal observation. Old measurement epochs remain stale. */
    gc.max_speed_mps=30.0; gc.max_altitude_m=20000.0; gc.max_variance_m2=100.0; gc.stale_age_us=20000u;
    context.boot_id=config.boot_id; context.time_epoch=1u; context.sensor_instance_id=11u;
    pvt.latitude_deg=31.0; pvt.longitude_deg=121.0; pvt.altitude_m=10.0; pvt.satellites=20u;
    pvt.fix_type=GNSS_FIX_RTK_FIXED;
    pvt.valid_flags=SENSOR_GNSS_PVT_POSITION_VALID|SENSOR_GNSS_PVT_VELOCITY_VALID|
                    SENSOR_GNSS_PVT_POSITION_COV_VALID|SENSOR_GNSS_PVT_VELOCITY_COV_VALID;
    for(i=0;i<3;i++) { pvt.position_variance_m2[i]=1.0f; pvt.velocity_variance_m2ps2[i]=0.1f; }
    CHECK(sensor_gnss_build_sample(&gc,&context,&state,&pvt,9000000u,9005000u,mapping,&sample)==SENSOR_OK);
    CHECK(sample.quality_flags&GNSS_Q_POSITION_VALID);
    pvt.fix_type=GNSS_FIX_NONE; pvt.valid_flags=0u;
    CHECK(sensor_gnss_build_sample(&gc,&context,&state,&pvt,9100000u,9105000u,mapping,&sample)==SENSOR_OK);
    CHECK(!(sample.quality_flags&(GNSS_Q_POSITION_VALID|GNSS_Q_VELOCITY_VALID)));
    CHECK(sensor_gnss_build_sample(&gc,&context,&state,&pvt,9000000u,9200000u,mapping,&sample)==SENSOR_ERR_STALE);
    /* Navigation unavailable while PPS/UTC remain healthy. */
    CHECK(edge(10ull*HZ,10u,TIMER_CAPTURE_VALID)==STATUS_OK);
    CHECK(utc(10,10)==STATUS_OK); s=status(); CHECK(s.state==TIME_QUALITY_LOCKED);

    /* UTC message loss with PPS still present must not masquerade as LOCKED. */
    for(i=11;i<=13;i++) CHECK(edge((uint64_t)i*HZ,i,TIMER_CAPTURE_VALID)==STATUS_OK);
    s=status(); d=discipline(); CHECK(s.state==TIME_QUALITY_HOLDOVER);
    CHECK(d.discipline_state==LOCAL_TIMEBASE_LOCKED);
    CHECK(utc(13,13)==STATUS_OK); s=status(); CHECK(s.state==TIME_QUALITY_LOCKED);
    last_valid=s.last_valid_pps_us; previous=s.monotonic_now_us;

    /* PPS loss: local monotonic and the session/continuity epoch survive. */
    raw=16ull*HZ; s=status(); d=discipline();
    CHECK(s.state==TIME_QUALITY_HOLDOVER && d.discipline_state==LOCAL_TIMEBASE_HOLDOVER);
    CHECK(s.monotonic_now_us>previous && s.utc_mapping_epoch==mapping);
    CHECK(clock_read(&instant)==STATUS_OK && instant.boot_id==config.boot_id);
    /* First returning edge has a long interval; it only seeds a new window. */
    CHECK(edge(17ull*HZ,14u,TIMER_CAPTURE_VALID)==STATUS_OUT_OF_RANGE);
    s=status(); CHECK(s.last_valid_pps_us==last_valid && s.state!=TIME_QUALITY_LOCKED);
    for(i=18;i<=25;i++) {
        CHECK(edge((uint64_t)i*HZ,i-3u,TIMER_CAPTURE_VALID)==STATUS_OK);
        d=discipline();
        CHECK((d.discipline_state==LOCAL_TIMEBASE_LOCKED)==(i==25u));
    }
    s=status(); CHECK(s.state==TIME_QUALITY_HOLDOVER); /* UTC has not returned yet */
    CHECK(utc(25,22)==STATUS_OK); s=status();
    CHECK(s.state==TIME_QUALITY_LOCKED && s.utc_mapping_epoch==mapping);
    last_valid=s.last_valid_pps_us;

    /* Repeated bad pulses do not refresh last valid PPS or retain LOCKED. */
    for(i=1;i<=3;i++) {
        CHECK(edge(25ull*HZ+(uint64_t)i*(HZ+20000u),22u+i,TIMER_CAPTURE_VALID)==STATUS_OUT_OF_RANGE);
        s=status(); CHECK(s.last_valid_pps_us==last_valid && s.state!=TIME_QUALITY_LOCKED);
    }
    d=discipline(); CHECK(d.discipline_state==LOCAL_TIMEBASE_HOLDOVER);
    CHECK(edge(raw,25u,TIMER_CAPTURE_VALID)==STATUS_STALE_DATA);
    CHECK(edge(raw+HZ,26u,TIMER_CAPTURE_OVERCAPTURE)==STATUS_OUT_OF_RANGE);
    s=status(); CHECK(s.last_valid_pps_us==last_valid && s.local_clock_valid);
    CHECK(systime_stop()==STATUS_OK);
    printf("GNSS runtime loss/recovery: %u checks, %u failures\n",checks,failures);
    return failures ? 1 : 0;
}
