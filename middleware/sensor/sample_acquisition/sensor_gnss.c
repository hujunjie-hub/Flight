#include "sample_acquisition/sensor_input.h"
#include "um982.h"
#include "unicore_ascii/unicore_ascii.h"
#include "systime.h"
#include <math.h>
#include <string.h>

typedef struct
{
    uint64_t mono_us;       /* mapped FCU monotonic epoch */
    uint32_t mapping_epoch; /* mapping used; 0 when unavailable */
    uint64_t uncertainty_ns;
} GnssTimeMap;

static Status gnss_time_map_from_status(const ClockDomainMappingStatus *status, uint64_t source_time_ns,
                               GnssTimeMap *out)
{
    if ((status == NULL) || (out == NULL))
    {
        return SENSOR_ERR_ARGUMENT;
    }
    if (status->mapping_state != CLOCK_MAPPING_LOCKED &&
        status->mapping_state != CLOCK_MAPPING_HOLDOVER)
    {
        return SENSOR_ERR_NOT_CONFIGURED;
    }
    if (status->mapping_epoch == 0u)
    {
        return SENSOR_ERR_NOT_CONFIGURED;
    }

    const int64_t delta_ns = (int64_t)source_time_ns - (int64_t)status->anchor_source_time_ns;
    const double scaled_us =
        (double)delta_ns * 1.0e-3 * (1.0 + ((double)status->source_rate_error_ppb * 1.0e-9));
    const double mono = (double)status->anchor_local_time_us + scaled_us;
    if (!(mono >= 0.0) || !isfinite(mono))
    {
        return SENSOR_ERR_RANGE;
    }

    out->mono_us = (uint64_t)(mono + 0.5);
    out->mapping_epoch = status->mapping_epoch;
    out->uncertainty_ns = status->uncertainty_ns;
    return SENSOR_OK;
}

static Status gnss_time_map_source(uint32_t source_clock_domain_id, uint64_t source_time_ns,
                          GnssTimeMap *out)
{
    ClockDomainMappingStatus status;
    memset(&status, 0, sizeof(status));
    if (time_get_mapping_status(source_clock_domain_id, &status) != 0)
    {
        return SENSOR_ERR_NOT_CONFIGURED;
    }
    {
        ClockInstant now;
        if (clock_read(&now) != STATUS_OK || status.local_time_epoch != now.time_epoch ||
            status.valid_until_local_us <= now.monotonic_us)
        {
            return SENSOR_ERR_NOT_CONFIGURED;
        }
    }
    return gnss_time_map_from_status(&status, source_time_ns, out);
}

static UnicoreAsciiParser gnss_parser;
static UnicoreAsciiMessage gnss_message;
static sensor_gnss_state_t gnss_state;
static sensor_context_t gnss_context;
static uint64_t last_receiver_time, mapping_reference_time;
static int64_t gps_to_utc_ns;
static uint32_t gnss_unmapped, gnss_rejected;
static uint8_t gnss_logs_requested;
static uint64_t next_log_request;
static rt_device_t gnss;
#if defined(BSP_USING_FLIGHT_DRIVERS) && !defined(FLIGHT_SIM_LINK)
static uint8_t rtcm_pending[1029];
static size_t rtcm_offset, rtcm_length;
static unsigned rtcm_sent, rtcm_dropped;

void sensor_gnss_queue_rtcm(const uint8_t *data, size_t length)
{
  if (!gnss || !gnss_logs_requested || rtcm_length != 0u || length > sizeof(rtcm_pending)) {
    ++rtcm_dropped;
    return;
  }
  const rt_ssize_t written = rt_device_write(gnss, 0, data, length);
  if (written == (rt_ssize_t)length) {
    ++rtcm_sent;
  } else {
    rtcm_offset = 0u;
    rtcm_length = length - (written > 0 ? (size_t)written : 0u);
    memcpy(rtcm_pending, data + length - rtcm_length, rtcm_length);
  }
}
#endif
static GnssSample latest_fix;
static double latest_altitude_msl_m;
#ifdef FLIGHT_LOGGER_ENABLE
static GnssLogStatus log_status;

void sensor_gnss_log_status(GnssLogStatus *out)
{
    rt_base_t level = rt_hw_interrupt_disable();
    *out = log_status;
    out->unmapped = gnss_unmapped;
    out->rejected = gnss_rejected;
    rt_hw_interrupt_enable(level);
}
#endif

bool sensor_gnss_latest(uint64_t now, GnssSample *sample, double *altitude_msl_m)
{
    /* Display age is independent of the estimator's tighter admission TTL. */
    if (!latest_fix.header.sequence || now < latest_fix.header.timestamp_sample_us ||
        now - latest_fix.header.timestamp_sample_us >= 500000u)
        return false;
    *sample = latest_fix;
    *altitude_msl_m = latest_altitude_msl_m;
    return true;
}

static void process_gnss_log(const UnicoreAsciiMessage *m, uint64_t now, SensorMeasurementSink submit_measurement) {
  if(m->kind==UNICORE_ASCII_RECTIME) {
    PpsCaptureEvent pps; uint64_t mono;
    GnssTimeObservation observation;
    if(time_get_latest_pps(&pps) || clock_ticks_to_mono_us(pps.capture_tick64,pps.time_epoch,&mono) ||
       now<mono || now-mono>200000 || m->output_delay_ms>150 ||
       now-mono+10000<(uint64_t)m->output_delay_ms*1000) { gnss_unmapped++; return; }
    memset(&observation,0,sizeof(observation));
    observation.utc_time_ns=m->utc_time_ns; observation.paired_pps_tick64=pps.capture_tick64;
    observation.pps_edge_sequence=pps.edge_sequence; observation.time_epoch=pps.time_epoch;
    observation.receiver_accuracy_ns=m->receiver_accuracy_ns;
    observation.time_scale=TIME_SCALE_UTC;
    observation.validity_flags=GNSS_TIME_UTC_VALID|GNSS_TIME_PPS_PAIRED;
    if(time_submit_gnss_time(&observation)) { gnss_unmapped++; return; }
    gps_to_utc_ns=(int64_t)m->utc_time_ns-(int64_t)m->gps_time_ns;
    mapping_reference_time=now; return;
  }
  if(m->kind==UNICORE_ASCII_BESTNAV) {
    SensorMeasurement measurement; GnssTimeMap map; ClockInstant clock;
    static const sensor_gnss_config_t cfg={100,100000,10000,20000};
    const UnicorePvt *decoded = &m->pvt;
#ifdef FLIGHT_LOGGER_ENABLE
    rt_base_t level = rt_hw_interrupt_disable();
    log_status.arrival_us = now;
    log_status.flags = decoded->valid_flags;
    log_status.fix_type = decoded->fix_type;
    log_status.satellites = decoded->satellites;
    rt_hw_interrupt_enable(level);
#endif
    /* Receiver status is for display only. The estimator still requires
     * the timestamp mapping and validated observations below. */
    latest_fix.header.sequence = 1;
    latest_fix.header.timestamp_sample_us = now;
    latest_fix.latitude_deg = decoded->latitude_deg;
    latest_fix.longitude_deg = decoded->longitude_deg;
    latest_fix.altitude_m = (float)decoded->altitude_m;
    latest_altitude_msl_m = decoded->altitude_msl_m;
    latest_fix.fix_type = decoded->fix_type;
    latest_fix.satellite_count = decoded->satellites;
    latest_fix.quality_flags =
      (decoded->valid_flags & UNICORE_PVT_POSITION_VALID ? GNSS_Q_POSITION_VALID : 0u) |
      (decoded->valid_flags & UNICORE_PVT_VELOCITY_VALID ? GNSS_Q_VELOCITY_VALID : 0u);
    memcpy(latest_fix.velocity_ned_mps, decoded->velocity_ned_mps, sizeof(latest_fix.velocity_ned_mps));
    if(!mapping_reference_time || now-mapping_reference_time>2500000 ||
       m->gps_time_ns<=last_receiver_time ||
       gnss_time_map_source(CLOCK_DOMAIN_UTC_POSIX,(uint64_t)((int64_t)m->gps_time_ns+gps_to_utc_ns),&map) ||
       map.uncertainty_ns>100000 || clock_read(&clock)) { gnss_unmapped++; return; }
    sensor_gnss_pvt_t pvt = {
      .latitude_deg = decoded->latitude_deg,
      .longitude_deg = decoded->longitude_deg,
      .altitude_m = decoded->altitude_m,
      .fix_type = decoded->fix_type,
      .satellites = decoded->satellites,
      .valid_flags =
          ((decoded->valid_flags & UNICORE_PVT_POSITION_VALID) ? SENSOR_GNSS_PVT_POSITION_VALID : 0u) |
          ((decoded->valid_flags & UNICORE_PVT_VELOCITY_VALID) ? SENSOR_GNSS_PVT_VELOCITY_VALID : 0u) |
          ((decoded->valid_flags & UNICORE_PVT_POSITION_COV_VALID) ? SENSOR_GNSS_PVT_POSITION_COV_VALID : 0u) |
          ((decoded->valid_flags & UNICORE_PVT_VELOCITY_COV_VALID) ? SENSOR_GNSS_PVT_VELOCITY_COV_VALID : 0u),
    };
    memcpy(pvt.velocity_ned_mps, decoded->velocity_ned_mps, sizeof(pvt.velocity_ned_mps));
    memcpy(pvt.position_variance_m2, decoded->position_variance_m2, sizeof(pvt.position_variance_m2));
    memcpy(pvt.velocity_variance_m2ps2, decoded->velocity_variance_m2ps2, sizeof(pvt.velocity_variance_m2ps2));
    gnss_context.boot_id=clock.boot_id; gnss_context.time_epoch=clock.time_epoch;
    gnss_context.profile_id=PRODUCT_PROFILE_ID; gnss_context.sensor_instance_id=4;
    memset(&measurement,0,sizeof(measurement));
    if(sensor_gnss_build_sample(&cfg,&gnss_context,&gnss_state,&pvt,map.mono_us,now,map.mapping_epoch,
                          &measurement.payload.gnss)) { gnss_rejected++; return; }
    measurement.payload.gnss.calibration_version=1;
    measurement.payload.gnss.header.source_clock_domain_id=CLOCK_DOMAIN_UTC_POSIX;
    measurement.payload.gnss.header.timestamp_flags|=TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
    measurement.payload.gnss.header.event_time_uncertainty_us=(uint32_t)((map.uncertainty_ns+999)/1000);
    measurement.header=measurement.payload.gnss.header; measurement.sensor_instance_id=4;
    measurement.measurement_kind=PRODUCT_MEASUREMENT_GNSS;
    if(submit_measurement(&measurement)) {
      gnss_rejected++;
    } else {
      last_receiver_time=m->gps_time_ns;
    }
  }
}
void sensor_gnss_poll(uint64_t now, SensorMeasurementSink submit_measurement) {
  uint8_t bytes[256]; rt_ssize_t n; unsigned i;
  if (!gnss) gnss = um982_open();
  if(gnss && !gnss_logs_requested && now >= next_log_request) {
    /* No reboot, FLASH save or protocol-dependent board reset. Receipt of
     * valid native log frames is the configuration confirmation. */
    next_log_request = now + 1000000u;
    gnss_logs_requested = um982_request_navigation_logs(gnss);
  }
  if(gnss) {
#if defined(BSP_USING_FLIGHT_DRIVERS) && !defined(FLIGHT_SIM_LINK)
    if (rtcm_length != 0u) {
      const rt_ssize_t written = rt_device_write(gnss, 0, rtcm_pending + rtcm_offset,
                                                rtcm_length - rtcm_offset);
      if (written > 0) rtcm_offset += (size_t)written;
      if (rtcm_offset == rtcm_length) {
        rtcm_length = 0u;
        ++rtcm_sent;
      }
    }
#endif
    /* At most 1024 bytes per wake; damaged streams cannot monopolize CPU. */
    for(i=0;i<4;++i) {
      unsigned j; n=rt_device_read(gnss,0,bytes,sizeof(bytes)); if(n<=0) break;
      for(j=0;j<(unsigned)n;++j) if(unicore_ascii_feed(&gnss_parser,bytes[j],&gnss_message)==1) {
        uint64_t arrival=now; (void)clock_now_us(&arrival);
        process_gnss_log(&gnss_message, arrival, submit_measurement);
      }
    }
  }
}

void sensor_gnss_print_diagnostics(void) {
  rt_kprintf("GNSS frames=%u crc=%u malformed=%u overflow=%u unmapped=%u rejected=%u\n",
    gnss_parser.valid_frames,gnss_parser.crc_errors,gnss_parser.malformed_frames,
    gnss_parser.overflows,gnss_unmapped,gnss_rejected);
#if defined(BSP_USING_FLIGHT_DRIVERS) && !defined(FLIGHT_SIM_LINK)
  rt_kprintf("GNSS RTCM queued to UART=%u dropped=%u\n", rtcm_sent, rtcm_dropped);
#endif
}
