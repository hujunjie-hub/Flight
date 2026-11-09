/* Low-priority flight recorder. This task alone calls the SD filesystem. */
#define DBG_TAG "flight.log"
#define DBG_LVL DBG_INFO

#include "task_config.h"
#include "runtime_internal.h"
#include "sample_acquisition/sensor_input.h"
#include <dfs_file.h>
#include <errno.h>
#include <rtdbg.h>

#define LOG_SD_DEVICE "sd"
#define LOG_ROOT "/LOG"
#define LOG_TIME_FORMAT "%u.%06u"
#define LOG_TIME_ARGS(us) (unsigned)((us) / 1000000u), (unsigned)((us) % 1000000u)
#define LOG_BOOT_FORMAT "%08x%08x"
#define LOG_BOOT_ARGS(id) (unsigned)((id) >> 32), (unsigned)(id)

enum {
    LOG_RETRY_MS = 2000,
    LOG_FLUSH_US = 1000000,
    LOG_EVENTS_PER_CYCLE = 8,
    LOG_DIRECTORY_ATTEMPTS = 16,
    LOG_SESSION_BYTES = 64 * 1024 * 1024
};

static const char state_header[] =
    "boot_id_hex,record_time_s,imu_time_s,imu_sequence,imu_quality,"
    "gyro_x_mrad_s,gyro_y_mrad_s,gyro_z_mrad_s,accel_x_mm_s2,accel_y_mm_s2,accel_z_mm_s2,"
    "nav_time_s,nav_valid_mask,state_epoch,north_mm,east_mm,down_mm,vn_mm_s,ve_mm_s,vd_mm_s,"
    "qw_1e6,qx_1e6,qy_1e6,qz_1e6,fms_time_s,arm,mode,authority,failsafe,mission,mission_id,"
    "demand_time_s,demand_mask,control_sequence,demand_0_mN,demand_1_mN,demand_2_mN,demand_3_mN,"
    "commit_time_s,commit_result,applied_sequence,committed_0_mN,committed_1_mN,"
    "committed_2_mN,committed_3_mN,event_drops,hil_build,"
    "time_epoch,pps_state,pps_rate_ppb,utc_valid,gnss_arrival_s,gnss_fix,gnss_sats,"
    "gnss_flags,gnss_unmapped,gnss_rejected\n";
static const char event_header[] =
    "boot_id_hex,event_time_s,event_id,source_id,reason_namespace,reason_code,severity,"
    "authority_epoch,directive_epoch,permit_epoch,time_epoch\n";

typedef struct {
    struct dfs_file state;
    struct dfs_file events;
    uint32_t next_directory;
    uint32_t bytes;
    bool mounted;
    bool state_open;
    bool events_open;
} LogSession;

static bool log_write(LogSession *session, struct dfs_file *file, const char *data, size_t length)
{
    if (dfs_file_write(file, data, length) != (ssize_t)length)
        return false;
    session->bytes += (uint32_t)length;
    return true;
}

static void log_close(LogSession *session)
{
    if (session->events_open)
    {
        (void)dfs_file_close(&session->events);
        session->events_open = false;
    }
    if (session->state_open)
    {
        (void)dfs_file_close(&session->state);
        session->state_open = false;
    }
    if (session->mounted && dfs_unmount("/") == 0)
        session->mounted = false;
}

static int log_create_directory(const char *path)
{
    struct dfs_file directory;
    fd_init(&directory);
    const int result = dfs_file_open(&directory, path, O_DIRECTORY | O_CREAT);
    if (result < 0)
        return result;
    return dfs_file_close(&directory);
}

static bool log_open(LogSession *session)
{
    char directory[sizeof(LOG_ROOT "/F1234567")];
    char path[sizeof(LOG_ROOT "/F1234567/EVENTS.CSV")];

    if (session->mounted || dfs_mount(LOG_SD_DEVICE, "/", "elm", 0, RT_NULL) != 0)
        return false;
    session->mounted = true;
    session->bytes = 0;
    const int root_result = log_create_directory(LOG_ROOT);
    if (root_result != 0 && root_result != -EEXIST)
        return false;

    for (unsigned attempt = 0; attempt < LOG_DIRECTORY_ATTEMPTS; ++attempt)
    {
        const unsigned id = (unsigned)(session->next_directory++ & 0x0fffffffu);
        (void)rt_snprintf(directory, sizeof(directory), LOG_ROOT "/F%07x", id);
        const int result = log_create_directory(directory);
        if (result == -EEXIST)
            continue;
        if (result != 0)
            return false;

        fd_init(&session->state);
        (void)rt_snprintf(path, sizeof(path), "%s/STATE.CSV", directory);
        if (dfs_file_open(&session->state, path, O_WRONLY | O_CREAT | O_EXCL) < 0)
            return false;
        session->state_open = true;

        fd_init(&session->events);
        (void)rt_snprintf(path, sizeof(path), "%s/EVENTS.CSV", directory);
        if (dfs_file_open(&session->events, path, O_WRONLY | O_CREAT | O_EXCL) < 0)
            return false;
        session->events_open = true;

        if (!log_write(session, &session->state, state_header, sizeof(state_header) - 1u) ||
            !log_write(session, &session->events, event_header, sizeof(event_header) - 1u) ||
            dfs_file_flush(&session->state) != 0 || dfs_file_flush(&session->events) != 0)
            return false;
        LOG_I("SD recording in %s", directory);
        return true;
    }
    return false;
}

static int log_scaled(float value, float scale)
{
    const float scaled = value * scale;
    return scaled > -2147483648.0f && scaled < 2147483648.0f ? (int)scaled : INT32_MIN;
}

static bool log_state(LogSession *session, const ClockInstant *clock)
{
    ImuSample imu;
    NavigationSnapshot nav;
    FmsStateSnapshot fms;
    ActuatorDemand demand;
    ActuatorCommitReceipt receipt;
    uint32_t event_drops;
    if (!runtime_data_lock(0))
        return true;

    imu = runtime_data.log_imu;
    nav = runtime_data.nav;
    fms = runtime_data.fms;
    demand = runtime_data.demand;
    receipt = runtime_data.receipt;
    event_drops = runtime_stats.log_event_drops;
    runtime_data_unlock();

    LocalTimebaseDisciplineStatus discipline = {0};
    TimeServiceStatus time = {0};
    GnssLogStatus gnss;
    if (time_get_local_discipline_status(&discipline) != STATUS_OK)
        discipline.discipline_state = LOCAL_TIMEBASE_INVALID;
    if (time_get_status(&time) != STATUS_OK)
        time.utc_valid = 0u;
    sensor_gnss_log_status(&gnss);

#ifdef FLIGHT_SIM_LINK
    const unsigned hil = 1;
#else
    const unsigned hil = 0;
#endif
    const float *gyro = imu.gyro_frd_radps;
    const float *accel = imu.accel_frd_mps2;
    const float *position = nav.core.position_ned_m;
    const float *velocity = nav.core.velocity_ned_mps;
    const float *quaternion = nav.core.q_nb_wxyz;
    const float *requested = demand.demand.motor_thrust_n;
    const float *committed = receipt.committed_demand.motor_thrust_n;
    char row[768];
    const int length = rt_snprintf(row, sizeof(row),
        LOG_BOOT_FORMAT "," LOG_TIME_FORMAT "," LOG_TIME_FORMAT ",%u,%u,%d,%d,%d,%d,%d,%d,"
        LOG_TIME_FORMAT ",%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
        LOG_TIME_FORMAT ",%u,%u,%u,%u,%u,%u,"
        LOG_TIME_FORMAT ",%u,%u,%d,%d,%d,%d,"
        LOG_TIME_FORMAT ",%u,%u,%d,%d,%d,%d,%u,%u,"
        "%u,%u,%d,%u," LOG_TIME_FORMAT ",%u,%u,%u,%u,%u\n",
        LOG_BOOT_ARGS(clock->boot_id), LOG_TIME_ARGS(clock->monotonic_us),
        LOG_TIME_ARGS(imu.header.timestamp_sample_us), (unsigned)imu.sample_sequence,
        (unsigned)imu.quality_flags,
        log_scaled(gyro[0], 1000), log_scaled(gyro[1], 1000), log_scaled(gyro[2], 1000),
        log_scaled(accel[0], 1000), log_scaled(accel[1], 1000), log_scaled(accel[2], 1000),
        LOG_TIME_ARGS(nav.header.timestamp_sample_us), (unsigned)nav.core.valid_mask,
        (unsigned)nav.state_epoch,
        log_scaled(position[0], 1000), log_scaled(position[1], 1000), log_scaled(position[2], 1000),
        log_scaled(velocity[0], 1000), log_scaled(velocity[1], 1000), log_scaled(velocity[2], 1000),
        log_scaled(quaternion[0], 1000000), log_scaled(quaternion[1], 1000000),
        log_scaled(quaternion[2], 1000000), log_scaled(quaternion[3], 1000000),
        LOG_TIME_ARGS(fms.header.timestamp_sample_us), (unsigned)fms.arm_state,
        (unsigned)fms.flight_mode, (unsigned)fms.authority, (unsigned)fms.failsafe_level,
        (unsigned)fms.mission_state, (unsigned)fms.active_mission_id,
        LOG_TIME_ARGS(demand.header.timestamp_sample_us), (unsigned)demand.valid_channel_mask,
        (unsigned)demand.control_sequence,
        log_scaled(requested[0], 1000), log_scaled(requested[1], 1000),
        log_scaled(requested[2], 1000), log_scaled(requested[3], 1000),
        LOG_TIME_ARGS(receipt.commit_time_us), (unsigned)receipt.result_code,
        (unsigned)receipt.applied_control_sequence,
        log_scaled(committed[0], 1000), log_scaled(committed[1], 1000),
        log_scaled(committed[2], 1000), log_scaled(committed[3], 1000), (unsigned)event_drops, hil,
        (unsigned)clock->time_epoch, (unsigned)discipline.discipline_state,
        (int)discipline.rate_correction_ppb, (unsigned)time.utc_valid,
        LOG_TIME_ARGS(gnss.arrival_us), (unsigned)gnss.fix_type, (unsigned)gnss.satellites,
        (unsigned)gnss.flags, (unsigned)gnss.unmapped, (unsigned)gnss.rejected);

    return length > 0 && (size_t)length < sizeof(row) &&
        log_write(session, &session->state, row, (size_t)length);
}

static bool log_events(LogSession *session)
{
    RuntimeEvent event;
    for (unsigned count = 0; count < LOG_EVENTS_PER_CYCLE && runtime_data_take_log_event(&event); ++count)
    {
        char row[160];
        const int length = rt_snprintf(row, sizeof(row),
            LOG_BOOT_FORMAT "," LOG_TIME_FORMAT ",%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
            LOG_BOOT_ARGS(event.boot_id), LOG_TIME_ARGS(event.time_us), (unsigned)event.event_id,
            (unsigned)event.source_id, (unsigned)event.reason_namespace, (unsigned)event.reason_code,
            (unsigned)event.severity, (unsigned)event.authority_epoch, (unsigned)event.directive_epoch,
            (unsigned)event.permit_epoch, (unsigned)event.time_epoch);
        if (length <= 0 || (size_t)length >= sizeof(row) ||
            !log_write(session, &session->events, row, (size_t)length))
        {
            rt_base_t level = rt_hw_interrupt_disable();
            runtime_stats.log_event_drops++;
            rt_hw_interrupt_enable(level);
            return false;
        }
    }
    return true;
}

static void task_logger_entry(void *parameter)
{
    LogSession session = {0};
    ClockInstant clock;
    bool storage_unavailable = false;
    (void)parameter;
    while (clock_read(&clock) != STATUS_OK)
    {
        if (!runtime_running())
            return;
        rt_thread_mdelay(TASK_LOGGER_PERIOD_MS);
    }
    session.next_directory = (uint32_t)clock.boot_id;

    while (runtime_running())
    {
        if (!log_open(&session))
        {
            if (!storage_unavailable)
                LOG_W("SD recording unavailable; retrying");
            storage_unavailable = true;
            log_close(&session);
            rt_thread_mdelay(LOG_RETRY_MS);
            continue;
        }
        storage_unavailable = false;
        runtime_stats.sd_recording = true;

        rt_tick_t last = rt_tick_get();
        uint64_t last_flush = runtime_now_us();
        bool write_ok = true;
        while (runtime_running() && session.bytes < LOG_SESSION_BYTES)
        {
            if (clock_read(&clock) == STATUS_OK)
            {
                write_ok = log_state(&session, &clock) && log_events(&session);
                if (write_ok && clock.monotonic_us - last_flush >= LOG_FLUSH_US)
                {
                    write_ok = dfs_file_flush(&session.state) == 0 && dfs_file_flush(&session.events) == 0;
                    last_flush = clock.monotonic_us;
                }
                if (!write_ok)
                    break;
            }
            rt_thread_delay_until(&last, rt_tick_from_millisecond(TASK_LOGGER_PERIOD_MS));
        }
        runtime_stats.sd_recording = false;
        log_close(&session);
        if (!write_ok)
        {
            runtime_stats.sd_write_errors++;
            LOG_W("SD recording interrupted");
            storage_unavailable = true;
            rt_thread_mdelay(LOG_RETRY_MS);
        }
    }
}

/* The kernel initializes the stack. SDMMC DMA uses the driver's staging buffer. */
static uint8_t task_logger_stack[4096] __attribute__((section(".logger_stack"), aligned(8)));

const TaskDescriptor task_logger = {
    "logger",
    task_logger_entry,
    task_logger_stack,
    sizeof(task_logger_stack),
    TASK_LOGGER_PRIORITY,
};
