#include "runtime_internal.h"
#include "contract_checks/algorithm_checks.h"
#include <string.h>

RuntimeData runtime_data;
RuntimeStats runtime_stats;

static uint64_t heartbeats[RUNTIME_OWNER_COUNT];
enum { EVENT_CAPACITY = 32 };
static struct rt_messagequeue event_queue, request_queue, measurement_queue;
rt_align(RT_ALIGN_SIZE) static uint8_t event_pool[RT_MQ_BUF_SIZE(sizeof(RuntimeEvent), EVENT_CAPACITY)];
rt_align(RT_ALIGN_SIZE) static uint8_t request_pool[RT_MQ_BUF_SIZE(sizeof(OperationRequest), PRODUCT_MAX_PENDING_OPERATION_REQUESTS)];
rt_align(RT_ALIGN_SIZE) static uint8_t measurement_pool[RT_MQ_BUF_SIZE(sizeof(SensorMeasurement), RUNTIME_MEASUREMENT_CAPACITY)];
static uint32_t event_sequence;
static uint32_t event_authority, event_directive, event_permit;
#ifdef FLIGHT_LOGGER_ENABLE
enum { LOG_EVENT_CAPACITY = 16 };
static struct rt_messagequeue log_event_queue;
/* CPU-only queue. rt_mq_init initializes the pool before producers use it. */
static uint8_t log_event_pool[RT_MQ_BUF_SIZE(sizeof(RuntimeEvent), LOG_EVENT_CAPACITY)]
    __attribute__((section(".logger_queue"), aligned(RT_ALIGN_SIZE)));
static bool log_event_queue_ready;
#endif
#ifdef FLIGHT_NOR_LOG_ENABLE
enum { CRITICAL_EVENT_CAPACITY = 16 };
static struct rt_messagequeue critical_queue;
static uint8_t critical_pool[RT_MQ_BUF_SIZE(sizeof(RuntimeEvent), CRITICAL_EVENT_CAPACITY)]
    __attribute__((section(".flash_work"), aligned(RT_ALIGN_SIZE)));
static bool critical_queue_ready;
#endif

/* Owners drain their queues on each wake, so one pending notification is enough. */
static int init_wakeup(struct rt_semaphore *event, const char *name)
{
    int result = rt_sem_init(event, name, 0, RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
        return result;
    result = rt_sem_control(event, RT_IPC_CMD_SET_VLIMIT, (void *)(uintptr_t)1u);
    if (result != RT_EOK)
        (void)rt_sem_detach(event);
    return result;
}

int runtime_data_init(uint64_t start_us)
{
    int result;
    memset(&runtime_data, 0, sizeof(runtime_data));
    memset(&runtime_stats, 0, sizeof(runtime_stats));
    event_sequence = 0u;
    event_authority = event_directive = event_permit = 0u;
    for (unsigned owner = 0; owner < RUNTIME_OWNER_COUNT; ++owner)
    {
        heartbeats[owner] = start_us;
    }

    result = rt_mutex_init(&runtime_data.mutex, "runtime_data", RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        return result;
    }
    result = init_wakeup(&runtime_data.imu_event, "imu_event");
    if (result != RT_EOK)
    {
        (void)rt_mutex_detach(&runtime_data.mutex);
        return result;
    }
    result = init_wakeup(&runtime_data.plan_event, "plan_event");
    if (result != RT_EOK)
    {
        goto detach_imu_event;
    }
    result = rt_mq_init(&event_queue, "fltevent", event_pool, sizeof(RuntimeEvent),
                        sizeof(event_pool), RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
        goto detach_plan_event;
    result = rt_mq_init(&request_queue, "fltreq", request_pool, sizeof(OperationRequest),
                        sizeof(request_pool), RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
        goto detach_event_queue;
    result = rt_mq_init(&measurement_queue, "fltmeas", measurement_pool, sizeof(SensorMeasurement),
                        sizeof(measurement_pool), RT_IPC_FLAG_FIFO);
    if (result == RT_EOK)
    {
#ifdef FLIGHT_LOGGER_ENABLE
        log_event_queue_ready = rt_mq_init(&log_event_queue, "log_event", log_event_pool,
                                          sizeof(RuntimeEvent), sizeof(log_event_pool),
                                          RT_IPC_FLAG_FIFO) == RT_EOK;
#endif
#ifdef FLIGHT_NOR_LOG_ENABLE
        critical_queue_ready = rt_mq_init(&critical_queue, "nor_event", critical_pool,
                                          sizeof(RuntimeEvent), sizeof(critical_pool),
                                          RT_IPC_FLAG_FIFO) == RT_EOK;
#endif
        return RT_EOK;
    }

    (void)rt_mq_detach(&request_queue);
detach_event_queue:
    (void)rt_mq_detach(&event_queue);
detach_plan_event:
    (void)rt_sem_detach(&runtime_data.plan_event);
detach_imu_event:
    (void)rt_sem_detach(&runtime_data.imu_event);
    (void)rt_mutex_detach(&runtime_data.mutex);
    return result;
}

void runtime_data_deinit(void)
{
#ifdef FLIGHT_NOR_LOG_ENABLE
    if (critical_queue_ready)
    {
        critical_queue_ready = false;
        (void)rt_mq_detach(&critical_queue);
    }
#endif
#ifdef FLIGHT_LOGGER_ENABLE
    if (log_event_queue_ready)
    {
        log_event_queue_ready = false;
        (void)rt_mq_detach(&log_event_queue);
    }
#endif
    (void)rt_mq_detach(&measurement_queue);
    (void)rt_mq_detach(&request_queue);
    (void)rt_mq_detach(&event_queue);
    (void)rt_sem_detach(&runtime_data.plan_event);
    (void)rt_sem_detach(&runtime_data.imu_event);
    (void)rt_mutex_detach(&runtime_data.mutex);
}

int runtime_data_lock(rt_int32_t wait)
{
    return rt_mutex_take(&runtime_data.mutex, wait) == RT_EOK;
}

void runtime_data_unlock(void)
{
    (void)rt_mutex_release(&runtime_data.mutex);
}

void runtime_data_set_event_epochs(const FmsStateSnapshot *snapshot)
{
    rt_base_t level = rt_hw_interrupt_disable();
    event_authority = snapshot->authority_epoch;
    event_directive = snapshot->directive_epoch;
    event_permit = snapshot->permit_epoch;
    rt_hw_interrupt_enable(level);
}

uint64_t runtime_data_get_heartbeat(RuntimeOwner owner)
{
    rt_base_t level = rt_hw_interrupt_disable();
    uint64_t last = heartbeats[owner];
    rt_hw_interrupt_enable(level);
    return last;
}

void runtime_data_record_event(unsigned source, unsigned ns, unsigned reason,
                              unsigned severity, uint64_t now)
{
    ClockInstant clock;
    if (clock_read(&clock))
    {
        return;
    }
    RuntimeEvent event = {
        .time_us = now,
        .boot_id = clock.boot_id,
        .time_epoch = clock.time_epoch,
        .source_id = (uint16_t)source,
        .reason_namespace = (uint16_t)ns,
        .reason_code = reason,
        .severity = (uint8_t)severity,
    };

    rt_base_t level = rt_hw_interrupt_disable();
    event.event_id = ++event_sequence;
    event.authority_epoch = event_authority;
    event.directive_epoch = event_directive;
    event.permit_epoch = event_permit;
    if (rt_mq_send(&event_queue, &event, sizeof(event)) != RT_EOK)
    {
        runtime_stats.event_drops++;
    }
#ifdef FLIGHT_LOGGER_ENABLE
    if (!log_event_queue_ready || rt_mq_send(&log_event_queue, &event, sizeof(event)) != RT_EOK)
        runtime_stats.log_event_drops++;
#endif
#ifdef FLIGHT_NOR_LOG_ENABLE
    if ((severity >= 2u || source == SOURCE_M4) &&
        (!critical_queue_ready || rt_mq_send(&critical_queue, &event, sizeof(event)) != RT_EOK))
        runtime_stats.critical_event_drops++;
#endif
    rt_hw_interrupt_enable(level);
}

#ifdef FLIGHT_NOR_LOG_ENABLE
bool runtime_data_take_critical_event(RuntimeEvent *event)
{
    return critical_queue_ready &&
        rt_mq_recv(&critical_queue, event, sizeof(*event), 0) == (rt_ssize_t)sizeof(*event);
}
#endif

#ifdef FLIGHT_LOGGER_ENABLE
bool runtime_data_take_log_event(RuntimeEvent *event)
{
    return log_event_queue_ready &&
        rt_mq_recv(&log_event_queue, event, sizeof(*event), 0) == (rt_ssize_t)sizeof(*event);
}
#endif

Status runtime_take_event(RuntimeEvent *out)
{
    if (!out)
    {
        return STATUS_INVALID_ARGUMENT;
    }
    if (!runtime_running())
        return STATUS_NOT_READY;
    return rt_mq_recv(&event_queue, out, sizeof(*out), 0) == (rt_ssize_t)sizeof(*out)
        ? STATUS_OK : STATUS_NOT_READY;
}

void runtime_data_mark_heartbeat(RuntimeOwner owner, uint64_t now)
{
    rt_base_t level = rt_hw_interrupt_disable();
    heartbeats[owner] = now;
    rt_hw_interrupt_enable(level);
}

int runtime_data_header_is_current(const ContractHeader *header, uint64_t now)
{
    ClockInstant clock;
    return !clock_read(&clock) && header_valid(header, now) &&
           clock.boot_id == header->boot_id && clock.time_epoch == header->time_epoch;
}

static Status lock_input(const ContractHeader *header)
{
    if (!runtime_running())
    {
        return STATUS_NOT_READY;
    }
    if (!header || !runtime_data_header_is_current(header, runtime_now_us()))
    {
        return STATUS_INVALID_ARGUMENT;
    }
    return runtime_data_lock(RT_WAITING_FOREVER) ? STATUS_OK : STATUS_NOT_READY;
}

Status runtime_submit_request(const OperationRequest *request)
{
    Status result = lock_input(request ? &request->header : NULL);
    if (result != STATUS_OK)
    {
        return result;
    }
    if (rt_mq_send(&request_queue, request, sizeof(*request)) != RT_EOK)
    {
        runtime_stats.queue_overruns++;
        runtime_data_unlock();
        return STATUS_BUFFER_FULL;
    }
    runtime_data_unlock();
    return STATUS_OK;
}

/* The mission owner drains the request batch under the runtime data lock. */
unsigned runtime_data_take_requests(OperationRequest *out)
{
    unsigned count = 0u;
    while (count < PRODUCT_MAX_PENDING_OPERATION_REQUESTS &&
           rt_mq_recv(&request_queue, &out[count], sizeof(*out), 0) == (rt_ssize_t)sizeof(*out))
        ++count;
    return count;
}

Status runtime_submit_operator(const OperatorInput *input)
{
    Status result = lock_input(input ? &input->header : NULL);
    if (result != STATUS_OK)
    {
        return result;
    }
    runtime_data.operator_input = *input;
    runtime_data.have_operator = 1u;
    runtime_data_unlock();
    return STATUS_OK;
}

enum { MISSION_SLOT_EMPTY, MISSION_SLOT_PENDING, MISSION_SLOT_BORROWED };

const MissionUploadCandidate *runtime_data_claim_mission(void)
{
    if (runtime_data.mission_slot_state != MISSION_SLOT_PENDING)
        return NULL;
    runtime_data.mission_slot_state = MISSION_SLOT_BORROWED;
    return &runtime_data.mission;
}

void runtime_data_release_mission(void)
{
    if (!runtime_data_lock(RT_WAITING_FOREVER))
        return;
    if (runtime_data.mission_slot_state == MISSION_SLOT_BORROWED)
        runtime_data.mission_slot_state = MISSION_SLOT_EMPTY;
    runtime_data_unlock();
}

Status runtime_submit_mission(const MissionUploadCandidate *candidate)
{
    Status result = lock_input(candidate ? &candidate->header : NULL);
    if (result != STATUS_OK)
    {
        return result;
    }
    if (runtime_data.mission_slot_state != MISSION_SLOT_EMPTY)
    {
        runtime_data_unlock();
        return STATUS_BUFFER_FULL;
    }
    runtime_data.mission = *candidate;
    runtime_data.mission_slot_state = MISSION_SLOT_PENDING;
    runtime_data_unlock();
    return STATUS_OK;
}

Status runtime_submit_measurement(const SensorMeasurement *measurement)
{
    Status result = lock_input(measurement ? &measurement->header : NULL);
    if (result != STATUS_OK)
    {
        return result;
    }
    if (rt_mq_send(&measurement_queue, measurement, sizeof(*measurement)) != RT_EOK)
    {
        runtime_stats.queue_overruns++;
        runtime_data_unlock();
        return STATUS_BUFFER_FULL;
    }
    runtime_data_unlock();
    (void)rt_sem_release(&runtime_data.imu_event);
    return STATUS_OK;
}

bool runtime_data_take_measurement(SensorMeasurement *out)
{
    return rt_mq_recv(&measurement_queue, out, sizeof(*out), 0) == (rt_ssize_t)sizeof(*out);
}

void runtime_get_stats(RuntimeStats *out)
{
    if (out)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        *out = runtime_stats;
        rt_hw_interrupt_enable(level);
    }
}

Status runtime_get_navigation(NavigationSnapshot *nav, NavigationCapability *capability)
{
    if (!nav || !capability || !runtime_running() || !runtime_data_lock(RT_WAITING_FOREVER))
    {
        return STATUS_NOT_READY;
    }
    *nav = runtime_data.nav;
    *capability = runtime_data.nav_cap;
    runtime_data_unlock();
    return STATUS_OK;
}

Status runtime_get_fms(FmsStateSnapshot *state, PreArmCheckResult *prearm)
{
    if (!state || !prearm || !runtime_running() || !runtime_data_lock(RT_WAITING_FOREVER))
    {
        return STATUS_NOT_READY;
    }
    *state = runtime_data.fms;
    *prearm = runtime_data.prearm;
    runtime_data_unlock();
    return STATUS_OK;
}

Status runtime_get_sensor_health(unsigned index, RuntimeSensorHealth *out)
{
    if (!out || index >= RUNTIME_SENSOR_COUNT)
    {
        return STATUS_INVALID_ARGUMENT;
    }
    if (!runtime_running() || !runtime_data_lock(RT_WAITING_FOREVER))
    {
        return STATUS_NOT_READY;
    }
    *out = runtime_data.sensors[index];
    runtime_data_unlock();
    return STATUS_OK;
}

Status runtime_get_control(ControlCapability *capability, ControlHealth *health,
                                  ActuatorDemand *demand, uint32_t *reference)
{
    if (!capability || !health || !demand || !reference)
    {
        return STATUS_INVALID_ARGUMENT;
    }
    if (!runtime_running() || !runtime_data_lock(RT_WAITING_FOREVER))
    {
        return STATUS_NOT_READY;
    }
    *capability = runtime_data.control_cap;
    *health = runtime_data.control_health;
    *demand = runtime_data.demand;
    *reference = runtime_data.control_reference_id;
    runtime_data_unlock();
    return STATUS_OK;
}

void runtime_data_controller_input(const ControlBundle *bundle, ControllerInput *input,
                                  uint64_t now)
{
    memset(input, 0, sizeof(*input));
    input->navigation = &bundle->nav;
    input->reference = &bundle->guidance.reference;
    input->pilot_control_reference = &bundle->guidance.pilot_control_reference;
    input->guidance_status = &bundle->guidance.guidance_status;
    input->directive = &bundle->directive;
    input->safety_envelope = &bundle->envelope;
    input->authority_status = &bundle->authority;
    input->actuation_permit = &bundle->permit;
    input->actuator_status = &bundle->actuator;
    input->actuator_capability = &bundle->actuator_cap;
    input->actuator_commit_receipt = &bundle->receipt;
    input->control_model_view = &bundle->model;
    input->control_time_us = bundle->nav.header.timestamp_sample_us;
    input->evaluation_time_us = now;
}
