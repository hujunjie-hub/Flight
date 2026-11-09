#include <winsock2.h>
#include <windows.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "ground_station.h"
#include "ground_log.h"
#include "calibration/sensor_calibration_service.h"
#include "mavlink/mavlink_action.h"
#include "mavlink/mavlink_frame.h"
#include "parameter_store.h"
#include "ground_parameters.h"

/* Standard MAVLink compatibility at the loopback boundary of the Host SIL. */
enum {
    HEARTBEAT = 0, SYS_STATUS = 1, PARAM_REQUEST_READ = 20, PARAM_REQUEST_LIST = 21,
    PARAM_VALUE = 22, PARAM_SET = 23, GPS_RAW_INT = 24, GPS_STATUS = 25, ATTITUDE = 30,
    GLOBAL_POSITION_INT = 33, MISSION_ITEM = 39, MISSION_REQUEST = 40,
    MISSION_CURRENT = 42, MISSION_REQUEST_LIST = 43, MISSION_COUNT = 44,
    MISSION_CLEAR_ALL = 45, MISSION_ITEM_REACHED = 46, MISSION_ACK = 47,
    MISSION_REQUEST_INT = 51, MISSION_ITEM_INT = 73,
    COMMAND_INT = 75, COMMAND_LONG = 76, COMMAND_ACK = 77,
    AUTOPILOT_VERSION = 148, BATTERY_STATUS = 147, HOME_POSITION = 242, EXTENDED_SYS_STATE = 245, STATUSTEXT = 253,
    AVAILABLE_MODES = 435, CURRENT_MODE = 436
};
enum { MAV_ACCEPTED = 0, MAV_TEMPORARILY_REJECTED = 1, MAV_DENIED = 2,
       MAV_UNSUPPORTED = 3, MAV_FAILED = 4, MAV_IN_PROGRESS = 5, MAV_CANCELLED = 6 };
enum { PARAMETER_CORE_COUNT = 1 };
#define CORE_PARAMETER_COUNT parameter_store_exposed_count()
#define PARAMETER_COUNT (parameter_store_exposed_count() + 1u)
static const char telemetry_rate_name[16] = "SIM_TLM_HZ";
static float telemetry_rate_hz = 5.0f;
static SOCKET connection = INVALID_SOCKET;
static struct sockaddr_in ground_target;
static uint8_t sequence;
static unsigned next_parameter;
static uint64_t next_heartbeat_ms, next_state;
static const double map_latitude = 31.2, map_longitude = 121.5;
static const double north_m_per_deg = 110875.0, east_m_per_deg = 95200.0;
enum { ITEM_LENGTH = 38, PLAN_UPLOAD = 1, PLAN_CLEAR = 2,
       PLAN_CAPACITY = PRODUCT_MAX_MISSION_WAYPOINTS + 2,
       FENCE_CAPACITY = PRODUCT_MAX_GEOFENCE_POLYGONS * PRODUCT_MAX_GEOFENCE_VERTICES,
       RALLY_CAPACITY = 32, PLAN_FENCE = 1, PLAN_RALLY = 2 };
enum { MISSION_OK = 0, MISSION_ERROR = 1, MISSION_FRAME = 2, MISSION_UNSUPPORTED = 3,
       MISSION_FULL = 4, MISSION_INVALID = 5, MISSION_SEQUENCE = 13, MISSION_DENIED = 14 };
typedef struct {
    float takeoff_altitude;
    float takeoff_ne[2];
    uint16_t finish_command;
} PlanActions;
static struct {
    uint8_t items[FENCE_CAPACITY][ITEM_LENGTH];
    unsigned count, next;
    uint8_t system, component, retries, type;
    uint64_t retry_at_ms;
    PlanActions actions;
} upload;
static uint8_t plan[PLAN_CAPACITY][ITEM_LENGTH];
static uint8_t fence_plan[FENCE_CAPACITY][ITEM_LENGTH];
static uint8_t rally_plan[RALLY_CAPACITY][ITEM_LENGTH];
static unsigned fence_count, rally_count;
static PlanActions plan_actions;
static unsigned plan_count;
static uint32_t plan_revision;
static unsigned reported_waypoint;
static uint16_t reported_mission_state;
typedef enum { ROUTE_IDLE, ROUTE_ARM, ROUTE_TAKEOFF, ROUTE_START, ROUTE_WAYPOINTS,
               ROUTE_FINISH, ROUTE_DISARM, ROUTE_COMPLETE, ROUTE_FAILED } RoutePhase;
static struct {
    RoutePhase phase, reported_phase;
    uint64_t deadline_us, progress_at_ms;
    uint16_t command;
    uint8_t system, component;
} route;
enum { SIL_TAKEOFF_MODE = FLIGHT_MODE_EMERGENCY_LAND + 1 };
static const struct {
    uint8_t standard, properties;
    const char *name;
} flight_modes[] = {
    {0, 2, "PreFlight"}, {0, 3, "Rate"}, {0, 2, "Attitude"}, {4, 2, ""},
    {1, 0, ""}, {6, 4, ""}, {5, 4, ""}, {7, 4, ""}, {0, 6, "Emergency Land"},
    {8, 4, ""}
};
static struct {
    uint64_t deadline_us, progress_at_ms;
    uint32_t request_id;
    uint16_t command, action;
    uint8_t system, component, plan_operation, route_operation;
} pending;

static void mission_ack(uint8_t system, uint8_t component, uint8_t result, uint8_t type);

void ground_station_apply_config(ProductControllerConfig *config)
{
    ground_configure_controller(config);
}

int ground_station_open(uint16_t port)
{
    WSADATA data;
    struct sockaddr_in endpoint = {0};
    u_long nonblocking = 1;
    if (port == 0 || WSAStartup(MAKEWORD(2, 2), &data) != 0)
        return 0;
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(port);
    endpoint.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ground_target = endpoint;
    connection = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (connection == INVALID_SOCKET || ioctlsocket(connection, FIONBIO, &nonblocking) != 0) {
        if (connection != INVALID_SOCKET)
            closesocket(connection);
        connection = INVALID_SOCKET;
        WSACleanup();
        return 0;
    }
    printf("[ground] Flight SIL -> 127.0.0.1:%u, %u active parameters\n", port, PARAMETER_COUNT);
    next_parameter = PARAMETER_COUNT;
    return 1;
}

void ground_station_close(void)
{
    if (connection != INVALID_SOCKET) {
        closesocket(connection);
        connection = INVALID_SOCKET;
        WSACleanup();
    }
}

static void emit(uint32_t id, uint8_t crc, const uint8_t *payload, size_t length)
{
    uint8_t frame[GROUND_FRAME_CAPACITY];
    const size_t size = wire_encode(id, crc, payload, length, sequence++, frame, sizeof(frame));
    if (size) {
        if (id != GROUND_LOG_ENTRY && id != GROUND_LOG_DATA) {
            FILETIME ft;
            GetSystemTimeAsFileTime(&ft);
            const uint64_t timestamp = ((uint64_t)ft.dwHighDateTime << 32) | ft.dwLowDateTime;
            ground_log_record((timestamp - UINT64_C(116444736000000000)) / 10u, frame, size);
        }
        (void)sendto(connection, (const char *)frame, (int)size, 0,
                     (const struct sockaddr *)&ground_target, sizeof(ground_target));
    }
}

static void mission_reached(unsigned item)
{
    uint8_t p[2];
    wire_u16(p, (uint16_t)item);
    emit(MISSION_ITEM_REACHED, 11, p, sizeof(p));
}

static void parameter(unsigned index)
{
    uint8_t payload[25] = {0};
    unsigned core_index = index;
    if (index == CORE_PARAMETER_COUNT) {
        wire_float(payload, telemetry_rate_hz);
        memcpy(payload + 8, telemetry_rate_name, 16);
    } else {
        core_index = parameter_store_exposed_at(index);
        if (core_index == UINT16_MAX)
            return;
        wire_float(payload, parameter_store_value(core_index));
        memcpy(payload + 8, parameter_store_name(core_index), 16);
    }
    wire_u16(payload + 4, (uint16_t)PARAMETER_COUNT);
    wire_u16(payload + 6, (uint16_t)index);
    payload[24] = 9; /* MAV_PARAM_TYPE_REAL32 */
    emit(PARAM_VALUE, 220, payload, sizeof(payload));
}

static void status_text(const char *message)
{
    uint8_t payload[51] = {4};
    size_t size = strlen(message);
    memcpy(payload + 1, message, size < 50 ? size : 50);
    emit(STATUSTEXT, 83, payload, sizeof(payload));
}

static void command_ack(uint16_t command, uint8_t result, uint16_t reason,
                        uint8_t system, uint8_t component)
{
    uint8_t payload[10] = {0};
    wire_u16(payload, command);
    payload[2] = result;
    payload[3] = 255; /* No percentage is available from FMS. */
    wire_u32(payload + 4, reason);
    payload[8] = system;
    payload[9] = component;
    emit(COMMAND_ACK, 143, payload, sizeof(payload));
}

static bool route_starting(void)
{
    return route.phase >= ROUTE_ARM && route.phase <= ROUTE_START;
}

static bool route_running(void)
{
    return route.phase >= ROUTE_ARM && route.phase <= ROUTE_DISARM;
}

static void route_failed(uint8_t result, uint16_t reason)
{
    if (route_starting())
        command_ack(route.command, result, reason, route.system, route.component);
    status_text("Flight SIL: plan stopped; check flight state");
    route.phase = ROUTE_FAILED;
}

void ground_station_ack(const CommandAck *ack)
{
    if (!pending.request_id || ack->request_id != pending.request_id)
        return;
    uint8_t result = MAV_FAILED;
    switch (ack->ack_state) {
    case COMMAND_ACK_ACCEPTED:
    case COMMAND_ACK_IN_PROGRESS: result = MAV_IN_PROGRESS; break;
    case COMMAND_ACK_COMPLETED: result = MAV_ACCEPTED; break;
    case COMMAND_ACK_REJECTED: result = MAV_DENIED; break;
    case COMMAND_ACK_CANCELLED: result = MAV_CANCELLED; break;
    }
    if (pending.plan_operation) {
        if (result == MAV_IN_PROGRESS) return;
        if (result == MAV_ACCEPTED) {
            plan_count = pending.plan_operation == PLAN_UPLOAD ? upload.count : 0;
            memcpy(plan, upload.items, plan_count * ITEM_LENGTH);
            plan_actions = pending.plan_operation == PLAN_UPLOAD ? upload.actions : (PlanActions){0};
            if (pending.plan_operation == PLAN_UPLOAD) ++plan_revision;
            route.phase = route.reported_phase = ROUTE_IDLE;
            reported_waypoint = 0;
            reported_mission_state = MISSION_STATE_IDLE;
        }
        mission_ack(pending.system, pending.component, result == MAV_ACCEPTED ? MISSION_OK : MISSION_DENIED, 0);
        upload.count = 0;
        pending.plan_operation = 0;
    } else if (pending.route_operation) {
        if (result != MAV_IN_PROGRESS) {
            if (result != MAV_ACCEPTED) {
                route_failed(result, ack->reason_code);
            } else {
                if (route.phase == ROUTE_TAKEOFF) mission_reached(0);
                if (route.phase == ROUTE_START)
                    command_ack(route.command, MAV_ACCEPTED, 0, route.system, route.component);
                if (route.phase == ROUTE_FINISH) mission_reached(plan_count - 1);
                route.phase = (RoutePhase)(route.phase + 1);
                route.deadline_us = ack->header.timestamp_publish_us + 8000000u;
            }
        }
    } else {
        command_ack(pending.command, result, ack->reason_code, pending.system, pending.component);
    }
    printf("[ground] command=%u request=%lu result=%u reason=%u\n", pending.command,
           (unsigned long)ack->request_id, result, ack->reason_code);
    if (result != MAV_IN_PROGRESS)
        pending.request_id = 0;
}

static void version(bool dynamic)
{
    uint8_t payload[60] = {0};
    /* MAVLink 2 plus the standard float parameter protocol. */
    wire_u64(payload, (1u << 0) | (1u << 2) | (1u << 3) | (1u << 13) | (1u << 17) |
             (dynamic ? (1u << 14) | (1u << 15) : 0u));
    wire_u32(payload + 16, dynamic ? 0x010902ffu : 0x010901ffu);
    if (dynamic)
        memcpy(payload + 36, "FLTSIL01", 8);
    emit(AUTOPILOT_VERSION, 178, payload, sizeof(payload));
}

static unsigned flight_mode(const FmsStateSnapshot *fms)
{
    return pending.request_id && pending.action == PRODUCT_CMD_TAKEOFF ?
        SIL_TAKEOFF_MODE : fms->flight_mode;
}

static void current_mode(const FmsStateSnapshot *fms)
{
    const unsigned mode = flight_mode(fms);
    if (mode >= sizeof(flight_modes) / sizeof(flight_modes[0])) return;
    uint8_t p[9] = {0};
    wire_u32(p, mode);
    p[8] = flight_modes[mode].standard;
    emit(CURRENT_MODE, 193, p, sizeof(p));
}

static bool available_modes(float requested, bool can_select)
{
    const unsigned count = sizeof(flight_modes) / sizeof(flight_modes[0]);
    if (!isfinite(requested) || requested < 0 || requested > count ||
        requested != (float)(unsigned)requested) return false;
    for (unsigned mode = 0; mode < count; ++mode) {
        if (requested != 0 && requested != (float)(mode + 1)) continue;
        uint8_t p[46] = {0};
        wire_u32(p, mode);
        wire_u32(p + 4, flight_modes[mode].properties | (can_select ? 0u : 2u));
        p[8] = (uint8_t)count;
        p[9] = (uint8_t)(mode + 1);
        p[10] = flight_modes[mode].standard;
        memcpy(p + 11, flight_modes[mode].name, strlen(flight_modes[mode].name));
        emit(AVAILABLE_MODES, 134, p, sizeof(p));
    }
    return true;
}

static void map_position(uint8_t *payload, const float ned[3])
{
    /* The map anchor is a display coordinate for the local SIL scene. */
    wire_u32(payload, (uint32_t)(int32_t)lround((map_latitude + ned[0] / north_m_per_deg) * 1e7));
    wire_u32(payload + 4, (uint32_t)(int32_t)lround((map_longitude + ned[1] / east_m_per_deg) * 1e7));
    wire_u32(payload + 8, (uint32_t)(int32_t)lround(-ned[2] * 1000.0));
}

static void send_home(const ProductHomeTargetSnapshot *home)
{
    uint8_t payload[60] = {0};
    map_position(payload, home->position_ned_m);
    for (unsigned axis = 0; axis < 3; ++axis) {
        wire_float(payload + 12 + 4 * axis, home->position_ned_m[axis]);
        wire_float(payload + 40 + 4 * axis, NAN);
    }
    wire_float(payload + 24, NAN); /* The FMS home target has no orientation. */
    wire_u64(payload + 52, home->captured_at_us);
    emit(HOME_POSITION, 104, payload, sizeof(payload));
}

static int targets(const uint8_t *p)
{
    return (p[0] == 0 || p[0] == 1) && (p[1] == 0 || p[1] == 1);
}

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (unsigned)p[1] << 8);
}

static float read_float(const uint8_t *p)
{
    const uint32_t bits = (uint32_t)read_u16(p) | (uint32_t)read_u16(p + 2) << 16;
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static int32_t read_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)read_u16(p) | (uint32_t)read_u16(p + 2) << 16);
}

static void mission_ack(uint8_t system, uint8_t component, uint8_t result, uint8_t type)
{
    const uint8_t p[4] = {system, component, result, type};
    emit(MISSION_ACK, 153, p, sizeof(p));
}

static void request_mission_item(uint64_t wall_ms)
{
    uint8_t p[5] = {0, 0, upload.system, upload.component, upload.type};
    wire_u16(p, (uint16_t)upload.next);
    emit(MISSION_REQUEST_INT, 196, p, sizeof(p));
    upload.retry_at_ms = wall_ms + 700u;
}

static void start_pending(uint64_t now_us, uint32_t request, uint16_t command,
                          uint8_t system, uint8_t component, uint8_t plan_operation)
{
    pending.request_id = request;
    pending.deadline_us = now_us + 45000000u;
    pending.progress_at_ms = GetTickCount64() + 1000u;
    pending.command = command;
    pending.action = 0;
    pending.system = system;
    pending.component = component;
    pending.plan_operation = plan_operation;
    pending.route_operation = 0;
}

static unsigned decode_waypoint(const uint8_t *item, float home_down,
                                ProductMissionWaypoint *waypoint)
{
    if (read_u16(item + 30) != 16) return MISSION_UNSUPPORTED; /* NAV_WAYPOINT */
    if (item[34] != 3 && item[34] != 6) return MISSION_FRAME; /* GLOBAL_RELATIVE_ALT[_INT] */
    const float hold = read_float(item), radius = read_float(item + 4);
    const float pass_radius = read_float(item + 8), yaw = read_float(item + 12);
    const float altitude = read_float(item + 24);
    const double north = (read_i32(item + 16) / 1e7 - 31.2) * 110875.0;
    const double east = (read_i32(item + 20) / 1e7 - 121.5) * 95200.0;
    if (hold != 0.0f || pass_radius != 0.0f || !item[36] ||
        !isfinite(radius) || radius < 0.0f || radius > 5.0f ||
        (!isnan(yaw) && (!isfinite(yaw) || yaw < -180.0f || yaw > 360.0f)) ||
        !isfinite(altitude) || altitude <= 0.0f || altitude > 10.0f ||
        hypot(north, east) > 100.0)
        return MISSION_INVALID;
    waypoint->waypoint_id = read_u16(item + 28) + 1u;
    waypoint->position_ned_m[0] = (float)north;
    waypoint->position_ned_m[1] = (float)east;
    waypoint->position_ned_m[2] = home_down - altitude;
    waypoint->yaw_rad = isnan(yaw) ? 0.0f : yaw * 0.01745329252f;
    waypoint->acceptance_radius_m = radius > 0.0f ? radius : 0.35f;
    waypoint->speed_limit_mps = 2.0f;
    return MISSION_OK;
}

static unsigned decode_plan(const NavigationSnapshot *nav, const ProductHomeTargetSnapshot *home,
                            ProductMissionPayload *candidate)
{
    unsigned first = 0, end = upload.count;
    upload.actions = (PlanActions){0};
    const uint8_t *takeoff = upload.items[0];
    if (read_u16(takeoff + 30) == 22) {
        if (takeoff[34] != 3 && takeoff[34] != 6) return MISSION_FRAME;
        const float altitude = read_float(takeoff + 24);
        const double north = (read_i32(takeoff + 16) / 1e7 - 31.2) * 110875.0;
        const double east = (read_i32(takeoff + 20) / 1e7 - 121.5) * 95200.0;
        if (!takeoff[36] || !isfinite(altitude) || altitude <= 0.0f || altitude > 10.0f ||
            hypot(north - nav->core.position_ned_m[0], east - nav->core.position_ned_m[1]) > 1.0)
            return MISSION_INVALID;
        upload.actions.takeoff_altitude = altitude;
        upload.actions.takeoff_ne[0] = (float)north;
        upload.actions.takeoff_ne[1] = (float)east;
        ++first;
    }
    const uint8_t *finish = upload.items[end - 1];
    const uint16_t finish_command = read_u16(finish + 30);
    if (finish_command == 20 || finish_command == 21) {
        upload.actions.finish_command = finish_command == 20 ? PRODUCT_CMD_RETURN : PRODUCT_CMD_LAND;
        --end;
    }
    if (first >= end) return MISSION_INVALID;
    if (end - first > PRODUCT_MAX_MISSION_WAYPOINTS) return MISSION_FULL;
    if (finish_command == 21) {
        const uint8_t *last_waypoint = upload.items[end - 1];
        if (finish[34] != 3 && finish[34] != 6) return MISSION_FRAME;
        if (read_float(finish + 24) != 0.0f ||
            memcmp(finish + 16, last_waypoint + 16, 8))
            return MISSION_INVALID;
    }
    candidate->waypoint_count = (uint16_t)(end - first);
    candidate->mission_revision = plan_revision + 1;
    candidate->mission_hash = 2166136261u;
    const float home_down = home ? home->position_ned_m[2] : nav->core.position_ned_m[2];
    for (unsigned i = 0; i < upload.count; ++i) {
        if (i >= first && i < end) {
            const unsigned result = decode_waypoint(upload.items[i], home_down,
                                                    &candidate->waypoints[i - first]);
            if (result != MISSION_OK) return result;
        }
        for (unsigned j = 0; j < ITEM_LENGTH; ++j)
            candidate->mission_hash = (candidate->mission_hash ^ upload.items[i][j]) * 16777619u;
    }
    if (!candidate->mission_hash) candidate->mission_hash = 1u;
    return MISSION_OK;
}

static unsigned commit_auxiliary_plan(const GroundStationActions *actions,
                                      const NavigationSnapshot *nav)
{
    if (!nav->origin_id || !nav->state_epoch)
        return MISSION_DENIED;
    ProductGeofenceSnapshot fences[PRODUCT_MAX_GEOFENCE_POLYGONS] = {0};
    unsigned polygons = 0, inclusions = 0, vertex = 0;
    for (unsigned index = 0; index < upload.count; ++index) {
        const uint8_t *item = upload.items[index];
        const uint16_t command = read_u16(item + 30);
        const float height = read_float(item + 24);
        const double north = (read_i32(item + 16) / 1e7 - 31.2) * 110875.0;
        const double east = (read_i32(item + 20) / 1e7 - 121.5) * 95200.0;
        if (hypot(north, east) > 100.0 || !isfinite(height))
            return MISSION_INVALID;
        if (upload.type == PLAN_RALLY) {
            if (command != 5100) return MISSION_UNSUPPORTED;
            if (item[34] != 3 && item[34] != 6) return MISSION_FRAME;
            if (height < 0.0f || height > 10.0f) return MISSION_INVALID;
            continue;
        }
        if (command < 5001 || command > 5004) return MISSION_UNSUPPORTED;
        if (item[34] != 0 && item[34] != 5) return MISSION_FRAME;
        const bool circle = command >= 5003;
        const float parameter = read_float(item);
        const float count = circle ? PRODUCT_MAX_GEOFENCE_VERTICES : parameter;
        if (!isfinite(parameter) || height != 0.0f ||
            (circle ? vertex || parameter < 0.25f || hypot(north, east) + parameter > 100.0
                    : count < 3 || count > PRODUCT_MAX_GEOFENCE_VERTICES || floorf(count) != count))
            return MISSION_INVALID;
        if (polygons == PRODUCT_MAX_GEOFENCE_POLYGONS) return MISSION_FULL;
        ProductGeofenceSnapshot *fence = &fences[polygons];
        const uint8_t kind = command == 5001 || command == 5003 ?
            PRODUCT_GEOFENCE_KEEP_IN : PRODUCT_GEOFENCE_KEEP_OUT;
        if (!vertex) {
            if (kind == PRODUCT_GEOFENCE_KEEP_IN && ++inclusions > 1u) return MISSION_UNSUPPORTED;
            fence->fence_id = polygons + 1u;
            fence->fence_hash = 2166136261u;
            fence->kind = kind;
            fence->vertex_count = (uint8_t)count;
            fence->validity = PRODUCT_GEOFENCE_VALID;
            fence->frame_id = FRAME_LOCAL_NED;
            fence->origin_id = nav->origin_id;
            fence->state_epoch = nav->state_epoch;
            fence->min_height_above_home_m = -2.0f;
            fence->max_height_above_home_m = 12.0f;
        }
        if (kind != fence->kind || count != fence->vertex_count) return MISSION_INVALID;
        if (circle) {
            /* Inscribe keep-in circles and circumscribe keep-out circles. */
            const double step = 6.283185307179586 / fence->vertex_count;
            const double radius = kind == PRODUCT_GEOFENCE_KEEP_IN ? parameter :
                                  parameter / cos(step * 0.5);
            for (unsigned i = 0; i < fence->vertex_count; ++i) {
                fence->vertices_ne_m[i][0] = (float)(north + radius * cos(i * step));
                fence->vertices_ne_m[i][1] = (float)(east + radius * sin(i * step));
            }
            vertex = fence->vertex_count - 1u;
        } else {
            fence->vertices_ne_m[vertex][0] = (float)north;
            fence->vertices_ne_m[vertex][1] = (float)east;
        }
        for (unsigned byte = 0; byte < 24; ++byte)
            fence->fence_hash = (fence->fence_hash ^ item[byte]) * 16777619u;
        if (++vertex == fence->vertex_count) {
            if (!fence->fence_hash) fence->fence_hash = 1u;
            vertex = 0;
            ++polygons;
        }
    }
    if (vertex) return MISSION_INVALID;
    if (upload.type == PLAN_FENCE) {
        if (actions->replace_geofences(fences, (uint16_t)polygons) != STATUS_OK)
            return MISSION_INVALID;
        memcpy(fence_plan, upload.items, upload.count * ITEM_LENGTH);
        fence_count = upload.count;
    } else {
        memcpy(rally_plan, upload.items, upload.count * ITEM_LENGTH);
        rally_count = upload.count;
    }
    return MISSION_OK;
}

static void receive_mission(unsigned id, const uint8_t *p, uint8_t system, uint8_t component,
                            uint64_t now_us, const NavigationSnapshot *nav,
                            const ProductHomeTargetSnapshot *home,
                            const FmsStateSnapshot *fms, const GroundStationActions *actions)
{
    const unsigned address = (id == MISSION_ITEM_INT || id == MISSION_ITEM) ? 32u :
        (id == MISSION_COUNT || id == MISSION_REQUEST || id == MISSION_REQUEST_INT) ? 2u : 0u;
    const uint8_t type = p[address == 32u ? 37u : address + 2u];
    if (!targets(p + address)) return;
    if (type > PLAN_RALLY) {
        mission_ack(system, component, MISSION_UNSUPPORTED, type);
        return;
    }
    const unsigned stored_count = type == PLAN_FENCE ? fence_count :
                                  type == PLAN_RALLY ? rally_count : plan_count;
    const uint8_t (*stored)[ITEM_LENGTH] = type == PLAN_FENCE ? fence_plan :
                                           type == PLAN_RALLY ? rally_plan : plan;
    if (id == MISSION_REQUEST_LIST) {
        uint8_t reply[5] = {0, 0, system, component, type};
        wire_u16(reply, (uint16_t)stored_count);
        emit(MISSION_COUNT, 221, reply, sizeof(reply));
        return;
    }
    if (id == MISSION_REQUEST || id == MISSION_REQUEST_INT) {
        const unsigned index = read_u16(p);
        if (index >= stored_count) {
            mission_ack(system, component, MISSION_SEQUENCE, type);
            return;
        }
        uint8_t item[ITEM_LENGTH];
        memcpy(item, stored[index], sizeof(item));
        item[32] = system;
        item[33] = component;
        if (id == MISSION_REQUEST) {
            wire_float(item + 16, (float)(read_i32(item + 16) / 1e7));
            wire_float(item + 20, (float)(read_i32(item + 20) / 1e7));
            if (item[34] == 5) item[34] = 0; /* GLOBAL_INT to GLOBAL */
            if (item[34] == 6) item[34] = 3; /* GLOBAL_RELATIVE_ALT_INT */
        }
        emit(id == MISSION_REQUEST_INT ? MISSION_ITEM_INT : MISSION_ITEM,
             id == MISSION_REQUEST_INT ? 38 : 254, item, sizeof(item));
        return;
    }
    if (!actions || p[address] != 1 || route_running() ||
        (type && fms->arm_state != ARM_STATE_DISARMED)) {
        mission_ack(system, component, MISSION_DENIED, type);
        return;
    }
    if (pending.request_id) {
        if (type || !pending.plan_operation || system != pending.system || component != pending.component)
            mission_ack(system, component, MISSION_DENIED, type);
        return;
    }
    if (upload.count && (type != upload.type || system != upload.system || component != upload.component)) {
        mission_ack(system, component, MISSION_DENIED, type);
        return;
    }
    if (id == MISSION_CLEAR_ALL || (id == MISSION_COUNT && read_u16(p) == 0)) {
        upload.count = 0;
        if (type) {
            unsigned result = MISSION_OK;
            if (type == PLAN_FENCE) {
                if (actions->replace_geofences(NULL, 0) == STATUS_OK) fence_count = 0;
                else result = MISSION_DENIED;
            } else {
                rally_count = 0;
            }
            mission_ack(system, component, (uint8_t)result, type);
            return;
        }
        const uint32_t request = actions->command(PRODUCT_CMD_MISSION_CLEAR, 0.0f);
        if (request) start_pending(now_us, request, PRODUCT_CMD_MISSION_CLEAR, system, component, PLAN_CLEAR);
        else mission_ack(system, component, MISSION_DENIED, 0);
        return;
    }
    if (id == MISSION_COUNT) {
        const unsigned count = read_u16(p);
        const unsigned capacity = type == PLAN_FENCE ? FENCE_CAPACITY :
                                  type == PLAN_RALLY ? RALLY_CAPACITY : PLAN_CAPACITY;
        if (count > capacity) {
            mission_ack(system, component, MISSION_FULL, type);
            return;
        }
        if (upload.count != count) {
            upload.count = count;
            upload.next = 0;
        }
        upload.system = system;
        upload.component = component;
        upload.type = type;
        upload.retries = 0;
        request_mission_item(GetTickCount64());
        return;
    }
    const unsigned index = read_u16(p + 28);
    if (!upload.count) {
        if (id == MISSION_ITEM_INT && type == upload.type && stored_count && index == stored_count - 1 &&
            system == upload.system && component == upload.component &&
            !memcmp(stored[index], p, ITEM_LENGTH))
            mission_ack(system, component, MISSION_OK, type); /* final ACK retry */
        else
            mission_ack(system, component, MISSION_SEQUENCE, type);
        return;
    }
    if (index != upload.next) {
        request_mission_item(GetTickCount64());
        return;
    }
    uint8_t *item = upload.items[index];
    memcpy(item, p, ITEM_LENGTH);
    if (id == MISSION_ITEM) {
        const float lat = read_float(p + 16), lon = read_float(p + 20);
        if (!isfinite(lat) || !isfinite(lon) || fabsf(lat) > 90.0f || fabsf(lon) > 180.0f) {
            upload.count = 0;
            mission_ack(system, component, MISSION_INVALID, type);
            return;
        }
        wire_u32(item + 16, (uint32_t)(int32_t)lround(lat * 1e7));
        wire_u32(item + 20, (uint32_t)(int32_t)lround(lon * 1e7));
    }
    if (++upload.next < upload.count) {
        upload.retries = 0;
        request_mission_item(GetTickCount64());
        return;
    }
    if (type) {
        const unsigned result = commit_auxiliary_plan(actions, nav);
        upload.count = 0;
        mission_ack(system, component, (uint8_t)result, type);
        return;
    }
    ProductMissionPayload candidate = {0};
    unsigned result = decode_plan(nav, home, &candidate);
    if (result == MISSION_OK) {
        const uint32_t request = actions->commit_mission(&candidate);
        if (request) {
            start_pending(now_us, request, PRODUCT_CMD_MISSION_COMMIT, system, component, PLAN_UPLOAD);
            return;
        }
        result = MISSION_DENIED;
    }
    upload.count = 0;
    mission_ack(system, component, (uint8_t)result, 0);
}

static void receive_command(uint64_t now_us, const uint8_t *p, uint8_t system, uint8_t component,
                            const GroundStationActions *actions, int long_format,
                            const ProductHomeTargetSnapshot *home, const FmsStateSnapshot *fms,
                            const MissionStatus *mission)
{
    if (p[30] != 1 || !targets(p + 30))
        return;
    const uint16_t command = read_u16(p + 28);
    const float param1 = read_float(p), param2 = read_float(p + 4);
    float argument = read_float(p + 24);
    uint32_t (*request)(uint16_t, float) = actions ? actions->command : NULL;
    float home_target[3];
    bool explicit_home = false;
    uint16_t action = 0;
    if ((command == 512 && param1 == AUTOPILOT_VERSION) || command == 520) {
        version(request != NULL);
        command_ack(command, MAV_ACCEPTED, 0, system, component);
        return;
    }
    if ((command == 512 && param1 == HOME_POSITION) || command == 410) {
        if (home)
            send_home(home);
        command_ack(command, home ? MAV_ACCEPTED : MAV_TEMPORARILY_REJECTED, 0, system, component);
        return;
    }
    if (command == 512 && (param1 == AVAILABLE_MODES || param1 == CURRENT_MODE)) {
        bool sent = true;
        if (param1 == AVAILABLE_MODES) sent = available_modes(param2, request != NULL);
        else current_mode(fms);
        command_ack(command, sent ? MAV_ACCEPTED : MAV_FAILED, 0, system, component);
        return;
    }
    if (long_format && command == 241) { /* MAV_CMD_PREFLIGHT_CALIBRATION */
        if (fms->arm_state != ARM_STATE_DISARMED) {
            status_text("Flight SIL: calibration requires DISARMED");
            command_ack(command, MAV_DENIED, 1, system, component);
            return;
        }
        const uint8_t kind = mavlink_calibration_kind(p);
        char error[40];
        if (kind == SENSOR_CAL_NONE ||
            !sensor_calibration_start(kind, now_us, error, sizeof(error))) {
            status_text(kind == SENSOR_CAL_NONE ? "Flight SIL: no calibration selected" : error);
            command_ack(command, MAV_DENIED, 1, system, component);
        } else {
            status_text("Flight SIL calibration started");
            command_ack(command, MAV_ACCEPTED, 0, system, component);
        }
        return;
    }
    if (long_format && command == 246) { /* MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN */
        if (fms->arm_state != ARM_STATE_DISARMED || param1 < 0.5f) {
            status_text("Flight SIL: reboot requires DISARMED");
            command_ack(command, MAV_DENIED, 1, system, component);
        } else {
            status_text("Flight SIL: reboot requested (simulation keeps running)");
            command_ack(command, MAV_ACCEPTED, 0, system, component);
        }
        return;
    }
    if (long_format && command == 31011) {
        char error[48] = "Flight SIL: invalid calibration action";
        bool accepted = false;
        if (fms->arm_state != ARM_STATE_DISARMED) {
            snprintf(error, sizeof(error), "Flight SIL: calibration requires DISARMED");
        } else if (param2 == 2.0f) {
            sensor_calibration_cancel(now_us);
            accepted = true;
        } else if (param2 == 3.0f) {
            accepted = sensor_calibration_finish_rc(now_us, error, sizeof(error));
        } else if (param2 == 5.0f) {
            SensorCalibrationStatus status;
            sensor_calibration_get_status(&status);
            if (status.state != SENSOR_CAL_RUNNING && actions->send_crsf_bind) {
                accepted = actions->send_crsf_bind();
                if (accepted)
                    status_text("CRSF bind request sent");
            }
        } else if (param2 == 4.0f) {
            accepted = sensor_calibration_copy_rc_trims(now_us, error, sizeof(error));
            if (accepted)
                status_text("RC TRIM copied");
        } else if (param2 == 1.0f && param1 >= 0.0f && param1 < SENSOR_CAL_ACCEL_FACES &&
                   param1 == (float)(uint8_t)param1) {
            accepted = sensor_calibration_capture_accel_face((uint8_t)param1, now_us, error, sizeof(error));
        }
        if (!accepted)
            status_text(error);
        command_ack(command, accepted ? MAV_ACCEPTED : MAV_DENIED, accepted ? 0 : 1, system, component);
        return;
    }
    MavlinkAction decoded;
    const MavlinkActionResult decode_result = mavlink_action_decode(command, long_format, p, &decoded);
    if (decode_result == MAVLINK_ACTION_INVALID ||
        (decode_result == MAVLINK_ACTION_VALID && decoded.altitude_m > 10.0f)) {
        command_ack(command, MAV_DENIED, 1, system, component);
        return;
    }
    if (decode_result == MAVLINK_ACTION_VALID) {
        action = decoded.product_command;
        argument = decoded.altitude_m;
    }
    if (!action) switch (command) {
    case 300: /* MAV_CMD_MISSION_START, whole mission only */
        if (plan_count && param1 == 0.0f &&
            (read_float(p + 4) == 0.0f || read_float(p + 4) == (float)(plan_count - 1u)))
            action = mission->mission_state == MISSION_STATE_PAUSED ?
                PRODUCT_CMD_MISSION_RESUME : PRODUCT_CMD_MISSION_START;
        break;
    case 193: /* MAV_CMD_DO_PAUSE_CONTINUE */
        if (param1 == 0.0f || param1 == 1.0f)
            action = param1 == 0.0f ? PRODUCT_CMD_MISSION_PAUSE : PRODUCT_CMD_MISSION_RESUME;
        break;
    case 31010: /* MAV_CMD_USER_1, Flight SIL mission abort */
        if (param1 == 0.0f) action = PRODUCT_CMD_MISSION_ABORT;
        break;
    case 179: /* MAV_CMD_DO_SET_HOME */
        if (!request) break;
        if (param1 == 1.0f) {
            action = PRODUCT_CMD_SET_HOME;
        } else if (param1 == 0.0f && actions->set_home) {
            const double lat = long_format ? read_float(p + 16) : read_i32(p + 16) / 1e7;
            const double lon = long_format ? read_float(p + 20) : read_i32(p + 20) / 1e7;
            const double north = (lat - map_latitude) * north_m_per_deg;
            const double east = (lon - map_longitude) * east_m_per_deg;
            /* The bounded SIL map uses a flat ground plane at zero MSL. */
            if ((!long_format && p[32] != 0u && p[32] != 5u) ||
                !isfinite(lat) || !isfinite(lon) || !isfinite(argument) ||
                fabs(lat) > 90.0 || fabs(lon) > 180.0 ||
                hypot(north, east) > 100.0 || fabsf(argument) > 2.0f) {
                command_ack(command, MAV_DENIED, 1, system, component);
                return;
            }
            home_target[0] = (float)north;
            home_target[1] = (float)east;
            home_target[2] = -argument;
            explicit_home = true;
            action = PRODUCT_CMD_SET_HOME;
        }
        break;
    case 176: /* MAV_CMD_DO_SET_MODE, custom mode */
    case 262: /* MAV_CMD_DO_SET_STANDARD_MODE */
        for (unsigned mode = FLIGHT_MODE_POS_HOLD; mode <= SIL_TAKEOFF_MODE; ++mode) {
            if (flight_modes[mode].properties & 2u) continue;
            if ((command == 176 && param1 == 1.0f && param2 == (float)mode) ||
                (command == 262 && param1 == flight_modes[mode].standard)) {
                switch (mode) {
                case FLIGHT_MODE_POS_HOLD:
                    if (request && fms->flight_mode == FLIGHT_MODE_POS_HOLD &&
                        (!pending.request_id || pending.action != PRODUCT_CMD_TAKEOFF)) {
                        command_ack(command, MAV_ACCEPTED, 0, system, component);
                        return;
                    }
                    if (mission->mission_state == MISSION_STATE_RUNNING) action = PRODUCT_CMD_MISSION_PAUSE;
                    break;
                case FLIGHT_MODE_AUTO_MISSION:
                    action = mission->mission_state == MISSION_STATE_PAUSED ?
                        PRODUCT_CMD_MISSION_RESUME : PRODUCT_CMD_MISSION_START;
                    break;
                case FLIGHT_MODE_AUTO_RETURN: action = PRODUCT_CMD_RETURN; break;
                case FLIGHT_MODE_AUTO_LAND: action = PRODUCT_CMD_LAND; break;
                case SIL_TAKEOFF_MODE: action = PRODUCT_CMD_TAKEOFF; argument = 2.0f; break;
                }
                break;
            }
        }
        break;
    }
    if (!action || !request) {
        command_ack(command, command == 262 ? MAV_FAILED : MAV_UNSUPPORTED, 0, system, component);
    } else if (route_starting() || pending.request_id) {
        const bool route_retry = route_starting() && command == route.command &&
                                 system == route.system && component == route.component;
        const int retry = !pending.plan_operation && command == pending.command && system == pending.system &&
                          component == pending.component;
        command_ack(command, retry || route_retry ? MAV_IN_PROGRESS : MAV_TEMPORARILY_REJECTED,
                    0, system, component);
    } else if (action == PRODUCT_CMD_MISSION_START &&
               (plan_actions.takeoff_altitude || plan_actions.finish_command)) {
        if (route_running() || mission->mission_state != MISSION_STATE_LOADED) {
            command_ack(command, MAV_DENIED, 0, system, component);
            return;
        }
        route.phase = plan_actions.takeoff_altitude ? ROUTE_ARM : ROUTE_START;
        route.deadline_us = now_us + 8000000u;
        route.progress_at_ms = GetTickCount64() + 1000u;
        route.command = command;
        route.system = system;
        route.component = component;
        command_ack(command, MAV_IN_PROGRESS, 0, system, component);
    } else {
        const uint32_t id = explicit_home ? actions->set_home(home_target) : request(action, argument);
        if (!id) {
            command_ack(command, MAV_TEMPORARILY_REJECTED, 0, system, component);
            return;
        }
        start_pending(now_us, id, command, system, component, 0);
        pending.action = action;
        if (route.phase == ROUTE_WAYPOINTS &&
            (action == PRODUCT_CMD_LAND || action == PRODUCT_CMD_RETURN))
            route.phase = ROUTE_FAILED;
    }
}

static void route_step(uint64_t now_us, const GroundStationActions *actions,
                       const NavigationSnapshot *nav, const FmsStateSnapshot *fms,
                       const LandedStateSnapshot *landed, const MissionStatus *mission,
                       const PreArmCheckResult *prearm)
{
    if (!route_running()) return;
    const uint64_t wall_ms = GetTickCount64();
    if (route_starting() && wall_ms >= route.progress_at_ms) {
        command_ack(route.command, MAV_IN_PROGRESS, 0, route.system, route.component);
        route.progress_at_ms = wall_ms + 1000u;
    }
    if (pending.request_id) return;
    if (fms->failsafe_level >= FAILSAFE_ACTION_REQUIRED) {
        route_failed(MAV_FAILED, 18);
        return;
    }
    if (route.phase == ROUTE_WAYPOINTS) {
        if (mission->mission_state == MISSION_STATE_ABORTED ||
            (mission->mission_state == MISSION_STATE_COMPLETED &&
             mission->result_code != MISSION_RESULT_SUCCESS)) {
            route_failed(MAV_CANCELLED, 0);
        } else if (mission->mission_state == MISSION_STATE_COMPLETED) {
            route.phase = plan_actions.finish_command ? ROUTE_FINISH : ROUTE_COMPLETE;
            route.deadline_us = now_us + 8000000u;
        }
        return;
    }
    if (now_us >= route.deadline_us) {
        route_failed(MAV_FAILED, 6);
        return;
    }
    uint16_t action = 0;
    switch (route.phase) {
    case ROUTE_ARM:
        if (landed->state != LANDED_STATE_LANDED) {
            if (fms->arm_state == ARM_STATE_ARMED) route_failed(MAV_DENIED, 11);
            return; /* A disarmed relaunch can wait for fresh ground evidence after touchdown. */
        }
        if (hypotf(nav->core.position_ned_m[0] - plan_actions.takeoff_ne[0],
                   nav->core.position_ned_m[1] - plan_actions.takeoff_ne[1]) > 1.0f) {
            route_failed(MAV_DENIED, 13);
            return;
        }
        if (fms->arm_state == ARM_STATE_ARMED) {
            route.phase = ROUTE_TAKEOFF;
            return;
        }
        if (prearm->state != PREARM_READY) return;
        action = PRODUCT_CMD_ARM;
        break;
    case ROUTE_TAKEOFF: action = PRODUCT_CMD_TAKEOFF; break;
    case ROUTE_START: action = PRODUCT_CMD_MISSION_START; break;
    case ROUTE_FINISH: action = plan_actions.finish_command; break;
    case ROUTE_DISARM: action = PRODUCT_CMD_DISARM; break;
    default: return;
    }
    const uint32_t request = actions->command(action, plan_actions.takeoff_altitude);
    if (!request) return; /* The bounded transition gate can be temporarily busy. */
    start_pending(now_us, request, route.command, route.system, route.component, 0);
    pending.route_operation = 1;
    pending.action = action;
}

static void receive(uint64_t now_us, const GroundStationActions *actions,
                    const NavigationSnapshot *navigation, const ProductHomeTargetSnapshot *home,
                    const FmsStateSnapshot *fms, const MissionStatus *mission)
{
    uint8_t buffer[2048];
    for (unsigned i = 0; i < 16; ++i) {
        struct sockaddr_in sender;
        int sender_length = sizeof(sender);
        const int bytes = recvfrom(connection, (char *)buffer, sizeof(buffer), 0,
                                   (struct sockaddr *)&sender, &sender_length);
        (void)sender;
        if (bytes <= 0)
            return;
        for (unsigned offset = 0; offset + 8u <= (unsigned)bytes;) {
            const uint8_t *frame = buffer + offset;
            const int v2 = frame[0] == MAVLINK_V2_MAGIC;
            if (!v2 && frame[0] != 0xfeu) {
                ++offset;
                continue;
            }
            const unsigned header = v2 ? 10u : 6u;
            const unsigned end = header + frame[1];
            const unsigned size = end + 2u + ((v2 && (frame[2] & 1u)) ? 13u : 0u);
            if (size > (unsigned)bytes - offset)
                break;
            offset += size;
            const uint8_t system = frame[v2 ? 5 : 3], component = frame[v2 ? 6 : 4];
            if ((v2 && frame[2] != 0) || !system || !component || system == 1)
                continue;
            const unsigned id = v2 ? frame[7] | (unsigned)frame[8] << 8 | (unsigned)frame[9] << 16
                                   : frame[5];
            unsigned extra;
            switch (id) {
            case PARAM_REQUEST_READ: extra = 214; break;
            case PARAM_REQUEST_LIST: extra = 159; break;
            case PARAM_SET: extra = 168; break;
            case MISSION_REQUEST_LIST: extra = 132; break;
            case MISSION_COUNT: extra = 221; break;
            case MISSION_CLEAR_ALL: extra = 232; break;
            case MISSION_REQUEST: extra = 230; break;
            case MISSION_REQUEST_INT: extra = 196; break;
            case MISSION_ITEM: extra = 254; break;
            case MISSION_ITEM_INT: extra = 38; break;
            case GROUND_LOG_REQUEST_LIST: extra = 128; break;
            case GROUND_LOG_REQUEST_DATA: extra = 116; break;
            case GROUND_LOG_ERASE: extra = 237; break;
            case GROUND_LOG_REQUEST_END: extra = 203; break;
            case COMMAND_INT: extra = 158; break;
            case COMMAND_LONG: extra = 152; break;
            default: continue;
            }
            uint16_t crc = 0xffffu;
            for (unsigned byte = 1; byte < end; ++byte)
                crc = mavlink_crc_accumulate(crc, frame[byte]);
            if (mavlink_crc_accumulate(crc, (uint8_t)extra) != read_u16(frame + end) || frame[1] > ITEM_LENGTH)
                continue;
            uint8_t payload[ITEM_LENGTH] = {0};
            memcpy(payload, frame + header, frame[1]);
            if (id >= GROUND_LOG_REQUEST_LIST && id <= GROUND_LOG_REQUEST_END) {
                const unsigned address = id == GROUND_LOG_REQUEST_LIST ? 4u :
                                         id == GROUND_LOG_REQUEST_DATA ? 10u : 0u;
                if (!targets(payload + address) ||
                    (id == GROUND_LOG_ERASE && payload[address] != 1)) continue;
                if (id == GROUND_LOG_ERASE && fms->arm_state != ARM_STATE_DISARMED) {
                    status_text("Flight SIL: log erase requires disarm");
                    continue;
                }
                ground_log_receive(id, payload, system, component, emit);
            } else if (id == COMMAND_LONG || id == COMMAND_INT) {
                receive_command(now_us, payload, system, component,
                                actions, id == COMMAND_LONG, home, fms, mission);
            } else if (id == PARAM_REQUEST_LIST && targets(payload)) {
                next_parameter = 0;
            } else if (id == PARAM_REQUEST_READ && targets(payload + 2)) {
                const uint16_t index = read_u16(payload);
                for (unsigned n = 0; n < PARAMETER_COUNT; ++n) {
                    const char *name = n == CORE_PARAMETER_COUNT ? telemetry_rate_name
                                                                 : parameter_store_name(parameter_store_exposed_at(n));
                    if (index == n || (index == UINT16_MAX && !memcmp(payload + 4, name, 16))) {
                        parameter(n);
                        break;
                    }
                }
            } else if (id == PARAM_SET && payload[4] == 1 && targets(payload + 4)) {
                if (!memcmp(payload + 6, telemetry_rate_name, 16)) {
                    const float rate = read_float(payload);
                    if (payload[22] == 9 && isfinite(rate) && rate >= 1.0f && rate <= 10.0f)
                        telemetry_rate_hz = rate;
                    else
                        status_text("Flight SIL: SIM_TLM_HZ range is 1 to 10");
                    parameter(CORE_PARAMETER_COUNT);
                    continue;
                }
                const int index = parameter_store_find_exposed((const char *)(payload + 6));
                if (index >= 0) {
                    if (payload[22] != 9 || !isfinite(read_float(payload)))
                        status_text("Flight: parameter type must be REAL32");
                    else if (!parameter_store_set((unsigned)index, read_float(payload)))
                        status_text("Flight: parameter rejected (range or armed)");
                    parameter((unsigned)parameter_store_exposed_ordinal((unsigned)index));
                } else {
                    status_text("Flight: parameter is not an exposed tuning parameter");
                }
            } else if (id != PARAM_REQUEST_LIST && id != PARAM_REQUEST_READ && id != PARAM_SET) {
                receive_mission(id, payload, system, component, now_us, navigation, home, fms, actions);
            }
        }
    }
}

static void publish_calibration_status(uint64_t now_us)
{
    static uint8_t last_state, last_kind, last_faces;
    static uint32_t last_started;
    static uint64_t last_publish;
    SensorCalibrationStatus status;
    sensor_calibration_get_status(&status);
    if (status.state == SENSOR_CAL_IDLE)
        return;
    const bool changed = status.state != last_state || status.kind != last_kind ||
                         status.face_mask != last_faces || status.started_us != last_started;
    if (!changed && (status.state != SENSOR_CAL_RUNNING || now_us - last_publish < 1000000u))
        return;
    char message[48];
    snprintf(message, sizeof(message), "FCAL %u %u %u %u", (unsigned)status.kind,
             (unsigned)status.state, (unsigned)status.face_mask,
             (unsigned)(status.progress_permille / 10u));
    status_text(message);
    if (status.state == SENSOR_CAL_DONE || status.state == SENSOR_CAL_FAILED)
        status_text(status.message);
    last_state = status.state;
    last_kind = status.kind;
    last_faces = status.face_mask;
    last_started = status.started_us;
    last_publish = now_us;
}

void ground_station_step(uint64_t now_us, const NavigationSnapshot *navigation,
                         const FmsStateSnapshot *fms, const LandedStateSnapshot *landed,
                         const ProductHomeTargetSnapshot *home,
                         const MissionStatus *mission, const PreArmCheckResult *prearm,
                         const GroundStationActions *actions)
{
    const uint64_t wall_ms = GetTickCount64();
    const int armed = fms->arm_state == ARM_STATE_ARMED;
    static uint8_t last_landed = UINT8_MAX;
    ground_log_set_armed(armed);
    if (home->validity != PRODUCT_HOME_VALID || now_us >= home->home_expires_at_us ||
        home->origin_id != navigation->origin_id || home->state_epoch != navigation->state_epoch)
        home = NULL;
    publish_calibration_status(now_us);
    if (wall_ms >= next_heartbeat_ms) {
        uint8_t heartbeat[9] = {0};
        wire_u32(heartbeat, flight_mode(fms));
        heartbeat[4] = 2; /* quadrotor */
        heartbeat[5] = 0; /* MAV_AUTOPILOT_GENERIC */
        switch (fms->flight_mode) {
        case FLIGHT_MODE_MANUAL_RATE: heartbeat[6] = 32; break;
        case FLIGHT_MODE_MANUAL_ATTITUDE: heartbeat[6] = 16; break;
        case FLIGHT_MODE_ALT_HOLD:
        case FLIGHT_MODE_POS_HOLD: heartbeat[6] = 8; break;
        case FLIGHT_MODE_AUTO_MISSION:
        case FLIGHT_MODE_AUTO_RETURN:
        case FLIGHT_MODE_AUTO_LAND:
        case FLIGHT_MODE_EMERGENCY_LAND: heartbeat[6] = 4; break;
        }
        if (armed) heartbeat[6] |= 128;
        heartbeat[6] |= 1; /* CUSTOM_MODE_ENABLED */
        heartbeat[7] = (uint8_t)(fms->failsafe_level >= FAILSAFE_ACTION_REQUIRED ? 5 : armed ? 4 : 3);
        heartbeat[8] = 3;
        emit(HEARTBEAT, 50, heartbeat, sizeof(heartbeat));
        current_mode(fms);
        uint8_t status[31] = {0};
        actions->write_sensor_status(status, armed || prearm->state == PREARM_READY);
        /* Simulated 4S pack: 16.8 V, 5 A, 85% remaining. */
        wire_u16(status + 14, 16800);
        wire_u16(status + 16, 500);
        status[30] = 85;
        emit(SYS_STATUS, 124, status, sizeof(status));
        uint8_t rc_channels[42];
        actions->write_rc_channels(rc_channels, (uint32_t)(now_us / 1000u));
        emit(65 /* RC_CHANNELS */, 118, rc_channels, sizeof(rc_channels));

        /* MAVLink wire order starts with the consumed counters, not the ID.
         * This pack is supplied only by the SIL telemetry fixture. */
        uint8_t battery[41] = {0};
        wire_u32(battery, UINT32_MAX);     /* consumed mAh unknown */
        wire_u32(battery + 4, UINT32_MAX); /* consumed hJ unknown */
        wire_u16(battery + 8, 2500);      /* 25.00 C */
        for (unsigned cell = 0; cell < 10; ++cell)
            wire_u16(battery + 10 + 2 * cell, cell < 4 ? 4200 : UINT16_MAX);
        wire_u16(battery + 30, 500);      /* 5 A */
        battery[34] = 1;                 /* MAV_BATTERY_TYPE_LIPO */
        battery[35] = 85;
        battery[40] = 1;                 /* MAV_BATTERY_CHARGE_STATE_OK */
        emit(BATTERY_STATUS, 154, battery, sizeof(battery));
        if (home)
            send_home(home);
        next_heartbeat_ms = wall_ms + 1000u;
        last_landed = UINT8_MAX; /* Repeat the last state for a newly connected GCS. */
    }
    if (landed->state != last_landed) {
        const uint8_t state[2] = {0, landed->state <= LANDED_STATE_LANDING ? landed->state : 0};
        emit(EXTENDED_SYS_STATE, 130, state, sizeof(state));
        last_landed = landed->state;
    }
    const NavStateCore *nav = &navigation->core;
    if (now_us >= next_state) {
        const bool fresh = navigation->header.timestamp_sample_us <= now_us &&
                           now_us < navigation->header.valid_until_us;
        const uint32_t valid = fresh ? nav->valid_mask : 0;
        const float local_altitude = valid & NAV_VALID_VERTICAL_POSITION ? -nav->position_ned_m[2] : NAN;
        const float relative_altitude = home ? local_altitude + home->position_ned_m[2] : NAN;
        const float speed = valid & NAV_VALID_HORIZONTAL_VELOCITY ?
                            hypotf(nav->velocity_ned_mps[0], nav->velocity_ned_mps[1]) : NAN;
        const uint32_t position_mask = NAV_VALID_HORIZONTAL_POSITION | NAV_VALID_VERTICAL_POSITION;
        if ((valid & position_mask) == position_mask) {
            uint8_t position[28] = {0}, gps[30] = {0};
            wire_u32(position, (uint32_t)(now_us / 1000u));
            map_position(position + 4, nav->position_ned_m);
            wire_u32(position + 16, (uint32_t)(int32_t)lround((home ? relative_altitude : local_altitude) * 1000.0));
            for (unsigned axis = 0; axis < 3; ++axis)
                wire_u16(position + 20 + 2 * axis, (uint16_t)(int16_t)lround(nav->velocity_ned_mps[axis] * 100.0));
            wire_u16(position + 26, UINT16_MAX);
            emit(GLOBAL_POSITION_INT, 104, position, sizeof(position));
            wire_u64(gps, now_us);
            memcpy(gps + 8, position + 4, 12);
            wire_u16(gps + 20, 100); /* eph 1.0 m */
            wire_u16(gps + 22, 150); /* epv 1.5 m */
            wire_u16(gps + 24, isfinite(speed) && speed < 655.35f ? (uint16_t)lroundf(speed * 100.0f) : UINT16_MAX);
            float course = atan2f(nav->velocity_ned_mps[1], nav->velocity_ned_mps[0]) * (180.0f / 3.14159265359f);
            if (course < 0.0f) course += 360.0f;
            wire_u16(gps + 26, speed >= 0.5f && isfinite(course) ?
                     (uint16_t)(lroundf(course * 100.0f) % 36000L) : UINT16_MAX);
            gps[28] = 3;  /* Simulated 3D fix. */
            gps[29] = 16; /* Simulated satellite count. */
            emit(GPS_RAW_INT, 24, gps, sizeof(gps));

            uint8_t gps_status[101] = {0};
            gps_status[0] = 16;
            for (unsigned sv = 0; sv < 16; ++sv) {
                gps_status[1 + sv] = (uint8_t)(sv + 1);  /* PRN */
                gps_status[21 + sv] = 1;                 /* used in solution */
                gps_status[41 + sv] = 45;                /* elevation */
                gps_status[61 + sv] = (uint8_t)(sv * 22);/* azimuth */
                gps_status[81 + sv] = 40;                /* SNR */
            }
            emit(GPS_STATUS, 23, gps_status, sizeof(gps_status));
        }
        if (valid & NAV_VALID_ATTITUDE) {
            uint8_t attitude[28] = {0};
            const float *q = nav->q_nb_wxyz;
            const float pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
            wire_u32(attitude, (uint32_t)(now_us / 1000u));
            wire_float(attitude + 4, atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                                           1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])));
            wire_float(attitude + 8, asinf(fminf(1.0f, fmaxf(-1.0f, pitch))));
            wire_float(attitude + 12, atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                                            1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])));
            for (unsigned axis = 0; axis < 3; ++axis)
                wire_float(attitude + 16 + 4 * axis, valid & NAV_VALID_BODY_RATE ?
                           nav->body_rate_frd_radps[axis] : NAN);
            emit(ATTITUDE, 39, attitude, sizeof(attitude));
        }
        uint8_t hud[20] = {0};
        wire_float(hud, NAN);
        wire_float(hud + 4, speed);
        wire_float(hud + 8, local_altitude); /* The SIL map anchor is at 0 m AMSL. */
        wire_float(hud + 12, valid & NAV_VALID_VERTICAL_VELOCITY ? -nav->velocity_ned_mps[2] : NAN);
        emit(74, 20, hud, sizeof(hud)); /* VFR_HUD */
        uint8_t altitude[MAVLINK_ALTITUDE_LEN];
        wire_altitude(altitude, fresh ? navigation->header.timestamp_sample_us : now_us,
                      local_altitude, local_altitude, relative_altitude);
        emit(MAVLINK_ALTITUDE_ID, MAVLINK_ALTITUDE_CRC, altitude, sizeof(altitude));
        next_state = now_us + (uint64_t)(1000000.0f / telemetry_rate_hz);
    }
    if (pending.request_id && now_us >= pending.deadline_us) {
        if (pending.plan_operation) {
            mission_ack(pending.system, pending.component, MISSION_ERROR, 0);
            upload.count = 0;
            pending.plan_operation = 0;
        } else if (pending.route_operation) {
            route_failed(MAV_FAILED, 6);
        } else {
            command_ack(pending.command, MAV_FAILED, 0, pending.system, pending.component);
        }
        pending.request_id = 0;
    }
    if (pending.request_id && !pending.plan_operation && !pending.route_operation &&
        wall_ms >= pending.progress_at_ms) {
        command_ack(pending.command, MAV_IN_PROGRESS, 0, pending.system, pending.component);
        pending.progress_at_ms = wall_ms + 1000u;
    }
    if (upload.count && !pending.request_id && wall_ms >= upload.retry_at_ms) {
        if (upload.retries++ < 4) {
            request_mission_item(wall_ms);
        } else {
            mission_ack(upload.system, upload.component, MISSION_ERROR, upload.type);
            upload.count = 0;
        }
    }
    if (actions) route_step(now_us, actions, navigation, fms, landed, mission, prearm);
    if (plan_count && (mission->mission_state != reported_mission_state ||
                       mission->current_item_id != reported_waypoint ||
                       route.phase != route.reported_phase)) {
        const unsigned waypoint = mission->current_item_id;
        if (reported_waypoint && (waypoint > reported_waypoint ||
            (mission->mission_state == MISSION_STATE_COMPLETED &&
             mission->result_code == MISSION_RESULT_SUCCESS)))
            mission_reached(reported_waypoint - 1u);
        const uint8_t states[] = {1, 2, 3, 4, 0, 5}; /* MAVLink MISSION_STATE */
        uint8_t current[6] = {0};
        unsigned seq = waypoint ? waypoint - 1u : 0u;
        if (mission->mission_state == MISSION_STATE_COMPLETED)
            seq = plan_count - 1u;
        wire_u16(current + 2, (uint16_t)plan_count);
        if (mission->mission_state < sizeof(states)) current[4] = states[mission->mission_state];
        current[5] = fms->flight_mode == FLIGHT_MODE_AUTO_MISSION ? 1 : 2;
        if (route_starting() || route.phase == ROUTE_FINISH || route.phase == ROUTE_DISARM) {
            current[4] = 3; /* Active, including the actions around FMS waypoints. */
            current[5] = 1;
        } else if (route.phase == ROUTE_FAILED) {
            current[4] = 0;
        }
        wire_u16(current, (uint16_t)seq);
        emit(MISSION_CURRENT, 28, current, sizeof(current));
        reported_waypoint = waypoint;
        reported_mission_state = mission->mission_state;
        route.reported_phase = route.phase;
    }
    receive(now_us, actions, navigation, home, fms, mission);
    for (unsigned sent = 0; sent < 4 && next_parameter < PARAMETER_COUNT; ++sent)
        parameter(next_parameter++);
    ground_log_poll(emit);
}
