#ifndef FLIGHT_SIL_GROUND_STATION_H
#define FLIGHT_SIL_GROUND_STATION_H

#include "fms_contract/fms_contract.h"

int ground_station_open(uint16_t port);
void ground_station_close(void);
void ground_station_apply_config(ProductControllerConfig *config);
typedef struct {
    uint32_t (*command)(uint16_t command, float argument);
    uint32_t (*set_home)(const float target_ned[3]);
    uint32_t (*commit_mission)(const ProductMissionPayload *mission);
    Status (*replace_geofences)(const ProductGeofenceSnapshot *fences, uint16_t count);
    void (*write_sensor_status)(uint8_t *payload, int prearm_ready);
    bool (*send_crsf_bind)(void);
    void (*write_rc_channels)(uint8_t *payload, uint32_t now_ms);
} GroundStationActions;
void ground_station_step(uint64_t now_us, const NavigationSnapshot *navigation,
                         const FmsStateSnapshot *fms, const LandedStateSnapshot *landed,
                         const ProductHomeTargetSnapshot *home,
                         const MissionStatus *mission, const PreArmCheckResult *prearm,
                         const GroundStationActions *actions);
void ground_station_ack(const CommandAck *ack);

#endif
