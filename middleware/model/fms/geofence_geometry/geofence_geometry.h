#ifndef GEOFENCE_GEOMETRY_H
#define GEOFENCE_GEOMETRY_H

#include "ports/port_api.h"

/* Geometry only: callers validate the snapshot header, frame and epochs. */
uint8_t fence_geometry_valid(const ProductGeofenceSnapshot *fence);
/* Signed horizontal clearance after the configured horizontal margin. */
float fence_clearance(const ProductGeofenceSnapshot *fence, float north, float east);
uint8_t fence_point_valid(const ProductGeofenceSnapshot *fence,
                             const float position_ned[3], float home_down_m);

#endif
