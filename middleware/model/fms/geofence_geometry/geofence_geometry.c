#include <math.h>
#include <stddef.h>
#include "geofence_geometry/geofence_geometry.h"

static double cross(const float a[2], const float b[2], const float c[2])
{
    return ((double)b[0] - a[0]) * ((double)c[1] - a[1]) -
           ((double)b[1] - a[1]) * ((double)c[0] - a[0]);
}

static uint8_t between(const float a[2], const float b[2], const float p[2])
{
    return (uint8_t)(p[0] >= fminf(a[0], b[0]) && p[0] <= fmaxf(a[0], b[0]) &&
                     p[1] >= fminf(a[1], b[1]) && p[1] <= fmaxf(a[1], b[1]));
}

static uint8_t intersects(const float a[2], const float b[2],
                           const float c[2], const float d[2])
{
    double x = cross(a, b, c), y = cross(a, b, d);
    double z = cross(c, d, a), w = cross(c, d, b);
    if (((x > 0 && y < 0) || (x < 0 && y > 0)) &&
        ((z > 0 && w < 0) || (z < 0 && w > 0))) return 1u;
    return (uint8_t)((x == 0 && between(a, b, c)) || (y == 0 && between(a, b, d)) ||
                     (z == 0 && between(c, d, a)) || (w == 0 && between(c, d, b)));
}

uint8_t fence_geometry_valid(const ProductGeofenceSnapshot *f)
{
    uint8_t i, j;
    double area = 0;
    if (f == NULL || f->validity != PRODUCT_GEOFENCE_VALID ||
        f->frame_id != FRAME_LOCAL_NED || f->vertex_count < 3u ||
        f->vertex_count > PRODUCT_MAX_GEOFENCE_VERTICES ||
        (f->kind != PRODUCT_GEOFENCE_KEEP_IN && f->kind != PRODUCT_GEOFENCE_KEEP_OUT) ||
        !isfinite(f->horizontal_margin_m) || f->horizontal_margin_m < 0 ||
        !isfinite(f->vertical_margin_m) || f->vertical_margin_m < 0 ||
        !isfinite(f->min_height_above_home_m) || !isfinite(f->max_height_above_home_m) ||
        f->max_height_above_home_m - f->vertical_margin_m <=
            f->min_height_above_home_m + f->vertical_margin_m) return 0u;
    for (i = 0; i < f->vertex_count; ++i) {
        const float *a = f->vertices_ne_m[i];
        const float *b = f->vertices_ne_m[(i + 1u) % f->vertex_count];
        const float *c = f->vertices_ne_m[(i + 2u) % f->vertex_count];
        if (!isfinite(a[0]) || !isfinite(a[1]) || !isfinite(b[0]) || !isfinite(b[1]) ||
            (a[0] == b[0] && a[1] == b[1])) return 0u;
        /* Adjacent edges may share a vertex, but cannot double back. */
        if (cross(a, b, c) == 0 &&
            (((double)a[0] - b[0]) * ((double)c[0] - b[0]) +
             ((double)a[1] - b[1]) * ((double)c[1] - b[1])) > 0) return 0u;
        area += (double)a[0] * b[1] - (double)a[1] * b[0];
        for (j = i + 1u; j < f->vertex_count; ++j) {
            uint8_t ni = (uint8_t)((i + 1u) % f->vertex_count);
            uint8_t nj = (uint8_t)((j + 1u) % f->vertex_count);
            if (ni == j || nj == i) continue;
            if (intersects(a, b, f->vertices_ne_m[j], f->vertices_ne_m[nj])) return 0u;
        }
    }
    return (uint8_t)(isfinite(area) && fabs(area) > 1e-6);
}

float fence_clearance(const ProductGeofenceSnapshot *f, float x, float y)
{
    uint8_t i, j, inside = 0;
    double best = INFINITY;
    for (i = 0, j = (uint8_t)(f->vertex_count - 1u); i < f->vertex_count; j = i++) {
        double ax = f->vertices_ne_m[j][0], ay = f->vertices_ne_m[j][1];
        double bx = f->vertices_ne_m[i][0], by = f->vertices_ne_m[i][1];
        double dx = bx - ax, dy = by - ay, len = dx * dx + dy * dy;
        double u = len > 0 ? (((double)x - ax) * dx + ((double)y - ay) * dy) / len : 0;
        double ex, ey, distance;
        u = fmax(0, fmin(1, u));
        ex = x - ax - u * dx;
        ey = y - ay - u * dy;
        distance = sqrt(ex * ex + ey * ey);
        if (distance < best) best = distance;
        if (((ay > y) != (by > y)) && x < (bx - ax) * (y - ay) / (by - ay) + ax)
            inside ^= 1u;
    }
    if (f->kind == PRODUCT_GEOFENCE_KEEP_OUT) inside ^= 1u;
    return (float)((inside ? best : -best) - f->horizontal_margin_m);
}

uint8_t fence_point_valid(const ProductGeofenceSnapshot *f, const float p[3], float home_d)
{
    float height = home_d - p[2];
    return (uint8_t)(isfinite(p[0]) && isfinite(p[1]) && isfinite(height) &&
        height >= f->min_height_above_home_m + f->vertical_margin_m &&
        height <= f->max_height_above_home_m - f->vertical_margin_m &&
        fence_clearance(f, p[0], p[1]) >= 0);
}
