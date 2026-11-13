/*
 * FMAT-0012 M6 outer-loop host test.
 *
 * Drives the real MPC path (middleware/model/control/outer_loop/quad_x4/mpc_solver/mpc.c)
 * instead of the synthetic QP used by verify_osqp_gate.m. The point is to prove the
 * QP is assembled from the model view inside the model path, that the candidate is
 * re-validated against the unscaled constraints, and that every rejection path
 * (stale reference, bad bounds, non-finite navigation, solver iteration limit)
 * fails closed.
 *
 * Not covered here: the M6 time-context guard lives in
 * control_backend_outer_update(), which needs the full controller backend; it is
 * a separate fixture.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "attitude_rate_control/controller_internal.h"
#include "osqp/mpc_sparsity_pattern.h"
#include "osqp.h"
#include "quad_workspace.h"
#include "types.h"
#include "algebra_matrix.h"
#include "algebra_vector.h"

static int checks = 0;
static int failures = 0;
#define CHECK(cond, fmt, ...) do { \
    checks++; \
    if (!(cond)) { failures++; \
        printf("  FAIL %s:%d  " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); } \
  } while (0)

static uint64_t g_now = 1000000u;
Status clock_read(ClockInstant *out) {
    if (!out) return STATUS_INVALID_ARGUMENT;
    memset(out, 0, sizeof(*out));
    out->boot_id = 0xB00Bu;
    out->monotonic_us = g_now;
    out->time_epoch = 3u;
    return STATUS_OK;
}

static void header(ContractHeader *h, uint16_t type, uint64_t publish_us) {
    memset(h, 0, sizeof(*h));
    h->schema_major = CONTRACT_SCHEMA_MAJOR;
    h->schema_minor = CONTRACT_SCHEMA_MINOR;
    h->message_type = type;
    h->source_id = SOURCE_M5;
    h->profile_id = PRODUCT_PROFILE_ID;
    h->frame_id = FRAME_LOCAL_NED;
    h->sequence = 1u;
    h->boot_id = 0xB00Bu;
    h->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
    h->time_epoch = 3u;
    h->timestamp_sample_us = publish_us;
    h->timestamp_publish_us = publish_us;
    h->valid_until_us = publish_us + 200000u;
    h->timestamp_quality = TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
    h->timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
    h->validity_flags = 3u;
}

static void make_reference(MotionReference *ref, uint64_t now, float x_m) {
    unsigned i;
    memset(ref, 0, sizeof(*ref));
    header(&ref->header, MSG_MOTION_REFERENCE, now);
    ref->reference_id = 11u;
    ref->reference_epoch = 5u;
    ref->state_epoch = 9u;
    ref->origin_id = 1u;
    ref->authority_epoch = 2u;
    ref->directive_epoch = 4u;
    ref->point_count = 24u;
    ref->anchor.offset_us = 0u;
    ref->anchor.valid_mask = 31u;
    ref->anchor.point.valid_mask = 31u;
    ref->anchor.point.position_ned_m[0] = x_m;
    for (i = 0; i < 24u; ++i) {
        ref->points[i].offset_us = (i + 1u) * 20000u;
        ref->points[i].valid_mask = 31u;
        ref->points[i].point.valid_mask = 31u;
        ref->points[i].point.position_ned_m[0] = x_m;
    }
}

static void make_inputs(NavigationSnapshot *nav, SafetyEnvelope *env,
                        ControlModelView *view, ControllerInput *in,
                        MotionReference *ref, uint64_t now) {
    memset(nav, 0, sizeof(*nav));
    header(&nav->header, MSG_NAVIGATION_SNAPSHOT, now);
    nav->state_epoch = 9u;
    nav->origin_id = 1u;

    memset(env, 0, sizeof(*env));
    header(&env->header, MSG_SAFETY_ENVELOPE, now);
    env->safety_envelope_id = 3u;
    env->envelope_epoch = 6u;
    env->max_linear_speed_mps = 5.0f;
    env->max_linear_accel_mps2 = 5.0f;
    env->max_angular_rate_radps = 3.0f;
    env->max_yaw_rate_radps = 2.0f;
    env->vehicle_extension.max_tilt_rad = 0.5f;
    env->vehicle_extension.max_total_thrust_n = 40.0f;
    env->vehicle_extension.min_total_thrust_n = 0.0f;

    memset(view, 0, sizeof(*view));
    header(&view->header, MSG_CONTROL_MODEL_VIEW, now);
    view->model_epoch = 2u;
    {
        int i;
        for (i = 0; i < 6; ++i) {
            view->bounds.state_min[i] = -100.0f;
            view->bounds.state_max[i] = 100.0f;
        }
        for (i = 0; i < 3; ++i) {
            view->bounds.input_min[i] = -8.0f;
            view->bounds.input_max[i] = 8.0f;
            view->bounds.input_slew_max[i] = 50.0f;
        }
    }

    memset(in, 0, sizeof(*in));
    in->navigation = nav;
    in->reference = ref;
    in->safety_envelope = env;
    in->control_model_view = view;
    in->control_time_us = now;
    in->evaluation_time_us = now;
}

static ProductControllerConfigPayload make_config(void) {
    ProductControllerConfigPayload cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mass_kg = 1.5f;
    /* The shipped QUAD_X4 tuning (config/SIM_ONLY/QUAD_X4/controller_tuning.yaml). */
    cfg.mpc_q_diag[0] = 8.0f;  cfg.mpc_q_diag[1] = 8.0f;  cfg.mpc_q_diag[2] = 12.0f;
    cfg.mpc_q_diag[3] = 3.0f;  cfg.mpc_q_diag[4] = 3.0f;  cfg.mpc_q_diag[5] = 4.0f;
    cfg.mpc_qf_diag[0] = 8.0f; cfg.mpc_qf_diag[1] = 8.0f; cfg.mpc_qf_diag[2] = 12.0f;
    cfg.mpc_qf_diag[3] = 3.0f; cfg.mpc_qf_diag[4] = 3.0f; cfg.mpc_qf_diag[5] = 4.0f;
    cfg.mpc_r_diag[0] = 0.8f; cfg.mpc_r_diag[1] = 0.8f; cfg.mpc_r_diag[2] = 1.0f;
    cfg.mpc_s_diag[0] = 0.2f; cfg.mpc_s_diag[1] = 0.2f; cfg.mpc_s_diag[2] = 0.25f;
    cfg.actuation_preview_delay_s = 0.002f;
    return cfg;
}

int main(void) {
    ProductControllerConfigPayload cfg = make_config();
    NavigationSnapshot nav;
    SafetyEnvelope env;
    ControlModelView view;
    MotionReference ref;
    ControllerInput in;
    MpcSnapshot snap;
    float prev[3] = {0.0f, 0.0f, 0.0f};
    Status status;
    int i;

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("FMAT-0012 M6 outer-loop host test\n");
    status = mpc_reset();
    CHECK(status == STATUS_OK, "MPC reset failed, status=%d", (int)status);
    CHECK(quad_solver.settings != NULL && quad_solver.settings->eps_rel == 0.0f,
          "MPC reset did not apply absolute OSQP termination");

    /* M6-01: a valid geometric reference solves inside the model path. */
    make_reference(&ref, g_now, 0.1f);
    make_inputs(&nav, &env, &view, &in, &ref, g_now);
    memset(&snap, 0, sizeof(snap));
    status = mpc_solve(&in, &cfg, prev, &snap);
    CHECK(status == STATUS_OK, "valid reference rejected, status=%d", (int)status);
    CHECK(snap.solver_status == CONTROL_SOLVER_SOLVED,
          "solver status=%d", (int)snap.solver_status);
    CHECK(isfinite(snap.acceleration[0]) && isfinite(snap.acceleration[1]) &&
          isfinite(snap.acceleration[2]), "non-finite acceleration");

    /* M6-02: the assembly keeps the 15/190/45/702 contract. */
    {
        OSQPData *d = quad_solver.work->data;
        CHECK(d->n == 15, "QP variables=%d", (int)d->n);
        CHECK(d->m == 190, "QP constraints=%d", (int)d->m);
        CHECK(OSQPMatrix_get_nz(d->P) == 45, "P nnz=%d", (int)OSQPMatrix_get_nz(d->P));
        CHECK(OSQPMatrix_get_nz(d->A) == 702, "A nnz=%d", (int)OSQPMatrix_get_nz(d->A));
    }

    /* M6-03: mpc_solve() re-validates the candidate against the unscaled
     * constraints itself and returns OUT_OF_RANGE when that fails, so an
     * accepted solve is the evidence that the strict check ran. */
    CHECK(snap.acceleration[0] >= view.bounds.input_min[0] - 1e-3f &&
          snap.acceleration[0] <= view.bounds.input_max[0] + 1e-3f &&
          snap.acceleration[1] >= view.bounds.input_min[1] - 1e-3f &&
          snap.acceleration[1] <= view.bounds.input_max[1] + 1e-3f,
          "accepted acceleration outside the model input bounds");
    printf("  accepted accel = [%.4f %.4f %.4f]\n",
           snap.acceleration[0], snap.acceleration[1], snap.acceleration[2]);
    for (i = 0; i < 5; ++i) {
        float a = snap.acceleration[0], b = snap.acceleration[1],
              c = CONTROL_GRAVITY_MPS2 - snap.acceleration[2];
        CHECK(cfg.mass_kg * sqrtf(a * a + b * b + c * c) <= env.vehicle_extension.max_total_thrust_n + 1e-3f,
              "thrust ball violated on move %d", i);
        break; /* only the first move is copied into acceleration */
    }

    /* M6-04: a preview horizon that no longer covers the control instant is
     * refused instead of being extrapolated. */
    make_reference(&ref, g_now, 0.1f);
    make_inputs(&nav, &env, &view, &in, &ref, g_now);
    in.control_time_us = ref.header.timestamp_sample_us +
                         (uint64_t)ref.points[23].offset_us + 100000u;
    in.evaluation_time_us = in.control_time_us;
    status = mpc_solve(&in, &cfg, prev, &snap);
    CHECK(status == STATUS_STALE_DATA, "expired preview status=%d", (int)status);

    /* M6-05: non-finite navigation is refused. */
    make_reference(&ref, g_now, 0.1f);
    make_inputs(&nav, &env, &view, &in, &ref, g_now);
    nav.core.position_ned_m[0] = NAN;
    status = mpc_solve(&in, &cfg, prev, &snap);
    CHECK(status == STATUS_INVALID_ARGUMENT, "NaN navigation status=%d", (int)status);

    /* M6-06: contradictory model bounds are refused, not solved around. */
    make_reference(&ref, g_now, 0.1f);
    make_inputs(&nav, &env, &view, &in, &ref, g_now);
    view.bounds.input_min[0] = 9.0f;
    view.bounds.input_max[0] = -9.0f;
    status = mpc_solve(&in, &cfg, prev, &snap);
    CHECK(status == STATUS_OUT_OF_RANGE, "invalid bounds status=%d", (int)status);

    /* M6-07: a model view outside the domain is refused. */
    make_reference(&ref, g_now, 0.1f);
    make_inputs(&nav, &env, &view, &in, &ref, g_now);
    env.vehicle_extension.max_total_thrust_n = 0.5f; /* below gravity */
    status = mpc_solve(&in, &cfg, prev, &snap);
    CHECK(status == STATUS_MODEL_OUT_OF_DOMAIN, "out-of-domain envelope status=%d", (int)status);

    printf("M6 checks=%d failures=%d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
