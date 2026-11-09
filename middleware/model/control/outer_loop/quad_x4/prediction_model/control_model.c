/* QUAD fixed ZOH double integrator; no physical-plant simulation is linked. */
#include "contract_checks/algorithm_checks.h"
#include "control_backend.h"

#define QUAD_LTI_ID 0x1501u
Status control_backend_model_get_descriptor(ControlModelDescriptor *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  out->model_id = QUAD_LTI_ID;
  out->model_version = 1;
  out->state_layout_id = PRODUCT_CONTROL_MODEL_STATE_LAYOUT_ID;
  out->input_layout_id = PRODUCT_CONTROL_MODEL_INPUT_LAYOUT_ID;
  out->model_hash = 0x514C5431u;
  out->model_kind = CONTROL_MODEL_KIND_LTI;
  return STATUS_OK;
}
Status control_backend_model_check_domain(const OperatingPoint *op) {
  if (!op || !finite_array(op->state.state, 6) ||
      !finite_array(op->input.input, 3) ||
      !isfinite(op->operating_speed_mps) || !isfinite(op->operating_voltage_v))
    return STATUS_INVALID_ARGUMENT;
  if (op->operating_speed_mps < 0 || op->operating_speed_mps > 30 ||
      op->operating_voltage_v <= 0 || op->operating_voltage_v > 60)
    return STATUS_MODEL_OUT_OF_DOMAIN;
  return STATUS_OK;
}
Status control_backend_model_get_linear_model(const OperatingPoint *op, float dt,
                                      ControlLinearModel *out) {
  unsigned i;
  if (!out || !isfinite(dt) || fabsf(dt - .02f) > 1e-7f)
    return STATUS_INVALID_ARGUMENT;
  if (control_backend_model_check_domain(op) != STATUS_OK)
    return STATUS_MODEL_OUT_OF_DOMAIN;
  memset(out, 0, sizeof(*out));
  for (i = 0; i < 6; ++i)
    out->A[6 * i + i] = 1;
  for (i = 0; i < 3; ++i) {
    out->A[6 * i + i + 3] = dt;
    out->B[3 * i + i] = .5f * dt * dt;
    out->B[3 * (i + 3) + i] = dt;
  }
  out->sample_time_s = dt;
  return STATUS_OK;
}
Status control_backend_model_get_constraints(const SafetyEnvelope *env,
                                     ControlModelBounds *out) {
  unsigned i;
  const ProductSafetyEnvelopePayload *e;
  if (!env || !out)
    return STATUS_INVALID_ARGUMENT;
  e = &env->vehicle_extension;
  if ((e->valid_mask & 7u) != 7u)
    return STATUS_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  for (i = 0; i < 3; ++i) {
    if (!(e->max_velocity_ned_mps[i] > 0) ||
        !(e->max_acceleration_ned_mps2[i] > 0) ||
        !(e->max_jerk_ned_mps3[i] > 0) ||
        !isfinite(e->max_velocity_ned_mps[i]) ||
        !isfinite(e->max_acceleration_ned_mps2[i]) ||
        !isfinite(e->max_jerk_ned_mps3[i]))
      return STATUS_INVALID_ARGUMENT;
    /* This ABI supplies height bounds relative to Home, while state is relative
       to Origin. Do not silently apply one datum as the other. M4/M5 own the
       spatial fence; this model has no horizontal position restriction. */
    out->state_min[i] = -1e30f;
    out->state_max[i] = 1e30f;
    out->state_min[i + 3] = -e->max_velocity_ned_mps[i];
    out->state_max[i + 3] = e->max_velocity_ned_mps[i];
    out->input_min[i] = -e->max_acceleration_ned_mps2[i];
    out->input_max[i] = e->max_acceleration_ned_mps2[i];
    out->input_slew_max[i] = e->max_jerk_ned_mps3[i];
  }
  return STATUS_OK;
}
Status control_backend_model_validate(const ControlModelState *x,
                              const ControlModelInput *u,
                              const OperatingPoint *op) {
  if (!x || !u || !finite_array(x->state, 6) ||
      !finite_array(u->input, 3))
    return STATUS_INVALID_ARGUMENT;
  return control_backend_model_check_domain(op);
}
Status control_backend_model_predict(const ControlModelState *x,
                             const ControlModelInput *u, float dt,
                             ControlModelState *next) {
  unsigned i;
  ControlModelState result;
  if (!x || !u || !next || !finite_array(x->state, 6) ||
      !finite_array(u->input, 3) || !isfinite(dt) ||
      fabsf(dt - .02f) > 1e-7f)
    return STATUS_INVALID_ARGUMENT;
  for (i = 0; i < 3; ++i) {
    result.state[i] =
        x->state[i] + dt * x->state[i + 3] + .5f * dt * dt * u->input[i];
    result.state[i + 3] = x->state[i + 3] + dt * u->input[i];
  }
  if (!finite_array(result.state, 6))
    return STATUS_NUMERIC_ERROR;
  *next = result;
  return STATUS_OK;
}
Status control_backend_model_get_view(const OperatingPoint *op, float dt,
                              const SafetyEnvelope *env,
                              ControlModelView *out) {
  ControlModelView result;
  Status s;
  if (!env || !out)
    return STATUS_INVALID_ARGUMENT;
  memset(&result, 0, sizeof(result));
  s = control_backend_model_get_linear_model(op, dt, &result.linear_model);
  if (s)
    return s;
  s = control_backend_model_get_constraints(env, &result.bounds);
  if (s)
    return s;
  control_backend_model_get_descriptor(&result.descriptor);
  result.header = env->header;
  result.header.message_type = MSG_CONTROL_MODEL_VIEW;
  result.header.frame_id = FRAME_LOCAL_NED;
  result.model_epoch = 1;
  result.operating_point_id = env->envelope_epoch;
  *out = result;
  return STATUS_OK;
}
