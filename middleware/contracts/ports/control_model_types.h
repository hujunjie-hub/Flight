/*
 * Flight public contract - ControlModelPort dimension types.
 *
 * Authority: spec/02_接口契约与公共数据规范.md (V1.9.1-GLOBAL-AUDIT-R3),
 *            section 4.5 (OperatingPoint/Control Model) and section 4.7.
 *
 * Dimensions are compile-time constants of the selected Product Contract:
 *   state[PRODUCT_CONTROL_NX], input[PRODUCT_CONTROL_NU], A[NX*NX], B[NX*NU].
 * Matrices are row-major float32 with a fixed stride.
 */
#ifndef CONTROL_MODEL_TYPES_H
#define CONTROL_MODEL_TYPES_H

#include "product_contract/product_contract.h"

EXTERN_C_BEGIN

typedef struct {
  float state[PRODUCT_CONTROL_NX];
} ControlModelState;

typedef struct {
  float input[PRODUCT_CONTROL_NU];
} ControlModelInput;

typedef struct {
  float A[PRODUCT_CONTROL_NX * PRODUCT_CONTROL_NX];
  float B[PRODUCT_CONTROL_NX * PRODUCT_CONTROL_NU];
  float c[PRODUCT_CONTROL_NX];
  float sample_time_s;
} ControlLinearModel;

typedef struct {
  float state_min[PRODUCT_CONTROL_NX];
  float state_max[PRODUCT_CONTROL_NX];
  float input_min[PRODUCT_CONTROL_NU];
  float input_max[PRODUCT_CONTROL_NU];
  float input_slew_max[PRODUCT_CONTROL_NU];
} ControlModelBounds;

typedef struct {
  ControlModelState state;
  ControlModelInput input;
  float operating_speed_mps;
  float operating_voltage_v;
} OperatingPoint;

typedef struct {
  uint32_t model_id;
  uint32_t model_version;
  uint32_t state_layout_id;
  uint32_t input_layout_id;
  uint32_t model_hash;
  uint8_t model_kind;
  uint8_t reserved[3];
} ControlModelDescriptor;

/* ControlModelDescriptor.model_kind (spec 02 section 4). */
#define CONTROL_MODEL_KIND_LTI      1u
#define CONTROL_MODEL_KIND_LTV      2u
#define CONTROL_MODEL_KIND_LPV      3u
#define CONTROL_MODEL_KIND_NONLINEAR 4u

EXTERN_C_END

#endif /* CONTROL_MODEL_TYPES_H */
