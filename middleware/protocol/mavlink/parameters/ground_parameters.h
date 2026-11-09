/* Generated from the five YAML files by generate_ground_config.py; do not edit. */
#ifndef GROUND_PARAMETERS_H
#define GROUND_PARAMETERS_H

#include "product_contract/product_contract.h"

typedef struct {
    char name[16];
    float value;
    float minimum;
    float maximum;
    unsigned char writable;
} GroundParameter;

static const GroundParameter ground_parameters[] = {
    {"VEH_MASS", 1.500000000e+00f, 1.000000000e-01f, 5.000000000e+01f, 1},
    {"CTL_ACT_DLY", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e-02f, 1},
    {"CTL_FB_KD_X", 2.000000000e+00f, 0.000000000e+00f, 5.000000000e+01f, 1},
    {"CTL_FB_KD_Y", 2.000000000e+00f, 0.000000000e+00f, 5.000000000e+01f, 1},
    {"CTL_FB_KD_Z", 2.000000000e+00f, 0.000000000e+00f, 5.000000000e+01f, 1},
    {"CTL_FB_KP_X", 2.000000000e+00f, 0.000000000e+00f, 5.000000000e+01f, 1},
    {"CTL_FB_KP_Y", 2.000000000e+00f, 0.000000000e+00f, 5.000000000e+01f, 1},
    {"CTL_FB_KP_Z", 2.000000000e+00f, 0.000000000e+00f, 5.000000000e+01f, 1},
    {"CTL_HOVER_THR", 5.000000000e-01f, 5.000000000e-02f, 9.500000000e-01f, 1},
    {"CTL_MPC_Q0", 8.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_Q1", 8.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_Q2", 1.200000000e+01f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_Q3", 3.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_Q4", 3.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_Q5", 4.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_QF0", 8.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_QF1", 8.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_QF2", 1.200000000e+01f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_QF3", 3.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_QF4", 3.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_QF5", 4.000000000e+00f, 1.000000000e-03f, 1.000000000e+03f, 1},
    {"CTL_MPC_R0", 8.000000000e-01f, 0.000000000e+00f, 1.000000000e+03f, 1},
    {"CTL_MPC_R1", 8.000000000e-01f, 0.000000000e+00f, 1.000000000e+03f, 1},
    {"CTL_MPC_R2", 1.000000000e+00f, 0.000000000e+00f, 1.000000000e+03f, 1},
    {"CTL_MPC_S0", 2.000000000e-01f, 1.000000000e-03f, 1.000000000e+02f, 1},
    {"CTL_MPC_S1", 2.000000000e-01f, 1.000000000e-03f, 1.000000000e+02f, 1},
    {"CTL_MPC_S2", 2.500000000e-01f, 1.000000000e-03f, 1.000000000e+02f, 1},
    {"CTL_SEED_POS", 5.000000000e-01f, 1.000000000e-02f, 1.000000000e+02f, 1},
    {"CTL_SEED_VEL", 1.000000000e+00f, 1.000000000e-02f, 1.000000000e+02f, 1},
    {"CTL_RATE_DCUT", 4.000000000e+01f, 1.000000000e+00f, 4.490000000e+02f, 1},
    {"CTL_RATE_ILIM_X", 2.000000000e-01f, 1.000000000e-03f, 1.000000000e+01f, 1},
    {"CTL_RATE_ILIM_Y", 2.000000000e-01f, 1.000000000e-03f, 1.000000000e+01f, 1},
    {"CTL_RATE_ILIM_Z", 2.000000000e-01f, 1.000000000e-03f, 1.000000000e+01f, 1},
    {"CTL_RATE_KAW_X", 5.000000000e+00f, 1.000000000e-02f, 1.000000000e+02f, 1},
    {"CTL_RATE_KAW_Y", 5.000000000e+00f, 1.000000000e-02f, 1.000000000e+02f, 1},
    {"CTL_RATE_KAW_Z", 5.000000000e+00f, 1.000000000e-02f, 1.000000000e+02f, 1},
    {"CTL_RATE_KD_X", 2.000000000e-03f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_RATE_KD_Y", 2.000000000e-03f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_RATE_KD_Z", 2.000000000e-03f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_RATE_KI_X", 5.000000000e-02f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_RATE_KI_Y", 5.000000000e-02f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_RATE_KI_Z", 5.000000000e-02f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_RATE_KP_X", 1.200000000e-01f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_RATE_KP_Y", 1.200000000e-01f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_RATE_KP_Z", 1.200000000e-01f, 0.000000000e+00f, 2.000000000e+01f, 1},
    {"CTL_SO3_X", 6.000000000e+00f, 1.000000000e-01f, 5.000000000e+01f, 1},
    {"CTL_SO3_Y", 6.000000000e+00f, 1.000000000e-01f, 5.000000000e+01f, 1},
    {"CTL_SO3_Z", 6.000000000e+00f, 1.000000000e-01f, 5.000000000e+01f, 1},
    {"VEH_IXX", 2.500000000e-02f, 1.000000000e-03f, 5.000000000e+00f, 1},
    {"VEH_IXY", 0.000000000e+00f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_IXZ", 0.000000000e+00f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_IYY", 2.500000000e-02f, 1.000000000e-03f, 5.000000000e+00f, 1},
    {"VEH_IYZ", 0.000000000e+00f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_IZZ", 4.000000000e-02f, 1.000000000e-03f, 5.000000000e+00f, 1},
    {"VEH_M0_X", 1.800000000e-01f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M0_Y", -1.800000000e-01f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M0_Z", 0.000000000e+00f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M1_X", 1.800000000e-01f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M1_Y", 1.800000000e-01f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M1_Z", 0.000000000e+00f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M2_X", -1.800000000e-01f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M2_Y", 1.800000000e-01f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M2_Z", 0.000000000e+00f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M3_X", -1.800000000e-01f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M3_Y", -1.800000000e-01f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M3_Z", 0.000000000e+00f, -1.000000000e+00f, 1.000000000e+00f, 1},
    {"VEH_M0_YAW", -2.000000000e-02f, -2.000000000e-01f, 2.000000000e-01f, 1},
    {"VEH_M1_YAW", 2.000000000e-02f, -2.000000000e-01f, 2.000000000e-01f, 1},
    {"VEH_M2_YAW", -2.000000000e-02f, -2.000000000e-01f, 2.000000000e-01f, 1},
    {"VEH_M3_YAW", 2.000000000e-02f, -2.000000000e-01f, 2.000000000e-01f, 1},
    {"CAL_GYRO_X", 0.000000000e+00f, -2.000000000e+01f, 2.000000000e+01f, 1},
    {"CAL_GYRO_Y", 0.000000000e+00f, -2.000000000e+01f, 2.000000000e+01f, 1},
    {"CAL_GYRO_Z", 0.000000000e+00f, -2.000000000e+01f, 2.000000000e+01f, 1},
    {"CAL_GYRO_OK", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"CAL_ACC_X", 0.000000000e+00f, -5.000000000e+00f, 5.000000000e+00f, 1},
    {"CAL_ACC_Y", 0.000000000e+00f, -5.000000000e+00f, 5.000000000e+00f, 1},
    {"CAL_ACC_Z", 0.000000000e+00f, -5.000000000e+00f, 5.000000000e+00f, 1},
    {"CAL_ACC_SX", 1.000000000e+00f, 8.000000000e-01f, 1.200000000e+00f, 1},
    {"CAL_ACC_SY", 1.000000000e+00f, 8.000000000e-01f, 1.200000000e+00f, 1},
    {"CAL_ACC_SZ", 1.000000000e+00f, 8.000000000e-01f, 1.200000000e+00f, 1},
    {"CAL_ACC_OK", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"CAL_MAG_X", 0.000000000e+00f, -5.000000000e+02f, 5.000000000e+02f, 1},
    {"CAL_MAG_Y", 0.000000000e+00f, -5.000000000e+02f, 5.000000000e+02f, 1},
    {"CAL_MAG_Z", 0.000000000e+00f, -5.000000000e+02f, 5.000000000e+02f, 1},
    {"CAL_MAG_SX", 1.000000000e+00f, 5.000000000e-01f, 2.000000000e+00f, 1},
    {"CAL_MAG_SY", 1.000000000e+00f, 5.000000000e-01f, 2.000000000e+00f, 1},
    {"CAL_MAG_SZ", 1.000000000e+00f, 5.000000000e-01f, 2.000000000e+00f, 1},
    {"CAL_MAG_OK", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"CAL_LVL_ROLL", 0.000000000e+00f, -1.000000000e+01f, 1.000000000e+01f, 1},
    {"CAL_LVL_PITCH", 0.000000000e+00f, -1.000000000e+01f, 1.000000000e+01f, 1},
    {"CAL_LVL_OK", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"CAL_BARO_P0", 1.013250000e+05f, 8.000000000e+04f, 1.100000000e+05f, 1},
    {"RC_MIN_1", 1.720000000e+02f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_MAX_1", 1.811000000e+03f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_TRIM_1", 9.920000000e+02f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_MIN_2", 1.720000000e+02f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_MAX_2", 1.811000000e+03f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_TRIM_2", 9.920000000e+02f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_MIN_3", 1.720000000e+02f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_MAX_3", 1.811000000e+03f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_TRIM_3", 1.720000000e+02f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_MIN_4", 1.720000000e+02f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_MAX_4", 1.811000000e+03f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_TRIM_4", 9.920000000e+02f, 0.000000000e+00f, 2.047000000e+03f, 1},
    {"RC_REV_1", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"RC_REV_2", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"RC_REV_3", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"RC_REV_4", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"RC_CAL_OK", 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 1},
    {"RC_MODE_CH", 0.000000000e+00f, 0.000000000e+00f, 1.600000000e+01f, 1},
    {"RC_MODE_1", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_MODE_2", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_MODE_3", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_MODE_4", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_MODE_5", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_MODE_6", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_6", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_7", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_8", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_9", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_10", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_11", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_12", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_13", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_14", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_15", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
    {"RC_AUX_16", 0.000000000e+00f, 0.000000000e+00f, 4.000000000e+00f, 1},
};
enum { GROUND_PARAMETER_COUNT = 127 };

static void ground_configure_controller(ProductControllerConfig *config)
{
    config->parameter_hash = 0x63547de6u;
    config->payload.inertia_frd_kgm2[0] = 2.500000000e-02f;
    config->payload.inertia_frd_kgm2[1] = 0.000000000e+00f;
    config->payload.inertia_frd_kgm2[2] = 0.000000000e+00f;
    config->payload.inertia_frd_kgm2[3] = 0.000000000e+00f;
    config->payload.inertia_frd_kgm2[4] = 2.500000000e-02f;
    config->payload.inertia_frd_kgm2[5] = 0.000000000e+00f;
    config->payload.inertia_frd_kgm2[6] = 0.000000000e+00f;
    config->payload.inertia_frd_kgm2[7] = 0.000000000e+00f;
    config->payload.inertia_frd_kgm2[8] = 4.000000000e-02f;
    config->payload.mass_kg = 1.500000000e+00f;
    config->payload.motor_position_frd_m[0][0] = 1.800000000e-01f;
    config->payload.motor_position_frd_m[0][1] = -1.800000000e-01f;
    config->payload.motor_position_frd_m[0][2] = 0.000000000e+00f;
    config->payload.motor_position_frd_m[1][0] = 1.800000000e-01f;
    config->payload.motor_position_frd_m[1][1] = 1.800000000e-01f;
    config->payload.motor_position_frd_m[1][2] = 0.000000000e+00f;
    config->payload.motor_position_frd_m[2][0] = -1.800000000e-01f;
    config->payload.motor_position_frd_m[2][1] = 1.800000000e-01f;
    config->payload.motor_position_frd_m[2][2] = 0.000000000e+00f;
    config->payload.motor_position_frd_m[3][0] = -1.800000000e-01f;
    config->payload.motor_position_frd_m[3][1] = -1.800000000e-01f;
    config->payload.motor_position_frd_m[3][2] = 0.000000000e+00f;
    config->payload.motor_yaw_torque_ratio_m[0] = -2.000000000e-02f;
    config->payload.motor_yaw_torque_ratio_m[1] = 2.000000000e-02f;
    config->payload.motor_yaw_torque_ratio_m[2] = -2.000000000e-02f;
    config->payload.motor_yaw_torque_ratio_m[3] = 2.000000000e-02f;
    config->payload.so3_gain_per_s[0] = 6.000000000e+00f;
    config->payload.so3_gain_per_s[1] = 6.000000000e+00f;
    config->payload.so3_gain_per_s[2] = 6.000000000e+00f;
    config->payload.rate_kp_nm_per_radps[0] = 1.200000000e-01f;
    config->payload.rate_kp_nm_per_radps[1] = 1.200000000e-01f;
    config->payload.rate_kp_nm_per_radps[2] = 1.200000000e-01f;
    config->payload.rate_ki_nm_per_rad[0] = 5.000000000e-02f;
    config->payload.rate_ki_nm_per_rad[1] = 5.000000000e-02f;
    config->payload.rate_ki_nm_per_rad[2] = 5.000000000e-02f;
    config->payload.rate_kd_nm_per_radps2[0] = 2.000000000e-03f;
    config->payload.rate_kd_nm_per_radps2[1] = 2.000000000e-03f;
    config->payload.rate_kd_nm_per_radps2[2] = 2.000000000e-03f;
    config->payload.rate_kaw_per_s[0] = 5.000000000e+00f;
    config->payload.rate_kaw_per_s[1] = 5.000000000e+00f;
    config->payload.rate_kaw_per_s[2] = 5.000000000e+00f;
    config->payload.rate_d_cutoff_hz = 4.000000000e+01f;
    config->payload.rate_integral_limit_nm[0] = 2.000000000e-01f;
    config->payload.rate_integral_limit_nm[1] = 2.000000000e-01f;
    config->payload.rate_integral_limit_nm[2] = 2.000000000e-01f;
    config->payload.manual_hover_throttle_norm = 5.000000000e-01f;
    config->payload.fallback_kp_per_s2[0] = 2.000000000e+00f;
    config->payload.fallback_kp_per_s2[1] = 2.000000000e+00f;
    config->payload.fallback_kp_per_s2[2] = 2.000000000e+00f;
    config->payload.fallback_kd_per_s[0] = 2.000000000e+00f;
    config->payload.fallback_kd_per_s[1] = 2.000000000e+00f;
    config->payload.fallback_kd_per_s[2] = 2.000000000e+00f;
    config->payload.mpc_q_diag[0] = 8.000000000e+00f;
    config->payload.mpc_q_diag[1] = 8.000000000e+00f;
    config->payload.mpc_q_diag[2] = 1.200000000e+01f;
    config->payload.mpc_q_diag[3] = 3.000000000e+00f;
    config->payload.mpc_q_diag[4] = 3.000000000e+00f;
    config->payload.mpc_q_diag[5] = 4.000000000e+00f;
    config->payload.mpc_qf_diag[0] = 8.000000000e+00f;
    config->payload.mpc_qf_diag[1] = 8.000000000e+00f;
    config->payload.mpc_qf_diag[2] = 1.200000000e+01f;
    config->payload.mpc_qf_diag[3] = 3.000000000e+00f;
    config->payload.mpc_qf_diag[4] = 3.000000000e+00f;
    config->payload.mpc_qf_diag[5] = 4.000000000e+00f;
    config->payload.mpc_r_diag[0] = 8.000000000e-01f;
    config->payload.mpc_r_diag[1] = 8.000000000e-01f;
    config->payload.mpc_r_diag[2] = 1.000000000e+00f;
    config->payload.mpc_s_diag[0] = 2.000000000e-01f;
    config->payload.mpc_s_diag[1] = 2.000000000e-01f;
    config->payload.mpc_s_diag[2] = 2.500000000e-01f;
    config->payload.actuation_preview_delay_s = 0.000000000e+00f;
    config->payload.mpc_seed_position_error_limit_m = 5.000000000e-01f;
    config->payload.mpc_seed_velocity_error_limit_mps = 1.000000000e+00f;
}

/* Copy the parameter catalog order into the controller payload. */
static void ground_payload_from_values(ProductControllerConfigPayload *p,
                                      const float *v)
{
    p->mass_kg = v[0];
    p->actuation_preview_delay_s = v[1];
    p->fallback_kd_per_s[0] = v[2];
    p->fallback_kd_per_s[1] = v[3];
    p->fallback_kd_per_s[2] = v[4];
    p->fallback_kp_per_s2[0] = v[5];
    p->fallback_kp_per_s2[1] = v[6];
    p->fallback_kp_per_s2[2] = v[7];
    p->manual_hover_throttle_norm = v[8];
    p->mpc_q_diag[0] = v[9];
    p->mpc_q_diag[1] = v[10];
    p->mpc_q_diag[2] = v[11];
    p->mpc_q_diag[3] = v[12];
    p->mpc_q_diag[4] = v[13];
    p->mpc_q_diag[5] = v[14];
    p->mpc_qf_diag[0] = v[15];
    p->mpc_qf_diag[1] = v[16];
    p->mpc_qf_diag[2] = v[17];
    p->mpc_qf_diag[3] = v[18];
    p->mpc_qf_diag[4] = v[19];
    p->mpc_qf_diag[5] = v[20];
    p->mpc_r_diag[0] = v[21];
    p->mpc_r_diag[1] = v[22];
    p->mpc_r_diag[2] = v[23];
    p->mpc_s_diag[0] = v[24];
    p->mpc_s_diag[1] = v[25];
    p->mpc_s_diag[2] = v[26];
    p->mpc_seed_position_error_limit_m = v[27];
    p->mpc_seed_velocity_error_limit_mps = v[28];
    p->rate_d_cutoff_hz = v[29];
    p->rate_integral_limit_nm[0] = v[30];
    p->rate_integral_limit_nm[1] = v[31];
    p->rate_integral_limit_nm[2] = v[32];
    p->rate_kaw_per_s[0] = v[33];
    p->rate_kaw_per_s[1] = v[34];
    p->rate_kaw_per_s[2] = v[35];
    p->rate_kd_nm_per_radps2[0] = v[36];
    p->rate_kd_nm_per_radps2[1] = v[37];
    p->rate_kd_nm_per_radps2[2] = v[38];
    p->rate_ki_nm_per_rad[0] = v[39];
    p->rate_ki_nm_per_rad[1] = v[40];
    p->rate_ki_nm_per_rad[2] = v[41];
    p->rate_kp_nm_per_radps[0] = v[42];
    p->rate_kp_nm_per_radps[1] = v[43];
    p->rate_kp_nm_per_radps[2] = v[44];
    p->so3_gain_per_s[0] = v[45];
    p->so3_gain_per_s[1] = v[46];
    p->so3_gain_per_s[2] = v[47];
    p->inertia_frd_kgm2[0] = v[48];
    p->inertia_frd_kgm2[1] = v[49];
    p->inertia_frd_kgm2[3] = v[49];
    p->inertia_frd_kgm2[2] = v[50];
    p->inertia_frd_kgm2[6] = v[50];
    p->inertia_frd_kgm2[4] = v[51];
    p->inertia_frd_kgm2[5] = v[52];
    p->inertia_frd_kgm2[7] = v[52];
    p->inertia_frd_kgm2[8] = v[53];
    p->motor_position_frd_m[0][0] = v[54];
    p->motor_position_frd_m[0][1] = v[55];
    p->motor_position_frd_m[0][2] = v[56];
    p->motor_position_frd_m[1][0] = v[57];
    p->motor_position_frd_m[1][1] = v[58];
    p->motor_position_frd_m[1][2] = v[59];
    p->motor_position_frd_m[2][0] = v[60];
    p->motor_position_frd_m[2][1] = v[61];
    p->motor_position_frd_m[2][2] = v[62];
    p->motor_position_frd_m[3][0] = v[63];
    p->motor_position_frd_m[3][1] = v[64];
    p->motor_position_frd_m[3][2] = v[65];
    p->motor_yaw_torque_ratio_m[0] = v[66];
    p->motor_yaw_torque_ratio_m[1] = v[67];
    p->motor_yaw_torque_ratio_m[2] = v[68];
    p->motor_yaw_torque_ratio_m[3] = v[69];
}

/* Read the controller payload back into the catalog order. */
static void ground_values_from_payload(float *v,
                                      const ProductControllerConfigPayload *p)
{
    v[0] = p->mass_kg;
    v[1] = p->actuation_preview_delay_s;
    v[2] = p->fallback_kd_per_s[0];
    v[3] = p->fallback_kd_per_s[1];
    v[4] = p->fallback_kd_per_s[2];
    v[5] = p->fallback_kp_per_s2[0];
    v[6] = p->fallback_kp_per_s2[1];
    v[7] = p->fallback_kp_per_s2[2];
    v[8] = p->manual_hover_throttle_norm;
    v[9] = p->mpc_q_diag[0];
    v[10] = p->mpc_q_diag[1];
    v[11] = p->mpc_q_diag[2];
    v[12] = p->mpc_q_diag[3];
    v[13] = p->mpc_q_diag[4];
    v[14] = p->mpc_q_diag[5];
    v[15] = p->mpc_qf_diag[0];
    v[16] = p->mpc_qf_diag[1];
    v[17] = p->mpc_qf_diag[2];
    v[18] = p->mpc_qf_diag[3];
    v[19] = p->mpc_qf_diag[4];
    v[20] = p->mpc_qf_diag[5];
    v[21] = p->mpc_r_diag[0];
    v[22] = p->mpc_r_diag[1];
    v[23] = p->mpc_r_diag[2];
    v[24] = p->mpc_s_diag[0];
    v[25] = p->mpc_s_diag[1];
    v[26] = p->mpc_s_diag[2];
    v[27] = p->mpc_seed_position_error_limit_m;
    v[28] = p->mpc_seed_velocity_error_limit_mps;
    v[29] = p->rate_d_cutoff_hz;
    v[30] = p->rate_integral_limit_nm[0];
    v[31] = p->rate_integral_limit_nm[1];
    v[32] = p->rate_integral_limit_nm[2];
    v[33] = p->rate_kaw_per_s[0];
    v[34] = p->rate_kaw_per_s[1];
    v[35] = p->rate_kaw_per_s[2];
    v[36] = p->rate_kd_nm_per_radps2[0];
    v[37] = p->rate_kd_nm_per_radps2[1];
    v[38] = p->rate_kd_nm_per_radps2[2];
    v[39] = p->rate_ki_nm_per_rad[0];
    v[40] = p->rate_ki_nm_per_rad[1];
    v[41] = p->rate_ki_nm_per_rad[2];
    v[42] = p->rate_kp_nm_per_radps[0];
    v[43] = p->rate_kp_nm_per_radps[1];
    v[44] = p->rate_kp_nm_per_radps[2];
    v[45] = p->so3_gain_per_s[0];
    v[46] = p->so3_gain_per_s[1];
    v[47] = p->so3_gain_per_s[2];
    v[48] = p->inertia_frd_kgm2[0];
    v[49] = p->inertia_frd_kgm2[1];
    v[50] = p->inertia_frd_kgm2[2];
    v[51] = p->inertia_frd_kgm2[4];
    v[52] = p->inertia_frd_kgm2[5];
    v[53] = p->inertia_frd_kgm2[8];
    v[54] = p->motor_position_frd_m[0][0];
    v[55] = p->motor_position_frd_m[0][1];
    v[56] = p->motor_position_frd_m[0][2];
    v[57] = p->motor_position_frd_m[1][0];
    v[58] = p->motor_position_frd_m[1][1];
    v[59] = p->motor_position_frd_m[1][2];
    v[60] = p->motor_position_frd_m[2][0];
    v[61] = p->motor_position_frd_m[2][1];
    v[62] = p->motor_position_frd_m[2][2];
    v[63] = p->motor_position_frd_m[3][0];
    v[64] = p->motor_position_frd_m[3][1];
    v[65] = p->motor_position_frd_m[3][2];
    v[66] = p->motor_yaw_torque_ratio_m[0];
    v[67] = p->motor_yaw_torque_ratio_m[1];
    v[68] = p->motor_yaw_torque_ratio_m[2];
    v[69] = p->motor_yaw_torque_ratio_m[3];
}

#endif
