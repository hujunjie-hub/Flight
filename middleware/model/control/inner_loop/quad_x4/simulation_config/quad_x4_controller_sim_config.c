/* Generated from SIM_ONLY vehicle_model.yaml + controller_tuning.yaml. */
#include "control_backend.h"
static const ProductControllerConfig sim_config = {
 .profile_hash=PRODUCT_PROFILE_ID,.parameter_set_id=1,.parameter_hash=0x1ba590cau,.config_version=1,
 .payload={
  .mass_kg=1.5f,
  .inertia_frd_kgm2={0.025f,0.0f,0.0f,0.0f,0.025f,0.0f,0.0f,0.0f,0.04f},
  .motor_position_frd_m={{0.18f,-0.18f,0.0f},{0.18f,0.18f,0.0f},{-0.18f,0.18f,0.0f},{-0.18f,-0.18f,0.0f}},
  .motor_yaw_torque_ratio_m={-0.02f,0.02f,-0.02f,0.02f},
  .so3_gain_per_s={6.0f,6.0f,6.0f},
  .rate_kp_nm_per_radps={0.12f,0.12f,0.12f},
  .rate_ki_nm_per_rad={0.05f,0.05f,0.05f},
  .rate_kd_nm_per_radps2={0.002f,0.002f,0.002f},
  .rate_kaw_per_s={5.0f,5.0f,5.0f},
  .rate_d_cutoff_hz=40.0f,
  .rate_integral_limit_nm={0.2f,0.2f,0.2f},
  .manual_hover_throttle_norm=0.5f,
  .fallback_kp_per_s2={2.0f,2.0f,2.0f},
  .fallback_kd_per_s={2.0f,2.0f,2.0f},
  .mpc_q_diag={8.0f,8.0f,12.0f,3.0f,3.0f,4.0f},
  .mpc_qf_diag={8.0f,8.0f,12.0f,3.0f,3.0f,4.0f},
  .mpc_r_diag={0.8f,0.8f,1.0f},
  .mpc_s_diag={0.2f,0.2f,0.25f},
  .actuation_preview_delay_s=0.0f,
  .mpc_seed_position_error_limit_m=0.5f,
  .mpc_seed_velocity_error_limit_mps=1.0f,
 }};

void control_backend_sim_config(ProductControllerConfig *out)
{
    if (out)
    {
        *out = sim_config;
    }
}
