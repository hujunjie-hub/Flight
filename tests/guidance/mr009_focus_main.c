/* Focused MR-009 production-path test runner.
 * Include the main Host test translation unit with its normal suite entrypoint
 * renamed so the exact static SafetyFx fixtures and production C hooks are used.
 */
#define main m5_full_suite_main
#include "test_m5_main.c"
#undef main

int main(void)
{
  test_m5_safety_mr009_horner_bound_extremes();
  test_m5_safety_mr009_consumer_rejection_stages();
  test_m5_safety_mr009_extreme_reference_rejected();
  test_m5_safety_floating_environment_rejected();
  test_m5_safety_consumer_time_error_c_path();
  printf("MR009_FOCUSED checks=%d failures=%d\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}
