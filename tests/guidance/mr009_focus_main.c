/* Focused MR-009 production-path test runner.
 * Include the main Host test translation unit with its normal suite entrypoint
 * renamed so the exact static SafetyFx fixture can be executed alone.
 */
#define main m5_full_suite_main
#include "test_m5_main.c"
#undef main

int main(void)
{
  test_m5_safety_mr009_extreme_reference_rejected();
  printf("MR009_PRODUCTION_PATH checks=%d failures=%d\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}
