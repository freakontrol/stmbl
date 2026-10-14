#include "hal_test_comp.h"
#include "hal.h"

/**
* ## Brief
* `hal_test` is a debugging component that burns CPU time in the rt and frt threads to test HAL timing and overrun handling. F4 component; not used by any config in `conf/`.
*
* ## Component Explanation
* 1. **Busy loops**:
* - `rt` runs an empty loop `rt_wait` times, `frt` runs one `frt_wait` times. Both default to 0 (no load).
*/

HAL_COMP(hal_test);

HAL_PIN(rt_wait);   // *parameter*, Busy loop iterations per rt period
HAL_PIN(frt_wait);  // *parameter*, Busy loop iterations per frt period

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct rev_ctx_t * ctx = (struct rev_ctx_t *)ctx_ptr;
  struct hal_test_pin_ctx_t *pins = (struct hal_test_pin_ctx_t *)pin_ptr;

  volatile uint32_t foo = 0;
  (void)foo;
  for(uint32_t i = 0; i < PIN(rt_wait); i++) {
    foo = 1;
  }
}

static void frt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct rev_ctx_t * ctx = (struct rev_ctx_t *)ctx_ptr;
  struct hal_test_pin_ctx_t *pins = (struct hal_test_pin_ctx_t *)pin_ptr;

  volatile uint32_t foo = 0;
  (void)foo;
  for(uint32_t i = 0; i < PIN(frt_wait); i++) {
    foo = 1;
  }
}

hal_comp_t hal_test_comp_struct = {
    .name      = "hal_test",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = frt_func,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct hal_test_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
