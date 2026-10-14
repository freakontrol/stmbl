#include "gain_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `gain` computes a velocity dependent factor between 0 and 1. F4 component; not used by any config in `conf/`.
*
* ## Component Explanation
* 1. **Output** (`rt`):
* - `scale = CLAMP(|vel / MAX(max_vel, 0.1)|, 0, 1)`, i.e. 0 at standstill rising linearly to 1 at `max_vel`.
*/

HAL_COMP(gain);

HAL_PIN(vel);      // *input*, Velocity (rad/s)
HAL_PIN(max_vel);  // *parameter*, Velocity for factor 1 (rad/s), min 0.1

HAL_PIN(scale);  // *output*, Factor 0..1


static void rt_func(float gainriod, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct gain_ctx_t * ctx = (struct gain_ctx_t *)ctx_ptr;
  struct gain_pin_ctx_t *pins = (struct gain_pin_ctx_t *)pin_ptr;

  PIN(scale) = CLAMP(ABS(PIN(vel) / MAX(PIN(max_vel), 0.1)), 0.0, 1.0);
}

hal_comp_t gain_comp_struct = {
    .name      = "gain",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct gain_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
