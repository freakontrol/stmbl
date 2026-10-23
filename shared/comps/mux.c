#include "mux_comp.h"
#include "hal.h"
#include "defines.h"

/**
* ## Brief
* `mux` selects one of eight banks of ten inputs and copies it to the ten outputs. F4 component, used in `conf/spindle_slip_uf.txt` to switch between parameter sets with an input (`mux0.mux = io0.ind0`).
*
* ## Component Explanation
* 1. **Selection** (`rt`):
* - The bank is `mux` clamped to 0..7 and truncated to an integer.
* - `out<i> = in<bank * 10 + i>` for `i` = 0..9, e.g. `mux` = 1 copies `in10`..`in19`.
*/

HAL_COMP(mux);

HAL_PINA(in, 80);   // *input*, 8 banks of 10 inputs, bank n is in(10n)..in(10n+9)
HAL_PINA(out, 10);  // *output*, Outputs of the selected bank
HAL_PIN(mux);       // *input*, Bank select 0..7


static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct mux_pin_ctx_t *pins = (struct mux_pin_ctx_t *)pin_ptr;
  for(int i = 0; i < 10; i++) {
    PINA(out, i) = PINA(in, i + (int)MIN(MAX(PIN(mux), 0.0), 7.0) * 10);
  }
}

hal_comp_t mux_comp_struct = {
    .name      = "mux",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct mux_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};