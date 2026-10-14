#include "en_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `en` is an enable sequencer: after its input has been active for a while it switches on two enable outputs one after the other, and it restarts when a fault occurs. It runs on the F4 board. `conf/template/en.txt` uses it to enable the drive automatically after power up: `en0.en_in = 1`, `fault0.en = en0.en_out0`, `en0.fault = fault0.fault`, `en0.time = 1`. In `conf/move.txt` `en_out1` enables the drive and homing.
*
* ## Component Explanation
* 1. **Timer** (in `rt`): `timer` counts up by `period` while `en_in > 0`, and is reset to 0 when `en_in <= 0` or `fault > 0`. It stops at `time`.
*
* 2. **Fault retry**:
* - With the `en.txt` wiring a fault resets the timer, `en_out0` drops, `fault0` goes to DISABLED and clears its fault, and the drive is enabled again after `time / 2`.
* - So the drive retries automatically after every fault.
*/

HAL_COMP(en);

HAL_PIN(en_in);    // *input*, Enable request
HAL_PIN(en_out0);  // *output*, 1 once timer > time / 2
HAL_PIN(en_out1);  // *output*, 1 once timer > time
HAL_PIN(fault);    // *input*, Fault, > 0 resets the timer

HAL_PIN(time);   // *parameter*, Sequence time (s, default 5)
HAL_PIN(timer);  // *output*, Time since en_in became active (s)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct en_pin_ctx_t *pins = (struct en_pin_ctx_t *)pin_ptr;
  PIN(time)                 = 5;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct en_ctx_t *ctx      = (struct en_ctx_t *)ctx_ptr;
  struct en_pin_ctx_t *pins = (struct en_pin_ctx_t *)pin_ptr;

  if(PIN(en_in) <= 0.0) {
    PIN(timer) = 0.0;
  } else {
    PIN(timer) += period;
  }

  if(PIN(fault) > 0.0) {
    PIN(timer) = 0.0;
  }

  if(PIN(timer) > PIN(time) / 2.0) {
    PIN(en_out0) = 1.0;
  } else {
    PIN(en_out0) = 0.0;
  }

  if(PIN(timer) > PIN(time)) {
    PIN(en_out1) = 1.0;
    PIN(timer)   = PIN(time);
  } else {
    PIN(en_out1) = 0.0;
  }
}

hal_comp_t en_comp_struct = {
    .name      = "en",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct en_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
