#include "reslimit_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `reslimit` quantizes a position to a given number of steps per turn. In `conf/template/pid.txt` and `mpid.txt` it sits between the position command and the controller (`reslimit0.pos_in = rev0.out`, `pid0.pos_ext_cmd = reslimit0.pos_out`) with `reslimit0.res = conf0.mot_fb_res`, so the command is limited to the resolution of the motor feedback. It runs on the F4 board.
*
* ## Component Explanation
* 1. **Quantization** (in `rt`):
* ```c
* pos_out = (int)(pos_in * res / (2 pi) + 0.5) / res * 2 pi;
* ```
*
* {{% hint warning %}}
* `res` must not be 0 (division by zero, TODO in the code). The `(int)` cast truncates towards zero, so rounding is wrong for negative positions and the step around 0 is twice as wide (the "offset at zerocross" TODO).
* {{% /hint %}}
*/

HAL_COMP(reslimit);

HAL_PIN(pos_in);   // *input*, Position (rad)
HAL_PIN(pos_out);  // *output*, Quantized position (rad)

HAL_PIN(res);  // *parameter*, Steps per turn, must not be 0

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct reslimit_ctx_t * ctx = (struct reslimit_ctx_t *)ctx_ptr;
  struct reslimit_pin_ctx_t *pins = (struct reslimit_pin_ctx_t *)pin_ptr;
  //TODO: offset at zerocross
  uint32_t r   = ABS(PIN(res));  //TODO: div by zero
  PIN(pos_out) = ((int)(PIN(pos_in) * r / 2.0 * M_1_PI + 0.5)) / (float)r * 2.0 * M_PI;
}

hal_comp_t reslimit_comp_struct = {
    .name      = "reslimit",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct reslimit_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
