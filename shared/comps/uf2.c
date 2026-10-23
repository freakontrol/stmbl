// deprecated: use vf (open loop) or vf with enc (vf_enc) instead.
// Known faults, kept as is: the slip uses the unsigned cur (hv0.abs_cur), so
// it adds the wrong way reversing or braking; scale (fault0.scale in uf.txt)
// scales the voltage too; polecount and scale have no defaults; registers as
// "uf", the same name as the unused uf.c
#include "uf2_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* **Deprecated**: use [vf](/docs/hal_components/vf.md) instead (templates `vf` and `vf_enc`), whose slip compensation is signed and which has damping, stall prevention and an encoder path. `uf` is kept for old configs and is not being fixed; its known faults are listed below.
*
* Open loop V/f (voltage over frequency) control for induction motors, with slip compensation. Note that this file registers the component as **`uf`** (`HAL_COMP(uf)`), so it is loaded with `load uf` and its pins are `uf0.*`; the older `uf.c` is no longer built. It runs on the F4 board and is loaded by `conf/template/uf.txt`: `uf0.u_cmd -> hv0.d_cmd` (with `hv0.cmd_mode = 0`, voltage mode), `uf0.com_pos -> hv0.pos`, `uf0.cur = hv0.abs_cur`, `uf0.scale = fault0.scale`, `uf0.polecount = conf0.polecount`. The speed command is set by the user, e.g. `uf0.vel_cmd = ramp0.vel_cmd` in `conf/bene_uf_2kw.txt`.
*
* ## Component Explanation
* All work is done in `rt`. `cur_n` has no default (0). `polecount` and `scale` have none either, so they must be linked or the output stays 0.
*
* 1. **Current filter**: `cur` is low pass filtered with a factor 0.01 per rt cycle (about 100 cycles time constant).
*
* 2. **Slip compensation**:
* ```c
* vel_cmd_out = vel_cmd + cur_filt / cur_n * slip_n * vel_n;
* ```
* - `slip_n` is the nominal slip as a fraction of `vel_n`.
*
* 3. **Voltage and angle**:
* - `u_cmd = vel_cmd_out / vel_n * u_n * scale` (linear V/f, no boost at low speed, negative for negative speed).
* - `com_pos = mod(com_pos + vel_cmd_out * polecount * period)` is the electrical angle (rad, +-pi).
* - `load = cur_filt / cur_n` is the relative load, used e.g. by `ramp0.load`.
*
* {{% hint warning %}}
* Slip compensation uses the unsigned current magnitude (`hv0.abs_cur`), so the slip is always added in the positive direction. It has the wrong sign when running in reverse or when braking (generating).
* In `conf/template/uf.txt` `fault0.scale` derates the output voltage. In V/f operation this changes the voltage for the same frequency (less flux) instead of cleanly limiting current. In the same template it also scales the `hv0` current limit (`hv0.scale`).
* {{% /hint %}}
*/

HAL_COMP(uf);

HAL_PIN(u_n);        // *parameter*, Nominal voltage at vel_n (V, default 155.6)
HAL_PIN(vel_n);      // *parameter*, Nominal velocity (rad/s, default 314.2)
HAL_PIN(polecount);  // *parameter*, Pole pairs, no default
HAL_PIN(cur_n);      // *parameter*, Nominal current (A), min 0.1
HAL_PIN(slip_n);     // *parameter*, Nominal slip as fraction of vel_n (default 0.01)


HAL_PIN(scale);  // *input*, Voltage scale, usually fault0.scale, no default

HAL_PIN(vel_cmd);  // *input*, Velocity command (rad/s)
HAL_PIN(cur);      // *input*, Motor current magnitude (A)

HAL_PIN(u_cmd);        // *output*, Voltage command, to hv0.d_cmd (V)
HAL_PIN(vel_cmd_out);  // *output*, Velocity command incl. slip compensation (rad/s)
HAL_PIN(com_pos);      // *output*, Electrical angle of the output voltage (rad)

HAL_PIN(load);  // *output*, Filtered current / cur_n

struct uf_ctx_t {
  float cur;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct uf_ctx_t *ctx      = (struct uf_ctx_t *)ctx_ptr;
  struct uf_pin_ctx_t *pins = (struct uf_pin_ctx_t *)pin_ptr;

  PIN(u_n)     = 220.0 * M_SQRT1_2;
  PIN(vel_n)   = 3000.0 / 60.0 * 2.0 * M_PI;
  PIN(com_pos) = 0.0;
  PIN(slip_n)  = 0.01;
  ctx->cur     = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct uf_ctx_t *ctx      = (struct uf_ctx_t *)ctx_ptr;
  struct uf_pin_ctx_t *pins = (struct uf_pin_ctx_t *)pin_ptr;

  ctx->cur = PIN(cur) * 0.01 + ctx->cur * 0.99;

  float vel_cmd    = PIN(vel_cmd) + ctx->cur / MAX(PIN(cur_n), 0.1) * PIN(slip_n) * PIN(vel_n);
  PIN(vel_cmd_out) = vel_cmd;
  PIN(u_cmd)       = vel_cmd / MAX(PIN(vel_n), 0.1) * PIN(u_n) * PIN(scale);
  PIN(com_pos)     = mod(PIN(com_pos) + vel_cmd * PIN(polecount) * period);
  PIN(load)        = ctx->cur / MAX(PIN(cur_n), 0.1);
}

const hal_comp_t uf_comp_struct = {
    .name      = "uf",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct uf_ctx_t),
    .pin_count = sizeof(struct uf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
