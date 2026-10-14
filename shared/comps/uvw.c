#include "uvw_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* The `uvw` component turns the three digital hall / commutation signals `u`, `v`, `w` of a motor into a commutation angle with 60 deg resolution. On the F4 board `conf/template/uvw_fb0.txt` (or `uvw_fb1.txt`) reads the signals from `io0.fb0a`, `io0.fb0b`, `io0.fb0z`, links `uvw0.amp = adc0.amp0` and feeds `fb_switch0.com_pos`, `com_abs_pos` and `com_state`. `conf/template/encws_fb0.txt` combines it with `enc_fb0` in line saving mode (`uvw0.mode = 1`).
*
* ## Component Explanation
*
* 1. **Hall code** (`rt`):
* - The code is `rpos = u + 2 * v + 4 * w` (0..7) and is always written to `rpos`, which helps to find the table values while turning the motor by hand.
*
* 2. **Code to sector table**:
* - The angle from the sector table `p0`..`p7` is
* ```c
* pos = mod(p[rpos] / 6.0 * 2.0 * M_PI);
* ```
* - Defaults set in `nrt_init`: `p0 = -1`, `p1 = 0`, `p2 = 2`, `p3 = 1`, `p4 = 4`, `p5 = 5`, `p6 = 3`, `p7 = -1` (codes 0 and 7 are faults). The table values are truncated to integers.
*
* 3. **Signal check**:
* - If `amp < 0.75`, the component sets `error = 1`, `state = 0` and resets `timer`; `pos` is not updated. `amp` is meant to carry the feedback signal amplitude (`adc0.amp0`); if it is left unlinked it stays 0 and the component is always in error, so link it or set it to 1.
*
* 4. **Mode 0, plain UVW** (`mode = 0`):
* - Valid code (table value `>= 0`): `state = 3` (absolute), `error = 0`, `pos` updated.
* - Invalid code: `error = 1`, `state = 0`, `pos` keeps its last value.
*
* 5. **Mode 1, line saving** (`mode = 1`):
* - Used by encoders that send the UVW signals on the A/B/Z lines only right after power-up. For the first `en_time / 2` seconds after `amp` becomes valid, `pos` is updated from the table and `timer` counts up by the rt period. After that `pos` is frozen, `state = 2` (start absolute position) and `error = 0`, so `fb_switch` can take over with the incremental encoder.
*
* {{% hint warning %}}
* In mode 1 the table value is not checked, so an invalid code (-1) during the startup window gives `pos` = -60 deg instead of an error. `state` and `error` are not written during that window.
* {{% /hint %}}
*/

HAL_COMP(uvw);

//u,v,w inputs
HAL_PIN(u);        // *input*, Hall signal U (high when > 0)
HAL_PIN(v);        // *input*, Hall signal V (high when > 0)
HAL_PIN(w);        // *input*, Hall signal W (high when > 0)

HAL_PIN(amp);      // *input*, Feedback signal amplitude, below 0.75 counts as signal loss (error)
HAL_PIN(timer);    // *output*, Time since signal became valid in mode 1 (s)
HAL_PIN(en_time);  // *parameter*, Mode 1: UVW is read for en_time / 2 after startup (s, default 0.01)
HAL_PIN(mode);     // *parameter*, 0 = uvw, 1 = uvw line saving

HAL_PIN(led);      // *output*, XOR of u, v and w

HAL_PIN(p0);       // *parameter*, Sector (0..5) for hall code 0, negative = invalid code
HAL_PIN(p1);       // *parameter*, Sector (0..5) for hall code 1, negative = invalid code
HAL_PIN(p2);       // *parameter*, Sector (0..5) for hall code 2, negative = invalid code
HAL_PIN(p3);       // *parameter*, Sector (0..5) for hall code 3, negative = invalid code
HAL_PIN(p4);       // *parameter*, Sector (0..5) for hall code 4, negative = invalid code
HAL_PIN(p5);       // *parameter*, Sector (0..5) for hall code 5, negative = invalid code
HAL_PIN(p6);       // *parameter*, Sector (0..5) for hall code 6, negative = invalid code
HAL_PIN(p7);       // *parameter*, Sector (0..5) for hall code 7, negative = invalid code

//rotor position output
HAL_PIN(pos);      // *output*, Commutation angle (rad, +-pi, 60 deg steps)
HAL_PIN(rpos);     // *output*, Raw hall code u + 2v + 4w (0..7)
HAL_PIN(state);    // *output*, 0 = error, 2 = start abs (mode 1 done), 3 = abs (mode 0)
HAL_PIN(error);    // *output*, 1 = invalid code or amp too low

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct uvw_ctx_t * ctx = (struct io_ctx_t *)ctx_ptr;
  struct uvw_pin_ctx_t *pins = (struct uvw_pin_ctx_t *)pin_ptr;

  PIN(p0)      = -1;  //fault
  PIN(p1)      = 0;   //u      = 0
  PIN(p2)      = 2;   //v      = 2.094395
  PIN(p3)      = 1;   //u + v  = 1.047198
  PIN(p4)      = 4;   //w      = -2.094395
  PIN(p5)      = 5;   //u + w  = -1.047198
  PIN(p6)      = 3;   //v + w  = -3.141593
  PIN(p7)      = -1;  //fault
  PIN(en_time) = 0.01;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct uvw_ctx_t * ctx = (struct uvw_ctx_t *)ctx_ptr;
  struct uvw_pin_ctx_t *pins = (struct uvw_pin_ctx_t *)pin_ptr;

  uint32_t rpos = (PIN(u) > 0.0) * 1.0 + (PIN(v) > 0.0) * 2.0 + (PIN(w) > 0.0) * 4.0;
  PIN(led)      = (PIN(u) > 0.0) ^ (PIN(v) > 0.0) ^ (PIN(w) > 0.0);
  //TODO: make this const, fault output
  int32_t t[8];
  t[0]      = PIN(p0);
  t[1]      = PIN(p1);
  t[2]      = PIN(p2);
  t[3]      = PIN(p3);
  t[4]      = PIN(p4);
  t[5]      = PIN(p5);
  t[6]      = PIN(p6);
  t[7]      = PIN(p7);
  PIN(rpos) = rpos;
  if(PIN(amp) < 0.75) {  // fix wire saving
    PIN(error) = 1.0;
    PIN(state) = 0.0;
    PIN(timer) = 0.0;
  } else {
    switch((int)PIN(mode)) {
      case 0:
        if(t[rpos] >= 0.0) {
          PIN(state) = 3.0;
          PIN(error) = 0.0;
          PIN(pos)   = mod((float)t[rpos] / 6.0 * 2.0 * M_PI);
        } else {
          PIN(error) = 1.0;
          PIN(state) = 0.0;
        }
        break;
      case 1:
        if(PIN(timer) < PIN(en_time) / 2.0) {
          PIN(pos) = mod((float)t[rpos] / 6.0 * 2.0 * M_PI);
          PIN(timer) += period;
        } else {
          PIN(state) = 2.0;
          PIN(error) = 0.0;
        }
    }
  }
}

hal_comp_t uvw_comp_struct = {
    .name      = "uvw",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct uvw_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
