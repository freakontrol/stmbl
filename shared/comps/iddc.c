#include "iddc_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `iddc` identifies a brushed DC motor: winding resistance `r`, inductance `l` and the torque/back-emf constant `psi`, and detects a reversed motor. It runs on the F4 board, drives `hv0` on the q axis only (the template sets `hv0.phase_mode = 3`, 180 degree 2 phase / H-bridge) and is loaded by the `id_dc` template. Afterwards continue with `id_mot`.
*
* ## Component Explanation
*
* 1. **Setup**:
* - At the console, `link id_dc`. The template wires `iddc0.en = fault0.en_out`, disables `pid0` and the position error fault, and connects `hv0.en`, `cur_bw`, `cmd_mode`, `q_cmd`, `pos`, `rev`, `r`, `l` to this component and `hv0.uq_fb`, `iq_fb`, `pwm_volt`, `vel1.vel` back into it.
* - `hv0.r` and `hv0.l` are wired to this component's `r` and `l`, so the current loop uses the estimate as it forms. While the state is 0 the nrt function resets `r` 0.1 ohm, `l` 1 mH, `psi` 0.055 and `out_rev` 0.
* - Set `test_cur` and `test_vel` after enabling or before; they are not reset.
*
* 2. **Resistance and inductance (`state` 1.x), rotor blocked**:
* - When `en` goes high the state goes to `1.0`; the nrt function prints "block the rotor" and waits in `1.1`. Set `iddc0.state = 1.2` to start. `en` low returns to `0` from any state.
* - `1.2` (2 s): current mode, `cur_bw` 10, `q_cmd = test_cur`. `r` is low-passed from `uq_fb / iq_fb` (factor 0.01 per rt tick). At the end `avg_test_volt = test_cur * r`, limited to `pwm_volt / 2`.
* - `1.3` (1 s): voltage mode. `q_cmd` alternates every rt tick between `1.5 * avg_test_volt` and `0.5 * avg_test_volt`; the current and voltage for both levels are low-passed into `tmp0`..`tmp3` and
* ```c
* l = |tmp1 - tmp3| / |tmp0 - tmp2| * period;
* ```
* - `1.4` (nrt): prints `conf0.r` and `conf0.l`, then goes to `2.0`, which prints "unblock the rotor" and waits (in state `3.1`).
*
* 3. **Torque constant (`state` 2.2), rotor free**:
* - Set `iddc0.state = 2.2`. For 5 s the motor runs in current mode (`cur_bw` 250) under a simple speed loop on `|vel_fb|` towards `test_vel`: the error is limited to `test_vel / 100`, integrated with `ki` into `cur_sum`, and `q_cmd = vel_bw * period * error + cur_sum`. `q_cmd` has no own limit; only the drive's current limits apply.
* - While `|vel_fb| > 0.1` rad/s, `psi = (uq_fb - iq_fb * r) / vel_fb` is low-passed with time constant `pi`.
* - At the end a negative `psi` is made positive and `out_rev = 1` is set. State `2.3` (nrt) prints `conf0.psi` (Vs/rad = Nm/A) and, if reversed, `conf0.out_rev = 1`, then goes to `2.4` (done).
*
* {{% hint warning %}}
* Untested per the template (no DC motor was available). The inductance test uses the per-tick dV/dI method that the `idacim` source documents as reading 2x to 6x high (the formula uses the full voltage swing, and the feedback delay makes the one-tick slope unpredictable), so treat `l` as a rough value. The 5 s speed test ramps `cur_sum` at no more than `ki * test_vel / 100` A/s (1 A/s by default), so the motor may not reach `test_vel`; `psi` is still averaged at whatever speed it reaches. After `1.4` the nrt function moves to state `3.1` but prints `iddc0.state = 2.2` as the start command, which is what you must type.
* {{% /hint %}}
*/

HAL_COMP(iddc);

HAL_PIN(q_cmd);     // *output*, q axis command to hv0.q_cmd, current (A) or voltage (V) depending on cmd_mode
HAL_PIN(com_pos);   // *output*, commutation position to hv0.pos, always 0
HAL_PIN(cmd_mode);  // *output*, to hv0.cmd_mode, 0 = voltage, 1 = current
HAL_PIN(en);        // *input*, enable, from fault0.en_out; low aborts (state -> 0)
HAL_PIN(en_out);    // *output*, enables hv0 while a test runs

HAL_PIN(iq_fb);   // *input*, q axis current from hv0.iq_fb (A)
HAL_PIN(uq_fb);   // *input*, q axis voltage from hv0.uq_fb (V)
HAL_PIN(pos_fb);  // *input*, motor position, unused
HAL_PIN(vel_fb);  // *input*, motor velocity from vel1.vel (rad/s)

HAL_PIN(state);  // *input/output*, 0 off, 1.1 wait, 1.2 r test, 1.3 l test, 1.4 print, 2.2 psi test, 2.3 print, 2.4 done
HAL_PIN(timer);  // *output*, time in the current test (s)

HAL_PIN(r);  // *output*, measured resistance (ohm), to hv0.r, result for conf0.r
HAL_PIN(l);  // *output*, measured inductance (H), to hv0.l, result for conf0.l

HAL_PIN(com_offset);  // *output*, unused, reset to 0
HAL_PIN(out_rev);     // *output*, 1 = motor turns backwards, to hv0.rev, result for conf0.out_rev

HAL_PIN(test_cur);  // *parameter*, test current (A), default 2
HAL_PIN(test_vel);  // *parameter*, target speed of the psi test (rad/s), default 50
HAL_PIN(ki);        // *parameter*, integral gain of the psi test speed loop (A/(rad/s)/s), default 2
HAL_PIN(vel_bw);    // *parameter*, proportional gain of the psi test speed loop (multiplied by period), default 20

HAL_PIN(pi);  // *parameter*, psi filter time constant (s), default 1

HAL_PIN(pwm_volt);  // *input*, usable voltage from hv0.pwm_volt (V), limits avg_test_volt

HAL_PIN(psi);  // *output*, measured back-emf / torque constant (Vs/rad), result for conf0.psi

HAL_PIN(cur_bw);   // *output*, current loop bandwidth to hv0.cur_bw, 10 for r/l, 250 for psi
HAL_PIN(cur_sum);  // *output*, integrator of the psi test speed loop (A)

HAL_PIN(tmp0);           // *output*, filtered current at the low voltage level of the l test (A)
HAL_PIN(tmp1);           // *output*, filtered voltage at the low voltage level of the l test (V)
HAL_PIN(tmp2);           // *output*, filtered current at the high voltage level of the l test (A)
HAL_PIN(tmp3);           // *output*, filtered voltage at the high voltage level of the l test (V)
HAL_PIN(avg_test_volt);  // *output*, voltage that drives test_cur, test_cur * r (V)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct iddc_ctx_t * ctx = (struct iddc_ctx_t *)ctx_ptr;
  struct iddc_pin_ctx_t *pins = (struct iddc_pin_ctx_t *)pin_ptr;
  PIN(test_cur)               = 2.0;
  PIN(test_vel)               = 50.0;
  PIN(ki)                     = 2.0;
  PIN(pi)                     = 1.0;
  PIN(vel_bw)                 = 20.0;
  PIN(cur_bw)                 = 1.0;
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct iddc_ctx_t * ctx = (struct iddc_ctx_t *)ctx_ptr;
  struct iddc_pin_ctx_t *pins = (struct iddc_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(r)          = 0.1;
      PIN(l)          = 0.001;
      PIN(psi)        = 0.055;
      PIN(com_offset) = 0.0;
      PIN(out_rev)    = 0.0;
      PIN(cur_bw)     = 1.0;
      break;

    case 10:  // r
      PIN(state)    = 1.1;
      PIN(timer)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;

      printf("Measure r, l\n");
      printf("<font color='green'>block the rotor</font>\n");
      printf("iddc0.state = 1.2 <font color='green'>to start</font>\n");
      break;

    case 14:
      printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
      printf("conf0.l = %f <font color='green'># append to config</font>\n", PIN(l));
      PIN(state) = 2.0;
      break;

    case 20:
      PIN(state)    = 3.1;
      PIN(timer)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_sum)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(cur_bw)   = 250.0;

      printf("Measure torque constant\n");
      printf("<font color='green'>unblock the rotor</font>\n");
      printf("the motor will move\n");
      printf("iddc0.state = 2.2 <font color='green'>to start</font>\n");
      break;

    case 23:
      printf("conf0.psi = %f <font color='green'># append to config</font>\n", PIN(psi));
      if(PIN(out_rev) > 0.0) {
        printf("conf0.out_rev = 1 <font color='green'># append to config</font>\n");
      }
      printf("done\n");
      printf("continue with id_mot\n");

      PIN(state) = 2.4;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct iddc_ctx_t * ctx = (struct iddc_ctx_t *)ctx_ptr;
  struct iddc_pin_ctx_t *pins = (struct iddc_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(en_out)   = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(timer)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 10.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1.0;
      }
      break;

    case 12:  // r
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 10.0;
      PIN(com_pos)  = 0.0;

      PIN(q_cmd) = PIN(test_cur);

      PIN(r) = PIN(r) * 0.99 + PIN(uq_fb) / MAX(PIN(iq_fb), 0.01) * 0.01;

      PIN(timer) += period;
      if(PIN(timer) >= 2.0) {
        PIN(timer)  = 0.0;
        PIN(state)  = 1.3;
        PIN(en_out) = 0.0;
        PIN(tmp0)   = 0.0;
        PIN(tmp1)   = 0.0;

        PIN(avg_test_volt) = PIN(test_cur) * MAX(PIN(r), 0.01);
        PIN(avg_test_volt) = LIMIT(PIN(avg_test_volt), PIN(pwm_volt) / 2.0);
      }
      break;

    case 13:  // l
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;
      PIN(cur_bw)   = 10.0;

      //PIN(l) = PIN(l) * 0.995 + ABS(PIN(uq_fb) - avg_test_volt / 2.0) / MAX(ABS(PIN(iq_fb) - PIN(test_cur)), 0.001) * period * 0.005;
      if(PIN(q_cmd) < PIN(avg_test_volt)) {
        PIN(tmp0)  = PIN(tmp0) * 0.99 + PIN(iq_fb) * 0.01;
        PIN(tmp1)  = PIN(tmp1) * 0.99 + PIN(uq_fb) * 0.01;
        PIN(q_cmd) = PIN(avg_test_volt) * 1.5;
      } else {
        PIN(tmp2)  = PIN(tmp2) * 0.99 + PIN(iq_fb) * 0.01;
        PIN(tmp3)  = PIN(tmp3) * 0.99 + PIN(uq_fb) * 0.01;
        PIN(q_cmd) = PIN(avg_test_volt) * 0.5;
      }

      PIN(timer) += period;
      if(PIN(timer) >= 1.0) {
        PIN(l)      = ABS(PIN(tmp1) - PIN(tmp3)) / ABS(PIN(tmp0) - PIN(tmp2)) * period;
        PIN(timer)  = 0.0;
        PIN(state)  = 1.4;
        PIN(q_cmd)  = PIN(avg_test_volt);
        PIN(en_out) = 0.0;
        // PIN(tmp0) = 0.0;
        // PIN(tmp1) = 0.0;
      }
      break;

    case 22:  // psi
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 250.0;

      float vel_error = PIN(test_vel) - ABS(PIN(vel_fb));
      vel_error       = LIMIT(vel_error, PIN(test_vel) / 100.0);

      PIN(cur_sum) += PIN(ki) * vel_error * period;

      PIN(q_cmd) = PIN(vel_bw) * period * vel_error + PIN(cur_sum);

      if(ABS(PIN(vel_fb)) > 0.1) {
        float psi = (PIN(uq_fb) - PIN(iq_fb) * PIN(r)) / (PIN(vel_fb));
        PIN(psi)  = PIN(psi) * (1.0 - period / PIN(pi)) + psi * period / PIN(pi);
      }

      //PIN(psi) = CLAMP(PIN(psi), 0.001, 1.0);

      PIN(timer) += period;
      if(PIN(timer) >= 5.0) {
        if(PIN(psi) < 0.0) {
          PIN(psi) *= -1.0;
          PIN(out_rev) = 1.0;
        }

        PIN(timer)    = 0.0;
        PIN(en_out)   = 0.0;
        PIN(q_cmd)    = 0.0;
        PIN(cur_sum)  = 0.0;
        PIN(cmd_mode) = 0.0;

        PIN(state) = 2.3;
      }
      break;
  }
}


hal_comp_t iddc_comp_struct = {
    .name      = "iddc",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,  //sizeof(struct iddc_ctx_t),
    .pin_count = sizeof(struct iddc_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};