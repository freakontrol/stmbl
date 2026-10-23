#include "rl_comp.h"
#include "hal.h"
#include "angle.h"
#include "defines.h"

/**
* ## Brief
* `rl` is a minimal resistance / inductance measurement that drives `hv0` in voltage mode on the d axis. It runs on the F4 board and is loaded by the `rl` template. It has no nrt function and prints nothing: the results are read from the `r`, `ld` and `et` pins (the template puts `ud_cmd`, `hv0.id_fb`, `r` and `ld` on the scope waves). For a full motor identification use `id_pmsm` / `idpmsm` instead.
*
* ## Component Explanation
*
* 1. **Setup and start**:
* - At the console, `link rl`. The template links the `pid` and `pmsm` templates, sets `conf0.max_ac_cur = 15`, and wires `hv0.en = rl0.en_out`, `hv0.d_cmd = rl0.ud_cmd`, `hv0.q_cmd = rl0.uq_cmd`, `hv0.cmd_mode = 0` (voltage) and `hv0.pos = 0`, plus the `hv0` d/q current and voltage feedback into this component.
* - `rl0.en` is not linked; set `rl0.en = 1` to start. The rotor is held on the d axis at electrical angle 0 by the test current (a PMSM aligns itself; block it if it must not move).
*
* 2. **Resistance (`state` 1)**:
* - `ud_cmd` is an integral current controller: `ud_cmd += ki * period * (r_test_cur - id_fb)`.
* - After `r_test_time`, while `|id_fb| > 0.1 * r_test_cur`, `ud_fb` and `id_fb` are low-passed into `ur` / `ir` (0.01 per tick) and `r = ur / ir`. After `2 * r_test_time` the state goes to 2.
*
* 3. **Inductance (`state` 2)**:
* - For `l_test_time`, `ud_cmd` alternates every rt tick between `l_test_volt` and `0.1 * l_test_volt`. The voltage and current seen at each level are low-passed into `u0`, `i0` (high) and `u1`, `i1` (low), and
* ```c
* ld = |((u1 - u0) / 2) / (i1 - i0)| * period;
* ```
* - The average voltage is `0.55 * l_test_volt`, so the mean current is about `5.5 V / r` with the defaults. Lower `l_test_volt` for low resistance motors.
*
* 4. **End (`state` 3)**:
* - Output off, `et = ld / r`, and the component sets its own `en` pin to 0, which returns it to state 0 on the next tick. Set `rl0.en = 1` again to repeat.
*
* {{% hint warning %}}
* Simple, older test. `r` includes the inverter dead time voltage (it is `ud_fb / id_fb`, not a two point fit), so it reads high on low resistance motors; `ld` uses the per-tick dV/dI slope that the `idacim` source documents as unreliable because of the feedback delay. The `ur`, `ir`, `u0`, `u1`, `i0`, `i1` filters are not reset between runs.
* {{% /hint %}}
*/

HAL_COMP(rl);

HAL_PIN(r_test_cur);   // *parameter*, current of the r test (A), default 5
HAL_PIN(l_test_volt);  // *parameter*, high voltage level of the l test (V), low level is 10%, default 10
HAL_PIN(r_test_time);  // *parameter*, settle time of the r test (s), total time is twice this, default 0.5
HAL_PIN(l_test_time);  // *parameter*, duration of the l test (s), default 1.5

HAL_PIN(ud_cmd);  // *output*, d axis voltage command to hv0.d_cmd (V)
HAL_PIN(ud_fb);   // *input*, d axis voltage from hv0.ud_fb (V)
HAL_PIN(id_fb);   // *input*, d axis current from hv0.id_fb (A)

HAL_PIN(uq_cmd);  // *output*, q axis voltage command to hv0.q_cmd, always 0
HAL_PIN(uq_fb);   // *input*, q axis voltage from hv0.uq_fb, unused
HAL_PIN(iq_fb);   // *input*, q axis current from hv0.iq_fb, unused

HAL_PIN(ki);  // *parameter*, integral gain of the r test current controller (V/(A s)), default 10

HAL_PIN(en);      // *input/output*, set to 1 to start, cleared by the component when done
HAL_PIN(en_out);  // *output*, enables hv0 during the test

HAL_PIN(state);  // *output*, 0 off, 1 r test, 2 l test, 3 done

HAL_PIN(timer);    // *output*, time in the current test (s)
HAL_PIN(counter);  // *output*, toggles between 1 and -1 each tick in the l test

HAL_PIN(r);   // *output*, measured resistance ur / ir (ohm)
HAL_PIN(ur);  // *output*, filtered voltage of the r test (V)
HAL_PIN(ir);  // *output*, filtered current of the r test (A)

HAL_PIN(ld);  // *output*, measured d axis inductance (H)
HAL_PIN(lq);  // *output*, unused
HAL_PIN(u0);  // *output*, filtered voltage at the high level of the l test (V)
HAL_PIN(u1);  // *output*, filtered voltage at the low level of the l test (V)
HAL_PIN(i0);  // *output*, filtered current at the high level of the l test (A)
HAL_PIN(i1);  // *output*, filtered current at the low level of the l test (A)

HAL_PIN(et);  // *output*, electrical time constant ld / r (s)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct rl_pin_ctx_t *pins = (struct rl_pin_ctx_t *)pin_ptr;

  PIN(r_test_cur)  = 5;
  PIN(r_test_time) = 0.5;
  PIN(l_test_volt) = 10;
  PIN(l_test_time) = 1.5;
  PIN(ki)          = 10;
}


static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct rl_ctx_t *ctx      = (struct rl_ctx_t *)ctx_ptr;
  struct rl_pin_ctx_t *pins = (struct rl_pin_ctx_t *)pin_ptr;


  if(PIN(en) <= 0.0) {
    PIN(state) = 0;
  }

  switch((int)PIN(state)) {
    case 0:  // off
      PIN(ud_cmd) = 0.0;
      PIN(uq_cmd) = 0.0;
      PIN(en_out) = 0.0;
      PIN(timer)  = 0.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1;
      }
      break;

    case 1:  // ramp up ud_cmd
      PIN(en_out) = 1;
      PIN(ud_cmd) += PIN(ki) * period * (PIN(r_test_cur) - PIN(id_fb));

      PIN(timer) += period;

      if(PIN(timer) > PIN(r_test_time)) {
        if(ABS(PIN(id_fb)) > ABS(PIN(r_test_cur)) * 0.1) {
          PIN(ur) = PIN(ur) * 0.99 + PIN(ud_fb) * 0.01;
          PIN(ir) = PIN(ir) * 0.99 + PIN(id_fb) * 0.01;
          PIN(r)  = PIN(ur) / PIN(ir);
        }
      }

      if(PIN(timer) > PIN(r_test_time) * 2.0) {
        PIN(state)   = 2;
        PIN(timer)   = 0;
        PIN(counter) = 1;
      }
      break;

    case 2:
      PIN(timer) += period;
      PIN(counter) *= -1;

      if(PIN(counter) > 0) {
        PIN(ud_cmd) = PIN(l_test_volt) * 0.1;
        PIN(u1)     = PIN(u1) * 0.998 + PIN(ud_fb) * 0.002;
        PIN(i1)     = PIN(i1) * 0.998 + PIN(id_fb) * 0.002;
      } else {
        PIN(ud_cmd) = PIN(l_test_volt);
        PIN(u0)     = PIN(u0) * 0.998 + PIN(ud_fb) * 0.002;
        PIN(i0)     = PIN(i0) * 0.998 + PIN(id_fb) * 0.002;
      }

      float dv = (PIN(u1) - PIN(u0)) / 2.0;
      float di = PIN(i1) - PIN(i0);

      PIN(ld) = ABS(dv / di) * period;

      if(PIN(timer) > PIN(l_test_time)) {
        PIN(state) = 3;
      }
      break;

    case 3:
      PIN(ud_cmd) = 0.0;
      PIN(uq_cmd) = 0.0;
      PIN(en_out) = 0.0;
      PIN(timer)  = 0.0;
      PIN(et)     = PIN(ld) / PIN(r);
      PIN(en)     = 0;
  }
}

const hal_comp_t rl_comp_struct = {
    .name      = "rl",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct rl_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
