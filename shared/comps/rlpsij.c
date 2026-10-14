#include "rlpsij_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `rlpsij` is an experimental all-in-one PMSM identification: resistance, feedback offset, pole pairs, flux `psi`, load torque, friction, damping and inertia `j` in one automatic sequence. It is built into the F4 firmware but no template in `conf/template` loads it, so it has to be loaded and wired by hand (`load rlpsij`, then connect `cmd_mode`, `d_cmd`, `q_cmd`, `com_pos`, `en_out` to `hv0`, the `hv0` d/q current and voltage feedback back in, and an absolute motor position to `abs_pos_fb`). It prints nothing; results are read from the pins. The maintained path is `id_pmsm` followed by `id_mot`.
*
* {{% hint danger %}}
* Experimental and buggy, see the notes at the end. In particular it cannot be stopped with `en` once it has started.
* {{% /hint %}}
*
* ## Component Explanation
*
* 1. **Velocity estimate (every rt tick)**:
* - `vel` and `acc` are derived from `abs_pos_fb` by differentiation (`minus()` handles the wrap) and low-passed with factor 0.99 per tick.
*
* 2. **`state` 0 -> 1, resistance and feedback offset**:
* - When `en > 0` the state goes to 1. Voltage mode, `com_pos = 0`: `d_cmd += rl_ki * period * (test_cur - id_fb)`. While `id_fb > test_cur / 2`, `r` is low-passed from `ud_fb / id_fb` with factor `lpf`.
* - After `rl_time` the aligned rotor position is stored in `fb_offset`.
*
* 3. **`state` 2 (placeholder for l)**:
* - Does nothing but reset `polecount` and `d_cmd`; `l` is never measured.
*
* 4. **`state` 3, pole pairs**:
* - Current mode, `d_cmd = test_cur`, `com_pos` rotates at one electrical turn per second. Each electrical turn increments `polecount`. After more than 1 s, once `abs_pos_fb` is back within 1% of a turn of `fb_offset`, the count is the number of pole pairs and the state goes to 4.
*
* 5. **`state` 4, psi, load, friction, damping**:
* - Commutation now follows the feedback: `com_pos = mod((abs_pos_fb - fb_offset) * polecount)`. A PI speed loop (`vel_kp`, `vel_ki`, integrator and output limited to `test_cur`) drives `q_cmd` through four quarters of `vel_time`: `test_vel / 10`, `test_vel`, `-test_vel / 10`, `-test_vel`.
* - While the speed is within 5% of the command, `psi = (uq_fb - iq_fb * r) / vel / polecount` and the mean `iq_fb` of each quarter (`i[0..3]`) are low-passed with `lpf`.
* - At the end: `load` = mean of the four currents, `damping = (i1 - i0 - i3 + i2) / 2 / (0.9 * test_vel)`, `friction = (i1 - i3 - 2 * damping * test_vel) / 2`, then converted from A to Nm with `3/2 * polecount * psi`.
*
* 6. **`state` 5, inertia**:
* - Open loop: `q_cmd = +test_cur` for `vel_time / 2`, then `-test_cur`. The torque `3/2 * polecount * psi * iq_fb - load - friction * sign(vel) - damping * vel` divided by `acc` is low-passed into `j` while `|acc| > test_vel`.
*
* 7. **`state` 6, done**:
* - `q_cmd` and `d_cmd` are 0, `com_pos` keeps following the feedback and `en_out` stays 1. `en` low returns to state 0.
*
* {{% hint warning %}}
* Known problems in the code:
* - `en` is only checked in states 0 and 6: once started, setting `en` to 0 does not stop the sequence or clear `en_out`.
* - The conversion to Nm multiplies `load` twice and never converts `friction`, so `load` is scaled by `(3/2 * polecount * psi)^2` and `friction` stays in A; `j` inherits both errors.
* - The inertia test applies `test_cur` open loop for `vel_time / 2` (2 s) in each direction with no speed limit.
* - `l` and `multi_pos` are never written; `r`, `psi`, `j` and `i[]` are not reset between runs.
* {{% /hint %}}
*/

HAL_COMP(rlpsij);

HAL_PIN(r);          // *output*, measured resistance (ohm)
HAL_PIN(l);          // *output*, never written, the l state is a placeholder
HAL_PIN(psi);        // *output*, measured flux linkage (Vs/rad electrical)
HAL_PIN(j);          // *output*, measured inertia (kgm^2), wrong because of the load/friction scaling bug
HAL_PIN(load);       // *output*, constant load torque (meant as Nm, scaled twice)
HAL_PIN(friction);   // *output*, coulomb friction (left in A, not converted)
HAL_PIN(damping);    // *output*, viscous damping (Nm/(rad/s))
HAL_PIN(polecount);  // *output*, counted pole pairs
HAL_PIN(fb_offset);  // *output*, feedback position at electrical angle 0 (rad)

HAL_PIN(cmd_mode);  // *output*, to hv0.cmd_mode, 0 = voltage (state 1), 1 = current
HAL_PIN(q_cmd);     // *output*, q axis current command to hv0.q_cmd (A)
HAL_PIN(d_cmd);     // *output*, d axis command to hv0.d_cmd, voltage in state 1, current in state 3
HAL_PIN(iq_fb);     // *input*, q axis current from hv0.iq_fb (A)
HAL_PIN(id_fb);     // *input*, d axis current from hv0.id_fb (A)
HAL_PIN(uq_fb);     // *input*, q axis voltage from hv0.uq_fb (V)
HAL_PIN(ud_fb);     // *input*, d axis voltage from hv0.ud_fb (V)

HAL_PIN(abs_pos_fb);  // *input*, absolute mechanical motor position (rad)
HAL_PIN(com_pos);     // *output*, commutation angle to hv0.pos (rad)
HAL_PIN(en_out);      // *output*, enables hv0, stays 1 until en is low in state 6

HAL_PIN(en);        // *input*, start; only checked in states 0 and 6
HAL_PIN(test_cur);  // *parameter*, test current and current limit (A), default 1
HAL_PIN(test_vel);  // *parameter*, speed of the psi test (rad/s), default 2 pi
HAL_PIN(rl_ki);     // *parameter*, integral gain of the r test current controller (V/(A s)), default 10
HAL_PIN(vel_ki);    // *parameter*, integral gain of the speed loop, default 0.01
HAL_PIN(vel_kp);    // *parameter*, proportional gain of the speed loop (A/(rad/s)), default 0.1
HAL_PIN(rl_time);   // *parameter*, duration of the r test (s), default 1
HAL_PIN(vel_time);  // *parameter*, duration of the psi test and of the j test (s), default 4
HAL_PIN(lpf);       // *parameter*, low pass factor per rt tick for all estimates, default 0.9955

HAL_PIN(timer);      // *output*, time in the current state (s)
HAL_PIN(multi_pos);  // *output*, unused
HAL_PIN(state);      // *output*, 0 off, 1 r, 2 (l placeholder), 3 pole pairs, 4 psi/friction, 5 j, 6 done
HAL_PIN(vel);        // *output*, filtered velocity from abs_pos_fb (rad/s)
HAL_PIN(acc);        // *output*, filtered acceleration (rad/s^2)
HAL_PIN(last_pos);   // *output*, abs_pos_fb of the previous tick (rad)
HAL_PIN(error_sum);  // *output*, speed loop integrator (A)
HAL_PINA(i, 4);      // *output*, mean q current of the four speed phases of state 4 (A)


static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct rlpsij_pin_ctx_t *pins = (struct rlpsij_pin_ctx_t *)pin_ptr;
  PIN(test_cur)                 = 1.0;
  PIN(test_vel)                 = 2.0 * M_PI * 1.0;
  PIN(rl_ki)                    = 10.0;
  PIN(vel_ki)                   = 0.01;
  PIN(vel_kp)                   = 0.1;
  PIN(rl_time)                  = 1.0;
  PIN(vel_time)                 = 4.0;
  PIN(lpf)                      = 0.9955;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct rlpsij_ctx_t * ctx = (struct rlpsij_ctx_t *)ctx_ptr;
  struct rlpsij_pin_ctx_t *pins = (struct rlpsij_pin_ctx_t *)pin_ptr;

  float vel     = minus(PIN(abs_pos_fb), PIN(last_pos)) / period;
  float acc     = (vel - PIN(vel)) / period;
  PIN(vel)      = PIN(vel) * 0.99 + vel * (1.0 - 0.99);
  PIN(acc)      = PIN(acc) * 0.99 + acc * (1.0 - 0.99);
  PIN(last_pos) = PIN(abs_pos_fb);

  PIN(timer) += period;

  float vel_cmd = 0.0;
  float error   = 0.0;

  switch((int)PIN(state)) {
    case 0:  // disabled
      PIN(timer)  = 0.0;
      PIN(en_out) = 0.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1;
      }
      break;

    case 1:  // r, fb_offset
      PIN(en_out)   = 1.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;  // volt mode

      PIN(d_cmd) += PIN(rl_ki) * period * (PIN(test_cur) - PIN(id_fb));

      if(PIN(id_fb) > PIN(test_cur) * 0.5) {
        PIN(r) = PIN(r) * PIN(lpf) + PIN(ud_fb) / PIN(id_fb) * (1.0 - PIN(lpf));
      }

      if(PIN(timer) > PIN(rl_time)) {
        PIN(fb_offset) = PIN(abs_pos_fb);
        PIN(timer)     = 0.0;
        PIN(state)     = 2.0;
      }
      break;

    case 2:  // l
      PIN(timer)     = 0.0;
      PIN(state)     = 3.0;
      PIN(polecount) = 0.0;
      PIN(d_cmd)     = 0.0;
      break;

    case 3:  // polecount
      PIN(cmd_mode) = 1.0;
      PIN(d_cmd)    = PIN(test_cur);

      PIN(com_pos) += 2.0 * M_PI * period;

      if(PIN(com_pos) >= M_PI) {
        PIN(polecount) += 1.0;
        PIN(com_pos) = -M_PI;
      }
      if(PIN(timer) > 1.0 && ABS(minus(PIN(abs_pos_fb), PIN(fb_offset))) < 2.0 * M_PI * 0.01) {
        PIN(d_cmd)     = 0.0;
        PIN(com_pos)   = 0.0;
        PIN(error_sum) = 0.0;
        PIN(timer)     = 0.0;
        PIN(state)     = 4.0;
      }
      break;

    case 4:  // psi, friction, damping, load
      vel_cmd   = PIN(test_vel) / 10.0;
      int phase = 0;
      if(PIN(timer) > PIN(vel_time) / 4.0) {
        phase   = 1;
        vel_cmd = PIN(test_vel);
      }
      if(PIN(timer) > PIN(vel_time) / 4.0 * 2.0) {
        phase   = 2;
        vel_cmd = -PIN(test_vel) / 10.0;
      }
      if(PIN(timer) > PIN(vel_time) / 4.0 * 3.0) {
        phase   = 3;
        vel_cmd = -PIN(test_vel);
      }

      PIN(com_pos) = mod(minus(PIN(abs_pos_fb), PIN(fb_offset)) * PIN(polecount));

      error = vel_cmd - PIN(vel);
      PIN(error_sum) += PIN(vel_ki) * period * (vel_cmd - PIN(vel));
      PIN(error_sum) = LIMIT(PIN(error_sum), PIN(test_cur));
      PIN(q_cmd)     = PIN(vel_kp) * error + PIN(error_sum);
      PIN(q_cmd)     = LIMIT(PIN(q_cmd), PIN(test_cur));

      if(ABS(vel_cmd - PIN(vel)) < PIN(test_vel) * 0.05) {
        float bemf = PIN(uq_fb) - PIN(iq_fb) * PIN(r);
        PIN(psi)   = PIN(psi) * PIN(lpf) + bemf / PIN(vel) / PIN(polecount) * (1.0 - PIN(lpf));

        PINA(i, phase) = PINA(i, phase) * PIN(lpf) + PIN(iq_fb) * (1.0 - PIN(lpf));
      }

      if(PIN(timer) > PIN(vel_time)) {
        PIN(load)     = (PINA(i, 0) + PINA(i, 1) + PINA(i, 2) + PINA(i, 3)) / 4.0;
        PIN(damping)  = (PINA(i, 1) - PINA(i, 0) - PINA(i, 3) + PINA(i, 2)) / 2.0 / (PIN(test_vel) * 0.9);
        PIN(friction) = (PINA(i, 1) - PIN(damping) * PIN(test_vel) - PINA(i, 3) - PIN(damping) * PIN(test_vel)) / 2.0;

        PIN(load) *= 3.0 / 2.0 * PIN(polecount) * PIN(psi);
        PIN(damping) *= 3.0 / 2.0 * PIN(polecount) * PIN(psi);
        PIN(load) *= 3.0 / 2.0 * PIN(polecount) * PIN(psi);

        PIN(q_cmd) = 0.0;
        PIN(timer) = 0.0;
        PIN(state) = 5.0;
      }
      break;

    case 5:  // j
      if(PIN(timer) < PIN(vel_time) / 2.0) {
        PIN(q_cmd) = PIN(test_cur);
      } else {
        PIN(q_cmd) = -PIN(test_cur);
      }

      PIN(com_pos) = mod(minus(PIN(abs_pos_fb), PIN(fb_offset)) * PIN(polecount));

      float t = 3.0 / 2.0 * PIN(polecount) * PIN(psi) * PIN(iq_fb);
      t -= PIN(load);
      t -= PIN(friction) * SIGN(PIN(vel));
      t -= PIN(damping) * PIN(vel);

      if(ABS(PIN(acc)) > PIN(test_vel)) {
        PIN(j) = PIN(j) * PIN(lpf) + t / PIN(acc) * (1.0 - PIN(lpf));
      }

      if(PIN(timer) > PIN(vel_time)) {
        PIN(state) = 6.0;
        PIN(q_cmd) = 0.0;
        PIN(d_cmd) = 0.0;
      }
      break;

    case 6:
      PIN(com_pos) = mod(minus(PIN(abs_pos_fb), PIN(fb_offset)) * PIN(polecount));
      if(PIN(en) <= 0.0) {
        PIN(state) = 0.0;
      }
      break;
  }
}

hal_comp_t rlpsij_comp_struct = {
    .name      = "rlpsij",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct rlpsij_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
