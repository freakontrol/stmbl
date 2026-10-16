#include "pid_comp.h"
/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2016 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2016 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `pid` is the standard cascaded position / velocity controller of the F4 board. It turns a position command into a torque command (Nm) using a proportional position loop, a PI velocity loop with inertia scaling, and friction / damping / inertia feedforward. It is loaded by `conf/template/pid.txt` (and most machine configs), where `pid0.pos_ext_cmd = reslimit0.pos_out`, `pid0.vel_ext_cmd = vel0.vel`, `pid0.pos_fb = fb_switch0.pos_fb`, `pid0.vel_fb = vel1.vel`, `pid0.en = fault0.en_pid`, `pid0.stop = fault0.rstop`, and `pid0.torque_cmd` feeds the torque-to-current component (`pmsm_ttc`, `dc_ttc`, ...). Gains and limits are linked from `conf0` (`pos_bw`, `vel_bw`, `vel_d`, `j`, `max_vel`, `max_force`, ...).
*
* ## Component Explanation
* All work is done in `rt`. When `en` is 0 every output, the integrator and the saturation timers are reset to 0.
*
* 1. **Position loop (pos -> vel)**, active when `pos_en > 0`:
* - `pos_error = minus(pos_ext_cmd, pos_fb)`, the shortest angular difference.
* - `vel_cmd = pos_error * pos_bw * scale`, clamped to `[-neg_min_vel * vel_g, max_vel * vel_g]`. `vel_g` therefore limits how much of the velocity range the position loop may use on its own.
* - While this clamp is active `vel_sat` counts up by `period`, otherwise it counts down to 0.
* - With `pos_en <= 0`, `pos_error`, the P velocity and `vel_sat` are 0 (pure velocity mode).
*
* 2. **Velocity loop (vel -> acc)**, active when `vel_en > 0`:
* - `vel_cmd += vel_ext_cmd` (velocity feedforward), then clamped to `[-neg_min_vel, max_vel]`.
* - `vel_error = vel_cmd - vel_fb`, `acc_cmd = vel_error * vel_bw * scale`. The P term is not clamped at `max_acc`: a clamped P term leaves the integrator to run the loop after a large error, which latches a torque-saturated limit cycle; the torque clamp is the limit.
* - The integral part is accumulated directly as torque:
* ```c
* torque_sum += vel_error * vel_bw^2 * scale^2 / MAX(vel_d, 0.1) * (j_mot + j_sys) * period;
* ```
* - `vel_d` acts like a damping factor: larger values give a slower integrator.
* - With `vel_en <= 0` the velocity error, `acc_cmd` and the integrator are 0.
*
* 3. **Acceleration to torque**:
* - `acc_cmd` is split by a first order low pass with cutoff `j_lpf` (Hz): `acc_cmd_lp` and the high pass rest `acc_cmd_hp`. With `j_lpf <= 0` the filter passes everything, so all of `acc_cmd` is in `acc_cmd_lp`.
* - `fb_torque_cmd = acc_cmd_hp * j_mot + acc_cmd_lp * (j_mot + j_sys)`, so the load inertia `j_sys` is only used for the low frequency part.
* - `fb_torque_cmd` is clamped to `[-neg_min_torque * torque_g, max_torque * torque_g]`, then the integrator is clamped dynamically to the remaining headroom (anti windup) and added.
* - `torque_sat` counts up by `period` while `fb_torque_cmd` is beyond 99 % of that limit, otherwise down to 0.
*
* 4. **Feedforward**:
* ```c
* ff_torque_cmd = acc_ext_cmd * (j_mot + j_sys) + d * vel_cmd + f * SIGN2(vel_cmd, max_vel * 0.001) + o;
* ```
* - `f` is Coulomb friction, applied with a linear ramp for speeds below 0.1 % of `max_vel`; `d` is viscous damping; `o` is a constant offset (e.g. gravity).
*
* 5. **Output**:
* - `torque_cmd = CLAMP(torque_ext_cmd + ff_torque_cmd + fb_torque_cmd, -neg_min_torque, max_torque)`.
* - `sat = MAX(vel_sat, torque_sat)` (s). It is usually linked to `fault0.sat`, which trips a saturation error when it exceeds `fault0.max_sat`.
*
* 6. **Regenerative stop (`stop`)**:
* - While `en > 0` and `stop > 0` (linked to `fault0.rstop`) the position loop is off (`pos_error`, the P velocity and `vel_sat` are 0) and the velocity, acceleration and torque feedforward inputs (`vel_ext_cmd`, `acc_ext_cmd`, `torque_ext_cmd`) are ignored, so the velocity loop brakes the motor to zero speed and the energy goes back into the DC link. The `d`, `f` and `o` feedforward terms still act on the (zero based) `vel_cmd`.
* - Because `pos_error` is held at 0, `fault0` cannot trip on a position error during the stop.
*
* 7. **Defaults (nrt_init)**:
* * - All limits (`max_vel`, `neg_min_vel`, `max_torque`, `neg_min_torque`) default to 0, which gives zero output; they must be linked (the template links them from `conf0`, with the negative limits set to the same value as the positive ones). `max_vel` must also be non zero because the friction feedforward divides by it.
*/

HAL_COMP(pid);

HAL_PIN(pos_ext_cmd);  // *input*, Position command (rad)
HAL_PIN(pos_fb);       // *input*, Position feedback (rad)
HAL_PIN(pos_error);    // *output*, Position error, wrapped to +-pi (rad)

HAL_PIN(vel_ext_cmd);  // *input*, Velocity feedforward command (rad/s)
HAL_PIN(vel_fb);       // *input*, Velocity feedback (rad/s)
HAL_PIN(vel_cmd);      // *output*, Clamped velocity command of the velocity loop (rad/s)
HAL_PIN(vel_error);    // *output*, Velocity error (rad/s)

HAL_PIN(acc_ext_cmd);  // *input*, Acceleration feedforward command (rad/s^2)
HAL_PIN(acc_cmd);      // *output*, Acceleration command of the velocity loop (rad/s^2)
HAL_PIN(acc_cmd_lp);   // *output*, Low pass part of acc_cmd, scaled with j_mot + j_sys (rad/s^2)
HAL_PIN(acc_cmd_hp);   // *output*, High pass part of acc_cmd, scaled with j_mot only (rad/s^2)

HAL_PIN(torque_ext_cmd);  // *input*, Additional torque command (Nm)
HAL_PIN(fb_torque_cmd);   // *output*, Feedback torque incl. integrator (Nm)
HAL_PIN(ff_torque_cmd);   // *output*, Feedforward torque (Nm)
HAL_PIN(torque_cmd);      // *output*, Total clamped torque command (Nm)
HAL_PIN(torque_sum);      // *output*, Velocity loop integrator (Nm)

HAL_PIN(en);      // *input*, Enable, 0 resets all outputs and the integrator
HAL_PIN(stop);    // *input*, > 0 = regenerative stop: position loop off, zero speed, no external feedforward (fault0.rstop)
HAL_PIN(pos_en);  // *parameter*, Enable position loop (default 1)
HAL_PIN(vel_en);  // *parameter*, Enable velocity loop (default 1)

HAL_PIN(pos_bw);  // *parameter*, Position loop bandwidth / P gain (1/s, default 100)

HAL_PIN(vel_bw);    // *parameter*, Velocity loop bandwidth / P gain (1/s, default 2000)
HAL_PIN(vel_d);     // *parameter*, Velocity loop damping, integrator gain is vel_bw^2 / vel_d (default 2, min 0.1)
HAL_PIN(vel_g);     // *parameter*, Fraction of max_vel the position loop may command (default 0.1)
HAL_PIN(torque_g);  // *parameter*, Fraction of max_torque the feedback path may command (default 1)

HAL_PIN(scale);  // *input*, Gain scale for pos_bw and vel_bw (default 1)

HAL_PIN(j_lpf);  // *parameter*, Cutoff of the j_sys low pass (Hz), 0 = no filter

HAL_PIN(acc_g);  // *parameter*, Not used by the code

HAL_PIN(j_mot);  // *parameter*, Motor inertia (kgm^2)
HAL_PIN(f);      // *parameter*, Coulomb friction feedforward (Nm)
HAL_PIN(d);      // *parameter*, Viscous damping feedforward (Nm/(rad/s))
HAL_PIN(j_sys);  // *parameter*, Load inertia (kgm^2)
HAL_PIN(o);      // *parameter*, Constant torque offset feedforward (Nm)

// user limits
HAL_PIN(max_vel);         // *parameter*, Maximum positive velocity (rad/s)
HAL_PIN(neg_min_vel);     // *parameter*, Maximum negative velocity as positive number (rad/s)
HAL_PIN(max_acc);         // *parameter*, Not used by the code, acc_cmd is not clamped (rad/s^2)
HAL_PIN(max_torque);      // *parameter*, Maximum positive torque (Nm)
HAL_PIN(neg_min_torque);  // *parameter*, Maximum negative torque as positive number (Nm)

HAL_PIN(vel_sat);     // *output*, Time the position loop output is saturated (s)
HAL_PIN(torque_sat);  // *output*, Time the feedback torque is saturated (s)
HAL_PIN(sat);         // *output*, max(vel_sat, torque_sat), to fault0.sat (s)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct pid_ctx_t *ctx      = (struct pid_ctx_t *)ctx_ptr;
  struct pid_pin_ctx_t *pins = (struct pid_pin_ctx_t *)pin_ptr;

  PIN(pos_en)   = 1.0;
  PIN(vel_en)   = 1.0;
  PIN(pos_bw)   = 100.0;   // (1/s)
  PIN(vel_bw)   = 2000.0;  // (1/s)
  PIN(vel_d)    = 2.0;
  PIN(vel_g)    = 0.1;
  PIN(torque_g) = 1.0;

  PIN(scale) = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct pid_ctx_t *ctx      = (struct pid_ctx_t *)ctx_ptr;
  struct pid_pin_ctx_t *pins = (struct pid_pin_ctx_t *)pin_ptr;

  float lpf = LP_HZ(MAX(PIN(j_lpf), 0.0));

  if(PIN(en) > 0.0) {
    const int stop = PIN(stop) > 0.0;
    if(PIN(pos_en) > 0.0 && !stop) {
      // pos -> vel
      PIN(pos_error) = minus(PIN(pos_ext_cmd), PIN(pos_fb));
      PIN(vel_cmd)   = PIN(pos_error) * PIN(pos_bw) * PIN(scale);                                     // p
      if(ABS(SAT2(PIN(vel_cmd), -PIN(neg_min_vel) * PIN(vel_g), PIN(max_vel) * PIN(vel_g))) > 0.0) {  // saturation
        PIN(vel_sat) += period;
      } else {
        PIN(vel_sat) = MAX(PIN(vel_sat) - period, 0.0);
      }
      PIN(vel_cmd) = CLAMP(PIN(vel_cmd), -PIN(neg_min_vel) * PIN(vel_g), PIN(max_vel) * PIN(vel_g));  // p clamping
    } else {
      PIN(pos_error) = 0.0;
      PIN(vel_cmd)   = 0.0;
      PIN(vel_sat)   = 0.0;
    }

    if(PIN(vel_en) > 0.0) {
      // vel -> acc
      PIN(vel_cmd) += stop ? 0.0 : PIN(vel_ext_cmd);                                                  // ff
      PIN(vel_cmd) = CLAMP(PIN(vel_cmd), -PIN(neg_min_vel), PIN(max_vel));                            // clamping
      PIN(vel_error) = PIN(vel_cmd) - PIN(vel_fb);
      // p, not clamped at max_acc: a clamped p term leaves the integrator to
      // run the loop after a large error, which latches a torque-saturated
      // limit cycle. The torque clamp below is the limit.
      PIN(acc_cmd)   = PIN(vel_error) * PIN(vel_bw) * PIN(scale);

      PIN(torque_sum) += PIN(vel_error) * PIN(vel_bw) * PIN(vel_bw) * PIN(scale) * PIN(scale) / MAX(PIN(vel_d), 0.1) * (PIN(j_mot) + PIN(j_sys)) * period;  // i
    } else {
      PIN(vel_error)  = 0.0;
      PIN(acc_cmd)    = 0.0;
      PIN(torque_sum) = 0.0;
    }


    // acc -> torque
    PIN(acc_cmd_lp) = PIN(acc_cmd) * lpf + PIN(acc_cmd_lp) * (1.0 - lpf);
    PIN(acc_cmd_hp) = PIN(acc_cmd) - PIN(acc_cmd_lp);

    PIN(fb_torque_cmd) = PIN(acc_cmd_hp) * PIN(j_mot) + PIN(acc_cmd_lp) * (PIN(j_mot) + PIN(j_sys));
    PIN(fb_torque_cmd) = CLAMP(PIN(fb_torque_cmd), -PIN(neg_min_torque) * PIN(torque_g), PIN(max_torque) * PIN(torque_g));                                         // clamping
    PIN(torque_sum)    = CLAMP(PIN(torque_sum), -PIN(neg_min_torque) * PIN(torque_g) - PIN(fb_torque_cmd), PIN(max_torque) * PIN(torque_g) - PIN(fb_torque_cmd));  // dynamic integral clamping
    PIN(fb_torque_cmd) += PIN(torque_sum);

    if(ABS(SAT2(PIN(fb_torque_cmd), -PIN(neg_min_torque) * PIN(torque_g) * 0.99, PIN(max_torque) * PIN(torque_g) * 0.99)) > 0.0) {  // saturation
      PIN(torque_sat) += period;
    } else {
      PIN(torque_sat) = MAX(PIN(torque_sat) - period, 0.0);
    }

    PIN(ff_torque_cmd) = (stop ? 0.0 : PIN(acc_ext_cmd)) * (PIN(j_mot) + PIN(j_sys));
    PIN(ff_torque_cmd) += PIN(d) * PIN(vel_cmd);
    PIN(ff_torque_cmd) += PIN(f) * SIGN2(PIN(vel_cmd), PIN(max_vel) * 0.001);
    PIN(ff_torque_cmd) += PIN(o);

    PIN(torque_cmd) = CLAMP((stop ? 0.0 : PIN(torque_ext_cmd)) + PIN(ff_torque_cmd) + PIN(fb_torque_cmd), -PIN(neg_min_torque), PIN(max_torque));

    // sat
    PIN(sat) = MAX(PIN(vel_sat), PIN(torque_sat));
  } else {
    PIN(pos_error)     = 0.0;
    PIN(vel_cmd)       = 0.0;
    PIN(vel_error)     = 0.0;
    PIN(acc_cmd)       = 0.0;
    PIN(torque_cmd)    = 0.0;
    PIN(acc_cmd_lp)    = 0.0;
    PIN(acc_cmd_hp)    = 0.0;
    PIN(vel_sat)       = 0.0;
    PIN(torque_sat)    = 0.0;
    PIN(sat)           = 0.0;
    PIN(torque_sum)    = 0.0;
    PIN(fb_torque_cmd) = 0.0;
    PIN(ff_torque_cmd) = 0.0;
    PIN(torque_cmd)    = 0.0;
  }
}

hal_comp_t pid_comp_struct = {
    .name      = "pid",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct pid_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
