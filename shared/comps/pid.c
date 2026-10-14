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
* 7. **Standstill hold (`hold_en`, off by default)**:
* - Why: with stiction a stop ends wherever in the friction band the last move left the torque (on a vertical axis: gravity plus the friction after an up move, gravity minus it after a down move), and the integrator keeps winding on the few encoder counts of error stiction leaves. On Z that is 3 to 18 A standing current depending on the direction of the last move.
* - At rest: the external velocity command has been below 0.1 % of `max_vel` and the external acceleration command below 1 rad/s^2 for `hold_t` seconds (not exactly 0: differentiating the in-drive stp position leaves float noise of about 1e-5 rad/s^2 at rest), and `abs(pos_error) < hold_arm` (or `< hold_win` once the error has not shrunk for `hold_stk`: the axis is stuck). `hold` is 1 then and stays 1 while `abs(pos_error) < hold_win`. Until then the loop runs normally and pulls the error in. Starting on `hold_win` froze the integrator on the way back from the stop overshoot, so on the Z bench every stop rested at the window edge (+1.99 mrad after up, -1.7 to -1.8 after down, overshoot 2.95 mrad); the rest error is now about `hold_arm` (harness Z with dob: 1.83 mrad on `hold_win`, 0.48 with `hold_arm` 0.5 mrad, 0.11 with 0.1 mrad, the hold 0.07 s later, same 10.8 A). `hold_stk` must leave the integrator time to wind through the band: on the Y plant 0.1 s started the hold at 0.2 mrad (1.2 without dob) where 0.3 s lets it reach 0.1 mrad.
* - While `hold` is 1 the integrator is frozen and slewed so that `torque_cmd` moves to `hold_o` at `hold_rate` Nm/s. `hold_o` is the moving offset `o` from `id_sys` (gravity on a vertical axis, about 0 on a horizontal one); no static friction value is needed, because the static band is not a stable number (on the Z bench the breakaway torque depended on the approach direction and scattered by 2 Nm).
* - If the real band does not reach `hold_o`, the axis creeps: the position error leaves `hold_win`, `hold` ends at once and the integrator takes over from where the hold left it. The hold then stays off until the next move (latched), so it cannot cycle at the window edge (hold, creep out, integrator pulls back, hold again). The window bounds the position error the hold can cost.
* - `hold_en` 2 is for use with the disturbance observer (`dob0.k` > 0): `hold` and the frozen integrator only, and `dob0.hold = pid0.hold` does the slew in dob, which would otherwise fight it (see dob). With dob, `hold_en` 1 must not be used.
* - Same on every axis. Harness (tests/sim, the Z and Y plants with the friction measured on the bench, `zm_*` / `ym_*`): Z no dob 17.9 / 3.3 A after an up / down stop becomes 10.8 / 10.8 A with `hold_en` 1 (the creep back to `hold_arm` takes about 0.9 s); with dob (f = d = 0, `dob0.max` 8) 17.6 / 3.3 A becomes 10.8 / 10.8 A with `hold_en` 2, started within 0.5 s.
*
* 8. **Defaults (nrt_init)**:
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

HAL_PIN(hold_en);    // *parameter*, 1 = standstill hold, 2 = hold flag and frozen integrator only (dob0 does the slew), default 0
HAL_PIN(hold_o);     // *parameter*, Torque the hold moves to at rest (Nm), conf0.o
HAL_PIN(hold_win);   // *parameter*, Position error window for the hold (rad), default 0.002
HAL_PIN(hold_arm);   // *parameter*, Position error the hold starts inside (rad), default 0.0001
HAL_PIN(hold_stk);   // *parameter*, Time the error has not shrunk before the hold starts anywhere inside hold_win (s), default 0.3
HAL_PIN(hold_t);     // *parameter*, Time at rest before the hold starts (s), default 0.1
HAL_PIN(hold_rate);  // *parameter*, Slew rate of the torque to hold_o (Nm/s), default 20
HAL_PIN(hold);       // *output*, 1 while the hold is active

struct pid_ctx_t {
  float rest;  // time the external command has been at rest (s)
  int drop;    // the hold ended on the window: off until the next move
  float emin;  // smallest abs(pos_error) at rest so far (rad)
  float still; // time abs(pos_error) has not set a new emin (s)
};

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

  PIN(hold_win)  = 0.002;
  PIN(hold_arm)  = 0.0001;
  PIN(hold_stk)  = 0.3;
  PIN(hold_t)    = 0.1;
  PIN(hold_rate) = 20.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct pid_ctx_t *ctx      = (struct pid_ctx_t *)ctx_ptr;
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

      // standstill hold: at rest the integrator is frozen and slewed so the
      // torque moves to hold_o (the last tick's torque_cmd)
      if(PIN(hold_en) > 0.0 && !stop && ABS(PIN(vel_ext_cmd)) < PIN(max_vel) * 0.001 && ABS(PIN(acc_ext_cmd)) < 1.0) {
        ctx->rest += period;
      } else {
        ctx->rest = 0.0;
        ctx->drop = 0;
        ctx->emin = 1e9;
      }
      if(ABS(PIN(pos_error)) < ctx->emin) {
        ctx->emin  = ABS(PIN(pos_error));
        ctx->still = 0.0;
      } else {
        ctx->still += period;
      }
      if(PIN(hold) > 0.0 && ABS(PIN(pos_error)) >= PIN(hold_win)) {
        ctx->drop = 1;
      }
      // start inside hold_arm, or inside hold_win once the error has stopped shrinking for hold_stk (stuck)
      float hold_lim = PIN(hold) > 0.0 || ctx->still >= PIN(hold_stk) ? PIN(hold_win) : MIN(PIN(hold_arm), PIN(hold_win));
      PIN(hold)      = PIN(hold_en) > 0.0 && !ctx->drop && ctx->rest >= PIN(hold_t) && ABS(PIN(pos_error)) < hold_lim;
      float hold_err = PIN(torque_cmd) - PIN(hold_o);
      if(PIN(hold) == 0.0) {
        PIN(torque_sum) += PIN(vel_error) * PIN(vel_bw) * PIN(vel_bw) * PIN(scale) * PIN(scale) / MAX(PIN(vel_d), 0.1) * (PIN(j_mot) + PIN(j_sys)) * period;  // i
      }
      if(PIN(hold) > 0.0 && PIN(hold_en) < 1.5) {
        PIN(torque_sum) -= CLAMP(hold_err, -PIN(hold_rate) * period, PIN(hold_rate) * period);
      }
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
    PIN(hold)          = 0.0;
    ctx->rest          = 0.0;
    ctx->drop          = 0;
    ctx->emin          = 1e9;
    ctx->still         = 0.0;
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
    .ctx_size  = sizeof(struct pid_ctx_t),
    .pin_count = sizeof(struct pid_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
