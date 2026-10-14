#include "idm_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"
#include <string.h>

/**
* ## Brief
* `idm` identifies the mechanical parameters of the axis: inertia, viscous damping, coulomb friction and a constant torque offset (e.g. gravity on a vertical axis). It runs on the F4 board and is loaded by the `id_mot` template (bare motor, results for `conf0.j`, `conf0.d`, `conf0.f`, `conf0.o`) and, through `id_sys` (which links `id_mot` and sets `idm0.sys = 1`), for the inertia of the coupled load (`conf0.j_sys`). It moves the axis back and forth through `pid0`, adapts the inertia (and first estimates of the others) until the feedback torque of the position/velocity loop no longer correlates with them, then fits friction, damping and offset over constant speed stretches at several speeds.
*
* ## Component Explanation
*
* 1. **Before you start**:
* - The motor must already run with a working current loop and commutation (run `id_pmsm`, `id_dc` or `id_acim` first).
* - The axis travels between `min_pos` and `max_pos` (about 3.2 turns each way) at up to `max_vel` and `max_acc`. The positions are relative to the start position, where the axis stands (`pos_fb`) when the run starts, so there is no step at the start, and the axis drives back to the start position at the end. Make sure there is room, or reduce these pins before enabling. The friction fit needs constant speed stretches, so a too short stroke leaves f, d and o as adapted (the console says so).
* - At the console, `link id_mot` (or `link id_sys` for the load inertia, which needs `conf0.j` from the motor run already in the config: it sets `pid0.j_mot = conf0.j` and `pid0.j_sys = idm0.inertia`). The template wires `idm0.en = fault0.en_out`, `idm0.pos_fb = fb_switch0.pos_fb`, the trajectory into `pid0.pos_ext_cmd` / `vel_ext_cmd` / `acc_ext_cmd`, the estimates back into `pid0.j_mot`, `pid0.d`, `pid0.f`, `pid0.o`, and `pid0.torque_cmd` / `pid0.fb_torque_cmd` into `torque` / `fb_torque`. It also sets `pid0.j_sys = 0`, `conf0.max_pos_error = 0`, `conf0.max_sat = 10`, `conf0.vel_g = 1`, `idm0.li = 0.005` and uses the soft loop gains `pos_bw`, `vel_bw`, `vel_d` (5, 40, 4) of this component for `pid0` during the test.
*
* 2. **Procedure (state machine, `state` pin)**:
* - `0`: idle. The trajectory follows `pos_fb`. When `en` goes high the rt function takes that position as the start position, resets the `*_sum` / `*_time` / `fit_*` pins and the plateau bins and goes to `1.0`. `en` low returns to `0` from any state.
* - `1.0` -> `1.1` (nrt): with `auto_step >= 1` it goes straight to `1.2`; otherwise it prints a prompt and waits in `1.1` for `idm0.state = 1.2`.
* - `1.2`: five ramp-up rounds (`sub_state` 1..5). Round k uses `max_acc * k / 5` and `max_vel * k / 5` and lasts `2 * (|max_pos - min_pos| / v + 2 * v / a)`, going to the start position + `max_pos` for the first half and back to the start position + `min_pos` for the second (about 22 s with the defaults).
* - `1.3`, adaptive phase: 45 s of moves at `max_vel` and `max_acc`, reversing whenever the position is within 0.1 rad of an end point.
* - `1.3`, speed plateaus: then the speed steps through 0.1, 0.25, 0.5 and 1.0 x `max_vel`, 4 moves (two each way) per level, at `max_acc`. On each constant speed stretch (`|vel_cmd|` within 1 % of the level speed), after 0.2 s of settling, the total torque `torque` and the speed `vel_cmd + vel_offset` are summed into a bin per level and direction; stretches shorter than 0.4 s are dropped. The adaptation keeps running through the plateaus.
* - At the end of `1.3` f, d and o are fitted (see 4.) and the axis drives back to the start position, then `1.4` (nrt) prints `conf0.j` (or `conf0.j_sys` when `sys > 0`), `conf0.o`, `conf0.d` and `conf0.f` as lines to append to the config, plus the fit's bin count, speed range and residual, and goes to `1.5` (done, the axis is held at the start position).
* - The whole run takes about two minutes with the defaults. Afterwards continue with `id_sys` or `id_pid`.
*
* 3. **Trajectory**:
* - `pos` is integrated from `vel_cmd` and `acc_cmd`; `pos_cmd` is `mod(pos)` (wrapped to +-pi). The generator is a bang-bang profile: `vel = LIMIT(acc * sqrt(2 * |to_go| / max_acc), max_vel)`, `acc_cmd = LIMIT((vel - vel_cmd) / period, max_acc)`.
* - `vel_offset` is added to the profile velocity for the damping and friction regressors, the plateau bins and in `vel_out = vel_cmd + vel_offset`; `pos` / `pos_cmd` / `vel_cmd` do not include it. The `id_sys_sl` template (sensorless, no encoder) sets `vel_offset = 125`, `max_vel = 75`, `min_pos`/`max_pos` = -50/50 and `vel_bw = 20`, so the speed sweeps 50..200 rad/s in one direction and the observer stays valid; it feeds `vel_out` into `sl_seq0.vel_cmd` and takes `en` from `sl_seq0.pid_en`.
*
* 4. **Estimation (rt, states 1.2 and 1.3)**:
* - Each estimate is a gradient loop on the feedback torque of `pid0`; because the estimates are fed back into the pid feedforward, each one stops moving once the feedback torque no longer contains its regressor:
* ```c
* inertia  += period / ji * fb_torque * acc_cmd * period;
* damping  += period / di * fb_torque * (vel_cmd + vel_offset) * period;
* friction += period / fi * fb_torque * SIGN(vel_cmd + vel_offset) * period;
* offset   += period / li * fb_torque * period;
* ```
* - `ji`, `di`, `fi`, `li` are divisors: larger means slower and smoother. The offset loop has a time constant of `li / period`, i.e. 50 s at the default 0.01 and the 5 kHz rt rate, which is why the template lowers it to 0.005 (25 s).
* - Clamps: `inertia` 5e-6..50 kgm^2 (0..50 when `sys > 0`), `damping` 0..100 Nm/(rad/s), `friction` 0..100 Nm, `offset` -100..100 Nm. `inertia` starts at 0.0002 from nrt_init; the others start at 0 (or at their last value on a rerun).
* - The inertia result is the adapted value. Friction, damping and offset are then replaced by a least squares fit of `T = f sign(v) + d v + o` over the plateau bin means (`fit_n` bins, `fit_lo`..`fit_hi` rad/s, residual `fit_rms`), because at one speed sign(v) and v move together and the adapted f/d split drifts from run to run. With bins in both directions (at least 3) all three are fitted. With every bin on one side (a `vel_offset` over `max_vel`) f and o cannot be told apart: d is fitted, o stays as adapted and f takes the rest. With fewer than 2 bins, or with only two bins in opposite directions (a short stroke that reaches one speed level only: f + d v is all there is and a fit would fold the friction into the damping), f, d and o stay as adapted, and so they do when the fit has no solution (one direction with every bin at the same speed, or a singular system). `fit_n` then reads below 2 (0 in all but the first case) and the console prints 'f, d, o not fitted'; `fit_lo`/`fit_hi` still show the bins' speed range when there were at least 2. The fitted values are clamped like the adapted ones.
* - In state 1.2 it also accumulates open-loop averages of the total torque (`inertia_sum`, `damping_sum`, `friction_sum`, `offset_sum`, divided by `acc_time`, `vel_time` and `time` at the end of 1.2). These are diagnostics only and are not used for the results.
*/
HAL_COMP(idm);

HAL_PIN(en);  // *input*, enable; high starts the identification, low aborts it (state -> 0)

HAL_PIN(state);  // *input/output*, state machine: 0 off, 1.1 wait for start, 1.2 ramp-up rounds, 1.3 adaptive phase and speed plateaus, 1.4 print results, 1.5 done
HAL_PIN(sub_state);  // *output*, round 1..5 in state 1.2, acc/vel scaled by sub_state/5
HAL_PIN(timer);  // *output*, time in the current round / in state 1.3 (s)
HAL_PIN(acc_time);  // *output*, time spent accelerating in state 1.2 (s)
HAL_PIN(vel_time);  // *output*, time spent moving in state 1.2 (s)
HAL_PIN(time);  // *output*, total time of state 1.2 (s)

HAL_PIN(freq);  // *parameter*, unused
HAL_PIN(amp);  // *output*, unused, only cleared to 0
HAL_PIN(min_pos);  // *parameter*, lower end of the travel relative to the start position (rad), default -20
HAL_PIN(max_pos);  // *parameter*, upper end of the travel relative to the start position (rad), default 20
HAL_PIN(max_vel);  // *parameter*, test velocity, top plateau speed (rad/s), default 50
HAL_PIN(max_acc);  // *parameter*, test acceleration (rad/s^2), default 250

HAL_PIN(pos);  // *output*, unwrapped trajectory position (rad)
HAL_PIN(pos_fb);  // *input*, feedback position from fb_switch0.pos_fb, followed while idle so the profile starts where the rotor is (rad)
HAL_PIN(pos_cmd);  // *output*, position command, pos wrapped to +-pi (rad), to pid0.pos_ext_cmd
HAL_PIN(vel_cmd);  // *output*, velocity command (rad/s), to pid0.vel_ext_cmd
HAL_PIN(acc_cmd);  // *output*, acceleration command (rad/s^2), to pid0.acc_ext_cmd
HAL_PIN(vel_offset);  // *parameter*, added to the profile velocity for the estimates and vel_out (rad/s), 0 = none; > max_vel keeps one direction (sensorless, id_sys_sl)
HAL_PIN(vel_out);  // *output*, vel_cmd + vel_offset (rad/s), to sl_seq0.vel_cmd in id_sys_sl

HAL_PIN(ji);  // *parameter*, inertia adaptation divisor, larger = slower, default 10
HAL_PIN(fi);  // *parameter*, friction adaptation divisor, default 0.001
HAL_PIN(di);  // *parameter*, damping adaptation divisor, default 1.0
HAL_PIN(li);  // *parameter*, offset adaptation divisor, time constant li/period, default 0.01 (id_mot sets 0.005)

HAL_PIN(torque);  // *input*, total torque command from pid0.torque_cmd (Nm), used for the plateau fit and the _sum diagnostics
HAL_PIN(fb_torque);  // *input*, feedback torque of the pid from pid0.fb_torque_cmd (Nm), drives the adaptation

HAL_PIN(inertia_sum);  // *output*, diagnostic, mean of acc_cmd * torque while accelerating in state 1.2
HAL_PIN(friction_sum);  // *output*, diagnostic, mean of sign(vel_cmd + vel_offset) * torque while moving in state 1.2 (Nm)
HAL_PIN(damping_sum);  // *output*, diagnostic, mean of (vel_cmd + vel_offset) * torque while moving in state 1.2
HAL_PIN(offset_sum);  // *output*, diagnostic, mean torque over state 1.2 (Nm)

HAL_PIN(inertia);  // *output*, estimated inertia (kgm^2), result for conf0.j or conf0.j_sys, default 0.0002
HAL_PIN(damping);  // *output*, viscous damping, adapted then fitted (Nm/(rad/s)), result for conf0.d
HAL_PIN(friction);  // *output*, coulomb friction, adapted then fitted (Nm), result for conf0.f
HAL_PIN(offset);  // *output*, constant torque e.g. gravity, adapted then fitted (Nm), result for conf0.o

HAL_PIN(pos_bw);  // *parameter*, position bandwidth used by pid0 during the test, default 5
HAL_PIN(vel_bw);  // *parameter*, velocity bandwidth used by pid0 during the test, default 40 (id_sys_sl sets 20)
HAL_PIN(vel_d);  // *parameter*, velocity loop damping used by pid0 during the test, default 4

HAL_PIN(sys);  // *parameter*, 0 = bare motor (prints conf0.j), >0 = load inertia (prints conf0.j_sys), set by id_sys

HAL_PIN(target);  // *output*, end point the trajectory is moving to (rad)
HAL_PIN(auto_step);  // *parameter*, >= 1 starts without waiting for state = 1.2, default 1.4

HAL_PIN(fit_n);  // *output*, plateau bins used in the f, d, o fit
HAL_PIN(fit_rms);  // *output*, rms residual of the fit over the bins (Nm)
HAL_PIN(fit_lo);  // *output*, lowest plateau speed in the fit (rad/s)
HAL_PIN(fit_hi);  // *output*, highest plateau speed in the fit (rad/s)

#define J_TIME 45.0        // 1.3: adaptive phase at max_vel [s]
#define N_LEVELS 4
#define LEVEL_FLIPS 4      // moves per speed level, two each way
#define PLAT_SETTLE 0.2    // skipped at the start of each constant speed stretch [s]
static const float levels[N_LEVELS] = {0.1, 0.25, 0.5, 1.0};

struct idm_ctx_t {
  int level;   // -1: adaptive phase, else the speed level
  int flips;   // moves at this level
  float plat;  // time on this constant speed stretch [s]
  float pst, psv;  // this stretch's torque and speed sums
  uint32_t pn;
  int ps, pl;  // its direction and level
  float st[N_LEVELS][2];  // torque sums per level and direction
  float sv[N_LEVELS][2];  // speed sums
  uint32_t n[N_LEVELS][2];
  float pos0;  // where the run started: the stroke is centred on it, the axis returns to it
  int home;    // 1.3 is done, the axis drives back to pos0
};

// least squares T = f sign(v) + d v + o over the bin means
static void fit_fdo(struct idm_ctx_t *ctx, struct idm_pin_ctx_t *pins) {
  float a[3][3] = {{0}}, b[3] = {0}, tv[2 * N_LEVELS], vv[2 * N_LEVELS];
  int m = 0, pos = 0, neg = 0;
  for(int k = 0; k < N_LEVELS; k++) {
    for(int s = 0; s < 2; s++) {
      if(ctx->n[k][s] > 0) {
        vv[m] = ctx->sv[k][s] / ctx->n[k][s];
        tv[m] = ctx->st[k][s] / ctx->n[k][s];
        if(vv[m] > 0.0) {
          pos++;
        } else {
          neg++;
        }
        m++;
      }
    }
  }
  PIN(fit_n) = m;
  if(m < 2) {
    return;
  }
  float lo = 1e9, hi = 0.0;
  for(int i = 0; i < m; i++) {
    lo = MIN(lo, ABS(vv[i]));
    hi = MAX(hi, ABS(vv[i]));
  }
  PIN(fit_lo) = lo;
  PIN(fit_hi) = hi;
  float f, d, o;
  if(pos > 0 && neg > 0 && m >= 3) {
    for(int i = 0; i < m; i++) {
      float x[3] = {vv[i] > 0.0 ? 1.0 : -1.0, vv[i], 1.0};
      for(int r = 0; r < 3; r++) {
        for(int c = 0; c < 3; c++) {
          a[r][c] += x[r] * x[c];
        }
        b[r] += x[r] * tv[i];
      }
    }
    float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if(ABS(det) < 1e-12) {
      PIN(fit_n) = 0;  // no solution: keep the adapted values, print 'not fitted'
      return;
    }
    f = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2]) + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2])) / det;
    d = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2]) - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0])) / det;
    o = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1]) - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0]) + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])) / det;
  } else if(pos == 0 || neg == 0) {  // one direction only: T = c + d v, f = (c - o) sign
    float mv = 0.0, mt = 0.0, sxx = 0.0, sxy = 0.0;
    for(int i = 0; i < m; i++) {
      mv += vv[i] / m;
      mt += tv[i] / m;
    }
    for(int i = 0; i < m; i++) {
      sxx += (vv[i] - mv) * (vv[i] - mv);
      sxy += (vv[i] - mv) * (tv[i] - mt);
    }
    if(sxx <= 0.0) {
      PIN(fit_n) = 0;  // no solution: keep the adapted values, print 'not fitted'
      return;
    }
    d = sxy / sxx;
    o = PIN(offset);
    f = (mt - d * mv - o) * (pos > 0 ? 1.0 : -1.0);
  } else {
    // both directions at one speed only (a short stroke): f + d v is all
    // there is, f and d can not be split. keep them as adapted
    PIN(fit_n) = 0;
    return;
  }
  float e = 0.0;
  for(int i = 0; i < m; i++) {
    float r = f * (vv[i] > 0.0 ? 1.0 : -1.0) + d * vv[i] + o - tv[i];
    e += r * r;
  }
  PIN(fit_rms)  = sqrtf(e / m);
  PIN(friction) = CLAMP(f, 0.0, 100.0);
  PIN(damping)  = CLAMP(d, 0.0, 100.0);
  PIN(offset)   = CLAMP(o, -100.0, 100.0);
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idm_ctx_t * ctx = (struct idm_ctx_t *)ctx_ptr;
  struct idm_pin_ctx_t *pins = (struct idm_pin_ctx_t *)pin_ptr;
  PIN(fi)                    = 0.001;
  PIN(li)                    = 0.01;
  PIN(di)                    = 1.0;
  PIN(ji)                    = 10.0;
  PIN(pos_bw)                = 5.0;
  PIN(vel_bw)                = 40.0;
  PIN(vel_d)                 = 4.0;
  PIN(max_vel)               = 50.0;
  PIN(max_acc)               = 250.0;
  PIN(min_pos)               = -20.0;
  PIN(max_pos)               = 20.0;
  PIN(auto_step)             = 1.4;
  PIN(inertia)               = 0.0002;
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idm_ctx_t * ctx = (struct idm_ctx_t *)ctx_ptr;
  struct idm_pin_ctx_t *pins = (struct idm_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      //PIN(inertia) = 0.0001;
      break;

    case 10:
      PIN(state)  = 1.1;
      PIN(timer)  = 0.0;

      if(PIN(auto_step) >= 1) {
        PIN(state) = 1.2;
      } else {
        printf("Measure friction, damping and inertia\n");
        printf("the motor will move\n");
        printf("idm0.state = 1.2 <font color='green'>to start</font>\n");
      }
      break;

    case 14:
      if(PIN(sys) > 0.0) {
        printf("conf0.j_sys = %f <font color='green'># append to config</font>\n", PIN(inertia));
      } else {
        printf("conf0.j = %f <font color='green'># append to config</font>\n", PIN(inertia));
      }
      printf("conf0.o = %f <font color='green'># append to config</font>\n", PIN(offset));
      printf("conf0.d = %f <font color='green'># append to config</font>\n", PIN(damping));
      printf("conf0.f = %f <font color='green'># append to config</font>\n", PIN(friction));
      if(PIN(fit_n) >= 2) {
        printf("<font color='green'># f, d, o fitted over %i constant speed stretches, %f to %f rad/s, residual %f Nm</font>\n", (int)PIN(fit_n), PIN(fit_lo), PIN(fit_hi), PIN(fit_rms));
      } else {
        printf("<font color='red'>f, d, o not fitted</font>: no constant speed stretches, left as adapted. lengthen max_pos - min_pos\n");
      }
      printf("done\n");
      if(PIN(sys) > 0.0) {
        printf("continue with id_pid\n");
      } else {
        printf("continue with id_sys or id_pid\n");
      }
      PIN(state) = 1.5;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idm_ctx_t *ctx = (struct idm_ctx_t *)ctx_ptr;
  struct idm_pin_ctx_t *pins = (struct idm_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  float to_go;
  float time_to_go;
  float acc;
  float vel;
  float max_acc;
  float max_vel;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(timer)   = 0.0;
      PIN(acc_cmd) = 0.0;
      PIN(vel_cmd) = 0.0;
      PIN(amp)     = 0.0;
      // follow the rotor while idle, so the profile starts without a step
      PIN(pos)     = PIN(pos_fb);
      PIN(pos_cmd) = mod(PIN(pos));

      if(PIN(en) > 0.0) {
        PIN(state)     = 1.0;
        PIN(sub_state) = 1.0;
        PIN(timer)     = 0.0;

        PIN(inertia_sum)  = 0.0;
        PIN(friction_sum) = 0.0;
        PIN(damping_sum)  = 0.0;
        PIN(offset_sum)   = 0.0;
        PIN(acc_time)     = 0.0;
        PIN(vel_time)     = 0.0;
        PIN(time)         = 0.0;
        PIN(fit_n)        = 0.0;
        PIN(fit_rms)      = 0.0;
        PIN(fit_lo)       = 0.0;
        PIN(fit_hi)       = 0.0;
        memset(ctx, 0, sizeof(struct idm_ctx_t));
        ctx->level = -1;
        ctx->pos0  = PIN(pos);
        PIN(target) = ctx->pos0 + PIN(min_pos);
      }
      break;

    case 12:
      max_acc = PIN(max_acc) / 5.0 * PIN(sub_state);
      max_vel = PIN(max_vel) / 5.0 * PIN(sub_state);

      PIN(pos) += PIN(vel_cmd) * period + PIN(acc_cmd) * period * period / 2.0;
      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) += PIN(acc_cmd) * period;
      to_go        = PIN(target) - PIN(pos);
      time_to_go   = sqrtf(2.0 * ABS(to_go) / max_acc);
      acc          = max_acc * SIGN(to_go);
      vel          = acc * time_to_go;
      vel          = LIMIT(vel, max_vel);
      acc          = (vel - PIN(vel_cmd)) / period;
      PIN(acc_cmd) = LIMIT(acc, max_acc);

      if(time_to_go < period) {
        time_to_go   = 0.0;
        to_go        = 0.0;
        PIN(acc_cmd) = 0.0;
        PIN(pos)     = PIN(target);
        PIN(vel_cmd) = 0.0;
      }

      if(ABS(PIN(acc_cmd)) > 0.0) {
        PIN(acc_time) += period;
        PIN(inertia_sum) += PIN(acc_cmd) * PIN(torque) * period;
      }

      if(ABS(PIN(vel_cmd) + PIN(vel_offset)) > 0.0) {
        PIN(vel_time) += period;
        PIN(damping_sum) += (PIN(vel_cmd) + PIN(vel_offset)) * PIN(torque) * period;
        PIN(friction_sum) += SIGN(PIN(vel_cmd) + PIN(vel_offset)) * PIN(torque) * period;
      }

      PIN(time) += period;
      PIN(offset_sum) += PIN(torque) * period;

      PIN(inertia) += period / PIN(ji) * PIN(fb_torque) * PIN(acc_cmd) * period;
      PIN(damping) += period / PIN(di) * PIN(fb_torque) * (PIN(vel_cmd) + PIN(vel_offset)) * period;
      PIN(friction) += period / PIN(fi) * PIN(fb_torque) * SIGN(PIN(vel_cmd) + PIN(vel_offset)) * period;
      PIN(offset) += period / PIN(li) * PIN(fb_torque) * period;

      if(PIN(sys) > 0.0) {
        PIN(inertia) = CLAMP(PIN(inertia), 0.0, 50.0);
      } else {
        PIN(inertia) = CLAMP(PIN(inertia), 0.000005, 50.0);
      }
      PIN(damping)  = CLAMP(PIN(damping), 0.0, 100.0);
      PIN(friction) = CLAMP(PIN(friction), 0.0, 100.0);
      PIN(offset)   = CLAMP(PIN(offset), -100.0, 100.0);

      PIN(timer) += period;
      if(PIN(timer) < (ABS(PIN(max_pos) - PIN(min_pos)) / max_vel + 2.0 * max_vel / max_acc)) {
        PIN(target) = ctx->pos0 + PIN(max_pos);
      } else {
        PIN(target) = ctx->pos0 + PIN(min_pos);
      }
      if(PIN(timer) > 2.0 * (ABS(PIN(max_pos) - PIN(min_pos)) / max_vel + 2.0 * max_vel / max_acc)) {
        PIN(timer) = 0.0;
        PIN(sub_state)
        ++;
      }

      if(PIN(sub_state) > 5.0) {
        PIN(state) = 1.3;

        if(PIN(acc_time) > period) {
          PIN(inertia_sum) /= PIN(acc_time);
        }

        if(PIN(vel_time) > period) {
          PIN(friction_sum) /= PIN(vel_time);
          PIN(damping_sum) /= PIN(vel_time);
        }

        if(PIN(time) > period) {
          PIN(offset_sum) /= PIN(time);
        }
      }
      break;

    case 13:  // j, d, f
      max_vel = PIN(max_vel) * (ctx->level >= 0 && ctx->level < N_LEVELS ? levels[ctx->level] : 1.0);
      PIN(pos) += PIN(vel_cmd) * period + PIN(acc_cmd) * period * period / 2.0;
      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) += PIN(acc_cmd) * period;
      to_go        = PIN(target) - PIN(pos);
      time_to_go   = sqrtf(2.0 * ABS(to_go) / PIN(max_acc));
      acc          = PIN(max_acc) * SIGN(to_go);
      vel          = acc * time_to_go;
      vel          = LIMIT(vel, max_vel);
      acc          = (vel - PIN(vel_cmd)) / period;
      PIN(acc_cmd) = LIMIT(acc, PIN(max_acc));

      if(ctx->home) {
        // back to where the run started, so repeated runs do not walk the
        // axis toward a stroke end
        if(time_to_go < period) {
          PIN(timer)   = 0.0;
          PIN(acc_cmd) = 0.0;
          PIN(vel_cmd) = 0.0;
          PIN(pos)     = PIN(target);
          PIN(pos_cmd) = mod(PIN(pos));
          PIN(state)   = 1.4;
        }
        break;
      }

      if(ABS(ctx->pos0 + PIN(max_pos) - PIN(pos)) < 0.1 && PIN(target) != ctx->pos0 + PIN(min_pos)) {
        PIN(target) = ctx->pos0 + PIN(min_pos);
        ctx->flips++;
      } else if(ABS(ctx->pos0 + PIN(min_pos) - PIN(pos)) < 0.1 && PIN(target) != ctx->pos0 + PIN(max_pos)) {
        PIN(target) = ctx->pos0 + PIN(max_pos);
        ctx->flips++;
      }

      // constant speed stretches: mean torque per level and direction. A
      // stretch counts only with PLAT_SETTLE left after settling, else the
      // speed loop's transient off the corner biases the mean.
      if(ctx->level >= 0 && ABS(ABS(PIN(vel_cmd)) - max_vel) <= 0.01 * max_vel) {
        ctx->plat += period;
        if(ctx->plat > PLAT_SETTLE) {
          ctx->pst += PIN(torque);
          ctx->psv += PIN(vel_cmd) + PIN(vel_offset);
          ctx->pn++;
          ctx->ps = PIN(vel_cmd) > 0.0 ? 0 : 1;
          ctx->pl = ctx->level;
        }
      } else {
        if(ctx->plat > 2.0 * PLAT_SETTLE && ctx->pn > 0) {
          ctx->st[ctx->pl][ctx->ps] += ctx->pst;
          ctx->sv[ctx->pl][ctx->ps] += ctx->psv;
          ctx->n[ctx->pl][ctx->ps] += ctx->pn;
        }
        ctx->plat = 0.0;
        ctx->pst  = 0.0;
        ctx->psv  = 0.0;
        ctx->pn   = 0;
      }

      PIN(inertia) += period / PIN(ji) * PIN(fb_torque) * PIN(acc_cmd) * period;
      PIN(damping) += period / PIN(di) * PIN(fb_torque) * (PIN(vel_cmd) + PIN(vel_offset)) * period;
      PIN(friction) += period / PIN(fi) * PIN(fb_torque) * SIGN(PIN(vel_cmd) + PIN(vel_offset)) * period;
      PIN(offset) += period / PIN(li) * PIN(fb_torque) * period;

      if(PIN(sys) > 0.0) {
        PIN(inertia) = CLAMP(PIN(inertia), 0.0, 50.0);
      } else {
        PIN(inertia) = CLAMP(PIN(inertia), 0.000005, 50.0);
      }
      PIN(damping)  = CLAMP(PIN(damping), 0.0, 100.0);
      PIN(friction) = CLAMP(PIN(friction), 0.0, 100.0);
      PIN(offset)   = CLAMP(PIN(offset), -100.0, 100.0);


      PIN(timer) += period;
      if(ctx->level < 0 && PIN(timer) > J_TIME) {
        ctx->level = 0;
        ctx->flips = -1;  // the move under way does not count
      } else if(ctx->level >= 0 && ctx->flips >= LEVEL_FLIPS) {
        ctx->level++;
        ctx->flips = 0;
      }
      if(ctx->level >= N_LEVELS) {
        PIN(amp) = 0.0;
        fit_fdo(ctx, pins);
        ctx->home   = 1;
        PIN(target) = ctx->pos0;
      }
      break;
  }
  PIN(vel_out) = PIN(vel_cmd) + PIN(vel_offset);
}


hal_comp_t idm_comp_struct = {
    .name      = "idm",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idm_ctx_t),
    .pin_count = sizeof(struct idm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
