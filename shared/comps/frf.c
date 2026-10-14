#include "frf_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `frf` measures the frequency response of the mechanics with the position loop running (stepped sine) and fits a two-mass model to it, mainly to find the coupling anti-resonance for `conf0.j_lpf`. It is loaded by `conf/template/id_frf.txt`, which replaces `id_lpf`'s ring down and is run before `id_pid` (`j_lpf` changes the plant the gains are tuned against). It sits between `pid0` and the torque controller (after `biquad0` if that is loaded):
* - `frf0.in = pid0.torque_cmd`, `pmsm_ttc0.torque = frf0.out`
* - `frf0.pos = fb_switch0.vel_fb`
* - `vel1.torque = frf0.out` and `vel2.torque = frf0.out`: the observers must see the injection
* - `frf0.rt_prio = 8.6`
* - creep: `frf0.en = fault0.en_out`, `frf0.pos_fb = fb_switch0.pos_fb`, `pid0.pos_ext_cmd = frf0.pos_cmd`, `pid0.vel_ext_cmd = frf0.vel_cmd`, `pid0.acc_ext_cmd = frf0.acc_cmd`
* - `frf0.amp = 1`, `frf0.creep_vel = 3`, `frf0.creep_acc = 20`, `frf0.min_pos = -10`, `frf0.max_pos = 10`, `conf0.max_pos_error = 0`; `term0` shows `frf0.freq` and `frf0.inj`.
*
* Start with `frf0.start = 1`. `frf0.start = 0` aborts a running sweep (with creep it first drives back to the start position), or clears the state after a sweep; the result pins keep the last sweep. The table, the fit and a `conf0.j_lpf` line print in the terminal.
*
* ## Component Explanation
* The sweep and the creep run in `rt`, printing and the fit in `nrt`.
*
* 1. **Stepped sine**:
* - A torque sine of `amp` is added to the torque command, `out = in + inj`, at `points` log-spaced frequencies from `f_start` to `f_end`.
* - At each frequency it waits `settle` cycles, then correlates over whole periods (`cycles`), so the correlation rejects DC and the other harmonics. The signals are `u = out` (torque into the current loop, injection included), `c = in` (the controller's torque) and `vel = minus(pos, last pos) / period`.
* - `vel` is the raw motor-side velocity: an observer such as `vel1` would mix its own model into the result. The half sample delay of the difference is taken out of the phase.
* - Plant `G = vel / u` (rad/s per Nm). Open loop `L = -c / u` at the injection frequency: the loop gain seen at the torque command.
* - `gj = abs(G) * w` is 1 / inertia: the total inertia below the coupling ring, the motor inertia above it, a dip at the anti-resonance and a peak at the resonance.
* - Each point prints as it comes in: `f`, `abs(G)`, G phase, `gj`, `abs(L)`, L phase.
*
* 2. **Creep**:
* - Friction sticks the motor at standstill and the low frequencies then read wrong, so the axis has to move. With `creep_vel > 0` `frf` drives it itself through `pid0`'s ext commands (as `ids` does): trapezoid moves back and forth at `creep_vel` and `creep_acc` between start + `min_pos` and start + `max_pos`.
* - It measures only while it cruises; after each reversal the current point starts over. When the sweep is done it drives back to the start, then `state` goes to 2.
* - While `en <= 0` the profile follows `pos_fb`, so enabling never jumps. Losing `en` during a sweep stops it and its injection where it is (`state` 3), with or without creep.
* - The lowest point has to fit in one cruise: `(max_pos - min_pos) / creep_vel - creep_vel / creep_acc` must exceed its settle plus measure time (the template: about 3 s at `f_start` 5 Hz). Otherwise, or with `max_pos <= 0`, `min_pos >= 0` or the drive disabled at the start, it stops at once (`state` 3).
* - `creep_vel` 3 rad/s keeps the load sliding through the sine near the anti-resonance, where it swings most (1 rad/s read the mode 10 % low in the simulation).
* - `creep_vel` 0 leaves the motion to someone else (a LinuxCNC jog) and measures all the time.
*
* 3. **Crossover**: after the sweep `f_cross` is the last point where `abs(L)` falls below 1, interpolated in log (`abs(L)` can dip under 1 around the anti-resonance and come back), and `pm = 180 + L phase` there. If `abs(L)` is still above 1 at the end of the band there is none (0).
*
* 4. **Two-mass fit**:
* - The complex inertia `J(jw) = 1 / (jw G)` of the points up to `fit_max` is fitted with `J = j_mot + j_load * (2 z wa s + wa^2) / (s^2 + 2 z wa s + wa^2) + b / s` (motor and load inertia, anti-resonance `wa`, its damping `z`, viscous `b`), after taking out a pure delay (current loop and feedback).
* - The model is linear in `j_mot`, `j_load` and `b`, so each (`wa`, `z`, delay) on a grid is one small weighted least squares (weight `1 / abs(J)`, so every point counts in proportion); the best one with positive inertias wins. Grid: 160 anti-resonances log-spaced from 0.7 times the lowest fitted frequency to the highest, 16 dampings from 0.02 to 0.8, 8 delays from 0 to 1.5 ms. One grid row runs per `nrt` pass, so `nrt` stays responsive.
* - It needs at least 6 points. The fit replaces picking the extremes of `gj`, which a second mode or noise moves. `f_anti` is the corner for `pid0.j_lpf` (what `ids`' ring down measured).
* - Verdict printed: no coupling resonance in the band (`f_anti` above 95 % of the band or `j_load < 0.1 j_mot`) gives `conf0.j_lpf = 0`; `f_anti` at the low end asks for a lower `f_start`; `fit_err > 0.15` is a poor fit (more than one mode or noise); otherwise it prints `conf0.j_lpf = f_anti`.
*
* 5. **Results** (commit message, Y axis bench): crossover 45 Hz, phase margin 47 deg, anti-resonance about 14 Hz, peak 26 Hz.
*
* {{% hint warning %}}
* - `amp` defaults to 0 (no injection): set it (the template uses 1 Nm).
* - Only one `frf` per drive: the fit arrays are shared (file statics), so a second instance stops at once (`state` 3) and prints that only the first frf instance can run.
* {{% /hint %}}
*/

#define FRF_N 48

HAL_COMP(frf);

HAL_PIN(in);       // *input*, Controller torque command (Nm), pid0.torque_cmd
HAL_PIN(out);      // *output*, Torque command with the injection (Nm), to the torque controller
HAL_PIN(pos);      // *input*, Motor-side position for the velocity (rad), fb_switch0.vel_fb
HAL_PIN(inj);      // *output*, Injected torque sine (Nm)
HAL_PIN(start);    // *input*, 1 = start the sweep, 0 = abort or clear
HAL_PIN(amp);      // *parameter*, Injection amplitude (Nm, default 0)
HAL_PIN(f_start);  // *parameter*, First frequency (Hz, default 5, min 0.5)
HAL_PIN(f_end);    // *parameter*, Last frequency (Hz, default 60)
HAL_PIN(points);   // *parameter*, Number of log-spaced frequencies (default 40, 1 to 48)
HAL_PIN(settle);   // *parameter*, Cycles before measuring (default 5, at least 0.05 s)
HAL_PIN(cycles);   // *parameter*, Cycles measured, whole periods (default 10, at least 0.1 s)
HAL_PIN(freq);     // *output*, Current frequency (Hz), 0 when idle
HAL_PIN(state);    // *output*, 0 idle, 1 running, 2 done, 3 stopped (en lost, stroke too short or not the first frf), start = 0 clears it

// creep
HAL_PIN(en);         // *input*, Drive enable, fault0.en_out: the profile follows pos_fb while disabled (default 1 when unlinked)
HAL_PIN(pos_fb);     // *input*, Position feedback (rad), fb_switch0.pos_fb
HAL_PIN(pos_cmd);    // *output*, Creep position command (rad), to pid0.pos_ext_cmd
HAL_PIN(vel_cmd);    // *output*, Creep velocity command (rad/s), to pid0.vel_ext_cmd
HAL_PIN(acc_cmd);    // *output*, Creep acceleration command (rad/s^2), to pid0.acc_ext_cmd
HAL_PIN(creep_vel);  // *parameter*, Creep velocity (rad/s, default 0 = no motion of its own)
HAL_PIN(creep_acc);  // *parameter*, Creep acceleration (rad/s^2, default 20, min 0.1)
HAL_PIN(min_pos);    // *parameter*, Lower end of the stroke about the start position (rad, default -10, must be below 0)
HAL_PIN(max_pos);    // *parameter*, Upper end of the stroke about the start position (rad, default 10, must be above 0)

// results
HAL_PIN(fit_max);  // *parameter*, Fit the points up to this frequency (Hz, default 0 = all)
HAL_PIN(f_anti);   // *output*, Fitted anti-resonance (Hz), the pid0.j_lpf corner
HAL_PIN(f_res);    // *output*, Fitted resonance f_anti * sqrt(1 + j_load / j_mot) (Hz)
HAL_PIN(zeta);     // *output*, Fitted damping of the anti-resonance
HAL_PIN(j_mot);    // *output*, Fitted inertia above the ring (kgm^2)
HAL_PIN(j_load);   // *output*, Fitted inertia behind the coupling (kgm^2)
HAL_PIN(b);        // *output*, Fitted viscous friction (Nm s/rad)
HAL_PIN(delay);    // *output*, Fitted delay (s)
HAL_PIN(fit_err);  // *output*, RMS fit error, fraction of abs(J)
HAL_PIN(f_cross);  // *output*, Crossover frequency where abs(L) = 1 (Hz), 0 = none in the band
HAL_PIN(pm);       // *output*, Phase margin at f_cross (deg)

#define FIT_FA 160  // anti-resonance grid, log-spaced over the fitted band
#define FIT_Z 16
#define FIT_D 8
static const float fit_z[FIT_Z] = {0.02, 0.026, 0.033, 0.042, 0.054, 0.07, 0.09, 0.11, 0.14, 0.18, 0.23, 0.3, 0.38, 0.48, 0.62, 0.8};
static const float fit_d[FIT_D] = {0.0, 0.0002, 0.0004, 0.0006, 0.0008, 0.001, 0.0012, 0.0015};

struct frf_ctx_t {
  uint32_t second;  // not the first frf instance: the fit arrays are file statics, so it never runs
  uint32_t idx;  // current point
  uint32_t n;    // points in this run
  uint32_t done; // points finished (rt writes, nrt prints)
  uint32_t printed;
  uint32_t measuring;
  uint32_t have_pos;
  float last_pos;
  float period;
  float ph;     // injection phase (rad)
  float t;      // time at this point (s)
  float t_set;  // settle time (s)
  float t_meas; // measure time (s)
  float f;
  float us, uc, cs, cc, vs, vc;
  float f_pt[FRF_N];
  float g_mag[FRF_N], g_ph[FRF_N];
  float l_mag[FRF_N], l_ph[FRF_N];
  // creep profile
  float p;     // position command, unwrapped (rad)
  float v;     // (rad/s)
  float p0;    // start position
  float dir;   // +1 / -1 toward max_pos / min_pos
  uint32_t home;     // the sweep is over, drive back to p0
  uint32_t cruising;
  // fit, run a grid row per nrt pass
  uint32_t fit_i;  // next anti-resonance grid row, FIT_FA + 1 = finished
  uint32_t fit_n;  // points used
  float fit_lo, fit_hi;
  float best_cost, best_fa, best_z, best_d, best_jm, best_jl, best_b;
};

static uint32_t frf_instances;  // the fit arrays below are shared: one frf per drive

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct frf_ctx_t *ctx      = (struct frf_ctx_t *)ctx_ptr;
  struct frf_pin_ctx_t *pins = (struct frf_pin_ctx_t *)pin_ptr;

  ctx->second = frf_instances++ > 0;

  PIN(amp)     = 0.0;
  PIN(f_start) = 5.0;
  PIN(f_end)   = 60.0;
  PIN(points)  = 40.0;
  PIN(settle)  = 5.0;
  PIN(cycles)  = 10.0;
  PIN(creep_vel) = 0.0;  // id_frf sets 3
  PIN(creep_acc) = 20.0;
  PIN(min_pos)   = -10.0;
  PIN(max_pos)   = 10.0;
  PIN(en)        = 1.0;  // unlinked: a jog-driven sweep needs no enable
}

static float point_freq(struct frf_ctx_t *ctx, struct frf_pin_ctx_t *pins, uint32_t i) {
  float f0 = MAX(PIN(f_start), 0.5);
  float f1 = MAX(PIN(f_end), f0);
  if(ctx->n < 2) {
    return f0;
  }
  return f0 * powf(f1 / f0, (float)i / (float)(ctx->n - 1));
}

static void point_start(struct frf_ctx_t *ctx, struct frf_pin_ctx_t *pins) {
  ctx->f         = point_freq(ctx, pins, ctx->idx);
  ctx->t         = 0.0;
  ctx->t_set     = MAX(MAX(PIN(settle), 0.0) / ctx->f, 0.05);
  // whole periods, so the correlation rejects DC and the other harmonics
  ctx->t_meas    = ceilf(MAX(MAX(PIN(cycles), 1.0) / ctx->f, 0.1) * ctx->f) / ctx->f;
  ctx->measuring = 0;
  ctx->us = ctx->uc = ctx->cs = ctx->cc = ctx->vs = ctx->vc = 0.0;
}

// one rt step of a trapezoid toward target at up to vmax; 1 once it is there
static uint32_t creep(struct frf_ctx_t *ctx, struct frf_pin_ctx_t *pins, float period, float target, float vmax) {
  float a   = MAX(PIN(creep_acc), 0.1);
  float rem = target - ctx->p;
  if(ABS(rem) < MAX(a * period * period, 1e-5) && ABS(ctx->v) <= 2.0 * a * period) {  // stops within a step
    ctx->p        = target;
    ctx->v        = 0.0;
    ctx->cruising = 0;
    PIN(acc_cmd)  = 0.0;
    return 1;
  }
  float v_stop  = sqrtf(2.0 * a * ABS(rem));  // fastest speed that still stops at the target
  float v_des   = SIGN(rem) * MIN(vmax, v_stop);
  float dv      = CLAMP(v_des - ctx->v, -a * period, a * period);
  float v_old   = ctx->v;
  ctx->v       += dv;
  ctx->p       += 0.5 * (v_old + ctx->v) * period;
  ctx->cruising = v_stop > vmax && ABS(ctx->v - v_des) < 1e-6 + 1e-4 * vmax;
  PIN(acc_cmd)  = dv / period;
  return 0;
}

static void idle_out(struct frf_pin_ctx_t *pins) {
  PIN(freq) = 0.0;
  PIN(inj)  = 0.0;
  PIN(out)  = PIN(in);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct frf_ctx_t *ctx      = (struct frf_ctx_t *)ctx_ptr;
  struct frf_pin_ctx_t *pins = (struct frf_pin_ctx_t *)pin_ptr;
  uint32_t creep_on          = PIN(creep_vel) > 0.0;

  // the profile follows the rotor while disabled, so enabling never jumps
  if(PIN(en) <= 0.0) {
    ctx->p        = PIN(pos_fb);
    ctx->v        = 0.0;
    ctx->cruising = 0;
    PIN(acc_cmd)  = 0.0;
    if(PIN(state) == 1.0) {  // stop the sweep and its injection, creep or not
      PIN(state) = 3.0;
    }
  }
  PIN(pos_cmd) = mod(ctx->p);
  PIN(vel_cmd) = ctx->v;

  if(PIN(start) <= 0.0) {  // abort or reset, the result pins keep the last sweep
    if(PIN(state) == 1.0 && creep_on && PIN(en) > 0.0) {  // abort: back to the start first, then idle
      if(creep(ctx, pins, period, ctx->p0, PIN(creep_vel))) {
        PIN(state) = 0.0;
      }
      PIN(pos_cmd) = mod(ctx->p);
      PIN(vel_cmd) = ctx->v;
      idle_out(pins);
      return;
    }
    PIN(state)   = 0.0;
    ctx->v       = 0.0;
    PIN(vel_cmd) = 0.0;
    PIN(acc_cmd) = 0.0;
    idle_out(pins);
    return;
  }

  if(PIN(state) == 0.0 && ctx->second) {  // see frf_instances
    PIN(state) = 3.0;
  }
  if(PIN(state) == 0.0) {  // rising start
    ctx->n        = (uint32_t)CLAMP(PIN(points), 1.0, FRF_N);
    ctx->idx      = 0;
    ctx->done     = 0;
    ctx->printed  = 0;
    ctx->ph       = 0.0;
    ctx->have_pos = 0;
    ctx->fit_i    = 0;
    ctx->home     = 0;
    ctx->dir      = 1.0;
    ctx->p0       = ctx->p;
    PIN(f_anti)   = 0.0;
    PIN(f_res)    = 0.0;
    PIN(zeta)     = 0.0;
    PIN(j_mot)    = 0.0;
    PIN(j_load)   = 0.0;
    PIN(b)        = 0.0;
    PIN(delay)    = 0.0;
    PIN(fit_err)  = 0.0;
    PIN(f_cross)  = 0.0;
    PIN(pm)       = 0.0;
    PIN(state)    = 1.0;
    point_start(ctx, pins);
    if(creep_on) {
      // the lowest point has to fit in one cruise between the reversals
      float vmax   = PIN(creep_vel);
      float t_run  = (PIN(max_pos) - PIN(min_pos)) / vmax - vmax / MAX(PIN(creep_acc), 0.1);
      if(PIN(en) <= 0.0 || PIN(max_pos) <= 0.0 || PIN(min_pos) >= 0.0 || t_run < ctx->t_set + ctx->t_meas) {
        PIN(state) = 3.0;
      }
    }
  }

  if(PIN(state) != 1.0) {  // done or stopped, waiting for start = 0
    idle_out(pins);
    return;
  }

  if(creep_on) {
    if(ctx->home) {
      if(creep(ctx, pins, period, ctx->p0, PIN(creep_vel))) {
        PIN(state) = 2.0;
      }
      PIN(pos_cmd) = mod(ctx->p);
      PIN(vel_cmd) = ctx->v;
      idle_out(pins);
      return;
    }
    float target = ctx->p0 + (ctx->dir > 0.0 ? PIN(max_pos) : PIN(min_pos));
    if(creep(ctx, pins, period, target, PIN(creep_vel))) {
      ctx->dir = -ctx->dir;
    }
    PIN(pos_cmd) = mod(ctx->p);
    PIN(vel_cmd) = ctx->v;
  } else {
    ctx->cruising = 1;  // someone else moves the axis
  }

  float vel     = ctx->have_pos ? minus(PIN(pos), ctx->last_pos) / period : 0.0;
  ctx->last_pos = PIN(pos);
  ctx->have_pos = 1;
  ctx->period   = period;

  if(!ctx->cruising) {  // reversal: friction flips, this point starts over once it cruises again
    ctx->measuring = 0;
    ctx->t         = 0.0;
    ctx->us = ctx->uc = ctx->cs = ctx->cc = ctx->vs = ctx->vc = 0.0;
  }

  // inputs belong to the injection of the previous rt step
  if(ctx->measuring) {
    float s = sinf(ctx->ph);
    float c = cosf(ctx->ph);
    ctx->us += PIN(out) * s;
    ctx->uc += PIN(out) * c;
    ctx->cs += PIN(in) * s;
    ctx->cc += PIN(in) * c;
    ctx->vs += vel * s;
    ctx->vc += vel * c;
  }

  ctx->t += period;
  if(!ctx->measuring && ctx->t >= ctx->t_set) {
    ctx->measuring = 1;
    ctx->t         = 0.0;
  } else if(ctx->measuring && ctx->t >= ctx->t_meas) {
    // complex ratios: X = xs + j xc for each signal
    float u2   = ctx->us * ctx->us + ctx->uc * ctx->uc;
    uint32_t i = ctx->idx;
    if(u2 > 0.0) {
      // G = v / u
      float gr = (ctx->vs * ctx->us + ctx->vc * ctx->uc) / u2;
      float gi = (ctx->vc * ctx->us - ctx->vs * ctx->uc) / u2;
      // L = -c / u
      float lr      = -(ctx->cs * ctx->us + ctx->cc * ctx->uc) / u2;
      float li      = -(ctx->cc * ctx->us - ctx->cs * ctx->uc) / u2;
      ctx->g_mag[i] = sqrtf(gr * gr + gi * gi);
      // the difference is the mean velocity half a step back: add that back
      ctx->g_ph[i]  = atan2f(gi, gr) * 180.0 / M_PI + 180.0 * ctx->f * period;
      ctx->l_mag[i] = sqrtf(lr * lr + li * li);
      ctx->l_ph[i]  = atan2f(li, lr) * 180.0 / M_PI;
    } else {
      ctx->g_mag[i] = ctx->g_ph[i] = ctx->l_mag[i] = ctx->l_ph[i] = 0.0;
    }
    ctx->f_pt[i] = ctx->f;
    ctx->done    = i + 1;
    ctx->idx++;
    if(ctx->idx >= ctx->n) {
      if(creep_on) {
        ctx->home = 1;
      } else {
        PIN(state) = 2.0;
      }
      idle_out(pins);
      return;
    }
    point_start(ctx, pins);
  }

  ctx->ph   = mod(ctx->ph + 2.0 * M_PI * ctx->f * period);
  PIN(freq) = ctx->f;
  PIN(inj)  = PIN(amp) * sinf(ctx->ph);
  PIN(out)  = PIN(in) + PIN(inj);
}

// fit data, filled once per sweep. Each point is weighted by 1 / |J| = w |G|, so every
// point counts in proportion; the weighted, delay-corrected target J e^(-jw delay) / |J|
// then has unit size and is kept for each delay on the grid.
static float fit_w[FRF_N];   // w (rad/s)
static float fit_wt[FRF_N];  // weight w |G|, divided by its mean (column scale)
static float fit_jr[FIT_D][FRF_N], fit_ji[FIT_D][FRF_N];
static float fit_s;  // mean weight: the unknowns come out multiplied by it
static float fit_wm; // geometric mid w: the b column is multiplied by it

static void fit_setup(struct frf_ctx_t *ctx, struct frf_pin_ctx_t *pins) {
  uint32_t n = 0;
  float sum  = 0.0;
  for(uint32_t i = 0; i < ctx->done; i++) {
    if(PIN(fit_max) > 0.0 && ctx->f_pt[i] > PIN(fit_max)) {
      break;
    }
    if(ctx->g_mag[i] <= 0.0) {
      continue;
    }
    float w   = 2.0 * M_PI * ctx->f_pt[i];
    float phi = DEG(ctx->g_ph[i]) + M_PI / 2.0;  // J = 1 / (jw G) = e^(-j phi) / (w |G|)
    fit_w[n]  = w;
    fit_wt[n] = w * ctx->g_mag[i];
    for(uint32_t d = 0; d < FIT_D; d++) {
      float an     = -phi - w * fit_d[d];
      fit_jr[d][n] = cosf(an);
      fit_ji[d][n] = sinf(an);
    }
    sum += fit_wt[n];
    n++;
  }
  ctx->fit_n = n;
  if(n < 6) {
    return;
  }
  fit_s = sum / n;
  for(uint32_t i = 0; i < n; i++) {
    fit_wt[i] /= fit_s;
  }
  fit_wm         = sqrtf(fit_w[0] * fit_w[n - 1]);
  ctx->fit_lo    = fit_w[0] / (2.0 * M_PI);
  ctx->fit_hi    = fit_w[n - 1] / (2.0 * M_PI);
  ctx->best_cost = 1e30;
}

static float fit_fa(struct frf_ctx_t *ctx, uint32_t row) {  // grid reaches a bit below the band
  float lo = ctx->fit_lo * 0.7;
  return lo * powf(ctx->fit_hi / lo, (float)row / (float)(FIT_FA - 1));
}

static float det3(float a11, float a12, float a13, float a22, float a23, float a33) {  // symmetric
  return a11 * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
}

// one anti-resonance of the grid, every damping and delay: weighted least squares
// for j_mot, j_load and b, normal equations solved by Cramer's rule
static void fit_row(struct frf_ctx_t *ctx, uint32_t row) {
  uint32_t n = ctx->fit_n;
  float fa   = fit_fa(ctx, row);
  float wa   = 2.0 * M_PI * fa;
  for(uint32_t zi = 0; zi < FIT_Z; zi++) {
    float z = fit_z[zi];
    float c2r[FRF_N], c2i[FRF_N], c3i[FRF_N];
    // columns: j_mot -> wt (real), j_load -> wt H, b -> wt / (jw) = -j wt / w (imaginary)
    float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
    for(uint32_t i = 0; i < n; i++) {
      float w  = fit_w[i];
      float nr = wa * wa;
      float ni = 2.0 * z * wa * w;
      float dr = wa * wa - w * w;
      float dd = dr * dr + ni * ni;
      float hr = (nr * dr + ni * ni) / dd;
      float hi = (ni * dr - nr * ni) / dd;
      float wt = fit_wt[i];
      c2r[i]   = wt * hr;
      c2i[i]   = wt * hi;
      c3i[i]   = -wt * fit_wm / w;
      a11 += wt * wt;
      a12 += wt * c2r[i];
      a22 += c2r[i] * c2r[i] + c2i[i] * c2i[i];
      a23 += c2i[i] * c3i[i];
      a33 += c3i[i] * c3i[i];
    }
    float det = det3(a11, a12, a13, a22, a23, a33);
    if(ABS(det) < 1e-12) {
      continue;
    }
    for(uint32_t d = 0; d < FIT_D; d++) {
      float r1 = 0, r2 = 0, r3 = 0, yy = 0;
      for(uint32_t i = 0; i < n; i++) {
        float yr = fit_jr[d][i];
        float yi = fit_ji[d][i];
        r1 += fit_wt[i] * yr;
        r2 += c2r[i] * yr + c2i[i] * yi;
        r3 += c3i[i] * yi;
        yy += yr * yr + yi * yi;
      }
      float x1 = r1 * (a22 * a33 - a23 * a23) - a12 * (r2 * a33 - a23 * r3) + a13 * (r2 * a23 - a22 * r3);
      float x2 = a11 * (r2 * a33 - r3 * a23) - r1 * (a12 * a33 - a23 * a13) + a13 * (a12 * r3 - r2 * a13);
      float x3 = a11 * (a22 * r3 - a23 * r2) - a12 * (a12 * r3 - r2 * a13) + r1 * (a12 * a23 - a22 * a13);
      x1 /= det;
      x2 /= det;
      x3 /= det;
      if(x1 <= 0.0 || x2 < 0.0) {  // inertias are positive
        continue;
      }
      float cost = yy - (x1 * r1 + x2 * r2 + x3 * r3);  // residual of the least squares solution
      if(cost < ctx->best_cost) {
        ctx->best_cost = cost;
        ctx->best_fa   = fa;
        ctx->best_z    = z;
        ctx->best_d    = fit_d[d];
        ctx->best_jm   = x1 / fit_s;
        ctx->best_jl   = x2 / fit_s;
        ctx->best_b    = x3 / fit_s * fit_wm;
      }
    }
  }
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct frf_ctx_t *ctx      = (struct frf_ctx_t *)ctx_ptr;
  struct frf_pin_ctx_t *pins = (struct frf_pin_ctx_t *)pin_ptr;

  if(PIN(state) == 3.0 && ctx->fit_i == 0) {
    ctx->fit_i = FIT_FA + 2;  // say it once
    if(ctx->second) {
      printf("frf stopped: only the first frf instance can run (shared fit memory)\n");
      return;
    }
    printf("frf stopped: drive disabled, or the stroke (min_pos..max_pos about the start, both sides) is too short\n");
    printf("for the lowest point at creep_vel. frf0.start = 0 clears it.\n");
    return;
  }

  if(ctx->printed == 0 && ctx->done > 0) {
    printf("f Hz, |G| rad/s/Nm, G deg, gj 1/kgm2, |L|, L deg\n");
  }
  while(ctx->printed < ctx->done) {
    uint32_t i = ctx->printed++;
    float w    = 2.0 * M_PI * ctx->f_pt[i];
    printf("%f %f %f %f %f %f\n", ctx->f_pt[i], ctx->g_mag[i], ctx->g_ph[i], ctx->g_mag[i] * w, ctx->l_mag[i], ctx->l_ph[i]);
  }

  // finished sweep, even if start is already 0
  if(ctx->fit_i == 0 && ctx->n > 0 && ctx->done == ctx->n) {
    uint32_t n = ctx->done;
    // crossover: the last point where |L| falls below 1, interpolated in log
    // (|L| can dip under 1 around the anti-resonance and come back); none if
    // |L| is still above 1 at the end of the band
    float fc = 0.0, pm = 0.0;
    for(uint32_t i = n - 1; i >= 1 && ctx->l_mag[n - 1] < 1.0; i--) {
      if(ctx->l_mag[i - 1] >= 1.0 && ctx->l_mag[i] < 1.0) {
        float a = logf(ctx->l_mag[i - 1]);
        float b = logf(ctx->l_mag[i]);
        float k = a / (a - b);
        fc      = ctx->f_pt[i - 1] * powf(ctx->f_pt[i] / ctx->f_pt[i - 1], k);
        float p = ctx->l_ph[i - 1] + k * (ctx->l_ph[i] - ctx->l_ph[i - 1]);
        pm      = 180.0 + p;
        break;
      }
    }
    PIN(f_cross) = fc;
    PIN(pm)      = pm;
    if(fc > 0.0) {
      printf("crossover %f Hz, phase margin %f deg\n", fc, pm);
    } else {
      printf("no crossover in the band (|L| at the last point %f)\n", ctx->l_mag[n - 1]);
    }
    fit_setup(ctx, pins);
    if(ctx->fit_n < 6) {
      printf("too few points to fit (%lu)\n", (unsigned long)ctx->fit_n);
      ctx->fit_i = FIT_FA + 2;
      return;
    }
    ctx->fit_i = 1;
    return;
  }

  if(ctx->fit_i >= 1 && ctx->fit_i <= FIT_FA) {  // a grid row per pass keeps nrt responsive
    fit_row(ctx, ctx->fit_i - 1);
    ctx->fit_i++;
    return;
  }

  if(ctx->fit_i == FIT_FA + 1) {
    ctx->fit_i = FIT_FA + 2;
    if(ctx->best_cost >= 1e30) {
      printf("<font color='red'>no two-mass fit</font> (no positive inertias): look at the table\n");
      return;
    }
    float err     = sqrtf(MAX(ctx->best_cost, 0.0) / (2.0 * ctx->fit_n));
    PIN(f_anti)   = ctx->best_fa;
    PIN(zeta)     = ctx->best_z;
    PIN(j_mot)    = ctx->best_jm;
    PIN(j_load)   = ctx->best_jl;
    PIN(f_res)    = ctx->best_fa * sqrtf(1.0 + ctx->best_jl / ctx->best_jm);
    PIN(b)        = ctx->best_b;
    PIN(delay)    = ctx->best_d;
    PIN(fit_err)  = err;
    printf("two-mass fit %f to %f Hz, %lu points, error %f %%\n", ctx->fit_lo, ctx->fit_hi, (unsigned long)ctx->fit_n, err * 100.0);
    printf("anti-resonance %f Hz (damping %f), resonance %f Hz\n", PIN(f_anti), PIN(zeta), PIN(f_res));
    printf("j_mot %f, j_load %f, total %f kgm2, b %f Nm s/rad, delay %f ms\n", PIN(j_mot), PIN(j_load), PIN(j_mot) + PIN(j_load), PIN(b), PIN(delay) * 1000.0);
    if(PIN(f_anti) > ctx->fit_hi * 0.95 || PIN(j_load) < 0.1 * PIN(j_mot)) {
      printf("<font color='green'>no coupling resonance below %f Hz</font>: the coupling is stiff here.\n", ctx->fit_hi);
      printf("conf0.j_lpf = 0 <font color='green'># or raise frf0.f_end and run again</font>\n");
    } else if(PIN(f_anti) < ctx->fit_lo * 1.05) {
      printf("<font color='red'>anti-resonance at the low end of the band</font>: lower f_start and run again\n");
    } else if(err > 0.15) {
      printf("<font color='red'>poor fit</font>: more than one mode or noise, look at the table\n");
    } else {
      printf("conf0.j_lpf = %f <font color='green'># append to config</font>\n", PIN(f_anti));
    }
  }
}

hal_comp_t frf_comp_struct = {
    .name      = "frf",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct frf_ctx_t),
    .pin_count = sizeof(struct frf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
