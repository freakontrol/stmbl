#include "idtune_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"
#include <math.h>

// Tunes the two running compensations that id_pmsm does not: hv0.adv, the
// commutation advance, and, for the latched dead time sign (drop_knee 0),
// hv0.drop_k. Run it after id_pmsm with its results in the config and the
// load off; hv0.pos and hv0.vel stay on the running path.
//
// drop_k (latch only): square steps on d through zero, no torque.
// Undercompensated the current creeps toward the command after each edge,
// overcompensated it overshoots; bisect on the signed mean of
// (i - cmd) * sign(cmd). With drop_knee > 0 drop_k comes from id_pmsm's curve
// fit and is kept: drop_k_cfg reads the config's hv0.drop_k because the
// template links it before it points hv0.drop_k at idtune0.drop_k.
//
// adv: speed dwells at adv_n speeds from 0.5 to 2 x test_vel. An angle lag d
// of the commutation turns part of uq into ud,
//   ud - ud_expected = -uq sin(d),    ud_expected = r id - w lq iq
// and d = w * (delay - adv), so fit the line
//   ud_err = -tau * uq * w + c         tau = delay - adv
// c takes what is left on d that is not lag (dead time residue, iron loss).
// adv moves by tau and the fit repeats until tau < ADV_TOL_FIT. The dwells run
// with drop_k 0: the latch's lag at an unloaded motor's ~1 A lands on d and
// cancels the slope.

/**
* ## Brief
* `idtune` tunes the running compensations that `idpmsm` does not: `hv0.adv`, the commutation advance in seconds (hv sends `pos + vel * adv`), and, for the latched dead time sign (`hv0.drop_knee = 0`), `hv0.drop_k`, the dead time (drop) compensation. With `drop_knee > 0` the config's `drop_k` from `idpmsm`'s curve fit is kept. It runs on the F4 board, is loaded by the `id_tune` config template and prints `hv0.drop_k` and `hv0.adv` lines to append to the config.
*
* ## How to run it
*
* 1. Run `id_pmsm` first and have its results in the config: `conf0.r`, `conf0.l`, `conf0.lq`, `conf0.psi`, `conf0.polecount`, `conf0.mot_fb_offset` (and `conf0.out_rev`), and `hv0.drop_k` / `hv0.drop_knee` if you use the dead time curve. A correct `conf0.lq` matters: the advance is found from the d voltage the motor should need, which is computed with `lq`.
* 2. Take the load off the shaft; the rotor must be free to spin at up to `2 * test_vel` (200 rad/s at the default).
* 3. With the drive disabled type `link id_tune`. The template does `load idtune` and wires:
* ```
* idtune0.en = fault0.en_out
* hv0.en = idtune0.en_out
* hv0.cmd_mode = idtune0.cmd_mode
* hv0.d_cmd = idtune0.d_cmd
* hv0.q_cmd = idtune0.q_cmd
* hv0.adv = idtune0.adv
* idtune0.drop_k_cfg = hv0.drop_k
* idtune0.drop_knee = hv0.drop_knee
* hv0.drop_k = idtune0.drop_k
* idtune0.vel_fb = vel1.vel
* idtune0.vel_e = vel2.vel
* idtune0.r = conf0.r
* idtune0.l = conf0.l
* idtune0.lq = conf0.lq
* ```
* - plus `ud_fb`, `uq_fb`, `id_fb`, `iq_fb`, `dc_volt` from `hv0`, and `fault0.pos_error = 0`, `fault0.sat = 0`, `pid0.en = 0`. `drop_k_cfg` is linked before `hv0.drop_k` is pointed at this component, so it reads the config's value. `hv0.pos` and `hv0.vel` stay on the normal running path, so the angle that is tuned is the angle the drive runs with.
* 4. Enable the drive. The tests run automatically: drop_k (about 3.5 s at the defaults, skipped with `drop_knee > 0`), then the adv speed dwells.
* 5. On success (state 1.3) it prints the last fit, `hv0.drop_k = ...` (or a note that the config's drop_k was kept) and `hv0.adv = ...` to append to the config, and goes to 1.5 (bridge off). On failure (state 1.4) it prints the reason and only the drop_k result.
* 6. Disable the drive and reload the normal config.
*
* ## Component Explanation
*
* `rt` does all the measuring, `nrt` only prints the result in states 1.3 and 1.4. `en <= 0` forces `state = 0`, which sets `en_out = 0`, `cmd_mode = 1` (current mode), zero commands, `cur_sum = 0` and `fail = 0`. On `en` going high the context is cleared, `adv = 0`, `drop_k = 0.6` and `state = 1.1`; with `drop_knee > 0` the config's `drop_k_cfg` is held as the result, `drop_k = 0` and it goes straight to `state = 1.2`.
*
* 1. **drop_k, bisection on d current steps (state 1.1, latch only)**:
* - `q_cmd = 0`; `d_cmd` is a square wave of +-`step_cur` at `step_freq`, so the current crosses zero on every edge with no torque (the rotor is held only by its cogging). `step_cur` is 6 A because the latch's model holds above about 2 A per phase.
* - With too little compensation the current creeps toward the command for tens of ms after each edge; with too much it overshoots. So the signed mean error is measured:
* ```c
* dk_err = mean((id_fb - cmd) / cmd);   // < 0: under compensated, > 0: over
* ```
* - A try is 5 square wave periods; the first period and the first 2 ms after every edge (the current loop's own rise) are not measured.
* - Bisection over `0 .. 1.2`: `dk_err < 0` raises the lower bound to `drop_k`, else the upper bound is lowered, and `drop_k` goes to the middle. After 7 tries (resolution 0.01) the result is held, `d_cmd = 0`, `drop_k = 0` and the state goes to 1.2. A result within one step of 0 or 1.2 is printed with a warning that it is not a measurement (raise `step_cur` or check the dc link).
*
* 2. **adv, from ud at `adv_n` speeds (state 1.2)**:
* - The dwells run with `drop_k = 0`: the latch's lag at an unloaded motor's ~1 A would land on d and cancel the slope.
* - `d_cmd = 0`; a speed loop on the mechanical `vel_fb` drives `q_cmd` towards the dwell speed `v_t`. The clamp slews only the integrator; the P term acts on the unclamped error:
* ```c
* vel_error = LIMIT(v_t - vel_fb, ABS(v_t) / 100);
* cur_sum   = LIMIT(cur_sum + ki * vel_error * period, test_cur);
* q_cmd     = LIMIT(vel_bw * period * (v_t - vel_fb) + cur_sum, test_cur);
* ```
* - The `adv_n` speeds (clamped 2..8) are spread evenly from `0.5 * test_vel` to `2 * test_vel` (50, 100, 150, 200 rad/s at the defaults). A negative `test_vel` runs the dwells in reverse: the clamp, the settle band and the stall check use `abs(v_t)`, so the loop works in either direction. At each speed: once the speed (filtered, 50 ms) is within 15% of `v_t` for 0.5 s, a 0.5 s dwell averages `ud_fb`, `uq_fb`, `id_fb`, `iq_fb` and the electrical speed `vel_e`. More than 8 s without settling fails with `fail = 2`. A filtered speed under 10% of `abs(v_t)` while `|cur_sum| > test_cur / 2` at any time fails with `fail = 1` (stalled, usually wrong polecount, offset or out_rev).
* - Without angle error the motor needs
* ```c
* ud_exp = r * id - w * lq * iq;   // w = electrical speed, lq = l if lq is 0
* ```
* - An angle lag `d` of the commutation turns part of uq into ud, `ud - ud_exp = -uq * sin(d)`, and `d = w * (delay - adv)`. After the last speed a least squares line is fitted through the dwells:
* ```c
* ud_err = -tau * uq * w + c;   // tau = delay - adv, c = what is on d that is not lag
* adv    = CLAMP(adv + tau, 0, 0.003);
* ```
* - `c` takes the dead time residue and iron loss, so low speed dwells no longer drag adv to its limit. The speed sweep repeats until a fit moves `adv` by less than 20 us (state 1.3); this is checked first, so a fit that settles at the 0 floor is a result (`adv = 0`). Otherwise `fail = 4` if `adv` hits 3 ms, or sits at 0 with a negative `tau` (the fit wants a negative advance); the message then says to check `conf0.lq` and `conf0.r`, raise `test_vel` and rerun. `fail = 3` after 4 fits without converging (check `conf0.lq` and `conf0.r`).
*
* 3. **Result (states 1.3, 1.4, 1.5)**:
* - The bridge is switched off (`en_out = 0`, commands and `cur_sum` zeroed) and `drop_k` is set back to the measured (or kept) value. `nrt` prints the result or the failure reason and sets `state = 1.5`, where it idles until the drive is disabled.
*
* {{% hint warning %}}
* - `drop_k_cfg` reads the config's `hv0.drop_k` only because `link id_tune` at the console links it before `hv0.drop_k` is re-pointed; do not save the id_tune lines into a config, since the boot time `relink` would chain `drop_k_cfg` to `idtune0.drop_k`.
* {{% /hint %}}
*/

HAL_COMP(idtune);

HAL_PIN(en);        // *input*, enable, usually fault0.en_out; low resets state to 0
HAL_PIN(en_out);    // *output*, bridge enable to hv0.en
HAL_PIN(state);     // *input/output*, 0 off, 1.1 drop_k, 1.2 adv, 1.3 done, 1.4 failed, 1.5 idle
HAL_PIN(cmd_mode);  // *output*, to hv0.cmd_mode, always 1 = current mode
HAL_PIN(d_cmd);     // *output*, d current command to hv0.d_cmd (A)
HAL_PIN(q_cmd);     // *output*, q current command to hv0.q_cmd (A)

HAL_PIN(adv);         // *output*, commutation advance to hv0.adv (s), 0 .. 0.003
HAL_PIN(drop_k);      // *output*, dead time compensation to hv0.drop_k, 0 .. 1.2, 0 during the adv dwells
HAL_PIN(drop_k_cfg);  // *input*, the config's hv0.drop_k, kept as the result when drop_knee > 0
HAL_PIN(drop_knee);   // *input*, hv0.drop_knee, > 0 skips the drop_k bisection

HAL_PIN(id_fb);    // *input*, d current from hv0 (A)
HAL_PIN(iq_fb);    // *input*, q current from hv0 (A)
HAL_PIN(ud_fb);    // *input*, d voltage command from hv0 (V)
HAL_PIN(uq_fb);    // *input*, q voltage command from hv0 (V)
HAL_PIN(vel_fb);   // *input*, mechanical speed for the speed loop (rad/s), vel1.vel
HAL_PIN(vel_e);    // *input*, electrical speed as hv0.vel sees it (rad/s), vel2.vel
HAL_PIN(dc_volt);  // *input*, dc link voltage from hv0 (V), only printed

HAL_PIN(r);   // *parameter*, winding resistance (ohm), conf0.r
HAL_PIN(l);   // *parameter*, d inductance (H), conf0.l
HAL_PIN(lq);  // *parameter*, q inductance (H), conf0.lq, 0 = same as l

HAL_PIN(step_cur);   // *parameter*, d step amplitude (A), default 6
HAL_PIN(step_freq);  // *parameter*, d step frequency (Hz), default 10, min 1
HAL_PIN(test_cur);   // *parameter*, q current limit of the speed loop (A), default 3
HAL_PIN(test_vel);   // *parameter*, base dwell speed, mechanical (rad/s), dwells at 0.5..2 x, negative runs them in reverse, default 100
HAL_PIN(ki);         // *parameter*, speed loop integral gain (A/rad), default 1
HAL_PIN(vel_bw);     // *parameter*, speed loop P gain, times period (A per rad/s), default 250
HAL_PIN(cur_sum);    // *output*, speed loop integrator (A)

HAL_PIN(dk_err);   // *output*, signed mean step error of the last drop_k try, fraction of step_cur
HAL_PIN(ud_err);   // *output*, ud minus ud expected in the last dwell (V)
HAL_PIN(ud_exp);   // *output*, ud expected in the last dwell, r id - w lq iq (V)
HAL_PIN(uq_mean);  // *output*, mean uq in the last dwell (V)
HAL_PIN(iq_mean);  // *output*, mean iq in the last dwell (A)
HAL_PIN(w_mean);   // *output*, mean electrical speed in the last dwell (rad/s)
HAL_PIN(fail);     // *output*, 0 none, 1 stalled, 2 never settled, 3 adv did not converge, 4 adv hit its limit (3 ms, or 0 with a negative fit)
HAL_PIN(adv_n);    // *parameter*, number of speeds in the adv fit, 2..8, default 4
HAL_PIN(adv_c);    // *output*, fit intercept, what is left on d that is not lag (V)
HAL_PIN(adv_tau);  // *output*, lag the last fit found on top of adv (s)

#define DK_MAX 1.2     // drop_k search range 0..DK_MAX
#define DK_ITER 7      // bisection steps, DK_MAX / 2^7 = 0.01
#define DK_CYCLES 5    // step cycles per try, the first one is not measured
#define DK_SKIP 0.002  // ignore this long after each edge: the loop's own rise [s]

#define ADV_MAX 0.003    // [s]
#define ADV_NMAX 8       // most speeds in the fit
#define ADV_PASSES 4     // fits before giving up
#define ADV_TOL_FIT 0.00002  // done when a fit moves adv less than this [s]
#define ADV_DWELL 0.5    // averaging time per dwell [s]
#define VEL_SETTLE 0.5   // speed inside the band this long before the first dwell [s]
#define VEL_BAND 0.15    // settled band, fraction of test_vel
#define VEL_SPINUP 8.0   // give up reaching test_vel after [s]
#define VEL_STALL 0.5    // q current, fraction of test_cur, that has to turn the rotor

struct idtune_ctx_t {
  float t;      // time in this try or stage
  float lo, hi; // drop_k bracket
  uint8_t it;   // drop_k tries done
  uint8_t stage;  // adv: 0 spin up, 1 dwell
  float se;     // sum of the signed step error
  uint32_t n;   // samples in the sums
  float vel_lp;
  float settle_t;
  float sud, suq, sid, siq, sw;
  uint8_t sp;    // adv fit: speed index
  uint8_t pass;  // adv fit: fits done
  float fx[ADV_NMAX], fy[ADV_NMAX], fw[ADV_NMAX];  // uq * w, ud_err, w of the last fit's dwells
  float dk;  // drop_k from the bisection, held while the adv dwells run with 0
  uint8_t kept;  // dk is the config's (drop_knee curve on), not measured
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idtune_pin_ctx_t *pins = (struct idtune_pin_ctx_t *)pin_ptr;

  // above ~2 A per phase the latch's model holds
  PIN(step_cur)  = 6.0;
  PIN(step_freq) = 10.0;
  PIN(test_cur)  = 3.0;
  PIN(test_vel)  = 100.0;
  PIN(ki)        = 1.0;
  PIN(vel_bw)    = 250.0;  // P = vel_bw * period, 0.05 A per rad/s at 5 kHz
  PIN(adv_n)     = 4.0;
}

static int adv_speeds(struct idtune_pin_ctx_t *pins) {
  return CLAMP((int)(PIN(adv_n) + 0.5), 2, ADV_NMAX);
}

static float adv_target(struct idtune_pin_ctx_t *pins, int sp) {
  return PIN(test_vel) * (0.5 + 1.5 * (float)sp / (float)(adv_speeds(pins) - 1));
}

static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idtune_ctx_t *ctx = (struct idtune_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idtune_ctx_t));
}

// an answer at either end of 0..DK_MAX is not a measurement
static void dk_edge_warn(float dk) {
  float step = DK_MAX / (float)(1 << DK_ITER);
  if(dk < step || dk > DK_MAX - step) {
    printf("<font color='red'># drop_k %f is at the edge of its 0..%f search range: not a result.\n", dk, DK_MAX);
    printf("# rerun at a higher step_cur, or check the dc link.</font>\n");
  }
}

static void print_dk(struct idtune_ctx_t *ctx, struct idtune_pin_ctx_t *pins, const char *tag) {
  if(ctx->kept) {
    printf("<font color='green'># hv0.drop_k %f kept: drop_knee %f is on, so drop_k comes from\n", ctx->dk, PIN(drop_knee));
    printf("# id_pmsm's curve fit, not from steps here.</font>\n");
    if(ctx->dk <= 0.0) {
      printf("<font color='red'># hv0.drop_k is 0 with drop_knee on: run id_pmsm and save its drop_k.</font>\n");
    }
  } else {
    printf("hv0.drop_k = %f <font color='green'># %s</font>\n", ctx->dk, tag);
    dk_edge_warn(ctx->dk);
  }
}

static void print_fit(struct idtune_ctx_t *ctx, struct idtune_pin_ctx_t *pins) {
  int n = adv_speeds(pins);
  printf("<font color='green'># adv fit, %i passes, ud_err = -tau uq w + c: tau %f ms on top, c %f V\n", ctx->pass, PIN(adv_tau) * 1000.0, PIN(adv_c));
  printf("# last pass, w el rad/s / ud_err V:");
  for(int i = 0; i < n; i++) {
    printf(" %.0f/%.3f", ctx->fw[i], ctx->fy[i]);
  }
  printf("</font>\n");
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idtune_ctx_t *ctx      = (struct idtune_ctx_t *)ctx_ptr;
  struct idtune_pin_ctx_t *pins = (struct idtune_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 13:
      print_fit(ctx, pins);
      print_dk(ctx, pins, "append to config");
      printf("hv0.adv = %f <font color='green'># append to config</font>\n", PIN(adv));
      if(!ctx->kept) {
        printf("<font color='green'># drop_k from +-%f A steps on d at %f V link.</font>\n", PIN(step_cur), PIN(dc_volt));
      }
      printf("<font color='green'># adv from ud at %f el. rad/s: %f V, expected %f V,\n", PIN(w_mean), PIN(ud_exp) + PIN(ud_err), PIN(ud_exp));
      printf("# uq %f V, iq %f A.</font>\n", PIN(uq_mean), PIN(iq_mean));
      printf("done\n");
      PIN(state) = 1.5;
      break;

    case 14:
      if(PIN(fail) == 1.0) {
        printf("<font color='red'>adv failed</font>: the rotor stalled. check the load is off and\n");
        printf("conf0.polecount, conf0.mot_fb_offset and out_rev\n");
      } else if(PIN(fail) == 2.0) {
        printf("<font color='red'>adv failed</font>: the speed never settled inside %i%% of %f rad/s\n", (int)(VEL_BAND * 100.0), PIN(test_vel));
      } else if(PIN(fail) == 4.0) {
        if(PIN(adv) <= 0.0) {
          printf("<font color='red'>adv failed</font>: the fit wants a negative advance (last move %f ms). ud at\n", PIN(adv_tau) * 1000.0);
        } else {
          printf("<font color='red'>adv failed</font>: it ran to the %f s limit. ud at\n", PIN(adv));
        }
        printf("this speed is not an angle lag; check conf0.lq and conf0.r, raise idtune0.test_vel (%f rad/s) and rerun\n", PIN(test_vel));
      } else {
        print_fit(ctx, pins);
        printf("<font color='red'>adv failed</font>: %i fits did not settle, last %f s,\n", ADV_PASSES, PIN(adv));
        printf("last move %f ms. check conf0.lq and conf0.r\n", PIN(adv_tau) * 1000.0);
      }
      print_dk(ctx, pins, "this part is measured");
      PIN(state) = 1.5;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idtune_ctx_t *ctx      = (struct idtune_ctx_t *)ctx_ptr;
  struct idtune_pin_ctx_t *pins = (struct idtune_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(en_out)   = 0.0;
      PIN(cmd_mode) = 1.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_sum)  = 0.0;
      PIN(fail)     = 0.0;
      if(PIN(en) > 0.0) {
        memset(ctx, 0, sizeof(struct idtune_ctx_t));
        ctx->hi       = DK_MAX;
        PIN(adv)      = 0.0;
        PIN(drop_k)   = DK_MAX / 2.0;
        PIN(state)    = 1.1;
        if(PIN(drop_knee) > 0.0) {
          ctx->kept   = 1;
          ctx->dk     = PIN(drop_k_cfg);
          PIN(drop_k) = 0.0;
          PIN(state)  = 1.2;
        }
      }
      break;

    case 11: {  // drop_k
      PIN(en_out) = 1.0;
      PIN(q_cmd)  = 0.0;
      float half  = 0.5 / MAX(PIN(step_freq), 1.0);
      float tc    = fmodf(ctx->t, 2.0 * half);
      float cmd   = tc < half ? PIN(step_cur) : -PIN(step_cur);
      float since = tc < half ? tc : tc - half;
      PIN(d_cmd)  = cmd;

      if(ctx->t >= 2.0 * half && since >= DK_SKIP) {
        ctx->se += (PIN(id_fb) - cmd) / cmd;
        ctx->n++;
      }
      ctx->t += period;

      if(ctx->t >= DK_CYCLES * 2.0 * half) {
        PIN(dk_err) = ctx->n ? ctx->se / (float)ctx->n : 0.0;
        if(PIN(dk_err) < 0.0) {
          ctx->lo = PIN(drop_k);  // current short of the command: under
        } else {
          ctx->hi = PIN(drop_k);
        }
        PIN(drop_k) = (ctx->lo + ctx->hi) / 2.0;
        ctx->t      = 0.0;
        ctx->se     = 0.0;
        ctx->n      = 0;
        if(++ctx->it >= DK_ITER) {
          PIN(d_cmd) = 0.0;
          ctx->it    = 0;
          ctx->stage = 0;
          ctx->dk    = PIN(drop_k);
          PIN(drop_k) = 0.0;
          PIN(state) = 1.2;
        }
      }
    } break;

    case 12: {  // adv
      int nsp   = adv_speeds(pins);
      float v_t = adv_target(pins, ctx->sp);
      ctx->vel_lp += (PIN(vel_fb) - ctx->vel_lp) * period / 0.05;
      ctx->t += period;

      PIN(en_out) = 1.0;
      PIN(d_cmd)  = 0.0;
      // the clamp slews the integrator only: a clamped P limit-cycles
      float vel_raw   = v_t - PIN(vel_fb);
      float vel_error = LIMIT(vel_raw, ABS(v_t) / 100.0);
      PIN(cur_sum) += PIN(ki) * vel_error * period;
      PIN(cur_sum) = LIMIT(PIN(cur_sum), PIN(test_cur));
      PIN(q_cmd)   = LIMIT(PIN(vel_bw) * period * vel_raw + PIN(cur_sum), PIN(test_cur));

      // stall on the integrator, not the P term, which spikes at spin up
      if(ABS(ctx->vel_lp) < ABS(v_t) * 0.1 && ABS(PIN(cur_sum)) > PIN(test_cur) * VEL_STALL) {
        PIN(fail) = 1.0;
      } else if(ctx->stage == 0) {  // spin up
        if(ABS(ctx->vel_lp - v_t) < ABS(v_t) * VEL_BAND) {
          ctx->settle_t += period;
        } else {
          ctx->settle_t = 0.0;
        }
        if(ctx->settle_t >= VEL_SETTLE) {
          ctx->stage = 1;
          ctx->t     = 0.0;
        } else if(ctx->t > VEL_SPINUP) {
          PIN(fail) = 2.0;
        }
      } else {  // dwell
        ctx->sud += PIN(ud_fb);
        ctx->suq += PIN(uq_fb);
        ctx->sid += PIN(id_fb);
        ctx->siq += PIN(iq_fb);
        ctx->sw += PIN(vel_e);
        ctx->n++;
        if(ctx->t >= ADV_DWELL) {
          float n      = (float)ctx->n;
          float lq     = PIN(lq) > 0.0 ? PIN(lq) : PIN(l);
          PIN(w_mean)  = ctx->sw / n;
          PIN(uq_mean) = ctx->suq / n;
          PIN(iq_mean) = ctx->siq / n;
          PIN(ud_exp)  = PIN(r) * ctx->sid / n - PIN(w_mean) * lq * PIN(iq_mean);
          PIN(ud_err)  = ctx->sud / n - PIN(ud_exp);
          ctx->sud = ctx->suq = ctx->sid = ctx->siq = ctx->sw = 0.0;
          ctx->n   = 0;
          ctx->t   = 0.0;
          ctx->fx[ctx->sp] = PIN(uq_mean) * PIN(w_mean);
          ctx->fy[ctx->sp] = PIN(ud_err);
          ctx->fw[ctx->sp] = PIN(w_mean);
          ctx->stage       = 0;  // settle at the next speed
          ctx->settle_t    = 0.0;
          if(++ctx->sp >= nsp) {
            // least squares line through (uq w, ud_err)
            float mx = 0.0, my = 0.0, sxx = 0.0, sxy = 0.0;
            for(int i = 0; i < nsp; i++) {
              mx += ctx->fx[i] / nsp;
              my += ctx->fy[i] / nsp;
            }
            for(int i = 0; i < nsp; i++) {
              sxx += (ctx->fx[i] - mx) * (ctx->fx[i] - mx);
              sxy += (ctx->fx[i] - mx) * (ctx->fy[i] - my);
            }
            float tau    = sxx > 0.0 ? -sxy / sxx : 0.0;
            PIN(adv_tau) = tau;
            PIN(adv_c)   = my + tau * mx;
            PIN(adv)     = CLAMP(PIN(adv) + tau, 0.0, ADV_MAX);
            ctx->sp      = 0;
            ctx->pass++;
            // converged first: a fit that settles at the 0 floor is a result
            if(ABS(tau) < ADV_TOL_FIT) {
              PIN(state) = 1.3;
            } else if(PIN(adv) >= ADV_MAX || (PIN(adv) <= 0.0 && tau < 0.0)) {
              PIN(fail) = 4.0;
            } else if(ctx->pass >= ADV_PASSES) {
              PIN(fail) = 3.0;
            }
          }
        }
      }
      if(PIN(fail) > 0.0) {
        PIN(state) = 1.4;
      }
    } break;

    case 13:
    case 14:
    case 15:
      PIN(drop_k)  = ctx->dk;
      PIN(en_out)  = 0.0;
      PIN(d_cmd)   = 0.0;
      PIN(q_cmd)   = 0.0;
      PIN(cur_sum) = 0.0;
      break;
  }
}

hal_comp_t idtune_comp_struct = {
    .name      = "idtune",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idtune_ctx_t),
    .pin_count = sizeof(struct idtune_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
