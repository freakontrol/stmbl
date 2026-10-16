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

HAL_COMP(idtune);

HAL_PIN(en);
HAL_PIN(en_out);
HAL_PIN(state);
HAL_PIN(cmd_mode);
HAL_PIN(d_cmd);
HAL_PIN(q_cmd);

HAL_PIN(adv);     // out, to hv0.adv [s]
HAL_PIN(drop_k);  // out, to hv0.drop_k
HAL_PIN(drop_k_cfg);  // the config's hv0.drop_k, kept when drop_knee > 0
HAL_PIN(drop_knee);   // hv0.drop_knee, > 0 skips the drop_k step

HAL_PIN(id_fb);
HAL_PIN(iq_fb);
HAL_PIN(ud_fb);
HAL_PIN(uq_fb);
HAL_PIN(vel_fb);  // mechanical speed, for the speed loop [rad/s]
HAL_PIN(vel_e);   // electrical speed, what hv0.vel sees [rad/s]
HAL_PIN(dc_volt);

HAL_PIN(r);
HAL_PIN(l);
HAL_PIN(lq);  // 0 = same as l

HAL_PIN(step_cur);   // *parameter*, d step amplitude [A]
HAL_PIN(step_freq);  // *parameter*, d step frequency [Hz]
HAL_PIN(test_cur);   // *parameter*, q current limit of the speed loop [A]
HAL_PIN(test_vel);   // *parameter*, dwell speed, mechanical [rad/s]
HAL_PIN(ki);
HAL_PIN(vel_bw);
HAL_PIN(cur_sum);

HAL_PIN(dk_err);   // signed mean step error of the last drop_k try, fraction of step_cur
HAL_PIN(ud_err);   // ud - ud expected in the last dwell [V]
HAL_PIN(ud_exp);   // ud expected in the last dwell [V]
HAL_PIN(uq_mean);  // uq in the last dwell [V]
HAL_PIN(iq_mean);  // iq in the last dwell [A]
HAL_PIN(w_mean);   // electrical speed in the last dwell [rad/s]
HAL_PIN(fail);     // 0 none, 1 stalled, 2 never settled, 3 adv did not converge, 4 adv hit its limit
HAL_PIN(adv_n);    // *parameter*, speeds in the adv fit, 2..8
HAL_PIN(adv_c);    // fit intercept: what is left on d that is not lag [V]
HAL_PIN(adv_tau);  // lag the last fit found on top of adv [s]

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
        printf("<font color='red'>adv failed</font>: it ran to %f s. ud at this speed is not\n", PIN(adv));
        printf("an angle lag; raise idtune0.test_vel (%f rad/s) and rerun\n", PIN(test_vel));
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
      float vel_error = LIMIT(vel_raw, v_t / 100.0);
      PIN(cur_sum) += PIN(ki) * vel_error * period;
      PIN(cur_sum) = LIMIT(PIN(cur_sum), PIN(test_cur));
      PIN(q_cmd)   = LIMIT(PIN(vel_bw) * period * vel_raw + PIN(cur_sum), PIN(test_cur));

      // stall on the integrator, not the P term, which spikes at spin up
      if(ABS(ctx->vel_lp) < v_t * 0.1 && ABS(PIN(cur_sum)) > PIN(test_cur) * VEL_STALL) {
        PIN(fail) = 1.0;
      } else if(ctx->stage == 0) {  // spin up
        if(ABS(ctx->vel_lp - v_t) < v_t * VEL_BAND) {
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
            if(PIN(adv) >= ADV_MAX || (PIN(adv) <= 0.0 && tau < 0.0)) {
              PIN(fail) = 4.0;
            } else if(ABS(tau) < ADV_TOL_FIT) {
              PIN(state) = 1.3;
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
