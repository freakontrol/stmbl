#include "idm_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"
#include <string.h>

/**
 * id_mot / id_sys (comp idm): j, f, d and o from a back and forth profile
 * between min_pos and max_pos about the start position, with each
 * estimate fed back into pid's feedforward.
 *
 * 1.2: five rounds, speed and acceleration stepping up to max_vel/max_acc.
 * 1.3: J_TIME s at max_vel and max_acc; j, f, d and o adapt on fb_torque.
 *      Then the speed steps through LEVELS x max_vel, LEVEL_FLIPS moves
 *      each, at max_acc. On every constant speed stretch, after PLAT_SETTLE,
 *      the mean torque_cmd and speed go into a bin per level and direction
 *      (stretches shorter than 2 PLAT_SETTLE are dropped).
 *      f, d and o come from a least squares fit of T = f sign(v) + d v + o
 *      over the bin means: at one speed sign(v) and v move together and the
 *      f/d split drifts from run to run. With every bin on one side (a
 *      vel_offset over max_vel), f and o can not be told apart: d is fitted
 *      and f takes the rest, o stays as adapted.
 *      At the end the axis drives back to where the run started.
 */
HAL_COMP(idm);

HAL_PIN(en);

HAL_PIN(state);
HAL_PIN(sub_state);
HAL_PIN(timer);
HAL_PIN(acc_time);
HAL_PIN(vel_time);
HAL_PIN(time);

HAL_PIN(freq);
HAL_PIN(amp);
HAL_PIN(min_pos);
HAL_PIN(max_pos);
HAL_PIN(max_vel);
HAL_PIN(max_acc);

HAL_PIN(pos);
HAL_PIN(pos_fb);      // fb_switch0.pos_fb: the profile starts where the rotor is
HAL_PIN(pos_cmd);
HAL_PIN(vel_cmd);
HAL_PIN(acc_cmd);
HAL_PIN(vel_offset);  // added to the profile's speed, 0 = none; > max_vel keeps one direction (sensorless)
HAL_PIN(vel_out);     // vel_cmd + vel_offset

HAL_PIN(ji);
HAL_PIN(fi);
HAL_PIN(di);
HAL_PIN(li);

HAL_PIN(torque);
HAL_PIN(fb_torque);

HAL_PIN(inertia_sum);
HAL_PIN(friction_sum);
HAL_PIN(damping_sum);
HAL_PIN(offset_sum);

HAL_PIN(inertia);
HAL_PIN(damping);
HAL_PIN(friction);
HAL_PIN(offset);

HAL_PIN(pos_bw);
HAL_PIN(vel_bw);
HAL_PIN(vel_d);

HAL_PIN(sys);

HAL_PIN(target);
HAL_PIN(auto_step);

HAL_PIN(fit_n);    // plateau bins in the f, d, o fit
HAL_PIN(fit_rms);  // fit residual over the bins [Nm]
HAL_PIN(fit_lo);   // lowest plateau speed [rad/s]
HAL_PIN(fit_hi);   // highest plateau speed [rad/s]

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
      return;
    }
    f = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2]) + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2])) / det;
    d = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2]) - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0])) / det;
    o = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1]) - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0]) + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])) / det;
  } else {  // one direction only: T = c + d v, f = (c - o) sign
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
      return;
    }
    d = sxy / sxx;
    o = PIN(offset);
    f = (mt - d * mv - o) * (pos > 0 ? 1.0 : -1.0);
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
