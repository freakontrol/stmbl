#include "iddac_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"
#include "common.h"
#include <math.h>

/**
 * id_dac (comp iddac): one shot search for the overcurrent comparator threshold
 * (hv0.dac) that trips at a wanted current. It prints the hv0.dac line to put
 * in the config; nothing is saved.
 *
 * The comparators are one-sided, so the bridge trips only with the current
 * vector at 60, 180 and 300 deg el. For each dac tried, d current ramps from
 * 0 at those three fixed angles (the unloaded rotor aligns to them) until the
 * f3 reports HV_OVERCURRENT_HW or the ramp reaches cur. A dac passes when all
 * three angles trip by cur. Bisection finds the highest passing dac between
 * dac_lo and dac_hi, so no angle trips above cur.
 *
 * mode 1, loaded or blocked axis: hv0.pos follows the rotor (com_pos) and
 * the current stays on d, so it makes no torque and the rotor need not move.
 * The current vector then sits at the rotor angle or opposite it; each ramp
 * takes the sign whose vector lies within 30 deg of a trip angle and counts
 * the current that phase sees, |i| cos(offset), against cur (the vector
 * goes up to 1.155 x cur). One ramp per dac tests only the comparator of
 * that phase; trip1/trip2 stay 0 and trip_phase says which one (0 = 60 deg).
 * Rotate the axis by hand between runs to check the others.
 *
 * mode 0, free axis: the three fixed angles, so all three comparators.
 *
 * Expect one comparator trip per ramp, a few dozen in all.
 * max_cur (conf0.max_ac_cur) must be over 1.05 x cur, 1.2 x in mode 1
 * (the vector goes up to 1.155 x cur); otherwise the run stops at once with
 * fail 1 and leaves hv0.dac at dac_lo.
 */
HAL_COMP(iddac);

HAL_PIN(en);        // fault0.en_out
HAL_PIN(en_out);    // to hv0.en
HAL_PIN(cmd_mode);  // to hv0.cmd_mode
HAL_PIN(d_cmd);     // to hv0.d_cmd
HAL_PIN(q_cmd);     // to hv0.q_cmd
HAL_PIN(pos);       // to hv0.pos
HAL_PIN(dac);       // to hv0.dac
HAL_PIN(hv_fault);  // in, hv0.fault
HAL_PIN(hv_error);  // out, to fault0.hv_error: hv0.fault without the trips we cause
HAL_PIN(max_cur);   // in, conf0.max_ac_cur
HAL_PIN(com_pos);   // in, rotor electrical angle (fb_switch0.com_fb), mode 1
HAL_PIN(mode);      // *parameter*, 0 free axis, 1 loaded or blocked axis

HAL_PIN(cur);       // *parameter*, wanted trip current [A peak]
HAL_PIN(cur_max);   // *parameter*, hard ceiling on cur [A], default 25
HAL_PIN(ramp);      // *parameter*, ramp rate [A/s], default 20
HAL_PIN(dac_lo);    // *parameter*, search floor, default 150 (0 A ringing trips below)
HAL_PIN(dac_hi);    // *parameter*, search ceiling, default 400

HAL_PIN(state);     // 0 idle, 1 running, 2 done, 3 failed
HAL_PIN(fail);      // 1 cur too high, 2 other hv fault, 3 fault did not clear, 4 dac_lo does not trip, 5 dac_hi trips
HAL_PIN(trip0);     // trip currents at the passing dac [A]: 60, 180, 300 deg
HAL_PIN(trip1);
HAL_PIN(trip2);
HAL_PIN(trip_phase);  // mode 1: the comparator tested, 0/1/2 = 60/180/300 deg

#define SETTLE 0.1   // after a dac change or before a ramp [s]
#define RESET_T 0.3  // bridge off after a trip [s]
#define CLEAR_T 2.0  // longest wait for hv0.fault to clear [s]
#define ALIGN 1.0    // start current, lets the rotor align [A]

enum { IDLE, SET_DAC, RAMP, OFF, DONE, FAILED };

struct iddac_ctx_t {
  int phase;
  int lo, hi, dac;  // bisection: lo passes, hi fails
  int lo_checked, hi_checked;
  int angle;
  float t;
  float i;
  float sign, c;  // mode 1: d sign and cos of the offset to the trip angle
  int phase_k;
  float trip[3];
  float best[3];
  int tripped;
  int printed;
  int failcode;
};

static const float angles[3] = {M_PI / 3.0, M_PI, 5.0 * M_PI / 3.0};

// mode 1: pick the d sign whose current vector is nearest a trip angle
static void pick_sign(struct iddac_ctx_t *ctx, float com) {
  float best = -2.0;
  for(int s = 0; s < 2; s++) {
    float v = com + (s ? M_PI : 0.0);
    for(int k = 0; k < 3; k++) {
      float c = cosf(minus(v, angles[k]));
      if(c > best) {
        best         = c;
        ctx->sign    = s ? -1.0 : 1.0;
        ctx->c       = c;
        ctx->phase_k = k;
      }
    }
  }
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct iddac_pin_ctx_t *pins = (struct iddac_pin_ctx_t *)pin_ptr;
  PIN(cur)     = 20.0;
  PIN(cur_max) = 25.0;
  PIN(ramp)    = 20.0;
  PIN(dac_lo)  = 150.0;
  PIN(dac_hi)  = 400.0;
  PIN(cmd_mode) = 1.0;  // CURRENT_MODE
}

static void finish(struct iddac_ctx_t *ctx, struct iddac_pin_ctx_t *pins, int fail) {
  ctx->phase  = fail ? FAILED : DONE;
  PIN(fail)   = fail;
  PIN(state)  = fail ? 3.0 : 2.0;
  PIN(en_out) = 0.0;
  PIN(d_cmd)  = 0.0;
  // leave the comparator at the result, or at the safe floor dac_lo when
  // the run stopped before the search started (fail 1 after boot: lo is 0)
  ctx->dac = ctx->lo > 0 ? ctx->lo : (int)PIN(dac_lo);
}

// a whole test of one dac is over: pass when all three angles tripped
static void next_dac(struct iddac_ctx_t *ctx, int pass) {
  if(!ctx->lo_checked) {
    ctx->lo_checked = 1;
    if(!pass) {
      ctx->failcode = 4;
      ctx->phase    = FAILED;
      return;
    }
    for(int k = 0; k < 3; k++) {
      ctx->best[k] = ctx->trip[k];
    }
    ctx->dac = ctx->hi;
  } else if(!ctx->hi_checked) {
    ctx->hi_checked = 1;
    if(pass) {
      ctx->failcode = 5;
      ctx->phase    = FAILED;
      return;
    }
  } else if(pass) {
    ctx->lo = ctx->dac;
    for(int k = 0; k < 3; k++) {
      ctx->best[k] = ctx->trip[k];
    }
  } else {
    ctx->hi = ctx->dac;
  }
  if(ctx->hi_checked) {
    if(ctx->hi - ctx->lo <= 1) {
      ctx->phase = DONE;
      return;
    }
    ctx->dac = (ctx->lo + ctx->hi) / 2;
  }
  ctx->angle   = 0;
  ctx->tripped = 0;
  ctx->phase   = SET_DAC;
  ctx->t       = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct iddac_ctx_t *ctx      = (struct iddac_ctx_t *)ctx_ptr;
  struct iddac_pin_ctx_t *pins = (struct iddac_pin_ctx_t *)pin_ptr;

  int f = (int)PIN(hv_fault);
  // trips we cause stay away from fault0 while running
  PIN(hv_error) = (ctx->phase != IDLE && ctx->phase != DONE && ctx->phase != FAILED && f == HV_OVERCURRENT_HW) ? 0.0 : PIN(hv_fault);

  if(PIN(en) <= 0.0) {
    // disabled: stop, a run cut short leaves state 0; the next enable runs again
    if(PIN(state) == 1.0 || PIN(state) == 3.0) {  // a result (2) stays until reset
      PIN(state) = 0.0;
      ctx->dac   = ctx->lo > 0 ? ctx->lo : (int)PIN(dac_lo);
    }
    ctx->phase  = IDLE;
    PIN(en_out) = 0.0;
    PIN(d_cmd)  = 0.0;
    PIN(q_cmd)  = 0.0;
    PIN(dac) = ctx->dac > 0 ? ctx->dac : PIN(dac_lo);
    return;
  }

  ctx->t += period;
  switch(ctx->phase) {
    case IDLE:
      if(PIN(state) >= 2.0) {  // finished: one shot, waits for en to drop
        PIN(en_out) = 0.0;
        PIN(d_cmd)  = 0.0;
        break;
      }
      if(PIN(cur) <= ALIGN || PIN(cur) > PIN(cur_max) || PIN(cur) * (PIN(mode) > 0.0 ? 1.2 : 1.05) > PIN(max_cur)) {
        finish(ctx, pins, 1);
        break;
      }
      ctx->lo         = (int)PIN(dac_lo);
      ctx->hi         = MAX((int)PIN(dac_hi), ctx->lo + 2);
      ctx->dac        = ctx->lo;
      ctx->lo_checked = 0;
      ctx->hi_checked = 0;
      ctx->angle      = 0;
      ctx->tripped    = 0;
      ctx->printed    = 0;
      ctx->failcode   = 0;
      ctx->t          = 0.0;
      ctx->phase      = SET_DAC;
      PIN(state)      = 1.0;
      PIN(fail)       = 0.0;
      break;

    case SET_DAC:  // bridge off, new dac on its way to the f3, fault clear
      PIN(en_out) = 0.0;
      PIN(d_cmd)  = 0.0;
      if(f != NO_ERROR) {
        if(ctx->t > CLEAR_T) {
          finish(ctx, pins, 3);
          ctx->printed = 0;
        }
        break;
      }
      if(ctx->t > SETTLE) {
        ctx->i      = ALIGN;
        ctx->t      = 0.0;
        ctx->sign   = 1.0;
        ctx->c      = 1.0;
        if(PIN(mode) > 0.0) {
          pick_sign(ctx, PIN(com_pos));
        }
        ctx->phase  = RAMP;
      }
      break;

    case RAMP:
      PIN(en_out) = 1.0;
      PIN(pos)    = PIN(mode) > 0.0 ? PIN(com_pos) : angles[ctx->angle];
      if(f == HV_OVERCURRENT_HW) {
        ctx->trip[ctx->angle] = ctx->i * ctx->c;  // what the tripping phase saw
        ctx->tripped++;
        ctx->phase = OFF;
        ctx->t     = 0.0;
        break;
      }
      if(f != NO_ERROR) {
        finish(ctx, pins, 2);
        break;
      }
      if(ctx->t > SETTLE) {
        ctx->i += PIN(ramp) * period;
      }
      if(ctx->i * ctx->c > PIN(cur)) {  // no trip by cur: this dac is too high
        ctx->trip[ctx->angle] = 0.0;
        PIN(d_cmd)            = 0.0;
        PIN(en_out)           = 0.0;
        next_dac(ctx, 0);
        break;
      }
      PIN(d_cmd) = ctx->sign * ctx->i;
      break;

    case OFF:
      PIN(en_out) = 0.0;
      PIN(d_cmd)  = 0.0;
      if(ctx->t > RESET_T) {
        if(ctx->angle < (PIN(mode) > 0.0 ? 0 : 2)) {
          ctx->angle++;
          ctx->phase = SET_DAC;
          ctx->t     = 0.0;
        } else {
          next_dac(ctx, ctx->tripped == (PIN(mode) > 0.0 ? 1 : 3));
        }
      }
      break;

    case DONE:
    case FAILED:
      PIN(en_out) = 0.0;
      PIN(d_cmd)  = 0.0;
      if(PIN(state) == 1.0) {
        finish(ctx, pins, ctx->phase == FAILED ? ctx->failcode : 0);
        PIN(trip0) = ctx->best[0];
        PIN(trip1) = ctx->best[1];
        PIN(trip2) = ctx->best[2];
        PIN(trip_phase) = ctx->phase_k;
      }
      break;
  }
  PIN(q_cmd) = 0.0;
  PIN(dac)   = ctx->dac;
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct iddac_ctx_t *ctx      = (struct iddac_ctx_t *)ctx_ptr;
  struct iddac_pin_ctx_t *pins = (struct iddac_pin_ctx_t *)pin_ptr;

  if(PIN(state) < 2.0 || ctx->printed) {
    if(PIN(state) < 2.0) {
      ctx->printed = 0;
    }
    return;
  }
  ctx->printed = 1;
  if(PIN(state) == 2.0) {
    if(PIN(mode) > 0.0) {
      printf("hv0.dac = %i <font color='green'># trips by %.1f A: %.1f A phase current, comparator at %i deg only</font>\n",
             ctx->lo, PIN(cur), PIN(trip0), 60 + 120 * (int)PIN(trip_phase));
    } else {
      printf("hv0.dac = %i <font color='green'># trips by %.1f A: %.1f / %.1f / %.1f A at 60 / 180 / 300 deg</font>\n",
             ctx->lo, PIN(cur), PIN(trip0), PIN(trip1), PIN(trip2));
    }
    printf("<font color='green'># append to config, then reset: iddac is a one shot</font>\n");
    return;
  }
  switch((int)PIN(fail)) {
    case 1:
      printf("<font color='red'>iddac failed</font>: cur %.1f A must be over %.1f A, at most cur_max %.1f A and under conf0.max_ac_cur / 1.05, / 1.2 in mode 1 (%.1f A)\n", PIN(cur), ALIGN, PIN(cur_max), PIN(max_cur));
      break;
    case 2:
      printf("<font color='red'>iddac failed</font>: hv fault %i during a ramp\n", (int)PIN(hv_fault));
      break;
    case 3:
      printf("<font color='red'>iddac failed</font>: hv0.fault %i did not clear %.0f s after a trip\n", (int)PIN(hv_fault), CLEAR_T);
      break;
    case 4:
      printf("<font color='red'>iddac failed</font>: dac_lo %.0f does not trip by %.1f A at every angle; lower dac_lo or raise cur\n", PIN(dac_lo), PIN(cur));
      break;
    case 5:
      printf("<font color='red'>iddac failed</font>: dac_hi %.0f still trips by %.1f A at every angle; raise dac_hi\n", PIN(dac_hi), PIN(cur));
      break;
  }
}

hal_comp_t iddac_comp_struct = {
    .name      = "iddac",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct iddac_ctx_t),
    .pin_count = sizeof(struct iddac_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
