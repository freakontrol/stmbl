#include "iddac_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"
#include "common.h"
#include <math.h>

/**
* ## Brief
* `iddac` (loaded by `conf/template/id_dac.txt` as `id_dac`) is a one shot identification on the F4 board that searches the F3 hardware overcurrent comparator threshold `hv0.dac` that trips at a wanted phase current `cur` (A peak). It drives `hv0` in current mode itself, bisects the dac, and prints the `hv0.dac = ...` line to append to the config. Nothing is saved; for normal operation use `ocdac` or a fixed `hv0.dac`.
*
* ## Component Explanation
*
* 1. **Procedure**:
* - Set `conf0.max_ac_cur` to at least 1.1 x `cur` (1.2 x in mode 1). `cur` must be over 1 A, at most `cur_max` and under `max_cur / 1.05` (mode 0) or `max_cur / 1.2` (mode 1), otherwise the run fails with `fail = 1`. Too low a `max_cur` also lets the F3 software trip (fault 15) or `fault0` fire first and stop the run.
* - `link id_dac` (or load the template). It disables `pid0`, `fault0.pos_error` and `fault0.sat`, routes `hv0.en`, `cmd_mode`, `d_cmd`, `q_cmd`, `pos` and `dac` from this component, and inserts `hv_error` between `hv0.fault` and `fault0.hv_error`. The template sets `mode = 1` and `cur = 20`.
* - Enable the drive. The run starts on `en > 0`, `state` goes to 1, and at the end the result is printed in the console (green) or the failure reason (red). Append the printed line to the config and reset: a finished run (`state = 2`) is a one shot and does not repeat until reset; after a failure, dropping `en` returns `state` to 0 and the next enable runs again.
* - Expect one comparator trip per ramp, a few dozen in all.
*
* 2. **Search (rt)**:
* - The comparators are one-sided, so the bridge trips only with the current vector at 60, 180 and 300 deg el.
* - For each dac tried: bridge off, wait for `hv_fault` to clear (up to 2 s, else `fail = 3`) and 0.1 s settle, then the d current ramps at `ramp` A/s from 1 A (after 0.1 s alignment) until the F3 reports `HV_OVERCURRENT_HW` or the ramp reaches `cur`. After a trip the bridge stays off 0.3 s.
* - A dac passes when every tested angle trips by `cur`. `dac_lo` must pass (else `fail = 4`) and `dac_hi` must fail (else `fail = 5`); then bisection finds the highest passing dac, so no angle trips above `cur`.
* - Any other hv fault during a ramp stops with `fail = 2`.
*
* 3. **Mode 0, free axis**:
* - `pos` is set to the three fixed angles 60, 180, 300 deg in turn (the unloaded rotor aligns to them), so all three comparators are tested.
*
* 4. **Mode 1, loaded or blocked axis**:
* - `pos` follows the rotor (`com_pos`) and the current stays on d, so it makes no torque and the rotor need not move.
* - The current vector then sits at the rotor angle or opposite it; each ramp takes the d sign whose vector lies within 30 deg of a trip angle and counts the current that phase sees, `|i| * cos(offset)`, against `cur` (the vector goes up to 1.155 x `cur`).
* - One ramp per dac tests only the comparator of that phase: the result is in `trip0`, `trip1`/`trip2` stay 0. Rotate the axis by hand between runs to check the others.
*
* 5. **Fault handling and outputs**:
* - While running, `HV_OVERCURRENT_HW` trips caused by the search are masked out of `hv_error`; any other `hv_fault` (and all faults when idle, done or failed) pass through to `fault0`.
* - With `en <= 0` the bridge is off and `dac` outputs the last result, or `dac_lo` before the first run; an interrupted or failed run returns `state` to 0, a finished one keeps `state = 2`.
* - On success or failure the comparator is left at the last passing dac (the result, or `dac_lo`).
* - `cmd_mode` is 1 (current mode) from nrt_init.
*
* {{% hint warning %}}
* After `fail = 1` (bad `cur`) on the first run, the `dac` output is 0 until `en` drops, because the search bounds were never set; the bridge stays off meanwhile. The console text for `fail = 3` also says "disabled", but disabling does not set `fail`.
* {{% /hint %}}
*/
HAL_COMP(iddac);

HAL_PIN(en);        // *input*, enable, from fault0.en_out
HAL_PIN(en_out);    // *output*, bridge enable, to hv0.en
HAL_PIN(cmd_mode);  // *output*, hv command mode, 1 = current, to hv0.cmd_mode
HAL_PIN(d_cmd);     // *output*, d current command (A peak), to hv0.d_cmd
HAL_PIN(q_cmd);     // *output*, q current command, always 0, to hv0.q_cmd
HAL_PIN(pos);       // *output*, electrical angle of the current vector (rad), to hv0.pos
HAL_PIN(dac);       // *output*, overcurrent comparator dac under test (0..4095), to hv0.dac
HAL_PIN(hv_fault);  // *input*, hv fault code, from hv0.fault
HAL_PIN(hv_error);  // *output*, hv0.fault without the HW overcurrent trips the search causes, to fault0.hv_error
HAL_PIN(max_cur);   // *input*, current limit (A peak), from conf0.max_ac_cur
HAL_PIN(com_pos);   // *input*, rotor electrical angle (rad), from fb_switch0.com_fb, used in mode 1
HAL_PIN(mode);      // *parameter*, 0 = free axis (three angles), 1 = loaded or blocked axis (rotor d axis)

HAL_PIN(cur);       // *parameter*, wanted trip current (A peak), default 20
HAL_PIN(cur_max);   // *parameter*, hard ceiling on cur (A peak), default 25
HAL_PIN(ramp);      // *parameter*, current ramp rate (A/s), default 20
HAL_PIN(dac_lo);    // *parameter*, search floor, must trip by cur, default 150 (ringing trips at 0 A below)
HAL_PIN(dac_hi);    // *parameter*, search ceiling, must not trip by cur, default 400

HAL_PIN(state);     // *output*, 0 = idle, 1 = running, 2 = done, 3 = failed
HAL_PIN(fail);      // *output*, 0 = ok, 1 = bad cur, 2 = other hv fault, 3 = fault did not clear, 4 = dac_lo does not trip, 5 = dac_hi trips
HAL_PIN(trip0);     // *output*, trip current at the result dac (A peak), 60 deg (mode 1: the tested phase)
HAL_PIN(trip1);     // *output*, trip current at the result dac (A peak), 180 deg, mode 0 only
HAL_PIN(trip2);     // *output*, trip current at the result dac (A peak), 300 deg, mode 0 only
HAL_PIN(trip_phase);  // *output*, mode 1: comparator tested, 0/1/2 = 60/180/300 deg

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
