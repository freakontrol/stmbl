#include "fault_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "main.h"
#include "common.h"

/**
* ## Brief
* `fault` is the central enable / fault state machine of the F4 board. It collects error flags, temperatures, voltages, currents and controller saturation, decides whether the drive is disabled, phasing, enabled or faulted, gates the other components with `en_out`, `en_fb` and `en_pid`, controls the motor brake and fans, outputs a derating factor `scale`, and after a stop can decelerate the motor regeneratively (`rstop`) or ask the F3 for short-circuit braking (`sbrake`). It is loaded by nearly every template (`pid.txt`, `mpid.txt`, `uf.txt`, the `id_*` templates, ...). Typical links: `hv0.en = fault0.en_out`, `fb_switch0.en = fault0.en_fb`, `pid0.en = fault0.en_pid`, `hv0.scale = fault0.scale`, `fault0.fb_ready = fb_switch0.state`, `fault0.hv_error = hv0.fault`, `fault0.sat = pid0.sat`, `io0.state = fault0.state`, `io0.fault = fault0.fault`, and the limits from `conf0`. pid.txt also links `hv0.sbrake = fault0.sbrake`, `hv0.sbrake_arm = fault0.sbrake_en`, `pid0.stop = fault0.rstop`, `fault0.vel_fb = vel1.vel`; ipm.txt links `fault0.ipm_temp = ipm0.temp`. The commands `enable` and `disable` set `fault0.en` to 1 or 0.
*
* ## Component Explanation
* The state machine and all checks run in `rt`; messages are printed in `nrt`.
*
* 1. **States** (`state` pin, `state_t` in `common.h`):
* - `0` DISABLED: all enables 0, brake engaged, fault cleared. Goes to PHASING when `en > 0`.
* - `2` PHASING: `en_out = en_fb = 1`, `en_pid = 0`. Waits for `fb_ready > 0` (commutation found), then goes to DELAYED_ENABLED if `brake_en_delay > 0`, else ENABLED. `mot_brake = !brake_during_phasing`.
* - `6` DELAYED_ENABLED: everything enabled but the brake stays engaged until `brake_timer` (loaded with `brake_en_delay`) reaches 0, then ENABLED.
* - `1` ENABLED: all enables 1, `mot_brake = 1` (released), `last_fault` cleared. When `en <= 0` goes to DELAYED_DISABLED if `brake_dis_delay > 0` and `rstop_en <= 0` (brake engaged, drive still enabled for that time), else DISABLED. With `rstop_en > 0` the brake delay is applied after the regenerative stop instead (see 5).
* - `7` DELAYED_DISABLED: goes to DISABLED when `brake_timer` reaches 0.
* - `3` SOFT_FAULT: all enables 0, brake engaged, fault code kept. Only `en <= 0` leaves it (to DISABLED, which clears `fault`; `last_fault` keeps the code), and only once the state has been held for `fault_hold`. So a fault taken while `en` is already 0 (a trip in a regenerative stop, hold or short brake) still reaches LinuxCNC: the 1 kHz sserial packet saw the one tick it used to last only about one time in five.
* - `4` HARD_FAULT and `5` LED_TEST: outputs off, never left (not entered by this component).
* - DELAYED_ENABLED and PHASING also go straight to DISABLED when `en <= 0`.
* - While `rstop` is 1 the enables are forced to 1 in any state (see 5).
*
* 2. **Fault checks** (any of them sets `fault` and `last_fault` and switches to SOFT_FAULT, from any state):
* - `1` CMD_ERROR, `2` MOT_FB_ERROR, `3` COM_FB_ERROR, `4` JOINT_FB_ERROR: the matching `*_error` input is > 0. Filtered: a counter goes up by 1 per rt cycle with error and down by 0.001 without, the fault trips at 5, so about 5 consecutive bad cycles are needed.
* - `5` POS_ERROR: `abs(pos_error) > max_pos_error` (only if `max_pos_error > 0`), not filtered.
* - `6` SAT_ERROR: `sat > max_sat` (only if `max_sat > 0`), not filtered.
* - `10` HV_TEMP_ERROR: `hv_temp > max_hv_temp`, filtered.
* - `11` HV_VOLT_ERROR: `dc_volt > max_dc_volt` or `dc_volt < min_dc_volt`, filtered.
* - `7` MOT_TEMP_ERROR: `mot_temp > max_mot_temp`, filtered.
* - `17` IPM_TEMP_ERROR: `ipm_temp > max_ipm_temp` (the bridge junction estimate of `ipm0`), filtered. `ipm_temp` stays 0 when `ipm` is not loaded, so the check never trips then.
* - Any `hv_error > 0`: its value is used as fault code, e.g. `8` HV_CRC_ERROR, `9` HV_TIMEOUT_ERROR, `12` HV_FAULT_ERROR, `13` HV_CURRENT_OFFSET_FAULT, `14` HV_OVERCURRENT_FILTERED, `15` HV_OVERCURRENT_PEAK, `16` HV_OVERCURRENT_HW.
* - `0` is NO_ERROR. The fault text is printed by `nrt` on state change. A fault taken while already disabled (in a regenerative stop) can leave SOFT_FAULT again before `nrt` sees the state (after `fault_hold`, or in the next tick with `fault_hold = 0`); `rt` keeps its code until `nrt` has printed it, so it is not lost.
*
* 3. **Derating** (`scale`, 0..1, the minimum of):
* - `hv_temp` from `high_hv_temp` (1) to `max_hv_temp` (0).
* - `dc_volt` from `high_dc_volt` to `max_dc_volt`.
* - `mot_temp` from `high_mot_temp` to `max_mot_temp`.
* - `ipm_temp` from `high_ipm_temp` to `max_ipm_temp`.
* - `ac_cur` from `max_ac_cur` to `1.1 * max_ac_cur`, and `dc_cur` from `max_dc_cur` to `1.1 * max_dc_cur`.
* - `max_ac_cur` and `max_dc_cur` have no default and must be set > 0, otherwise `scale` becomes 0.
* - The pid, uf and vf templates link `fault0.dc_cur = hv0.dc_cur` and `fault0.max_dc_cur = conf0.max_dc_cur` (default 15 A). Older configs did not read `dc_cur`, so a config whose `conf0.max_dc_cur` is below the real link current now derates: set it to the supply rating.
* - In `nrt`, once per second at most, a warning is printed while `dc_volt > high_dc_volt`, `mot_temp > high_mot_temp`, `hv_temp > high_hv_temp` or `ipm_temp > high_ipm_temp`.
*
* 4. **Brake chopper, fans and brake**:
* - `dc_brake` ramps from 0 at `high_dc_volt` to 1 at `max_dc_volt`.
* - `hv_fan` switches on at `fan_hv_temp` and off below 90 % of it; `mot_fan` likewise with `mot_temp` / `fan_mot_temp`.
* - `brake_release > 0` forces `mot_brake` to 1 in every state.
*
* 5. **Stopping: regenerative stop, then short brake**:
* - A stop is the edge from a powered state (ENABLED, DELAYED_ENABLED, DELAYED_DISABLED, PHASING) to DISABLED or SOFT_FAULT, i.e. a disable or a fault.
* - Regenerative stop (`rstop_en > 0`): if the fault is one the drive can ride through (none, CMD_ERROR, JOINT_FB_ERROR, POS_ERROR, SAT_ERROR: motor feedback, F3 link and DC link still good) and the stop is not out of PHASING, `rstop` goes to 1 for up to `rstop_time`. `en_out`, `en_fb`, `en_pid` are held at 1 and `mot_brake` released, while `pid0.stop` makes the controller command zero speed without position loop or feed forward, so the energy goes back into the DC link.
* - Vertical axes: SAT_ERROR and POS_ERROR also start the regenerative stop, so after such a fault the brake stays released for up to `rstop_time` while the controller may no longer hold the load. On an axis with gravity load keep `rstop_en` 0 or `rstop_time` short.
* - When `abs(vel_fb) < rstop_vel` the motor counts as stopped: `rstop` stays 1 for another `brake_dis_delay` with the drive still enabled and `mot_brake` engaged (0), so the bridge holds zero speed while the brake closes. Enabling again ends both phases.
* - Short-circuit brake (`sbrake_en > 0`): when the regenerative stop is off or not allowed for this fault, runs out of `rstop_time`, is switched off (`rstop_en`), comes out of PHASING (the commutation is not found yet, so the bridge must not be driven again for a regenerative stop or the hold) or is hit by a fault it cannot ride through, `sbrake` goes to 1 for `sbrake_time`. `hv` passes it to the F3, which turns all three low sides on and chops the current (see hv). It is only kept while the drive stays unpowered, `rstop` is 0 and the fault is one that leaves the bridge healthy (none, CMD/MOT_FB/COM_FB/JOINT_FB, POS, SAT, HV_CRC or HV_TIMEOUT error); HV temperature, voltage, overcurrent, gate fault or IPM faults cancel it.
* - `sbrake_en` is also wired to `hv0.sbrake_arm`, which lets the F3 brake on its own when the F4 link goes silent.
*
* {{% hint warning %}}
* There is no velocity error fault. The pid, mpid, uf and vf templates link `min_dc_volt = conf0.low_dc_volt` (24 V in the conf template), which overrides the 20 V default. In `uf.txt` the `scale` output also scales the V/f output voltage (see `uf2`).
* {{% /hint %}}
*/

HAL_COMP(fault);

HAL_PIN(en);          // *input*, Enable request, set by the enable / disable commands
HAL_PIN(state);       // *output*, State (0 disabled, 1 enabled, 2 phasing, 3 soft fault, 4 hard fault, 5 led test, 6 delayed enabled, 7 delayed disabled)
HAL_PIN(fault);       // *output*, Active fault code (fault_t), 0 = no error
HAL_PIN(last_fault);  // *output*, Last fault code, cleared when ENABLED is reached
HAL_PIN(en_out);      // *output*, Enable for the power stage (hv0.en), also 1 while rstop
HAL_PIN(en_fb);       // *output*, Enable for feedback / commutation (fb_switch0.en), also 1 while rstop
HAL_PIN(en_pid);      // *output*, Enable for the controller (pid0.en), also 1 while rstop

HAL_PIN(fb_ready);  // *input*, Commutation found, PHASING ends when > 0

HAL_PIN(cmd_error);       // *input*, Command source error, > 0 = error
HAL_PIN(mot_fb_error);    // *input*, Motor feedback error, > 0 = error
HAL_PIN(com_fb_error);    // *input*, Commutation feedback error, > 0 = error
HAL_PIN(joint_fb_error);  // *input*, Joint feedback error, > 0 = error
HAL_PIN(hv_error);        // *input*, Fault code from the power stage (hv0.fault)

HAL_PIN(hv_temp);        // *input*, Power stage temperature (degC)
HAL_PIN(mot_temp);       // *input*, Motor temperature (degC)
HAL_PIN(max_hv_temp);    // *parameter*, Power stage fault temperature (degC, default 90)
HAL_PIN(max_mot_temp);   // *parameter*, Motor fault temperature (degC, default 100)
HAL_PIN(high_hv_temp);   // *parameter*, Power stage derating start (degC, default 70)
HAL_PIN(high_mot_temp);  // *parameter*, Motor derating start (degC, default 80)
HAL_PIN(fan_hv_temp);    // *parameter*, Power stage fan on temperature (degC, default 60)
HAL_PIN(fan_mot_temp);   // *parameter*, Motor fan on temperature (degC, default 60)

HAL_PIN(ipm_temp);       // *input*, Bridge junction temperature estimate (degC, ipm0.temp), 0 when ipm is not linked
HAL_PIN(max_ipm_temp);   // *parameter*, Junction fault temperature (degC, default 140)
HAL_PIN(high_ipm_temp);  // *parameter*, Junction derating start (degC, default 125)

HAL_PIN(scale);  // *output*, Derating factor 0..1

HAL_PIN(dc_volt);       // *input*, DC link voltage (V)
HAL_PIN(min_dc_volt);   // *parameter*, Undervoltage fault (V, default 20)
HAL_PIN(high_dc_volt);  // *parameter*, Overvoltage derating / brake chopper start (V, default 370)
HAL_PIN(max_dc_volt);   // *parameter*, Overvoltage fault (V, default 390)

HAL_PIN(dc_cur);      // *input*, DC link current (A)
HAL_PIN(max_dc_cur);  // *parameter*, DC current derating start (A), must be set

HAL_PIN(ac_cur);      // *input*, Motor current magnitude (A)
HAL_PIN(max_ac_cur);  // *parameter*, AC current derating start (A), must be set

HAL_PIN(pos_error);      // *input*, Position error (rad)
HAL_PIN(max_pos_error);  // *parameter*, Position error fault limit (rad), 0 = off

HAL_PIN(sat);      // *input*, Controller saturation time (s)
HAL_PIN(max_sat);  // *parameter*, Saturation fault limit (s), 0 = off

HAL_PIN(mot_brake);             // *output*, Motor brake, 1 = released
HAL_PIN(brake_during_phasing);  // *parameter*, 1 = keep the brake engaged while phasing
HAL_PIN(dc_brake);              // *output*, Brake chopper duty 0..1
HAL_PIN(brake_en_delay);        // *parameter*, Delay between brake release and ENABLED (s)
HAL_PIN(brake_dis_delay);       // *parameter*, Time the drive stays enabled after the brake engages (s)

HAL_PIN(hv_fan);   // *output*, Power stage fan
HAL_PIN(mot_fan);  // *output*, Motor fan

HAL_PIN(print);  // *input/output*, Set > 0 to print the current state once, reset to 0

HAL_PIN(brake_release);  // *input*, > 0 forces the brake released

HAL_PIN(warn_timer);   // *output*, Warning message rate limit timer (s)
HAL_PIN(error_timer);  // *output*, Counts down to 0, otherwise unused (s)
HAL_PIN(brake_timer);  // *output*, Remaining brake delay (s)

HAL_PIN(sbrake_en);    // *parameter*, 1 = short-circuit brake (f3 io.c) after a stop that leaves the bridge healthy, also arms the F3 to brake on a link loss (hv0.sbrake_arm), default 0
HAL_PIN(sbrake_time);  // *parameter*, Short-circuit braking time (s, default 1)
HAL_PIN(fault_hold);   // *parameter*, Minimum time a SOFT_FAULT is held before en 0 may clear it (s, default 0.02)
HAL_PIN(sbrake);       // *output*, Short-circuit braking request (hv0.sbrake)

HAL_PIN(rstop_en);    // *parameter*, 1 = regenerative stop (pid0.stop, zero speed) before the short brake, default 0
HAL_PIN(rstop_time);  // *parameter*, Longest regenerative stop (s, default 1)
HAL_PIN(rstop_vel);   // *parameter*, Stop counts as done below this speed (rad/s, default 2)
HAL_PIN(vel_fb);      // *input*, Motor speed (rad/s)
HAL_PIN(rstop);       // *output*, Regenerative stop or hold running (pid0.stop)

//fault strings for fault_t form common.h
static const char *fault_string[] = {
    "no error",
    "CMD error",
    "mot FB error",
    "com FB error",
    "joint FB error",
    "position error",
    "saturation error",
    "Motor overtemperature",
    "HV crc error",
    "HV timeout error",
    "HV overtemperature",
    "HV volt error",
    "HV fault error",
    "Current offset fault",
    "HV overcurrent filtered",
    "HV overcurrent peak",
    "HV overcurrent hw limit",
    "IPM junction overtemperature",
};

struct fault_ctx_t {
  state_t state;
  fault_t fault;
  float cmd_error;
  float mot_fb_error;
  float com_fb_error;
  float joint_fb_error;
  float hv_temp_error;
  float dc_volt_error;
  float mot_temp_error;
  float sbrake_timer;
  float rstop_timer;
  float rhold_timer;  // after the stop: brake engaging, bridge still holding
  float fault_hold_timer;
  float ipm_temp_error;
  // a fault taken while already disabled (in a regenerative stop) leaves
  // SOFT_FAULT in the next tick, before nrt sees the state: rt keeps it here
  // until nrt has printed it
  volatile uint32_t fault_print;
};

// stops after which the bridge may still be used to brake the motor
static int sbrake_safe(fault_t fault) {
  switch(fault) {
    case NO_ERROR:
    case CMD_ERROR:
    case MOT_FB_ERROR:
    case COM_FB_ERROR:
    case JOINT_FB_ERROR:
    case POS_ERROR:
    case SAT_ERROR:
    case HV_CRC_ERROR:
    case HV_TIMEOUT_ERROR:
      return 1;
    default:
      return 0;
  }
}

// stops after which the drive may keep running to decelerate the motor:
// the feedback, the f3 link and the link voltage have to be good
static int rstop_safe(fault_t fault) {
  switch(fault) {
    case NO_ERROR:
    case CMD_ERROR:
    case JOINT_FB_ERROR:
    case POS_ERROR:
    case SAT_ERROR:
      return 1;
    default:
      return 0;
  }
}

// states in which the bridge is driving the motor
static int powered(state_t state) {
  return state == ENABLED || state == DELAYED_ENABLED || state == DELAYED_DISABLED || state == PHASING;
}

void enable(char *ptr) {
  hal_parse("fault0.en = 1");
}

COMMAND("enable", enable, "enable");

void disable(char *ptr) {
  hal_parse("fault0.en = 0");
}

COMMAND("disable", disable, "disable");


static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fault_ctx_t *ctx      = (struct fault_ctx_t *)ctx_ptr;
  struct fault_pin_ctx_t *pins = (struct fault_pin_ctx_t *)pin_ptr;

  ctx->state         = DISABLED;
  ctx->fault         = NO_ERROR;
  PIN(last_fault)    = NO_ERROR;
  PIN(min_dc_volt)   = 20.0;
  PIN(high_dc_volt)  = 370.0;
  PIN(max_dc_volt)   = 390.0;
  PIN(max_hv_temp)   = 90.0;
  PIN(max_mot_temp)  = 100.0;
  PIN(high_hv_temp)  = 70.0;
  PIN(high_mot_temp) = 80.0;
  PIN(fan_hv_temp)   = 60.0;
  PIN(fan_mot_temp)  = 60.0;
  PIN(sbrake_time)   = 1.0;
  PIN(fault_hold)    = 0.02;
  ctx->fault_hold_timer = 0.0;
  ctx->sbrake_timer  = 0.0;
  PIN(rstop_time)    = 1.0;
  PIN(rstop_vel)     = 2.0;
  ctx->rstop_timer   = 0.0;
  ctx->rhold_timer   = 0.0;
  ctx->fault_print   = NO_ERROR;
  PIN(max_ipm_temp)  = 140.0;
  PIN(high_ipm_temp) = 125.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fault_ctx_t *ctx      = (struct fault_ctx_t *)ctx_ptr;
  struct fault_pin_ctx_t *pins = (struct fault_pin_ctx_t *)pin_ptr;

  state_t last_state = ctx->state;

  switch(ctx->state) {
    case DISABLED:
      if(PIN(en) > 0.0) {
        ctx->state = PHASING;
      }
      break;

    case DELAYED_DISABLED:
      if (PIN(brake_timer) <= 0.0) {
        ctx->state = DISABLED;
      }
      break;

    case ENABLED:
      if(PIN(en) <= 0.0) {
        // with rstop the brake delay runs after the motor has stopped
        if(PIN(brake_dis_delay) > 0.0 && PIN(rstop_en) <= 0.0) {
          ctx->state = DELAYED_DISABLED;
          PIN(brake_timer) = PIN(brake_dis_delay);
        } else {
          ctx->state = DISABLED;
        }
      }
      break;

    case DELAYED_ENABLED:
      if (PIN(brake_timer) <= 0.0) {
        ctx->state = ENABLED;
      }

      if(PIN(en) <= 0.0) {
        ctx->state = DISABLED;
      }
      break;

    case PHASING:
      if(PIN(fb_ready) > 0.0) {
        if (PIN(brake_en_delay) > 0.0) {
          ctx->state = DELAYED_ENABLED;
          PIN(brake_timer) = PIN(brake_en_delay);
        } else {
          ctx->state = ENABLED;
        }
      }

      if(PIN(en) <= 0.0) {
        ctx->state = DISABLED;
      }
      break;

    case SOFT_FAULT:
      if(PIN(en) <= 0.0 && ctx->fault_hold_timer <= 0.0) {
        ctx->state = DISABLED;
      }
      break;

    case LED_TEST:
    case HARD_FAULT:
      break;
  }

  if(err_filter(&(ctx->cmd_error), 5.0, 0.001, PIN(cmd_error) > 0.0)) {
    ctx->fault      = CMD_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(err_filter(&(ctx->mot_fb_error), 5.0, 0.001, PIN(mot_fb_error) > 0.0)) {
    ctx->fault      = MOT_FB_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(err_filter(&(ctx->com_fb_error), 5.0, 0.001, PIN(com_fb_error) > 0.0)) {
    ctx->fault      = COM_FB_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(err_filter(&(ctx->joint_fb_error), 5.0, 0.001, PIN(joint_fb_error) > 0.0)) {
    ctx->fault      = JOINT_FB_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(PIN(max_pos_error) > 0 && (ABS(PIN(pos_error)) > PIN(max_pos_error))) {
    ctx->fault      = POS_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(PIN(max_sat) > 0 && (PIN(sat) > PIN(max_sat))) {
    ctx->fault      = SAT_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(err_filter(&(ctx->hv_temp_error), 5.0, 0.001, PIN(hv_temp) > PIN(max_hv_temp))) {
    ctx->fault      = HV_TEMP_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(err_filter(&(ctx->dc_volt_error), 5.0, 0.001, PIN(dc_volt) > PIN(max_dc_volt) || PIN(dc_volt) < PIN(min_dc_volt))) {
    ctx->fault      = HV_VOLT_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(err_filter(&(ctx->mot_temp_error), 5.0, 0.001, PIN(mot_temp) > PIN(max_mot_temp))) {
    ctx->fault      = MOT_TEMP_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  if(err_filter(&(ctx->ipm_temp_error), 5.0, 0.001, PIN(ipm_temp) > PIN(max_ipm_temp))) {
    ctx->fault      = IPM_TEMP_ERROR;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  float hv_error = PIN(hv_error);
  if(hv_error > 0.0) {
    ctx->fault      = hv_error;
    PIN(last_fault) = ctx->fault;
    ctx->state      = SOFT_FAULT;
  }

  float scale = 1.0;
  scale       = MIN(scale, SCALE(PIN(hv_temp), PIN(high_hv_temp), PIN(max_hv_temp)));
  scale       = MIN(scale, SCALE(PIN(dc_volt), PIN(high_dc_volt), PIN(max_dc_volt)));
  scale       = MIN(scale, SCALE(PIN(mot_temp), PIN(high_mot_temp), PIN(max_mot_temp)));
  scale       = MIN(scale, SCALE(PIN(ipm_temp), PIN(high_ipm_temp), PIN(max_ipm_temp)));
  scale       = MIN(scale, SCALE(PIN(ac_cur), PIN(max_ac_cur), PIN(max_ac_cur) * 1.1));
  scale       = MIN(scale, SCALE(PIN(dc_cur), PIN(max_dc_cur), PIN(max_dc_cur) * 1.1));

  PIN(dc_brake) = SCALE(PIN(dc_volt), PIN(max_dc_volt), PIN(high_dc_volt));

  if(PIN(hv_temp) >= PIN(fan_hv_temp)) {
    PIN(hv_fan) = 1.0;
  }

  if(PIN(hv_temp) < PIN(fan_hv_temp) * 0.9) {
    PIN(hv_fan) = 0.0;
  }

  if(PIN(mot_temp) >= PIN(fan_mot_temp)) {
    PIN(mot_fan) = 1.0;
  }

  if(PIN(mot_temp) < PIN(fan_mot_temp) * 0.9) {
    PIN(mot_fan) = 0.0;
  }

  if(ctx->state == SOFT_FAULT && last_state != SOFT_FAULT) {
    ctx->fault_hold_timer = PIN(fault_hold);
    ctx->fault_print = ctx->fault;
  }

  int stop_edge = powered(last_state) && !powered(ctx->state) && ctx->state != HARD_FAULT && ctx->state != LED_TEST;
  int fallback  = 0;  // regenerative stop given up: short brake instead
  if(stop_edge) {
    // not out of PHASING: commutation is not found yet, so the bridge must
    // not be driven again for a regenerative stop or the hold
    if(PIN(rstop_en) > 0.0 && rstop_safe(ctx->fault) && PIN(rstop_time) > 0.0 && last_state != PHASING) {
      ctx->rstop_timer = PIN(rstop_time);
    } else {
      fallback = 1;
    }
  }
  if(ctx->rstop_timer > 0.0) {
    if(powered(ctx->state)) {  // enabled again: nothing to stop
      ctx->rstop_timer = 0.0;
    } else if(ABS(PIN(vel_fb)) < PIN(rstop_vel)) {  // stopped: hold while the brake closes
      ctx->rstop_timer = 0.0;
      ctx->rhold_timer = MAX(PIN(brake_dis_delay), 0.0);
    } else if(PIN(rstop_en) <= 0.0 || !rstop_safe(ctx->fault) || ctx->rstop_timer <= period) {
      ctx->rstop_timer = 0.0;  // cannot or did not finish
      fallback         = 1;
    } else {
      ctx->rstop_timer -= period;
    }
  }
  if(ctx->rhold_timer > 0.0) {
    if(powered(ctx->state) || PIN(rstop_en) <= 0.0 || !rstop_safe(ctx->fault)) {
      ctx->rhold_timer = 0.0;
    } else {
      ctx->rhold_timer = MAX(ctx->rhold_timer - period, 0.0);
    }
  }
  PIN(rstop) = ctx->rstop_timer > 0.0 || ctx->rhold_timer > 0.0;

  if(PIN(sbrake_en) > 0.0 && fallback) {
    ctx->sbrake_timer = PIN(sbrake_time);
  }
  if(PIN(sbrake_en) <= 0.0 || powered(ctx->state) || !sbrake_safe(ctx->fault) || PIN(rstop) > 0.0) {
    ctx->sbrake_timer = 0.0;
  }
  PIN(sbrake)       = ctx->sbrake_timer > 0.0;
  ctx->sbrake_timer = MAX(ctx->sbrake_timer - period, 0.0);

  switch(ctx->state) {
    case DISABLED:
      ctx->fault = NO_ERROR;
      /* FALLTHRU */
    case SOFT_FAULT:
    case LED_TEST:
    case HARD_FAULT:
      PIN(mot_brake) = 0.0;
      PIN(en_out)    = 0.0;
      PIN(en_fb)     = 0.0;
      PIN(en_pid)    = 0.0;
      break;

    case DELAYED_DISABLED:
    case DELAYED_ENABLED:
      PIN(mot_brake) = 0.0;
      PIN(en_out)    = 1.0;
      PIN(en_fb)     = 1.0;
      PIN(en_pid)    = 1.0;
      ctx->fault     = NO_ERROR;
      break;

    case ENABLED:
      PIN(mot_brake)  = 1.0;
      PIN(en_out)     = 1.0;
      PIN(en_fb)      = 1.0;
      PIN(en_pid)     = 1.0;
      ctx->fault      = NO_ERROR;
      PIN(last_fault) = NO_ERROR;
      break;

    case PHASING:
      PIN(mot_brake) = !PIN(brake_during_phasing);
      ctx->fault     = NO_ERROR;
      PIN(en_pid)    = 0.0;
      PIN(en_fb)     = 1.0;
      PIN(en_out)    = 1.0;
      break;
  }

  if(PIN(rstop) > 0.0) {  // keep driving until stopped and the brake has closed
    PIN(mot_brake) = ctx->rstop_timer > 0.0;  // released while decelerating, engaging in the hold
    PIN(en_out)    = 1.0;
    PIN(en_fb)     = 1.0;
    PIN(en_pid)    = 1.0;
  }

  PIN(fault) = ctx->fault;
  PIN(state) = ctx->state;
  PIN(scale) = scale;

  if(PIN(brake_release) > 0.0) {
    PIN(mot_brake) = 1.0;
  }

  PIN(warn_timer)  = MAX(PIN(warn_timer) - period, 0.0);
  ctx->fault_hold_timer = MAX(ctx->fault_hold_timer - period, 0.0);
  PIN(error_timer) = MAX(PIN(error_timer) - period, 0.0);
  if (ctx->state == DELAYED_ENABLED || ctx->state == DELAYED_DISABLED) {
    PIN(brake_timer) = MAX(PIN(brake_timer) - period, 0.0);
  }
}


static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fault_ctx_t *ctx      = (struct fault_ctx_t *)ctx_ptr;
  struct fault_pin_ctx_t *pins = (struct fault_pin_ctx_t *)pin_ptr;

  if(PIN(warn_timer) <= 0.0) {
    if(PIN(dc_volt) > PIN(high_dc_volt)) {
      printf("<font color='orange'>WARNING:</font> over voltage current clamping active\n");
      PIN(warn_timer) = 1.0;
    }

    if(PIN(mot_temp) > PIN(high_mot_temp)) {
      printf("<font color='orange'>WARNING:</font> over temperature (motor) current clamping active\n");
      PIN(warn_timer) = 1.0;
    }

    if(PIN(hv_temp) > PIN(high_hv_temp)) {
      printf("<font color='orange'>WARNING:</font> over temperature (driver) current clamping active\n");
      PIN(warn_timer) = 1.0;
    }

    if(PIN(ipm_temp) > PIN(high_ipm_temp)) {
      printf("<font color='orange'>WARNING:</font> over temperature (ipm junction) current clamping active\n");
      PIN(warn_timer) = 1.0;
    }
  }

  // read and clear in one step: rt can write a new fault between a load and a store
  uint32_t fault_print = __atomic_exchange_n(&ctx->fault_print, NO_ERROR, __ATOMIC_RELAXED);
  if(fault_print != NO_ERROR) {
    printf("ERROR: Fault %lu: %s\n", fault_print, fault_print < sizeof(fault_string) / sizeof(fault_string[0]) ? fault_string[fault_print] : "unknown");
  }

  //TODO: fix EDGE
  if(EDGE(ctx->state) || PIN(print) > 0.0) {
    PIN(print) = 0.0;
    switch((state_t)ctx->state) {
      case DISABLED:
        printf("INFO: Disabled \n");
        break;

      case DELAYED_DISABLED:
        printf("INFO: Delayed disabled \n");
        break;

      case ENABLED:
        printf("INFO: Enabled \n");
        break;

      case DELAYED_ENABLED:
        printf("INFO: Delayed enabled \n");
        break;

      case PHASING:
        printf("INFO: Phasing \n");
        break;

      case SOFT_FAULT:
        if(fault_print == NO_ERROR) {  // not printed just above
          printf("ERROR: Fault %lu: %s\n", (uint32_t)ctx->fault, fault_string[(uint32_t)ctx->fault]);
        }
        break;

      case HARD_FAULT:
        printf("ERROR: Hard fault: \n");
        break;

      default:
        break;
    }
  }
}

hal_comp_t fault_comp_struct = {
    .name      = "fault",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct fault_ctx_t),
    .pin_count = sizeof(struct fault_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
