#include "fault_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "main.h"
#include "common.h"

HAL_COMP(fault);

HAL_PIN(en);
HAL_PIN(state);
HAL_PIN(fault);
HAL_PIN(last_fault);
HAL_PIN(en_out);
HAL_PIN(en_fb);
HAL_PIN(en_pid);

HAL_PIN(fb_ready);

HAL_PIN(cmd_error);
HAL_PIN(mot_fb_error);
HAL_PIN(com_fb_error);
HAL_PIN(joint_fb_error);
HAL_PIN(hv_error);

HAL_PIN(hv_temp);
HAL_PIN(mot_temp);
HAL_PIN(max_hv_temp);
HAL_PIN(max_mot_temp);
HAL_PIN(high_hv_temp);
HAL_PIN(high_mot_temp);
HAL_PIN(fan_hv_temp);
HAL_PIN(fan_mot_temp);

// bridge junction estimate from ipm0.temp, 0 when ipm is not linked
HAL_PIN(ipm_temp);
HAL_PIN(max_ipm_temp);
HAL_PIN(high_ipm_temp);

HAL_PIN(scale);

HAL_PIN(dc_volt);
HAL_PIN(min_dc_volt);
HAL_PIN(high_dc_volt);
HAL_PIN(max_dc_volt);

HAL_PIN(dc_cur);
HAL_PIN(max_dc_cur);

HAL_PIN(ac_cur);
HAL_PIN(max_ac_cur);

HAL_PIN(pos_error);
HAL_PIN(max_pos_error);

HAL_PIN(sat);
HAL_PIN(max_sat);

HAL_PIN(mot_brake);
HAL_PIN(brake_during_phasing);
HAL_PIN(dc_brake);
HAL_PIN(brake_en_delay);
HAL_PIN(brake_dis_delay);

HAL_PIN(hv_fan);
HAL_PIN(mot_fan);

HAL_PIN(print);

HAL_PIN(brake_release);

HAL_PIN(warn_timer);
HAL_PIN(error_timer);
HAL_PIN(brake_timer);

// Short-circuit braking (f3 io.c) after a stop that leaves the bridge
// healthy: a disable, or a command, feedback, following, saturation or link
// fault. sbrake_en also arms the f3 to brake on its own on a link loss.
HAL_PIN(sbrake_en);
HAL_PIN(sbrake_time);  // [s], default 1
// SOFT_FAULT is held at least this long [s] before en 0 may clear it, so a
// fault taken while en is already 0 (a trip in a regenerative stop, hold or
// short brake) reaches LinuxCNC: the 1 kHz sserial packet sampled the one
// tick it used to last about one time in five. Default 0.02.
HAL_PIN(fault_hold);
HAL_PIN(sbrake);       // request to hv0.sbrake, out

// Regenerative stop: on the same edge, if the feedback and the f3 link still
// work, keep the bridge, feedback and pid on for up to rstop_time while
// pid0.stop (rstop) commands zero speed, so the energy goes back into the
// link. Done below rstop_vel; out of time, or a fault it cannot ride through,
// hands over to the short brake. mot_brake stays released while decelerating,
// then the bridge holds zero speed for brake_dis_delay with it engaged.
HAL_PIN(rstop_en);    // *parameter*, 1 = regenerative stop first
HAL_PIN(rstop_time);  // *parameter*, longest regenerative stop [s], default 1
HAL_PIN(rstop_vel);   // *parameter*, done below this speed [rad/s], default 2
HAL_PIN(vel_fb);      // motor speed [rad/s], in
HAL_PIN(rstop);       // regenerative stop running, out, to pid0.stop

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
