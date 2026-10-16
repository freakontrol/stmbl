// Host stand-in for stm32f303/src/comps/io.c: phase current and voltage
// sampling, the trips and the bridge enable/short-brake state machine, with
// the same pins. The ADC values come from the plant (runner/plant.c, sampled
// at the tick), the bridge enable is sim_tim8.moe, and the over-current
// comparators are modelled in the plant (sim_tim8.brk). The trip and enable
// logic follows the firmware. Not modelled: the reconstruction of a phase
// the ADC could not sample (the plant samples every phase cleanly, recon_phase
// stays 0), the fault pin, the hv_temp hold while disabled, the brake output.
#include "io_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "periph.h"
#include "f3hw.h"
#include "common.h"
#include "sim_hw.h"

HAL_COMP(io);

HAL_PIN(led);

//phase current
HAL_PIN(iu);
HAL_PIN(iv);
HAL_PIN(iw);
//total current
HAL_PIN(iabs);

// Rebuild the phase whose low side was not on for the whole sample window
// from the other two (see rt_func). recon 0 turns it off; recon_phase reports
// which phase was rebuilt in this tick, 0 none, 1 u, 2 v, 3 w.
HAL_PIN(recon);
HAL_PIN(recon_phase);

// software overcurrent trip: iabs above oc_k * max_cur, at least oc_min and
// at most ABS_MAX_CURRENT (the measurement range), stops the bridge in the
// same tick. max_cur (conf0.max_ac_cur) sets the module limit, so this is the
// main software trip; ABS_MAX_CURRENT only backs it up
HAL_PIN(max_cur);
HAL_PIN(oc_k);    // default 1.3
HAL_PIN(oc_min);  // default 5 A
HAL_PIN(oc_lim);  // the limit in force [A], out

//phase voltage
HAL_PIN(u);
HAL_PIN(v);
HAL_PIN(w);
//dclink voltage
HAL_PIN(udc);
// the link for the duty division (hv0) and ls0.pwm_volt: 0.5 IIR, about 1.4 ticks, so
// sag on acceleration and rise on regen reach the duty within ~100 us
// instead of udc's 1.3 ms (0.05 IIR, kept for display and trips)
HAL_PIN(udc_duty);

// phase voltages to ground, one unfiltered adc sample per pwm period, for emf0
HAL_PIN(ur);
HAL_PIN(vr);
HAL_PIN(wr);

//driver temoerature
HAL_PIN(hv_temp);
// The IPM's NTC shares VFO, which the module pulls low while the bridge is
// disabled. hv_temp is read while HV_EN is low: enabled, or short braking.
// Otherwise the last value is held and decays toward the coolest live reading
// since power up, since the IPM only heats while switching.
// hv_temp_ok: 0 never read, 1 live, 2 held.
HAL_PIN(hv_temp_ok);
//motor temperature
HAL_PIN(mot_temp);

//ADC offset outputs
HAL_PIN(uo);
HAL_PIN(vo);
HAL_PIN(wo);

//DAC value for comperators
HAL_PIN(dac);

//comperator outputs
HAL_PIN(cu);
HAL_PIN(cv);
HAL_PIN(cw);

//enable in
HAL_PIN(hv_en);

//fault output
HAL_PIN(fault);
HAL_PIN(ignore_fault_pin);

HAL_PIN(brk_present);
HAL_PIN(brk);

// Short-circuit braking while disabled: all three low sides on, so the back
// emf drives current round the windings and the energy stays in the motor.
// Refused after any trip since the last enable (a failed switch plus three
// low sides on is another short). Chopped: a tick with iabs above the limit
// turns the bridge off for the next tick.
HAL_PIN(sbrake);      // request, in
HAL_PIN(sbrake_cur);  // chop threshold [A]; 0 = max_cur, or 10 A without it
HAL_PIN(sbrake_on);   // braking now, out; hv0 holds all compares at 0 on it

struct io_ctx_t {
  float u_offset;
  float v_offset;
  float w_offset;
  float overtemp_error;
  float overvoltage_error;
  float overcurrent_error;
  uint32_t offset_count;
  uint32_t offset_bad;
  uint32_t fault;
  uint32_t enabled;
  uint32_t sbrake_ok;
  uint32_t sbrake_ticks;
};

#define IO_LP_K (750.0 / PWM_FREQ)
#define IO_ERR_TICKS (PWM_FREQ / 3000.0)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_ctx_t *ctx      = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  PIN(brk_present)       = 0.0;
  PIN(brk)               = 0.0;
  ctx->offset_count      = 0;
  ctx->u_offset          = 0.0;
  ctx->v_offset          = 0.0;
  ctx->w_offset          = 0.0;
  ctx->fault             = NO_ERROR;
  ctx->offset_bad        = 0;
  ctx->overtemp_error    = 0;
  ctx->overvoltage_error = 0;
  ctx->overcurrent_error = 0;
  ctx->enabled           = 0;
  ctx->sbrake_ok         = 0;
  ctx->sbrake_ticks      = 0;
  PIN(hv_temp_ok)        = 0.0;
  PIN(recon)             = 1.0;
  PIN(recon_phase)       = 0.0;
  PIN(dac)               = 0;
  PIN(sbrake)            = 0.0;
  PIN(sbrake_cur)        = 0.0;
  PIN(sbrake_on)         = 0.0;
  PIN(oc_k)              = 1.3;
  PIN(oc_min)            = 5.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_ctx_t *ctx      = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  // the board waits 100 ticks, then averages 100 for the offsets (bridge off)
  if(ctx->offset_count < 100) {
    ctx->offset_count++;
  } else if(ctx->offset_count < 100 + 100) {
    ctx->u_offset += sim_f3_adc.iu / 100.0;
    ctx->v_offset += sim_f3_adc.iv / 100.0;
    ctx->w_offset += sim_f3_adc.iw / 100.0;
    ctx->offset_count++;
  } else if(ctx->offset_count < 100 + 100 + 1) {
    if(ABS(ctx->u_offset) > 5.0 || ABS(ctx->v_offset) > 5.0 || ABS(ctx->w_offset) > 5.0) {
      ctx->fault      = HV_CURRENT_OFFSET_FAULT;
      ctx->offset_bad = 1;
    }
    ctx->offset_count++;
  } else {
    PIN(uo)       = ctx->u_offset;
    PIN(vo)       = ctx->v_offset;
    PIN(wo)       = ctx->w_offset;
    PIN(iu)       = sim_f3_adc.iu - ctx->u_offset;
    PIN(iv)       = sim_f3_adc.iv - ctx->v_offset;
    PIN(iw)       = sim_f3_adc.iw - ctx->w_offset;
    PIN(ur)       = sim_f3_adc.ur;
    PIN(vr)       = sim_f3_adc.vr;
    PIN(wr)       = sim_f3_adc.wr;
    PIN(w)        = PIN(wr) * IO_LP_K + PIN(w) * (1.0 - IO_LP_K);
    PIN(v)        = PIN(vr) * IO_LP_K + PIN(v) * (1.0 - IO_LP_K);
    PIN(u)        = PIN(ur) * IO_LP_K + PIN(u) * (1.0 - IO_LP_K);
    float udc_raw = sim_f3_adc.udc;
    PIN(udc)      = udc_raw * IO_LP_K + PIN(udc) * (1.0 - IO_LP_K);
    PIN(udc_duty) = udc_raw * 0.5 + PIN(udc_duty) * 0.5;

    PIN(recon_phase) = 0.0;
    PIN(iabs)        = MAX3(ABS(PIN(iu)), ABS(PIN(iv)), ABS(PIN(iw)));
    PIN(hv_temp)     = sim_f3_adc.hv_temp;
    PIN(hv_temp_ok)  = 1.0;

    if(err_filter(&(ctx->overtemp_error), IO_ERR_TICKS, 0.001, PIN(hv_temp) > ABS_MAX_TEMP)) {
      ctx->fault = HV_TEMP_ERROR;
    }

    if(err_filter(&(ctx->overvoltage_error), IO_ERR_TICKS, 0.001, PIN(udc) > ABS_MAX_VOLT)) {
      ctx->fault = HV_VOLT_ERROR;
    }

    if(err_filter(&(ctx->overcurrent_error), IO_ERR_TICKS, 0.001, PIN(iabs) > ABS_MAX_CURRENT * 0.95)) {
      ctx->fault = HV_OVERCURRENT_FILTERED;
    }

    PIN(oc_lim) = PIN(max_cur) > 0.0 ? CLAMP(PIN(oc_k) * PIN(max_cur), PIN(oc_min), ABS_MAX_CURRENT) : ABS_MAX_CURRENT;
    if(PIN(iabs) > PIN(oc_lim)) {
      ctx->fault = HV_OVERCURRENT_PEAK;
    }

    if(PIN(iabs) > ABS_MAX_CURRENT) {
      ctx->fault = HV_OVERCURRENT_PEAK;
    }

    if(ctx->offset_bad) {
      ctx->fault = HV_CURRENT_OFFSET_FAULT;
    }

    PIN(fault) = ctx->fault;

    if(PIN(hv_en) > 0.0) {
      PIN(sbrake_on)    = 0.0;
      ctx->sbrake_ticks = 0;
      if(!ctx->enabled) {  // rising edge of enable
        sim_tim8.brk = 0;
        sim_tim8.moe = 1;
        ctx->enabled   = 1;
        ctx->sbrake_ok = ctx->fault == NO_ERROR;
      }
      if(ctx->fault != NO_ERROR) {
        ctx->sbrake_ok = 0;
      }
      if(ctx->fault == NO_ERROR) {
        if(!sim_tim8.moe || sim_tim8.brk) {
          ctx->fault = HV_OVERCURRENT_HW;
        }
      }
      if(ctx->fault != NO_ERROR) {
        sim_tim8.moe = 0;
      }
    } else if(PIN(sbrake) > 0.0 && ctx->sbrake_ok && ctx->offset_count > 200) {
      ctx->enabled = 0;
      if(ctx->sbrake_ticks == 0) {
        sim_tim8.brk = 0;
      }
      if(ctx->fault == NO_ERROR) {
        if(sim_tim8.brk) {
          ctx->fault = HV_OVERCURRENT_HW;
        }
      }

      if(ctx->fault != NO_ERROR) {
        ctx->sbrake_ok    = 0;
        ctx->sbrake_ticks = 0;
        PIN(sbrake_on)    = 0.0;
        sim_tim8.moe      = 0;
      } else {
        float lim = PIN(sbrake_cur) > 0.0 ? PIN(sbrake_cur) : (PIN(max_cur) > 0.0 ? PIN(max_cur) : 10.0);
        lim       = MIN(lim, 0.8 * PIN(oc_lim));

        PIN(sbrake_on) = 1.0;
        if(ctx->sbrake_ticks < 2) {
          ctx->sbrake_ticks++;
          sim_tim8.moe = 0;
        } else if(PIN(iabs) > lim) {
          sim_tim8.moe = 0;
        } else {
          sim_tim8.moe = 1;
        }
      }
    } else {
      ctx->enabled = 0;
      if(PIN(sbrake) <= 0.0) {
        ctx->fault = NO_ERROR;
      }
      ctx->sbrake_ticks = 0;
      PIN(sbrake_on)    = 0.0;
      sim_tim8.moe      = 0;
    }
  }

  sim_tim8.dac = CLAMP(PIN(dac), 0, 4095);
  PIN(cu)      = (sim_tim8.cmp & 1) > 0;
  PIN(cv)      = (sim_tim8.cmp & 2) > 0;
  PIN(cw)      = (sim_tim8.cmp & 4) > 0;
}

hal_comp_t io_comp_struct = {
    .name      = "io",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct io_ctx_t),
    .pin_count = sizeof(struct io_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
