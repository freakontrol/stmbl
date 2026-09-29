#include "io_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f3xx_hal.h"
#include "f3hw.h"
#include "common.h"

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
// at most ABS_MAX_CURRENT, stops the bridge in the same tick
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

// phase voltages to ground, one unfiltered adc sample per pwm period, for emf0
HAL_PIN(ur);
HAL_PIN(vr);
HAL_PIN(wr);

//driver temoerature
HAL_PIN(hv_temp);
// The IPM's NTC shares VFO, which the module pulls low while the bridge is
// disabled. hv_temp is read only while enabled and held otherwise (the IPM
// only heats while switching). hv_temp_ok: 0 never read, 1 live, 2 held.
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


volatile uint32_t adc_12_buf[ADC_SEQ_LEN];
volatile uint32_t adc_34_buf[ADC_SEQ_LEN];

struct io_ctx_t {
  float u_offset;
  float v_offset;
  float w_offset;
  float overtemp_error;
  float overvoltage_error;
  float overcurrent_error;
  float fault_pin_error;
  uint32_t offset_count;
  uint32_t hv_temp;
  uint32_t mot_temp;
  uint32_t fault;
  uint32_t enabled;
  uint32_t en_ticks;  // rt ticks since the enable edge, saturating
  uint32_t sbrake_ok;     // no trip since the last enable edge
  uint32_t sbrake_ticks;  // ticks since braking started
  int32_t lo_u;  // low-side half pulse of the previous tick, in timer ticks
  int32_t lo_v;
  int32_t lo_w;
};

#define ARES 4096.0  // analog resolution, 12 bit
#define ADC(a) ((a) / ARES * AREF)

#define HV_TEMP_PULLUP 3900
#define HV_R(a) (HV_TEMP_PULLUP / (AREF / (a)-1))
#define HV_TEMP_SETTLE 75  // rt ticks after enable before VFO counts as released, 5 ms
#define HV_TEMP_MIN_V 0.3  // below this the pin is held low, not an NTC reading [V]

#define MOT_TEMP_PULLUP 10000
#define MOT_TEMP_PULLMID 51000
#define MOT_TEMP_PULLDOWN 10000
#define MOT_TEMP_REF 15.26
#define MOT_REF(a) ((a) * (MOT_TEMP_PULLMID + MOT_TEMP_PULLDOWN) / MOT_TEMP_PULLDOWN)
#define MOT_R(a) (MOT_TEMP_PULLUP / (MOT_TEMP_REF / (a)-1))

#define VOLT(a) ((a) / (ARES) * (AREF) / (VDIVDOWN) * ((VDIVUP) + (VDIVDOWN)))
//#define TEMP(a) (log10f((a) * (AREF) / (ARES) * (TPULLUP) / ((AREF) - (a) * (AREF) / (ARES))) * (-53.0) + 290.0)

#define SHUNT_GAIN 16.0

#define AMP(a, gain) (((a)*AREF / ARES / (gain)-AREF / (SHUNT_PULLUP + SHUNT_SERIE) * SHUNT_SERIE) / (SHUNT * SHUNT_PULLUP) * (SHUNT_PULLUP + SHUNT_SERIE))
// AMP and VOLT folded to one multiply for the rt path; sum is the sum of
// ADC_CUR_SAMPLES samples
#define AMP_K ((float)(AREF / ARES / SHUNT_GAIN / ADC_CUR_SAMPLES / (SHUNT * SHUNT_PULLUP) * (SHUNT_PULLUP + SHUNT_SERIE)))
#define AMP_0 ((float)(AREF / (SHUNT_PULLUP + SHUNT_SERIE) * SHUNT_SERIE / (SHUNT * SHUNT_PULLUP) * (SHUNT_PULLUP + SHUNT_SERIE)))
#define AMPN(sum) ((float)(sum)*AMP_K - AMP_0)
#define VOLT_K ((float)(AREF / ARES / VDIVDOWN * (VDIVUP + VDIVDOWN)))

float r2temp(float r) {
    if (r < 1000)
      return 0;

    const float B  = 4092.0;      // Beta coefficient
    const float T0 = 298.15;      // reference temp in Kelvin (25 degC)
    const float R0 = 85000.0;     // resistance at 25 degC, in ohms
    float tempK = 1.0 / (1.0 / T0 + (1.0 / B) * log(r / R0));
    return tempK - 273.15;        // convert to Celsius
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_ctx_t *ctx      = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  PIN(brk_present) = 0.0;
  PIN(brk)         = 0.0;

  GPIO_InitTypeDef GPIO_InitStruct;
  //LED
  GPIO_InitStruct.Pin   = LED_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  // BRK
  GPIO_InitStruct.Pin   = BRK_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BRK_PORT, &GPIO_InitStruct);

  if(HAL_GPIO_ReadPin(BRK_PORT, BRK_PIN)) {  // BRK circuit detected
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BRK_PORT, &GPIO_InitStruct);
    PIN(brk_present) = 1.0;
  }

  // the ADC DMA is set up in main.c, before the ADCs start (RM0316 15.5.4)

  ctx->offset_count      = 0;
  ctx->u_offset          = 0.0;
  ctx->v_offset          = 0.0;
  ctx->w_offset          = 0.0;
  ctx->fault             = NO_ERROR;
  ctx->overtemp_error    = 0;
  ctx->overvoltage_error = 0;
  ctx->overcurrent_error = 0;
  ctx->fault_pin_error   = 0;
  ctx->hv_temp           = 0;
  ctx->mot_temp          = 0;
  ctx->enabled           = 0;
  ctx->en_ticks          = 0;
  PIN(hv_temp_ok)        = 0.0;
  ctx->sbrake_ok         = 0;
  ctx->sbrake_ticks      = 0;
  ctx->lo_u              = 0;
  ctx->lo_v              = 0;
  ctx->lo_w              = 0;
  PIN(recon)             = 1.0;
  PIN(recon_phase)       = 0.0;


#ifdef HV_EN_PIN
  GPIO_InitStruct.Pin   = HV_EN_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(HV_EN_PORT, &GPIO_InitStruct);
#endif

#ifdef HV_FAULT_PIN
  GPIO_InitStruct.Pin  = HV_FAULT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HV_FAULT_PORT, &GPIO_InitStruct);
#endif
  PIN(dac) = 0;
  PIN(sbrake)     = 0.0;
  PIN(sbrake_cur) = 0.0;
  PIN(sbrake_on)  = 0.0;
  PIN(oc_k)   = 1.3;
  PIN(oc_min) = 5.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_ctx_t *ctx      = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  // The sequence ends ~6 us after the tick; if the ADC DMA stalls (it only
  // requests once both ADCs of a pair finish) waiting here forever would hang
  // the rt with irqs of its priority blocked. Give up after ~50 us and stop
  // the rt instead: the watchdog then resets the F3.
  for(uint32_t n = 0; !((DMA1->ISR & DMA_ISR_TCIF1) && (DMA2->ISR & DMA_ISR_TCIF5)); n++) {
    if(n > 1000) {
      hal_stop();
      return;
    }
  }

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  DMA2->IFCR = DMA_IFCR_CTCIF5;

  // ranks 1-3 of each pair are current samples, rank 4 the voltage (adc.c)
  uint32_t a12 = adc_12_buf[0] + adc_12_buf[1] + adc_12_buf[2];
  uint32_t a34 = adc_34_buf[0] + adc_34_buf[1] + adc_34_buf[2];

  if(ctx->offset_count < 100) {
    ctx->offset_count++;
  } else if(ctx->offset_count < 100 + 100) {
    ctx->w_offset += AMP((float)(a12 & 0xFFFF) / (float)ADC_CUR_SAMPLES, SHUNT_GAIN) / 100.0;
    ctx->u_offset += AMP((float)(a12 >> 16) / (float)ADC_CUR_SAMPLES, SHUNT_GAIN) / 100.0;
    ctx->v_offset += AMP((float)(a34 & 0xFFFF) / (float)ADC_CUR_SAMPLES, SHUNT_GAIN) / 100.0;
    ctx->offset_count++;
  } else if(ctx->offset_count < 100 + 100 + 1) {
    if(ABS(ctx->u_offset) > 5.0 || ABS(ctx->v_offset) > 5.0 || ABS(ctx->w_offset) > 5.0) {
      ctx->fault = HV_CURRENT_OFFSET_FAULT;
    }
    ctx->offset_count++;
  } else {
    PIN(uo)       = ctx->u_offset;
    PIN(vo)       = ctx->v_offset;
    PIN(wo)       = ctx->w_offset;
    PIN(iw)       = -AMPN(a12 & 0xFFFF) + ctx->w_offset;  // 1u
    PIN(iu)       = -AMPN(a12 >> 16) + ctx->u_offset;
    PIN(iv)       = -AMPN(a34 & 0xFFFF) + ctx->v_offset;
    PIN(wr)       = (float)(adc_12_buf[ADC_SEQ_LEN - 1] & 0xFFFF) * VOLT_K;
    PIN(vr)       = (float)(adc_12_buf[ADC_SEQ_LEN - 1] >> 16) * VOLT_K;
    PIN(ur)       = (float)(adc_34_buf[ADC_SEQ_LEN - 1] & 0xFFFF) * VOLT_K;
    PIN(w)        = PIN(wr) * 0.05 + PIN(w) * 0.95;  // 0.6u
    PIN(v)        = PIN(vr) * 0.05 + PIN(v) * 0.95;
    PIN(u)        = PIN(ur) * 0.05 + PIN(u) * 0.95;
    PIN(udc)      = (float)(adc_34_buf[ADC_SEQ_LEN - 1] >> 16) * VOLT_K * 0.05 + PIN(udc) * 0.95;

    // Which phase could not be sampled. TIM8 is centre aligned and the ADC is
    // triggered at the counter extreme where the low sides conduct, so a
    // phase's low side is on from (half its off pulse) before the extreme,
    // less the dead time the DTG adds to its turn-on, to (half its off pulse)
    // after it. The half before came from the compare value of the previous
    // tick, the half after from the one loaded at this extreme, which is what
    // TIM8->CCRx reads now (io runs before hv rewrites it). The sample window
    // sits in the half after. So the sample is good only if the half after
    // outlasts the window and the half before outlasts the dead time.
    //
    // Until 2026-09-15 min_off was 5 us against a 417 ns dead time, so the
    // low side had 4.6 us at maximum duty. That day min_off went to 3 us
    // (1e8c578) and the dead time to 2.0 us (c8ebe87), which leaves a phase
    // at maximum duty with its low side on for 1 us inside the sample window
    // (ADC_CUR_WINDOW_TICKS), and while its current flows into the bridge the
    // shunt carries nothing during the dead time, so the phase reads low and the loop
    // pushes harder. That is a torque reversal at speed. Raising min_off
    // costs output voltage; instead rebuild the phase from the other two,
    // which are always valid (a phase at the top of the range forces the
    // others down): with no neutral the three sum to zero. At most one phase
    // is at the top at a time, so at most one is rebuilt.
#ifdef PWM_INVERT
    int32_t lo_u = (int32_t)TIM8->CCR3;
    int32_t lo_v = (int32_t)TIM8->CCR2;
    int32_t lo_w = (int32_t)TIM8->CCR1;
#else
    int32_t arr  = (int32_t)TIM8->ARR;
    int32_t lo_u = arr - (int32_t)TIM8->CCR3;
    int32_t lo_v = arr - (int32_t)TIM8->CCR2;
    int32_t lo_w = arr - (int32_t)TIM8->CCR1;
#endif
    PIN(recon_phase) = 0.0;
    if(PIN(recon) > 0.0) {
      int bad_u = lo_u < ADC_CUR_WINDOW_TICKS || ctx->lo_u < PWM_DEADTIME_TICKS;
      int bad_v = lo_v < ADC_CUR_WINDOW_TICKS || ctx->lo_v < PWM_DEADTIME_TICKS;
      int bad_w = lo_w < ADC_CUR_WINDOW_TICKS || ctx->lo_w < PWM_DEADTIME_TICKS;
      // if more than one is flagged (a spread wider than min_off leaves room
      // for, which hv.c prevents), rebuild the one with the shortest pulse
      // and leave the rest alone
      if(bad_u && (!bad_v || lo_u <= lo_v) && (!bad_w || lo_u <= lo_w)) {
        PIN(iu)          = -(PIN(iv) + PIN(iw));
        PIN(recon_phase) = 1.0;
      } else if(bad_v && (!bad_w || lo_v <= lo_w)) {
        PIN(iv)          = -(PIN(iu) + PIN(iw));
        PIN(recon_phase) = 2.0;
      } else if(bad_w) {
        PIN(iw)          = -(PIN(iu) + PIN(iv));
        PIN(recon_phase) = 3.0;
      }
    }
    ctx->lo_u = lo_u;
    ctx->lo_v = lo_v;
    ctx->lo_w = lo_w;
    PIN(iabs)     = MAX3(ABS(PIN(iu)), ABS(PIN(iv)), ABS(PIN(iw)));
    ctx->hv_temp  = adc_34_buf[1];  // ADC4 rank 2: the second hv_temp sample, settled
    if(!ctx->enabled) {
      ctx->en_ticks = 0;
    } else if(ctx->en_ticks < 0xFFFF) {
      ctx->en_ticks++;
    }
    ctx->mot_temp = adc_34_buf[2];  // ADC4 rank 3

    if(err_filter(&(ctx->overtemp_error), 5.0, 0.001, PIN(hv_temp) > ABS_MAX_TEMP)) {
      ctx->fault = HV_TEMP_ERROR;
    }

    if(err_filter(&(ctx->overvoltage_error), 5.0, 0.001, PIN(udc) > ABS_MAX_VOLT)) {
      ctx->fault = HV_VOLT_ERROR;
    }

    if(err_filter(&(ctx->overcurrent_error), 5.0, 0.001, PIN(iabs) > ABS_MAX_CURRENT * 0.95)) {
      ctx->fault = HV_OVERCURRENT_RMS;
    }

    PIN(oc_lim) = PIN(max_cur) > 0.0 ? CLAMP(PIN(oc_k) * PIN(max_cur), PIN(oc_min), ABS_MAX_CURRENT) : ABS_MAX_CURRENT;
    if(PIN(iabs) > PIN(oc_lim)) {
      ctx->fault = HV_OVERCURRENT_PEAK;
    }

    if(PIN(iabs) > ABS_MAX_CURRENT) {
      ctx->fault = HV_OVERCURRENT_PEAK;
    }

    PIN(fault) = ctx->fault;

    if(PIN(hv_en) > 0.0) {
      PIN(sbrake_on)    = 0.0;
      ctx->sbrake_ticks = 0;
      if(!ctx->enabled) {  //rising edge of enable
        //set timer master out enable
        TIM8->BDTR |= TIM_BDTR_MOE;
#ifdef HV_EN_PIN
        //clear driver enable pin
        HAL_GPIO_WritePin(HV_EN_PORT, HV_EN_PIN, GPIO_PIN_RESET);
#endif
        ctx->enabled   = 1;
        ctx->sbrake_ok = ctx->fault == NO_ERROR;
      }
      if(ctx->fault != NO_ERROR) {
        ctx->sbrake_ok = 0;
      }
      if(ctx->fault == NO_ERROR) {
#ifdef HV_FAULT_PIN
        //read fault pin from driver
        if(PIN(ignore_fault_pin) <= 0.0 && err_filter(&(ctx->fault_pin_error), 5.0, 0.01, HAL_GPIO_ReadPin(HV_FAULT_PORT, HV_FAULT_PIN) == HV_FAULT_POLARITY)) {
          ctx->fault = HV_FAULT_ERROR;
        }
#endif
        //Master out enable is cleared by timer break input.
        //Timer break input is connected to comperators
        if(!(TIM8->BDTR & TIM_BDTR_MOE)) {
          ctx->fault = HV_OVERCURRENT_HW;
        }
      } else {
        ctx->fault_pin_error = 0;
#ifdef HV_EN_PIN
        //set driver enable pin
        HAL_GPIO_WritePin(HV_EN_PORT, HV_EN_PIN, GPIO_PIN_SET);
#endif
      }
    } else if(PIN(sbrake) > 0.0 && ctx->sbrake_ok && ctx->offset_count > 200) {
      ctx->enabled = 0;
      if(ctx->sbrake_ticks == 0) {
        // a break flag left from before braking is not ours to judge
        TIM8->SR = ~(TIM_SR_BIF | TIM_SR_B2IF);
      }
      if(ctx->fault == NO_ERROR) {
#ifdef HV_FAULT_PIN
        if(PIN(ignore_fault_pin) <= 0.0 && HAL_GPIO_ReadPin(HV_FAULT_PORT, HV_FAULT_PIN) == HV_FAULT_POLARITY) {
          ctx->fault = HV_FAULT_ERROR;
        }
#endif
        // the comparators clear MOE and set a break flag even while this
        // code holds MOE off for a chop, so the flag is the trip signal here
        if(TIM8->SR & (TIM_SR_BIF | TIM_SR_B2IF)) {
          ctx->fault = HV_OVERCURRENT_HW;
        }
      }

      if(ctx->fault != NO_ERROR) {
        // tripped while braking: off for good, and the fault goes to the f4
        ctx->sbrake_ok    = 0;
        ctx->sbrake_ticks = 0;
        PIN(sbrake_on)    = 0.0;
        TIM8->BDTR &= ~TIM_BDTR_MOE;
#ifdef HV_EN_PIN
        HAL_GPIO_WritePin(HV_EN_PORT, HV_EN_PIN, GPIO_PIN_SET);
#endif
      } else {
        float lim = PIN(sbrake_cur) > 0.0 ? PIN(sbrake_cur) : (PIN(max_cur) > 0.0 ? PIN(max_cur) : 10.0);
        lim       = MIN(lim, 0.8 * PIN(oc_lim));

        PIN(sbrake_on) = 1.0;
#ifdef HV_EN_PIN
        HAL_GPIO_WritePin(HV_EN_PORT, HV_EN_PIN, GPIO_PIN_RESET);
#endif
        // two ticks for hv0's zero compares to reach the timer
        if(ctx->sbrake_ticks < 2) {
          ctx->sbrake_ticks++;
          TIM8->BDTR &= ~TIM_BDTR_MOE;
        } else if(PIN(iabs) > lim) {
          TIM8->BDTR &= ~TIM_BDTR_MOE;
        } else {
          TIM8->BDTR |= TIM_BDTR_MOE;
        }
      }
    } else {
      ctx->enabled = 0;
      // a trip during braking stays reported until the request goes away,
      // so the f4 sees it and drops the request instead of retrying
      if(PIN(sbrake) <= 0.0) {
        ctx->fault = NO_ERROR;
      }
      ctx->sbrake_ticks = 0;
      PIN(sbrake_on)    = 0.0;
      TIM8->BDTR &= ~TIM_BDTR_MOE;
#ifdef HV_EN_PIN
      //set driver enable pin
      HAL_GPIO_WritePin(HV_EN_PORT, HV_EN_PIN, GPIO_PIN_SET);
#endif
    }

    if(PIN(brk) > 0.0) {
      HAL_GPIO_WritePin(BRK_PORT, BRK_PIN, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(BRK_PORT, BRK_PIN, GPIO_PIN_SET);
    }
  }

  //dac output for comperators
  DAC1->DHR12R1 = CLAMP((uint32_t)PIN(dac), 0, 4095);

  //comperator outputs for debugging
  // COMP2 watches PA7 = U, COMP4 PB0 = V, COMP1 PA1 = W (schematic A_IU/A_IV/A_IW)
  PIN(cu) = (COMP2->CSR & COMP_CSR_COMPxOUT) > 0;
  PIN(cv) = (COMP4->CSR & COMP_CSR_COMPxOUT) > 0;
  PIN(cw) = (COMP1->CSR & COMP_CSR_COMPxOUT) > 0;
}

void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_ctx_t *ctx      = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  uint32_t led = (uint32_t)PIN(led);

  if(hal.hal_state != HAL_OK2) {
    led = 2;
  }

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, BLINK(led) > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

  float hv_v = ADC(ctx->hv_temp >> 16);
  if(ctx->enabled && ctx->en_ticks > HV_TEMP_SETTLE && hv_v > HV_TEMP_MIN_V) {
    float t = r2temp(HV_R(hv_v));
    // first reading since power up: start from it, not from 0
    PIN(hv_temp)    = PIN(hv_temp_ok) > 0.0 ? t * 0.01 + PIN(hv_temp) * 0.99 : t;
    PIN(hv_temp_ok) = 1.0;
  } else if(PIN(hv_temp_ok) > 0.0) {
    PIN(hv_temp_ok) = 2.0;
  }
  PIN(mot_temp) = r2temp(MOT_R(MOT_REF(ADC(ctx->mot_temp >> 16)))) * 0.01 + PIN(mot_temp) * 0.99;
}

hal_comp_t io_comp_struct = {
    .name      = "io",
    .nrt       = nrt_func,
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
