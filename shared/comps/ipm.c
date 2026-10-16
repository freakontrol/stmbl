#include "ipm_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(ipm);

/*
Bridge junction temperature estimate, for a derate and trip between the
instantaneous overcurrent trip and the NTC derate. The module NTC lags the
dies by seconds, so it carries the slow part (case_temp) and this models the
rise above it.

Each of the twelve dies (per leg: high and low IGBT, high and low diode) runs
its own three pole Foster network on its instantaneous loss: conduction from
the leg current and duty, switching from the current and link voltage, after
the CIPOS Mini application note. At speed the networks average the cycle; at
standstill the one leg carrying dc heats on its own, which a mean value model
over a sine understates about 3.3x.

rth_i and rth_d are the datasheet Rth(J-C) times a factor that reproduces an
SOA chart with this model (1.5 for the IKCM30F60GD, calibrated on the
IM535-U6D's chart). Trim them against the real heatsink. Defaults are for the
IKCM30F60GD fitted to v5.0.
*/

// in
HAL_PIN(id);         // *input*, hv0.id_fb
HAL_PIN(iq);         // *input*, hv0.iq_fb
HAL_PIN(ud);         // *input*, hv0.ud_fb
HAL_PIN(uq);         // *input*, hv0.uq_fb
HAL_PIN(pos);        // *input*, the electrical angle hv0 commutates with
HAL_PIN(dc_volt);    // *input*, hv0.dc_volt
HAL_PIN(case_temp);  // *input*, hv0.hv_temp, the module NTC

// device, common
HAL_PIN(f_sw);    // *parameter*, switching frequency (Hz)
HAL_PIN(e_volt);  // *parameter*, link voltage the switching energies were measured at (V)
HAL_PIN(v_tc);    // *parameter*, rise of the conduction drops with junction temperature (1/K)

// transistor
HAL_PIN(vi);     // *parameter*, on state knee voltage at 25 C (V)
HAL_PIN(ri);     // *parameter*, on state slope resistance at 25 C (ohm)
HAL_PIN(ei);     // *parameter*, (Eon + Eoff) per amp switched at e_volt, 25 C (J/A)
HAL_PIN(ei_tc);  // *parameter*, rise of that energy with junction temperature (1/K)
HAL_PIN(rth_i);  // *parameter*, junction to case (K/W)

// diode
HAL_PIN(vd);     // *parameter*, forward knee voltage at 25 C (V)
HAL_PIN(rd);     // *parameter*, forward slope resistance at 25 C (ohm)
HAL_PIN(ed);     // *parameter*, Erec per amp recovered at e_volt, 25 C (J/A)
HAL_PIN(ed_tc);  // *parameter*, rise of that energy with junction temperature (1/K)
HAL_PIN(rth_d);  // *parameter*, junction to case (K/W)

// junction to case impedance shape, three pole Foster network: shared time
// constants, separate splits; the last share is what the first two leave
HAL_PIN(tau1);
HAL_PIN(tau2);
HAL_PIN(tau3);
HAL_PIN(fi1);
HAL_PIN(fi2);
HAL_PIN(fd1);
HAL_PIN(fd2);

// out
HAL_PIN(p_igbt);   // mean per transistor (W)
HAL_PIN(p_diode);  // mean per diode (W)
HAL_PIN(power);    // all twelve dies, what the heatsink sees (W)
HAL_PIN(t_igbt);   // hottest transistor (C)
HAL_PIN(t_diode);  // hottest diode (C)
HAL_PIN(temp);     // the hotter of the two, what the derate watches (C)

struct ipm_ctx_t {
  float die[12][3];  // per leg: high igbt, low igbt, high diode, low diode
  float tdie[12];    // their junctions, for the temperature coefficients
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ipm_pin_ctx_t *pins = (struct ipm_pin_ctx_t *)pin_ptr;

  PIN(f_sw)   = 15000.0;  // the f3 PWM rate, the templates link hv0.pwm_freq
  PIN(e_volt) = 300.0;
  PIN(v_tc)   = 0.00155;  // 1.55 V to 1.85 V between 25 C and 150 C

  PIN(vi)    = 0.667;  // 1.55 V at 20 A, split 0.43 knee to 0.57 slope
  PIN(ri)    = 0.0442;
  PIN(ei)    = 56.65e-6;  // (698 + 435) uJ at 300 V, 20 A, 25 C
  PIN(ei_tc) = 0.00316;   // 1133 uJ to 1580 uJ between 25 C and 150 C
  PIN(rth_i) = 2.37;      // 1.58 K/W times 1.5

  // fitted at 150 C and divided back by the shared v_tc: the diode's own
  // drop barely moves with temperature
  PIN(vd)    = 0.729;
  PIN(rd)    = 0.0306;
  PIN(ed)    = 4.75e-6;  // 95 uJ at 300 V, 20 A, 25 C
  PIN(ed_tc) = 0.00665;  // 95 uJ to 174 uJ between 25 C and 150 C
  PIN(rth_d) = 3.08;     // 2.05 K/W times 1.5

  // the GD's Zth(J-C); the diode shares scaled as the IM535-U6D's lead its IGBT's
  PIN(tau1) = 0.00041;
  PIN(tau2) = 0.0143;
  PIN(tau3) = 0.229;
  PIN(fi1)  = 0.054;
  PIN(fi2)  = 0.170;
  PIN(fd1)  = 0.110;
  PIN(fd2)  = 0.247;

  PIN(t_igbt)  = 25.0;
  PIN(t_diode) = 25.0;
  PIN(temp)    = 25.0;
}

// one first order lag per pole, clamped so a tau under the tick cannot ring
static float foster(float *state, float power, float rth, float f1, float f2, float t1, float t2, float t3, float period) {
  const float f3 = MAX(1.0 - f1 - f2, 0.0);

  state[0] += (power * rth * f1 - state[0]) * period / MAX(t1, period);
  state[1] += (power * rth * f2 - state[1]) * period / MAX(t2, period);
  state[2] += (power * rth * f3 - state[2]) * period / MAX(t3, period);

  return state[0] + state[1] + state[2];
}

// deliberately no rt_start: a module does not cool because the drive was
// disabled, so the state survives a stop
static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ipm_ctx_t *ctx      = (struct ipm_ctx_t *)ctx_ptr;
  struct ipm_pin_ctx_t *pins = (struct ipm_pin_ctx_t *)pin_ptr;

  const float udc = MAX(PIN(dc_volt), 1.0);

  // ipm0.pos = hv0.pos links input to input and PIN() reads one level, so
  // walk the chain to the pin that is driven
  volatile hal_pin_inst_t *src = pins->pos.source;
  for(int hop = 0; hop < 4 && src->source != src; hop++) {
    src = src->source;
  }
  const float th = src->value;

  // phase currents and voltages, amplitude invariant inverse park and clarke
  float c, s;
  sincos_fast(th, &s, &c);
  float ia = PIN(id) * c - PIN(iq) * s, ib = PIN(id) * s + PIN(iq) * c;
  float ua = PIN(ud) * c - PIN(uq) * s, ub = PIN(ud) * s + PIN(uq) * c;
  float il[3] = {ia, -0.5 * ia + 0.5 * M_SQRT3 * ib, -0.5 * ia - 0.5 * M_SQRT3 * ib};
  float ul[3] = {ua, -0.5 * ua + 0.5 * M_SQRT3 * ub, -0.5 * ua - 0.5 * M_SQRT3 * ub};

  const float ksw = PIN(f_sw) * udc / MAX(PIN(e_volt), 1.0);
  float sum_i = 0.0, sum_d = 0.0, t_i = -300.0, t_d = -300.0;
  for(int l = 0; l < 3; l++) {
    float a    = ABS(il[l]);
    float duty = CLAMP(0.5 + ul[l] / udc, 0.0, 1.0);  // leg duty, common mode ignored
    for(int k = 0; k < 4; k++) {  // high igbt, low igbt, high diode, low diode
      float tj  = ctx->tdie[l * 4 + k];
      int diode = k >= 2;
      float kvt = CLAMP(1.0 + PIN(v_tc) * (tj - 25.0), 0.5, 3.0);
      float ket = CLAMP(1.0 + (diode ? PIN(ed_tc) : PIN(ei_tc)) * (tj - 25.0), 0.5, 3.0);
      // current out of the leg: high igbt for the duty, low diode for the
      // rest; into the leg: low igbt for the rest, high diode for the duty
      int on_hi  = (k == 0 || k == 2);
      int active = il[l] >= 0.0 ? (k == 0 || k == 3) : (k == 1 || k == 2);
      float d    = on_hi ? duty : 1.0 - duty;
      float cond = diode ? kvt * (PIN(vd) * a + PIN(rd) * a * a) : kvt * (PIN(vi) * a + PIN(ri) * a * a);
      float sw   = (diode ? PIN(ed) : PIN(ei)) * ksw * a * ket;
      float pw   = active ? d * cond + sw : 0.0;
      float rise = foster(ctx->die[l * 4 + k], pw, diode ? PIN(rth_d) : PIN(rth_i), diode ? PIN(fd1) : PIN(fi1), diode ? PIN(fd2) : PIN(fi2), PIN(tau1), PIN(tau2), PIN(tau3), period);
      ctx->tdie[l * 4 + k] = PIN(case_temp) + rise;
      if(diode) {
        sum_d += pw;
        t_d = MAX(t_d, ctx->tdie[l * 4 + k]);
      } else {
        sum_i += pw;
        t_i = MAX(t_i, ctx->tdie[l * 4 + k]);
      }
    }
  }
  PIN(p_igbt)  = sum_i / 6.0;
  PIN(p_diode) = sum_d / 6.0;
  PIN(power)   = sum_i + sum_d;
  PIN(t_igbt)  = t_i;
  PIN(t_diode) = t_d;
  PIN(temp)    = MAX(t_i, t_d);
}

hal_comp_t ipm_comp_struct = {
    .name      = "ipm",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct ipm_ctx_t),
    .pin_count = sizeof(struct ipm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
