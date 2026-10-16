#include "idacim_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(idacim);

HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(com_pos);
HAL_PIN(cmd_mode);
HAL_PIN(en);
HAL_PIN(en_out);

HAL_PIN(id_fb);
HAL_PIN(ud_fb);

HAL_PIN(state);
HAL_PIN(timer);

HAL_PIN(r);
HAL_PIN(l);
HAL_PIN(l_half);  // *parameter*, l test half period [s]
HAL_PIN(tau);     // measured current time constant l/r [s]
HAL_PIN(l_ok);    // 1 = l is a measurement, 0 = it is not
HAL_PIN(drop);          // dead time volts per phase at the top dwell [V]
HAL_PIN(r_known);       // *parameter*, measured winding resistance, 0 = fit it
HAL_PIN(fit_di);        // dwell current separation, 0 = r/drop fit did not run
HAL_PIN(r_2p);          // two dwell chord slope, 0 = the fit did not run
HAL_PIN(drop_slope);    // *parameter*, the chord's dead time bias, ohm A per volt
HAL_PIN(r_bias);        // what was subtracted from the chord to get r
HAL_PIN(r_ok);          // this run produced a resistance

HAL_PIN(pp);
HAL_PIN(out_rev);

HAL_PIN(test_cur);
HAL_PIN(test_vel);

HAL_PIN(vel_fb);

HAL_PIN(pwm_volt);
HAL_PIN(dc_volt);

HAL_PIN(cur_bw);

HAL_PIN(tmp0);
HAL_PIN(tmp1);
HAL_PIN(tmp2);
HAL_PIN(tmp3);
HAL_PIN(avg_test_volt);

// l test state
struct idacim_ctx_t {
  uint8_t was_high;  // last observed level of ud_fb
  uint8_t have_a;    // a settled low value has been captured
  uint16_t end_n;    // samples in the settled tail of this half
  uint16_t cyc;      // observed cycles, the first is discarded
  uint16_t tau_n;    // cycles that yielded a time constant
  float t_cmd;       // phase of the commanded square wave
  float t_in;        // time since the last observed edge
  float t_hi;        // measured length of the high half
  float area;        // integral of id_fb across the high half
  float prev;        // previous id_fb, for the trapezoid
  float end_sum;     // settled tail accumulator
  float i_a;         // settled low current
  float tau_sum;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;
  PIN(r_known)                  = 0.0;
  PIN(drop_slope)               = 0.0039;
  PIN(test_cur)                 = 3.0;
  PIN(test_vel)                 = 50.0;
  PIN(cur_bw)                   = 1.0;
}

// ctx survives a stop
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx = (struct idacim_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idacim_ctx_t));
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(r)       = 0.1;
      PIN(l)       = 0.001;
      PIN(l_half)  = 0.1;
      PIN(drop)    = 0.0;
      PIN(out_rev) = 0.0;
      PIN(cur_bw)  = 1.0;
      break;

    case 10:  // r, l
      PIN(state)    = 1.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(tmp0)     = 0.0;
      PIN(tmp1)     = 0.0;
      PIN(tmp2)     = 0.0;
      PIN(tmp3)     = 0.0;

      printf("Measure r, l\n");
      printf("<font color='green'>block the rotor</font>\n");
      printf("idacim0.state = 1.2 <font color='green'>to start</font>\n");
      break;

    case 14:
      if(PIN(r_ok) > 0.0) {
        printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
        if(PIN(l_ok) > 0.0) {
          printf("conf0.l = %f <font color='green'># append to config</font>\n", PIN(l));
          printf("<font color='green'># from tau = %f ms against the r above.</font>\n", PIN(tau) * 1000.0);
        } else if(PIN(tau) <= 0.0) {
          printf("<font color='red'>l not measured</font>: no usable transient\n");
          printf("check that the bridge is enabled and idacim0.ud_fb is wired\n");
        } else if(PIN(tau) > PIN(l_half) / 8.0) {
          printf("<font color='red'>l not measured</font>: tau = %f ms needs a longer half period\n", PIN(tau) * 1000.0);
          printf("raise idacim0.l_half above %f s and rerun\n", PIN(tau) * 8.0);
        } else {
          printf("<font color='red'>l not measured</font>: tau = %f ms is too fast to time here\n", PIN(tau) * 1000.0);
          printf("use an LCR meter: line to line at 1 kHz, halved\n");
        }
        printf("<font color='green'># dead time %f V per phase at the %f A dwell, %f V link\n", PIN(drop), PIN(test_cur), PIN(dc_volt));
        if(PIN(r_known) > 0.0) {
          printf("# read against the r you gave.</font>\n");
        } else {
          printf("# r is the chord %f less its dead time bias %f (idacim0.drop_slope).\n", PIN(r_2p), PIN(r_bias));
          printf("# below about 5 A the bias is understated; a four wire r in\n");
          printf("# idacim0.r_known is better.</font>\n");
        }
      } else {
        if(PIN(r_known) > 0.0) {
          printf("<font color='red'>r read failed</font>: the top dwell did not reach half of %f A\n", PIN(test_cur));
        } else {
          printf("<font color='red'>r fit failed</font>: the two dwells differ by %f A\n", PIN(fit_di));
        }
        printf("nothing below is measured, do not append it\n");
        printf("check that idacim0.test_cur (%f) is under conf0.max_ac_cur\n", PIN(test_cur));
      }
      // walk on only if r worked: hv0.r is this pin
      PIN(state) = PIN(r_ok) > 0.0 ? 2.0 : 0.0;
      break;

    case 20:  // pp
      PIN(state)    = 2.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;

      printf("Measure polepairs\n");
      printf("<font color='green'>unblock the rotor, it will move</font>\n");
      printf("idacim0.state = 2.2 <font color='green'>to start</font>\n");
      break;

    case 24:
      printf("conf0.polecount = %f <font color='green'># append to config</font>\n", PIN(pp));
      if(PIN(out_rev) > 0.0) {
        printf("conf0.out_rev = 1 <font color='green'># append to config</font>\n");
      }
      printf("done\n");
      PIN(state) = 3.0;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx        = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(en_out)   = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1.0;
      }
      break;

    case 12:  // r -- rotor blocked, current control: dwell at two currents and
              // fit the voltage the current loop needs to sustain them.
              // com_pos is held at 0 throughout -- an induction motor has no rotor
              // flux to align to, so the injection axis is arbitrary.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 1.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;

      // ud = r * id + 4/3 * drop: at angle 0 the phase currents are id,
      // -id/2, -id/2, and the three per phase dead time drops land 4/3 on d.
      // Two dwells split r (the slope) from the drop. Full current first, so
      // the first dwell aligns what the second holds.
      if(PIN(timer) < 2.0) {
        PIN(d_cmd) = PIN(test_cur);
        PIN(tmp2)  = PIN(tmp2) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp3)  = PIN(tmp3) * 0.999 + PIN(ud_fb) * 0.001;
      } else {
        PIN(d_cmd) = PIN(test_cur) * 0.5;
        PIN(tmp0)  = PIN(tmp0) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp1)  = PIN(tmp1) * 0.999 + PIN(ud_fb) * 0.001;
      }

      // hv0.r is this pin, the loop's plant model: the ud/id ratio bootstraps
      // it so current flows at cur_bw 1; the fit replaces it
      PIN(r) = PIN(r) * 0.99 + PIN(ud_fb) / MAX(PIN(id_fb), 0.01) * 0.01;

      PIN(timer) += period;
      if(PIN(timer) >= 4.0) {
        // the filters are linear and alike, so the fit holds on the filtered
        // pair even where neither dwell reaches its command
        float di    = PIN(tmp2) - PIN(tmp0);
        PIN(fit_di) = di;
        PIN(r_2p) = di > 0.01 ? MAX((PIN(tmp3) - PIN(tmp1)) / di, 0.001) : 0.0;

        float r_ok = 0.0;

        if(PIN(r_known) > 0.0) {
          // the drop is read at the top dwell, which has to be near its command
          if(PIN(tmp2) > PIN(test_cur) * 0.5) {
            PIN(r)    = PIN(r_known);
            PIN(drop) = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
            r_ok      = 1.0;
          }
        } else if(di > 0.01) {
          // The dead time drop rises as K ln(i), which biases the chord by
          // 8/3 K ln2 / test_cur. drop_slope = 8/3 ln2 K / Vdc, 0.0039 ohm A
          // per volt on the bridge it was fitted on. Skipped without dc_volt.
          PIN(r_bias) = PIN(dc_volt) > 1.0 ? PIN(drop_slope) * PIN(dc_volt) / MAX(PIN(test_cur), 0.1) : 0.0;
          PIN(r)      = MAX(PIN(r_2p) - PIN(r_bias), 0.001);
          PIN(drop)   = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
          r_ok        = 1.0;
        }

        PIN(r_ok) = r_ok;

        // the l test needs the voltage that holds test_cur, dead time
        // included, and never one built from a failed measurement
        if(r_ok > 0.0) {
          PIN(avg_test_volt) = PIN(r) * PIN(test_cur) + 4.0 / 3.0 * PIN(drop);
          PIN(avg_test_volt) = LIMIT(PIN(avg_test_volt), PIN(pwm_volt) / 2.0);
        } else {
          PIN(avg_test_volt) = 0.0;
        }

        PIN(timer)  = 0.0;
        PIN(state)  = r_ok > 0.0 ? 1.3 : 1.4;
        PIN(d_cmd)  = 0.0;
        PIN(en_out) = 0.0;
        PIN(tmp0)   = 0.0;
        PIN(tmp1)   = 0.0;
        PIN(tmp2)   = 0.0;
        PIN(tmp3)   = 0.0;
      }
      break;

    case 13: {  // l, from the current's relaxation time constant
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;  // volt cmd
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      // Step between two currents of the same sign and take the relaxation
      // time constant by the area method:
      //
      //   i(t) = i_f + (i_a - i_f) exp(-t/tau),  tau = l/r
      //   => tau = (i_b * T - integral i dt) / (i_b - i_a)     for T >> tau
      //
      // The dead time drop is a constant offset while the current keeps its
      // sign, so it cancels out of a time constant.
      float half = MAX(PIN(l_half), 0.01);
      float v_hi = PIN(avg_test_volt);
      float v_lo = PIN(avg_test_volt) - PIN(r) * PIN(test_cur) * 0.5;
      float mid  = (v_hi + v_lo) * 0.5;

      // time the window off the step as it comes back in ud_fb: it shares
      // id_fb's packet, so the pipeline delay cancels
      uint8_t hi = PIN(ud_fb) > mid;

      if(hi && !ctx->was_high) {  // observed rising edge
        if(ctx->end_n > 0) {
          ctx->i_a    = ctx->end_sum / (float)ctx->end_n;
          ctx->have_a = 1;
        }
        ctx->end_sum = 0.0;
        ctx->end_n   = 0;
        ctx->area    = 0.0;
        ctx->t_hi    = 0.0;
        ctx->t_in    = 0.0;
        ctx->prev    = PIN(id_fb);
      } else if(!hi && ctx->was_high) {  // observed falling edge
        if(ctx->end_n > 0 && ctx->have_a) {
          float i_b = ctx->end_sum / (float)ctx->end_n;
          float di  = i_b - ctx->i_a;
          // the step has to have moved the current, and one whole cycle has to
          // have gone by, before any of it means anything
          if(di > PIN(test_cur) * 0.05 && ctx->t_hi > 0.0 && ctx->cyc > 0) {
            float tau = (i_b * ctx->t_hi - ctx->area) / di;
            if(tau > 0.0) {
              ctx->tau_sum += tau;
              ctx->tau_n++;
            }
          }
        }
        ctx->end_sum = 0.0;
        ctx->end_n   = 0;
        ctx->t_in    = 0.0;
        ctx->cyc++;
      }

      if(hi) {
        // trapezoid: a left hand sum is about a percent of tau off
        ctx->area += (PIN(id_fb) + ctx->prev) * 0.5 * period;
        ctx->t_hi += period;
      }

      ctx->t_in += period;
      if(ctx->t_in > half * 0.75) {  // settled tail of whichever half we are in
        ctx->end_sum += PIN(id_fb);
        ctx->end_n++;
      }

      ctx->prev     = PIN(id_fb);
      ctx->was_high = hi;

      // the commanded square wave runs on its own clock; what we measure
      // against is the echo, not this
      ctx->t_cmd += period;
      if(ctx->t_cmd >= half * 2.0) {
        ctx->t_cmd = 0.0;
      }
      PIN(d_cmd) = ctx->t_cmd < half ? v_lo : v_hi;

      PIN(timer) += period;
      if(PIN(timer) >= 2.0) {
        float tau = ctx->tau_n > 0 ? ctx->tau_sum / (float)ctx->tau_n : 0.0;
        float l_ok = 0.0;

        PIN(tau) = tau;

        // too slow for the half period truncates the tail, too fast leaves a
        // handful of samples
        if(ctx->tau_n >= 3 && tau >= 10.0 * period && tau <= half / 8.0) {
          PIN(l) = tau * PIN(r);
          l_ok   = 1.0;
        } else {
          PIN(l) = 0.0;
        }

        PIN(l_ok)   = l_ok;
        PIN(timer)  = 0.0;
        PIN(state)  = 1.4;
        PIN(d_cmd)  = PIN(avg_test_volt);
        PIN(en_out) = 0.0;
      }
      break;
    }

    case 22:  // pp -- rotor free to turn: hold q_cmd at 0 and ramp com_pos open-loop
              // at test_vel while injecting d_cmd, forcing the rotor to follow the
              // rotating stator field (same technique as a sensorless PMSM I/F
              // startup). Comparing the commanded electrical rate against the
              // measured mechanical rate gives pole pairs directly.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(com_pos) += PIN(test_vel) * period;
      PIN(com_pos) = mod(PIN(com_pos));

      if(ABS(PIN(vel_fb)) > 0.1) {
        PIN(pp) = PIN(pp) * 0.995 + PIN(test_vel) / PIN(vel_fb) * 0.005;
      }

      PIN(timer) += period;
      if(PIN(timer) >= 3.0) {
        PIN(timer) = 0.0;

        if(PIN(pp) < 0.0) {
          PIN(out_rev) = 1.0;
          PIN(pp) *= -1.0;
        }
        PIN(pp) = (int)(PIN(pp) + 0.5);

        PIN(en_out)   = 0.0;
        PIN(d_cmd)    = 0.0;
        PIN(cmd_mode) = 0.0;

        PIN(state) = 2.4;
      }
      break;
  }
}

hal_comp_t idacim_comp_struct = {
    .name      = "idacim",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idacim_ctx_t),
    .pin_count = sizeof(struct idacim_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
