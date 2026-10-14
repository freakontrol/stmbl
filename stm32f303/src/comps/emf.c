#include "emf_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `emf` measures the back EMF of a coasting motor on the F3 (HV board): with the bridge off it demodulates every PWM period's unfiltered phase voltage sample against the rotor angle and sums the flux linkage `psi`, its 5th and 7th harmonic back EMF, and `psi` per magnet pole, to tell a demagnetised magnet from a weak rotor. It is loaded by `stm32f303/src/main.c` as `emf0` (rt_prio 7, the last in the chain). It is controlled from the F4 through config words and read back one result at a time through a state word: `idpmsm`'s psi test drives it (conf/template/id_pmsm.txt links `hv0.emf_run/emf_sel/emf_pp` to `idpmsm0` and `idpmsm0.emf_val = hv0.emf_val`). It runs on the F3 because the F4 only sees each phase voltage every ~2 ms and through `io0`'s filter.
*
* ## Component Explanation
*
* 1. **Wiring on the F3** (fixed in main.c):
* - `emf0.u/v/w = io0.ur/vr/wr`, `emf0.pos = ls0.pos`, `emf0.si/co = dq0.si/co`, `emf0.en = ls0.en`.
* - `emf0.run/sel/pp = ls0.emf_run/emf_sel/emf_pp` (F4 config words), `ls0.emf_val = emf0.val` (F4 state word).
*
* 2. **Speed and turns** (rt, always):
* - `vel` is the electrical speed from the change of `pos` per tick (the angle steps once per F4 packet, every `PWM_TICKS_PER_PACKET` ticks, 3 at the default 15 kHz PWM).
* - A wrap of `pos` from +pi to -pi counts one electrical cycle forward, the other way one back.
*
* 3. **When it sums** (rt):
* - Only with the bridge off: `en` must have been 0 for 30 ms (the winding current has to die out first), and `|vel|` must be above 20 rad/s electrical.
* - `run` < 0 clears the sums every tick while it stays negative. rt_start clears them too.
* - The live `psi` pin is updated whenever the conditions above hold, also while `run` is 0.
*
* 4. **Demodulation** (rt):
* - The phase voltages are Clarke transformed (amplitude invariant, so the common mode drops out) into `e = a + j b`. With the rotor flux `psi(th) = sum psi_k exp(j k th)` over the harmonics k = 1, -5, 7, the back EMF is `e = d/dt psi`, so
* ```c
* e * exp(-j k th) / w  averages to  j k psi_k   // w = electrical speed
* ```
* - This holds whatever the direction of rotation, so `psi` comes out in Vs (peak, per electrical rad) at any speed.
* - Every sample adds `e exp(-j th) / w`, `e exp(+j 5 th) / w` and `e exp(-j 7 th) / w` to three sums, and `n` counts them.
* - Per pole bins: the fundamental is also summed per half electrical cycle, `bins = 2 * pp` bins over a mechanical turn (`pp` rounded, 1..8, so up to 16 bins), which is one bin per magnet pole. Bin 0 is wherever the count started, not a fixed rotor position, so compare the bins with each other: a weak pole reads low in its bin on every run.
*
* 5. **DC offset removal**:
* - A DC offset between the phase voltage channels (divider or ADC offsets that differ per phase) averages out over a whole electrical cycle but not over half of one, so it would make the bins alternate high and low.
* - The offset `d` is estimated as the mean Clarke vector over whole electrical cycles only (the back EMF integrates to 0 over each one): summing starts at the first wrap of `pos` after the first sample, and a snapshot is taken at every later wrap. Each bin also keeps the sum of `exp(-j th) / w`, so its share of `d` is subtracted when the bin is read. Until one whole cycle has been seen, `d` reads 0.
*
* 6. **Results** (rt, one per tick):
* - `sel` picks what `val` returns:
* - 0 (or below): the live `psi` (low pass filtered |psi_1| with a 20 ms time constant).
* - 1: number of samples summed.
* - 2, 3: psi_1 real and imaginary part, averaged over the whole coast.
* - 4, 5: 5th harmonic back EMF (-5 psi_-5), real and imaginary, on the same scale as psi_1.
* - 6, 7: 7th harmonic back EMF (7 psi_7), real and imaginary.
* - 8: number of bins in use (2 * pp).
* - 9 + 3b, 10 + 3b, 11 + 3b: bin b (0..15), psi real, psi imaginary (DC offset removed, averaged over the bin's samples), sample count.
* - 57, 58: the DC offset `d`, alpha and beta (V).
* - Anything above returns 0.
* - On the F4 the value arrives through the rotating state words of the link, so after changing `sel` the reader must wait a few ms before `emf_val` holds the new result (`idpmsm` does).
*
* {{% hint warning %}}
* - `vel` comes from differencing `pos`, which on the F3 is the F4's angle extrapolated between packets, so the F4 must keep sending the rotor angle while the bridge is off.
* - Bins past `2 * pp` stay 0, and changing `pp` between clears mixes bin indices.
* - The bin count is limited to 16 (8 pole pairs) by the F3's HAL memory (HAL_MAX_CTX); a motor with more pole pairs gets bins that each span several poles.
* {{% /hint %}}
*/

// Back emf map of a coasting rotor, for telling a demagnetised magnet from a
// weak rotor.
//
// With the bridge off the terminals carry the back emf alone. Their Clarke
// vector is e = d/dt psi(th) with psi(th) = sum psi_k exp(j k th) over the
// flux harmonics k = 1, -5, 7, so
//
//   e exp(-j k th) / w  averages to  j k psi_k       w = d th / dt
//
// whatever the direction. This runs on the f3 because the f4 sees each phase
// voltage only every ~2 ms and through io0's filter; here every pwm period's
// unfiltered sample counts.
//
// The fundamental is also summed per half electrical cycle, 2 * pp bins over
// a mechanical turn, which is one bin per magnet pole. Bin 0 is wherever the
// count started, not a fixed rotor position, so compare the bins with each
// other: a weak pole reads low in its bin on every run.
//
// A dc offset d between the phase voltage channels (divider or adc offsets
// that differ per phase; the common part drops out in Clarke) adds
// d exp(-j th) / w to every sample. Over a whole electrical cycle that
// averages out, over half of one it does not, and it adds with opposite sign
// in the two halves: the bins alternate high and low. d is the time average
// of the Clarke vector over whole cycles, because the back emf integrates to
// psi(th + 2 pi) - psi(th) = 0 over each one at any speed. Each bin also
// keeps sum exp(-j th) / w, so its share of d is taken out when it is read.
//
// Results, picked by sel and returned in val:
//   0     live |psi| (low passed), also on the psi pin
//   1     samples summed
//   2, 3  psi_1 re, im over the whole coast
//   4, 5  -5 psi_-5 re, im: 5th harmonic emf, same scale as psi_1
//   6, 7  7 psi_7 re, im: 7th harmonic emf
//   8     bins in use (2 * pp)
//   9 + 3 b, 10 + 3 b, 11 + 3 b    bin b: psi re, im (dc removed), samples
//   9 + 3 EMF_BINS, + 1              dc offset d: alpha, beta [V]

HAL_COMP(emf);

HAL_PIN(u);  // *input*, U phase voltage to ground (V), unfiltered, from io0.ur
HAL_PIN(v);  // *input*, V phase voltage to ground (V), unfiltered, from io0.vr
HAL_PIN(w);  // *input*, W phase voltage to ground (V), unfiltered, from io0.wr
HAL_PIN(pos);  // *input*, Electrical rotor angle (rad), from ls0.pos
HAL_PIN(si);   // *input*, Sine of pos, from dq0.si
HAL_PIN(co);   // *input*, Cosine of pos, from dq0.co
HAL_PIN(en);   // *input*, Bridge enable from ls0.en, summing only runs 30 ms after it went to 0
HAL_PIN(run);  // *input*, 1 = sum, 0 = hold, -1 = clear, from ls0.emf_run
HAL_PIN(sel);  // *input*, Result number returned in val, from ls0.emf_sel
HAL_PIN(pp);   // *input*, Pole pairs for the per pole bins (1..8), from ls0.emf_pp

HAL_PIN(vel);  // *output*, Electrical speed from pos (rad/s), 4 ms low pass
HAL_PIN(psi);  // *output*, Live flux linkage amplitude (Vs), 20 ms low pass
HAL_PIN(n);    // *output*, Number of samples summed
HAL_PIN(val);  // *output*, Result number sel, to ls0.emf_val

#define EMF_BINS 16         // 2 * pp, up to 8 pole pairs (the f3 HAL_MAX_CTX limit)
#define EMF_VEL_TAU 0.004   // speed filter [s]
#define EMF_PSI_TAU 0.02    // live psi filter [s]
#define EMF_HOLDOFF 0.03    // wait after the bridge turns off, for the winding current to die [s]
#define EMF_MIN_VEL 20.0    // slowest electrical speed summed [rad/s]

struct emf_ctx_t {
  float last_pos;
  float off_t;  // time since the bridge turned off
  int32_t cycle;
  uint32_t n;
  float z1_re, z1_im;
  float z5_re, z5_im;
  float z7_re, z7_im;
  float b_re[EMF_BINS];
  float b_im[EMF_BINS];
  uint32_t b_n[EMF_BINS];
  float b_c[EMF_BINS];  // sum cos(th) / w, sin(th) / w: the bin's weight on d
  float b_s[EMF_BINS];
  float da, db;         // raw Clarke sums since the clear
  uint32_t dn;
  float da_w, db_w;     // the same at the last whole cycle
  uint32_t dn_w;
  int32_t d_cycle;      // cycle at the last wrap seen
  int32_t d_started;    // a wrap has been seen since the clear
};

static void clear(struct emf_ctx_t *ctx) {
  ctx->n     = 0;
  ctx->cycle = 0;
  ctx->z1_re = ctx->z1_im = 0.0;
  ctx->z5_re = ctx->z5_im = 0.0;
  ctx->z7_re = ctx->z7_im = 0.0;
  for(int i = 0; i < EMF_BINS; i++) {
    ctx->b_re[i] = 0.0;
    ctx->b_im[i] = 0.0;
    ctx->b_n[i]  = 0;
    ctx->b_c[i]  = 0.0;
    ctx->b_s[i]  = 0.0;
  }
  ctx->da = ctx->db = ctx->da_w = ctx->db_w = 0.0;
  ctx->dn = ctx->dn_w = 0;
  ctx->d_started       = 0;
}

static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct emf_ctx_t *ctx = (struct emf_ctx_t *)ctx_ptr;
  clear(ctx);
  ctx->last_pos = 0.0;
  ctx->off_t    = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct emf_ctx_t *ctx      = (struct emf_ctx_t *)ctx_ptr;
  struct emf_pin_ctx_t *pins = (struct emf_pin_ctx_t *)pin_ptr;

  // speed and whole electrical turns from the angle the f4 sends. It steps
  // once per packet, every PWM_TICKS_PER_PACKET ticks; the filter smooths
  // that out.
  float raw     = PIN(pos) - ctx->last_pos;
  ctx->last_pos = PIN(pos);
  if(raw < -M_PI) {  // wrapped forwards
    ctx->cycle++;
  } else if(raw > M_PI) {  // backwards
    ctx->cycle--;
  }
  PIN(vel) += (mod(raw) / period - PIN(vel)) * period / EMF_VEL_TAU;

  if(PIN(en) > 0.0) {
    ctx->off_t = 0.0;
  } else if(ctx->off_t < EMF_HOLDOFF) {
    ctx->off_t += period;
  }

  if(PIN(run) < 0.0) {
    clear(ctx);
  }

  float vel = PIN(vel);
  if(ctx->off_t >= EMF_HOLDOFF && ABS(vel) > EMF_MIN_VEL) {
    // Clarke, amplitude invariant; the common mode drops out
    float a = (PIN(u) * 2.0 - PIN(v) - PIN(w)) / 3.0;
    float b = (PIN(v) - PIN(w)) * M_SQRT1_3;
    float k = 1.0 / vel;

    float c = PIN(co);
    float s = PIN(si);
    // e exp(-j th) / w
    float p1_re = (a * c + b * s) * k;
    float p1_im = (b * c - a * s) * k;
    PIN(psi) += (sqrtf(p1_re * p1_re + p1_im * p1_im) - PIN(psi)) * period / EMF_PSI_TAU;

    if(PIN(run) > 0.0) {
      // raw sums for d from the first wrap of pos on, snapshotted at every
      // later wrap, so the snapshot always spans whole cycles
      if(ctx->d_started == 0) {
        if(ctx->n == 0) {
          ctx->d_cycle = ctx->cycle;
        } else if(ctx->cycle != ctx->d_cycle) {
          ctx->d_cycle   = ctx->cycle;
          ctx->d_started = 1;
        }
      } else if(ctx->cycle != ctx->d_cycle) {
        ctx->d_cycle = ctx->cycle;
        ctx->da_w    = ctx->da;
        ctx->db_w    = ctx->db;
        ctx->dn_w    = ctx->dn;
      }
      if(ctx->d_started) {
        ctx->da += a;
        ctx->db += b;
        ctx->dn++;
      }

      // exp(j 5 th) and exp(j 7 th) by repeated products
      float c2 = c * c - s * s, s2 = 2.0 * c * s;
      float c4 = c2 * c2 - s2 * s2, s4 = 2.0 * c2 * s2;
      float c5 = c4 * c - s4 * s, s5 = c4 * s + s4 * c;
      float c7 = c5 * c2 - s5 * s2, s7 = c5 * s2 + s5 * c2;

      ctx->z1_re += p1_re;
      ctx->z1_im += p1_im;
      float ak = a * k, bk = b * k;
      ctx->z5_re += ak * c5 - bk * s5;  // e exp(+j 5 th) / w
      ctx->z5_im += bk * c5 + ak * s5;
      ctx->z7_re += ak * c7 + bk * s7;  // e exp(-j 7 th) / w
      ctx->z7_im += bk * c7 - ak * s7;
      ctx->n++;

      int32_t bins = CLAMP((int32_t)(PIN(pp) + 0.5), 1, EMF_BINS / 2) * 2;
      int32_t i    = (ctx->cycle * 2 + (PIN(pos) >= 0.0 ? 1 : 0)) % bins;
      if(i < 0) {
        i += bins;
      }
      ctx->b_re[i] += p1_re;
      ctx->b_im[i] += p1_im;
      ctx->b_n[i]++;
      ctx->b_c[i] += c * k;
      ctx->b_s[i] += s * k;
    }
  }
  PIN(n) = ctx->n;

  // one result per tick, as the f4 asks for it
  int32_t sel = (int32_t)(PIN(sel) + 0.5);
  float nn    = MAX((float)ctx->n, 1.0);
  float out   = 0.0;
  if(sel <= 0) {
    out = PIN(psi);
  } else if(sel == 1) {
    out = ctx->n;
  } else if(sel < 8) {
    float z[6] = {ctx->z1_re, ctx->z1_im, ctx->z5_re, ctx->z5_im, ctx->z7_re, ctx->z7_im};
    out        = z[sel - 2] / nn;
  } else if(sel == 8) {
    out = CLAMP((int32_t)(PIN(pp) + 0.5), 1, EMF_BINS / 2) * 2;
  } else if(sel < 9 + 3 * EMF_BINS + 2) {
    // d over whole cycles only; before the first one there is no estimate
    float dnw = MAX((float)ctx->dn_w, 1.0);
    float da  = ctx->da_w / dnw;
    float db  = ctx->db_w / dnw;
    int32_t b = (sel - 9) / 3;
    float bn  = b < EMF_BINS ? MAX((float)ctx->b_n[b], 1.0) : 1.0;
    if(sel == 9 + 3 * EMF_BINS) {
      out = da;
    } else if(sel == 9 + 3 * EMF_BINS + 1) {
      out = db;
    } else {
      switch((sel - 9) % 3) {
        case 0:  // d exp(-j th) / w summed over the bin: re = da c + db s
          out = (ctx->b_re[b] - (da * ctx->b_c[b] + db * ctx->b_s[b])) / bn;
          break;
        case 1:  // im = db c - da s
          out = (ctx->b_im[b] - (db * ctx->b_c[b] - da * ctx->b_s[b])) / bn;
          break;
        default:
          out = ctx->b_n[b];
      }
    }
  }
  PIN(val) = out;
}

hal_comp_t emf_comp_struct = {
    .name      = "emf",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct emf_ctx_t),
    .pin_count = sizeof(struct emf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
