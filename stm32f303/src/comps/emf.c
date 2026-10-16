#include "emf_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

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

HAL_PIN(u);  // phase voltages to ground, unfiltered [V]
HAL_PIN(v);
HAL_PIN(w);
HAL_PIN(pos);  // electrical angle
HAL_PIN(si);   // sin and cos of pos, from dq0
HAL_PIN(co);
HAL_PIN(en);   // bridge enable; summing only runs with the bridge off
HAL_PIN(run);  // 1 sum, 0 hold, -1 clear
HAL_PIN(sel);
HAL_PIN(pp);

HAL_PIN(vel);  // electrical speed from pos [rad/s]
HAL_PIN(psi);  // live |psi|
HAL_PIN(n);    // samples summed
HAL_PIN(val);

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
  // once per packet, every third tick; the filter smooths that out.
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
