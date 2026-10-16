#include "sim_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `sim` is a test signal generator. It outputs a sine wave, its first and second integral, a square wave and a ramp, with adjustable amplitude, frequency and offset. It is used to drive a motor or feedback test without a real command, for example `conf/template/com_test.txt` links `hv0.pos = sim0.vel` to turn the field at a fixed speed, and the `pid`, `mpid` and `uf` templates load it. It runs in the rt.
*
* ## Component Explanation
*
* 1. **Amplitude and frequency**:
* - `amp` and `freq` are low pass filtered (0.1 % new value per tick), so changes take effect smoothly over about 1000 ticks.
*
* 2. **Waveforms**, with `A` = filtered amplitude, `f` = filtered frequency, `o` = `offset`:
* - `sin = A * sin(2 pi f t) + o`.
* - `sin2 = A / (2 pi f) * sin(2 pi f t) + o`: a sine whose derivative has amplitude `A`, so as a position it has a peak velocity of `A`.
* - `sin3 = A / (2 pi f)^2 * sin(2 pi f t) + o`: as a position its peak acceleration is `A`.
* - `square = +A + o` while the sine is positive, else `-A + o`.
* - Used as a position, `vel` turns at `f` revolutions per second. It does not include `offset`.
* - Below 0.01 Hz, `sin2` and `sin3` are 0 (plus `offset`).
*
* {{% hint warning %}}
* The time is wrapped with a simple check after each period (marked TODO in the code). When `freq` changes, the phase of the sine can jump.
* {{% /hint %}}
*/

HAL_COMP(sim);

HAL_PIN(amp);     // *parameter*, Amplitude, low pass filtered, default 3.1
HAL_PIN(freq);    // *parameter*, Frequency (Hz), low pass filtered, default 1
HAL_PIN(sin);     // *output*, Sine, amp * sin + offset
HAL_PIN(msin);    // *output*, sin wrapped to +-pi and quantised to 1/res
HAL_PIN(sin2);    // *output*, Sine with peak derivative amp, amp / (2 pi freq) * sin + offset
HAL_PIN(msin2);   // *output*, sin2 wrapped to +-pi and quantised to 1/res, constant max velocity = amp
HAL_PIN(sin3);    // *output*, Sine with peak second derivative amp, amp / (2 pi freq)^2 * sin + offset
HAL_PIN(msin3);   // *output*, sin3 wrapped to +-pi and quantised to 1/res, constant max acceleration = amp
HAL_PIN(square);  // *output*, Square wave, +-amp + offset
HAL_PIN(vel);     // *output*, Ramp, angle turning at freq (rad), wrapped to +-pi
HAL_PIN(res);     // *parameter*, Quantisation steps per unit for the m outputs, min 1, default 100000
HAL_PIN(offset);  // *parameter*, Offset added to the sine and square outputs

struct sim_ctx_t {
  float time;
  float amp;
  float freq;
  float vel;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct sim_ctx_t * ctx = (struct sim_ctx_t *)ctx_ptr;
  struct sim_pin_ctx_t *pins = (struct sim_pin_ctx_t *)pin_ptr;

  PIN(amp)  = 3.1;
  PIN(freq) = 1.0;
  PIN(res)  = 100000.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct sim_ctx_t *ctx      = (struct sim_ctx_t *)ctx_ptr;
  struct sim_pin_ctx_t *pins = (struct sim_pin_ctx_t *)pin_ptr;

  ctx->amp  = PIN(amp) * 0.001 + ctx->amp * 0.999;
  ctx->freq = PIN(freq) * 0.001 + ctx->freq * 0.999;
  float sin;
  float sin2;
  float sin3;
  float amp2;
  float r = MAX(PIN(res), 1);

  if(ABS(ctx->freq) > 0.01) {
    amp2 = 1 / (ctx->freq * 2.0 * M_PI);
  } else {
    amp2 = 0;
  }

  ctx->time += period;

  if(ABS(ctx->freq * ctx->time) >= 1.0 && ABS(ctx->freq) > 0.0) {  // TODO fix
    ctx->time -= 1.0 / ABS(ctx->freq);
  }

  float co = 0.0;
  float si = 0.0;
  sincos_fast(ctx->freq * ctx->time * 2.0 * M_PI, &si, &co);

  sin  = ctx->amp * si;
  sin2 = sin * amp2;
  sin3 = sin2 * amp2;

  float s = sin;
  float o = PIN(offset);
  ctx->vel += ctx->freq * 2.0 * M_PI * period;
  ctx->vel = mod(ctx->vel);

  PIN(sin)    = s + o;
  PIN(sin2)   = sin2 + o;
  PIN(sin3)   = sin3 + o;
  PIN(msin)   = ((int)(mod(s + o) * r)) / r;
  PIN(msin2)  = ((int)(mod(sin2 + o) * r)) / r;
  PIN(msin3)  = ((int)(mod(sin3 + o) * r)) / r;
  PIN(square) = (sin > 0.0) ? (ctx->amp + o) : (-ctx->amp + o);
  PIN(vel)    = ctx->vel;  //mod(((int)(vel * r)) / r + o);
}

hal_comp_t sim_comp_struct = {
    .name      = "sim",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct sim_ctx_t),
    .pin_count = sizeof(struct sim_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
