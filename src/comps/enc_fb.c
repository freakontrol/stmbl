#include "enc_fb_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"

/**
* ## Brief
* `enc_fb` reads an incremental quadrature encoder (A/B with optional index Z, optionally with analog sin/cos tracks) on the FB0 connector of the F4 board. It is loaded by `conf/template/enc_fb0.txt` (and `encws_fb0.txt`, which disables the index), which links `enc_fb0.pos`, `abs_pos` and `state` to `fb_switch0.mot_*`, `sin`/`cos`/`quad`/`amp` to the `adc0` sin0l/cos0l/quad/amp0 pins and `vel` to `vel1.vel`.
*
* ## Component Explanation
*
* 1. **Hardware (hw_init)**:
* - FB0 A = PD12, B = PD13, Z = PD14 are connected to TIM4 (V4 board) in encoder mode (TI1 and TI2 edges, 4 counts per line, B input inverted), auto-reload = `res - 1`, with `res` clamped to >= 1.
* - A and B use the timer's digital input filter at fCK_INT with N = 8 (about 95 ns at 84 MHz), so short glitches are ignored.
* - Channel 3 of the same timer captures the counter value on the Z (index) input.
*
* 2. **Count and quadrant correction (rt)**:
* - The counter and the A/B port are latched together at the start of the rt interrupt (`DMA2_Stream0_IRQHandler` in `main.c`, about 4 us after the last sin/cos sample), close to the time the analog quadrant was sampled.
* - `qdiff = quad - oquad`, where `quad` is the quadrant computed by the ADC component from the analog sin/cos signals. If they differ by one quadrant the count is corrected by +-1 so the digital count matches the analog signals, then wrapped to 0..res-1.
* - `pos = mod(count * 2 * pi / res)`. If `res` changes at runtime the timer auto-reload is updated.
*
* 3. **Index and absolute position (rt)**:
* - If `en_index > 0` and an index capture happened, the captured count is stored as the offset and `state` becomes 3.
* - `abs_pos = minus(pos, offset)`. Before the first index the offset is 0.
* - If `indexprint > 0`, the nrt function prints the captured counter value (`cnt = ...`) once per index event.
*
* 4. **Signal check and interpolation (rt)**:
* - If `amp > min_amp` or `|vel| > 0.15`: `error = 0`, `state` is at least 1, and the interpolated position is computed from the analog angle:
* ```c
* ipos = mod(pos + (int)(ires * mod(4 * atan2(sin, cos) + pi) / pi) / ires * pi / res);
* ```
* - i.e. the sin/cos phase adds a fine offset of +-half a count, quantised to `ires` steps.
* - Otherwise `error = 1` and `state = 0`. For a plain digital encoder without sin/cos `amp` must be set to a constant above `min_amp` (e.g. `enc_fb0.amp = 5` in `conf/haas_spindle_slip_uf.txt`), otherwise the encoder is reported as faulty at standstill.
*
* {{% hint warning %}}
* The `state` drops back to 0 whenever the amplitude/velocity check fails, even after an index was found (the stored index offset is kept).
* {{% /hint %}}
*/

HAL_COMP(enc_fb);

HAL_PIN(res);         // *parameter*, Counts per revolution (4 x lines, default 2048)
HAL_PIN(ires);        // *parameter*, Interpolation steps for ipos (default 1024)
HAL_PIN(pos);         // *output*, Incremental position (rad, +-pi)
HAL_PIN(abs_pos);     // *output*, Position relative to the index (rad, +-pi)
HAL_PIN(state);       // *output*, 0 = no signal, 1 = position valid, 3 = index found
HAL_PIN(index);       // *output*, Raw level of the Z input (0/1)
HAL_PIN(a);           // *output*, Raw level of the A input (0/1)
HAL_PIN(b);           // *output*, Raw level of the B input (0/1)
HAL_PIN(ipos);        // *output*, Position interpolated with the sin/cos signals (rad, +-pi)
HAL_PIN(sin);         // *input*, Analog sine signal from adc
HAL_PIN(cos);         // *input*, Analog cosine signal from adc
HAL_PIN(quad);        // *input*, Analog quadrant (1..4) from adc
HAL_PIN(oquad);       // *output*, Digital quadrant (1..4) from the A/B levels
HAL_PIN(oquadoff);    // *parameter*, Not used
HAL_PIN(qdiff);       // *output*, quad - oquad, +-1 corrects the count
HAL_PIN(error);       // *output*, 1 if amp is below min_amp and abs(vel) below 0.15 (mot fb error at standstill)
HAL_PIN(amp);         // *input*, Analog signal amplitude, > min_amp counts as valid
HAL_PIN(min_amp);     // *parameter*, Amplitude below which error is set at standstill (default 0.25)
HAL_PIN(vel);         // *input*, Velocity (rad/s), abs > 0.15 counts as valid
HAL_PIN(ccr3);        // *output*, Not used
HAL_PIN(en_index);    // *parameter*, > 0 enables the index capture
HAL_PIN(indexprint);  // *parameter*, > 0 prints the counter value at each index


struct enc_fb_ctx_t {
  int e_res;
  float absoffset;
};

// latched in DMA2_Stream0_IRQHandler (main.c), see there
extern volatile uint32_t fb0_cnt_latch;
extern volatile uint32_t fb0_idr_latch;

static int indexpos   = 0;
static int indexprint = 0;

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_fb_ctx_t *ctx      = (struct enc_fb_ctx_t *)ctx_ptr;
  struct enc_fb_pin_ctx_t *pins = (struct enc_fb_pin_ctx_t *)pin_ptr;
  ctx->e_res                    = 0;
  ctx->absoffset                = 0.0;
  PIN(res)                      = 2048.0;
  PIN(ires)                     = 1024.0;
  PIN(min_amp)                  = 0.25;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_fb_ctx_t *ctx      = (struct enc_fb_ctx_t *)ctx_ptr;
  struct enc_fb_pin_ctx_t *pins = (struct enc_fb_pin_ctx_t *)pin_ptr;
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  LL_GPIO_StructInit(&GPIO_InitStructure);

  /* **************** port 1, quadrature , sin/cos or resolver *********************/
  ctx->e_res = (int)PIN(res);
  if(ctx->e_res < 1) {
    ctx->e_res = 1;
  }
  // enable clocks
  LL_APB1_GRP1_EnableClock(FB0_ENC_TIM_RCC);

  // pin mode: af
  GPIO_InitStructure.Pin   = FB0_A_PIN;
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Alternate = FB0_ENC_TIM_AF;
  LL_GPIO_Init(FB0_A_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = FB0_B_PIN;
  GPIO_InitStructure.Alternate = FB0_ENC_TIM_AF;
  LL_GPIO_Init(FB0_B_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = FB0_Z_PIN;
  GPIO_InitStructure.Alternate = FB0_ENC_TIM_AF;
  LL_GPIO_Init(FB0_Z_PORT, &GPIO_InitStructure);

  // pin af -> tim

  // enc res / turn
  LL_TIM_SetAutoReload(FB0_ENC_TIM, ctx->e_res - 1);

  // quad
  LL_TIM_DisableCounter(FB0_ENC_TIM);
  // X4 on TI1/TI2, B inverted. Input filter on A and B: fCK_INT with N = 8,
  // about 95 ns at 84 MHz (RM0090 18.4.7). Not 0xF (fDTS/32, N = 8: 3 us),
  // which would cap the line rate near 160 kHz.
  LL_TIM_ENCODER_Init(FB0_ENC_TIM, &(LL_TIM_ENCODER_InitTypeDef){
                                       .EncoderMode    = LL_TIM_ENCODERMODE_X4_TI12,
                                       .IC1Polarity    = LL_TIM_IC_POLARITY_RISING,
                                       .IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
                                       .IC1Prescaler   = LL_TIM_ICPSC_DIV1,
                                       .IC1Filter      = LL_TIM_IC_FILTER_FDIV1_N8,
                                       .IC2Polarity    = LL_TIM_IC_POLARITY_FALLING,
                                       .IC2ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
                                       .IC2Prescaler   = LL_TIM_ICPSC_DIV1,
                                       .IC2Filter      = LL_TIM_IC_FILTER_FDIV1_N8,
                                   });
  LL_TIM_EnableCounter(FB0_ENC_TIM);
  FB0_ENC_TIM->CCMR2 |= TIM_CCMR2_CC3S_0;  //CC3 channel is configured as input, IC3 is mapped on CH3
  FB0_ENC_TIM->CCER |= TIM_CCER_CC3E;      //Capture enabled
}


// static void frt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
//   struct enc_fb_ctx_t *ctx      = (struct enc_fb_ctx_t *)ctx_ptr;
//   struct enc_fb_pin_ctx_t *pins = (struct enc_fb_pin_ctx_t *)pin_ptr;

//   float p  = mod(LL_TIM_GetCounter(FB0_ENC_TIM) * 2.0f * M_PI / (float)ctx->e_res);
//   PIN(pos) = p;
//   //TODO: this gets triggered by wire saving abs encoders. add timeout?
//   if(RISING_EDGE(!LL_GPIO_IsInputPinSet(FB0_Z_PORT, FB0_Z_PIN))) {
//     // TODO: fix
//     ctx->absoffset = -p;
//     PIN(state)     = 3.0;

//   }
//   PIN(index)  = LL_GPIO_IsInputPinSet(FB0_Z_PORT, FB0_Z_PIN);
//   PIN(abs_pos) = mod(p + ctx->absoffset);
// }

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_fb_ctx_t *ctx      = (struct enc_fb_ctx_t *)ctx_ptr;
  struct enc_fb_pin_ctx_t *pins = (struct enc_fb_pin_ctx_t *)pin_ptr;

  // timer value and timer pins, sampled together at the start of the rt
  // interrupt, about 4 us after the last sin/cos sample. Read here they were
  // about 50 us later (spindle bench, 128 lines): qdiff reached 2 from
  // 220 rad/s and went uncorrected. Latched, no qdiff 2 up to 838 rad/s.
  int32_t tim     = fb0_cnt_latch;
  uint32_t scgpio = fb0_idr_latch;

  float p = 0.0;
  int r   = (int)PIN(res);
  if(r < 1) {
    r = 1;
  }

  float ir = PIN(ires);
  if(ir < 1) {
    ir = 1;
  }

  float s = PIN(sin);
  float c = PIN(cos);

  int q;

  //calculate quadrant of timer
  if((scgpio & FB0_A_PIN)) {  //TODO: invert for v3... check: plot oquad vs quad
    if(scgpio & FB0_B_PIN) {
      q = 1;
    } else {
      q = 2;
    }
  } else {
    if(scgpio & FB0_B_PIN) {
      q = 4;
    } else {
      q = 3;
    }
  }
  //TODO: sincos stuff at speed
  //analog quadrant is calculated by adc component
  int qdiff = PIN(quad) - q;

  switch(qdiff) {
    case 1:
    case -3:
      tim++;
      break;
    case -1:
    case 3:
      tim--;
      break;
    default:
      break;
  }

  if(tim >= ctx->e_res) {
    tim = 0;
  } else if(tim < 0) {
    tim = ctx->e_res - 1;
  }

  PIN(qdiff) = qdiff;

  PIN(a) = (scgpio & FB0_A_PIN) > 0;  //TODO: invert for v3
  PIN(b) = (scgpio & FB0_B_PIN) > 0;

  PIN(oquad) = q;

  p        = mod(tim * 2.0f * M_PI / (float)ctx->e_res);
  PIN(pos) = p;

  if((PIN(en_index) > 0.0) && (FB0_ENC_TIM->SR & TIM_SR_CC3IF)) {
    int cc         = FB0_ENC_TIM->CCR3;
    PIN(state)     = 3.0;
    ctx->absoffset = mod(cc * 2.0f * M_PI / (float)ctx->e_res);
    if(PIN(indexprint) > 0.0) {
      indexpos   = cc;
      indexprint = 1;
    }
  }
  PIN(abs_pos) = minus(p, ctx->absoffset);
  PIN(index)   = LL_GPIO_IsInputPinSet(FB0_Z_PORT, FB0_Z_PIN);

  if(PIN(amp) > PIN(min_amp) || ABS(PIN(vel)) > 0.15) {
    PIN(error) = 0.0;
    PIN(state) = MAX(PIN(state), 1.0);
    PIN(ipos)  = mod(p + ((int)(ir * mod(atan2_fast(s, c) * 4.0 + M_PI) * M_1_PI)) / ir * M_PI / (float)ctx->e_res);
  } else {
    PIN(error) = 1.0;
    PIN(state) = 0.0;
  }

  if(ctx->e_res != r) {
    ctx->e_res = r;
    LL_TIM_SetAutoReload(FB0_ENC_TIM, ctx->e_res - 1);
  }
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct enc_fb_ctx_t *ctx      = (struct enc_fb_ctx_t *)ctx_ptr;
  // struct enc_fb_pin_ctx_t *pins = (struct enc_fb_pin_ctx_t *)pin_ptr;
  if(indexprint == 1) {
    indexprint = 0;
    printf("cnt = %i\n", indexpos);
  }
}

const hal_comp_t enc_fb_comp_struct = {
    .name      = "enc_fb",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct enc_fb_ctx_t),
    .pin_count = sizeof(struct enc_fb_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
