#include "res_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"

/**
* ## Brief
* `res` generates the resolver excitation (reference) signal on the FB0 connector of the F4 board and computes the rotor angle from the demodulated sin/cos signals delivered by the `adc` component. It is loaded by `conf/template/res_fb0.txt`, which links `res0.sin`/`cos`/`quad` to `adc0.sin0`/`cos0`/`quad`, `adc0.res_mode = res0.res_mode`, `res0.poles = conf0.mot_fb_polecount`, `res0.vel = fb_switch0.mot_vel` and `res0.pos` to `fb_switch0.mot_pos`/`mot_abs_pos`.
*
* ## Component Explanation
*
* 1. **Reference signal (hw_init and rt)**:
* - V4 board: TIM4 is clocked by the ADC master timer trigger (TIM3, `ADC_TRIGGER_FREQ` = 1.2 MHz). TIM4 channel 3 toggles the reference output on FB0 Z/RES_REF (PD14); the FB0 line driver enable (PD15) is switched on.
* - Every rt cycle `freq` is rounded to a multiple of the rt frequency (5 kHz) and clamped to 5..20 kHz (the shipped configs use 5 kHz); the rounded value is written back to `freq`.
* ```c
* mult     = CLAMP(freq / 5000 + 0.5, 1, 4);
* ARR      = 1.2e6 / 2 / (5000 * mult) - 1;
* CCR3     = CLAMP(phase * ARR, 0, ARR - 1);
* res_mode = ADC_GROUPS / 2 / mult;   // = 12 / mult, ADC sample groups per half wave
* ```
* - `phase` must be tuned so that `amp` is maximal (the configs use 0.89..0.9).
*
* 2. **Signal check (rt)**:
* - `amp = sqrt(sin^2 + cos^2)`. If `amp < min_amp`: `error = 1`, `state = 0`, `pos` is not updated. Otherwise `error = 0`, `state = 3`.
*
* 3. **Angle (rt)**:
* - `pos_e = atan2(sin, cos)` (computed with `atan2_fast()`, a polynomial approximation with about 3e-7 rad maximum error), plus a delay compensation of half an rt period: `dpos = vel * period / 2`.
* - `poles = 1`: `pos = mod(pos_e + dpos)`.
* - `poles > 1` (multi-speed resolver): the electrical cycle is counted by watching `quad` (quadrant from `adc`) change 2 -> 3 (+1) or 3 -> 2 (-1), wrapped to 0..poles-1, and
* ```c
* pos = mod((pos_e + cycle * 2 * pi) / poles + dpos);   // rad, +-pi
* ```
*
* {{% hint warning %}}
* With `poles > 1` the cycle counter starts at 0 at power up, so the mechanical position is only relative (not absolute) and a missed quadrant change shifts it by one electrical cycle. The rt code uses the V4 macros `RT_FREQ`, `ADC_TRIGGER_FREQ` and `ADC_GROUPS`, so the component only builds for V4 (the source comment "clk = TIM_MASTER(TIM2)" is stale, TIM_MASTER is TIM3 on V4).
* {{% /hint %}}
*/

HAL_COMP(res);

HAL_PIN(pos);       // *output*, Rotor position (rad, +-pi)
HAL_PIN(amp);       // *output*, Amplitude of the sin/cos signals
HAL_PIN(quad);      // *input*, Quadrant of the sin/cos signals from adc (1..4)
HAL_PIN(poles);     // *parameter*, Resolver speed, electrical cycles per revolution (default 1)
HAL_PIN(min_amp);   // *parameter*, Minimum amplitude, below it error = 1 (default 0.15)

HAL_PIN(vel);       // *input*, Velocity for delay compensation (rad/s)

HAL_PIN(sin);       // *input*, Demodulated sine signal from adc
HAL_PIN(cos);       // *input*, Demodulated cosine signal from adc

HAL_PIN(enable);    // *input*, Not used
HAL_PIN(error);     // *output*, 1 = amplitude below min_amp
HAL_PIN(state);     // *output*, 0 = error, 3 = position valid
HAL_PIN(phase);     // *parameter*, Reference phase adjust 0..1 relative to ADC sampling (default 0.85)
HAL_PIN(res_mode);  // *output*, ADC sample groups per half wave for adc0.res_mode (12 / (freq / 5 kHz))
HAL_PIN(freq);      // *input/output*, Reference frequency, rounded to 5, 10, 15 or 20 kHz (default 10000)

// TODO: in hal stop, reset adc dma

struct res_ctx_t {
  int lastq;   // last quadrant
  int abspos;  // multiturn position
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct res_ctx_t *ctx      = (struct res_ctx_t *)ctx_ptr;
  struct res_pin_ctx_t *pins = (struct res_pin_ctx_t *)pin_ptr;
  PIN(poles)                 = 1.0;
  PIN(phase)                 = 0.85;
  PIN(min_amp)               = 0.15;
  PIN(freq)                  = 10000;
}
static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct res_ctx_t *ctx = (struct res_ctx_t *)ctx_ptr;
  // struct res_pin_ctx_t *pins = (struct res_pin_ctx_t *)pin_ptr;

  ctx->abspos = 0;
  ctx->lastq  = 0;

  LL_TIM_OC_InitTypeDef TIM_OCInitStructure;
  LL_TIM_OC_StructInit(&TIM_OCInitStructure);
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  LL_GPIO_StructInit(&GPIO_InitStructure);

#ifdef V4
  LL_TIM_InitTypeDef TIM_TimeBaseStructure;
  LL_TIM_StructInit(&TIM_TimeBaseStructure);

  //timer init for v4, v3 uses slave timer
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  TIM_TimeBaseStructure.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.CounterMode       = LL_TIM_COUNTERMODE_UP;
  TIM_TimeBaseStructure.Autoreload            = ADC_TRIGGER_FREQ / FRT_FREQ - 1;  // 20kHz
  TIM_TimeBaseStructure.Prescaler         = 0;
  TIM_TimeBaseStructure.RepetitionCounter = 0;
  LL_TIM_Init(TIM4, &TIM_TimeBaseStructure);
  // Rising edges of the selected trigger (TRGI) clock the counter, clk = TIM_MASTER(TIM2) trigger out
  LL_TIM_SetTriggerInput(TIM4, LL_TIM_TS_ITR2);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_EnableCounter(TIM4);
#endif


  //12-1 40khz
  //15-1 35khz
  //20-1 30khz 2
  //24-1 25khz
  //30-1 20khz 3
  //40-1 15khz 4
  //60-1 10khz 6 default
  //120-1 5khz 12
  //res_en = ADC_GROUPS/2/(res_freq/rt_freq)
  //arr = ADC_TRIGGER_FREQ/2/res_freq

  // resolver reference signal OC
  TIM_OCInitStructure.OCMode       = LL_TIM_OCMODE_TOGGLE;
  TIM_OCInitStructure.OCState      = LL_TIM_OCSTATE_ENABLE;
  TIM_OCInitStructure.OCNState     = LL_TIM_OCSTATE_DISABLE;
  TIM_OCInitStructure.CompareValue = 0;
  TIM_OCInitStructure.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCIdleState  = LL_TIM_OCIDLESTATE_HIGH;
  TIM_OCInitStructure.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  //ref is always OC3
  LL_TIM_OC_Init(FB0_RES_REF_TIM, LL_TIM_CHANNEL_CH3, &TIM_OCInitStructure);
  LL_TIM_OC_EnablePreload(FB0_RES_REF_TIM, LL_TIM_CHANNEL_CH3);
  if(IS_TIM_BREAK_INSTANCE(FB0_RES_REF_TIM)) {
    LL_TIM_EnableAllOutputs(FB0_RES_REF_TIM);
  }

  //resolver ref signal generation
  GPIO_InitStructure.Pin   = FB0_RES_REF_PIN;
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Alternate = FB0_RES_REF_TIM_AF;
  LL_GPIO_Init(FB0_RES_REF_PORT, &GPIO_InitStructure);

  //txen
  GPIO_InitStructure.Pin   = FB0_Z_TXEN_PIN;
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FB0_Z_TXEN_PORT, &GPIO_InitStructure);
  LL_GPIO_SetOutputPin(FB0_Z_TXEN_PORT, FB0_Z_TXEN_PIN);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct res_ctx_t *ctx      = (struct res_ctx_t *)ctx_ptr;
  struct res_pin_ctx_t *pins = (struct res_pin_ctx_t *)pin_ptr;
  //TODO: arr can change!
  uint32_t mult         = CLAMP(PIN(freq) / RT_FREQ + 0.5, 1, 4);
  PIN(freq)             = RT_FREQ * mult;
  FB0_RES_REF_TIM->ARR  = ADC_TRIGGER_FREQ / 2 / (RT_FREQ * mult) - 1;
  FB0_RES_REF_TIM->CCR3 = (int)CLAMP(PIN(phase) * FB0_RES_REF_TIM->ARR, 0, FB0_RES_REF_TIM->ARR - 1);
  PIN(res_mode)         = ADC_GROUPS / 2 / mult;
  float s               = 0.0;
  float c               = 0.0;
  float a               = 0.0;

  s = PIN(sin);
  c = PIN(cos);
  a = sqrtf(s * s + c * c);

  float p = MAX(1.0, PIN(poles));

  float pos  = atan2_fast(s, c);
  float dpos = PIN(vel) * period / 2.0;

  if(a < PIN(min_amp)) {
    PIN(error) = 1.0;
    PIN(state) = 0.0;
  } else {
    PIN(error) = 0.0;
    PIN(state) = 3.0;
    if(p == 1.0f) {
      PIN(pos) = mod(pos + dpos);
    } else {
      int q = PIN(quad);  // current quadrant

      if(ctx->lastq == 2 && q == 3)
        ctx->abspos++;
      if(ctx->lastq == 3 && q == 2)
        ctx->abspos--;

      if(ctx->abspos >= p) {
        ctx->abspos = 0;
      }
      if(ctx->abspos <= -1) {
        ctx->abspos = p - 1;
      }

      ctx->lastq = q;
      //TODO: clamp ctx->abspos
      float absa = pos + ctx->abspos * M_PI * 2.0f;
      PIN(pos)   = mod(absa / p + dpos);
    }
  }
  PIN(amp) = a;
}

const hal_comp_t res_comp_struct = {
    .name      = "res",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct res_ctx_t),
    .pin_count = sizeof(struct res_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
