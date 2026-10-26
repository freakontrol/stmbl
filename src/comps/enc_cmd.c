#include "enc_cmd_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"

/**
* ## Brief
* `enc_cmd` reads an incremental position command (quadrature or step/dir) with an STM32 hardware timer on the F4 board and outputs it as an angle. It is loaded by `conf/template/enc_cmd.txt` (`enc_cmd0.res = conf0.cmd_res`, `rev0.in = enc_cmd0.pos`, `fault0.cmd_error = enc_cmd0.error`), so the command position goes through `rev0` to build a multiturn command. It can also drive a fault/ready line back to the controller.
*
* ## Component Explanation
*
* 1. **Connector and timer selection (`remap`, read once in hw_init)**:
* - `remap = 0` (default): CMD connector, A = PA15, B = PB3 on TIM2 (32 bit). Fault line = CMD C (PB5), its line driver enable PB9 is switched on.
* - `remap = 1`: FB0 connector, A = PD12, B = PD13 on TIM4. Fault line = FB0 Z (PD14), enable PD15.
* - `remap = 2`: FB1 connector, A = PE9, B = PE11 on TIM1. Fault line = FB1 Z (PE13), enable PE14.
* - `remap = 3` ("bene style", used by `conf/bene_sanyo.txt` and `conf/spindle_slip_uf.txt`): A = PA8, B = PA9 on TIM1. Fault line = CMD D (PB8), enable PB2.
* - Any other value returns from hw_init without configuring anything, `pos` then reads a timer that was never set up.
* - A and B get the timer alternate function with pull-ups, the fault pin and its enable are push-pull outputs.
*
* 2. **Signal type (`mode`, read once in hw_init)**:
* - `mode = 0` (default): quadrature. The timer runs in X4 encoder mode (counts every edge of A and B, so 4 counts per line), auto-reload = `res - 1`.
* - `mode = 1`: step/dir, step on A, direction on B. `mode = 2`: dir/step, direction on A, step on B. Direction high counts up.
* - In step/dir the timer is a free running 32 bit up counter clocked by the rising edges of the step input. Each edge of the direction input captures the count in hardware (input capture), and the TIM2 interrupt (priority 1, above the rt) adds the steps since the last direction edge with the old sign. So the step at which the direction changed is exact whatever the interrupt latency; it only has to be taken before the next direction edge, else the capture overflow flag marks it as lost. The STM32 encoder modes cannot decode step/dir: they count a step pulse's rising and falling edge with opposite signs.
* - `mode = 3` (up/down) is not supported, and neither is step/dir on `remap` 1..3 (16 bit timers, their DMA belongs to the feedback components). Then `error` is set and the timer is not set up.
* - Both inputs use the digital input filter `input_filter` (IC1F/IC2F code), clamped, without prescaler.
*
* 3. **Position output (rt)**:
* - Quadrature: each rt cycle the counter is read and converted:
* ```c
* pos = mod(counter * 2 * pi / res);   // rad, wrapped to +-pi
* ```
* - In quadrature `res` is counts per revolution at X4, so lines x 4 (a 2000 line encoder is `res = 8000`), the same convention as `enc_fb`. The counter wraps once per revolution; the multiturn tracking is done downstream (e.g. `rev0`).
* - Step/dir: the rt takes a pending direction edge, reads the count and adds the steps since the last edge with the current sign, with interrupts off. The signed step count is kept modulo `res`, and `pos = mod(steps * 2 * pi / res)`, so in step/dir `res` is steps per revolution.
* - `res` is clamped to >= 1, and in quadrature on the 16 bit timers (`remap` 1..3) to <= 65536. If it changes at runtime the auto-reload register is updated (quadrature only).
*
* 4. **Error and fault output (rt)**:
* - The template links `error` to `fault0.cmd_error`, so it trips fault 1 (CMD error).
* - A `fault` of 0 drives the fault line low (e.g. `enc_cmd0.fault = fault0.fault` or an inverted enable signal).
*
* {{% hint warning %}}
* Breaking change: before this version quadrature `res` was counts per revolution at 2 counts per line (lines x 2), with auto-reload `2 * res - 1`. Configs that use quadrature `enc_cmd` need `res` (`conf0.cmd_res`) doubled. With an old value the drive moves twice the commanded distance, and with commutation from the command input (autophasing) the commutation angle is wrong. Step/dir is unchanged.
* {{% /hint %}}
*
* {{% hint warning %}}
* With TIM1/TIM4 (`remap` 1..3) the counter is 16 bit, so quadrature `res` is clamped to 65536. The pin/timer definitions exist only for the V4 board.
* {{% /hint %}}
*/

HAL_COMP(enc_cmd);

HAL_PIN(res);           // *parameter*, Quadrature: counts per revolution at X4 (lines x 4, <= 65536 on remap 1..3); step/dir: steps per revolution; default 4096
HAL_PIN(pos);           // *output*, Command position (rad, +-pi)
HAL_PIN(a);             // *output*, Raw level of the A input (0/1)
HAL_PIN(b);             // *output*, Raw level of the B input (0/1)
HAL_PIN(fault);         // *input*, > 0 drives the fault/ready line high
HAL_PIN(mode);          // *parameter*, 0 = quadrature, 1 = step/dir (step on A), 2 = dir/step (step on B); 3 = up/down is not supported; read at start
HAL_PIN(remap);         // *parameter*, Input connector: 0 = CMD (TIM2), 1 = FB0 (TIM4), 2 = FB1 (TIM1), 3 = PA8/PA9 bene style (TIM1); step/dir only on 0
HAL_PIN(input_filter);  // *parameter*, Timer input filter setting 0..15 (default 3)
HAL_PIN(error);         // *output*, 1 = mode not supported on this remap, or a direction change was lost (latched)

struct enc_cmd_ctx_t {
  int e_res;
  int step;        // 0 = quadrature, 1 = step/dir or dir/step on TIM2
  uint32_t cc_ch;  // the dir channel's capture: CC2 (mode 1) or CC1 (mode 2)
  int32_t steps;   // signed step count up to the last direction edge, mod e_res
  uint32_t last;   // raw step count at the last direction edge
  int32_t sign;    // count direction since that edge
  int lost;        // a direction edge came before the previous one was taken
  uint32_t a_pin, b_pin, c_pin, c_en_pin, tim_af, tim_rcc;
  GPIO_TypeDef *a_port, *b_port, *c_port, *c_en_port;
  TIM_TypeDef *tim;
};

// Step/dir: the timer runs as an up counter clocked by the step input
// (external clock mode 1, rising edges), so it counts every step without
// software. Each edge on the dir input captures that count into CCRx in
// hardware, so the step where the direction changed is exact whatever the
// interrupt latency; the interrupt only has to take the capture before the
// next direction change, else CCxOF flags the loss. The STM32 encoder modes
// cannot do this: they count a step pulse's rising and falling edge with
// opposite signs.
static struct enc_cmd_ctx_t *step_ctx;

static void take_dir_edge(struct enc_cmd_ctx_t *ctx) {
  TIM_TypeDef *tim = ctx->tim;
  uint32_t sr      = tim->SR;
  uint32_t ccif    = ctx->cc_ch == LL_TIM_CHANNEL_CH1 ? TIM_SR_CC1IF : TIM_SR_CC2IF;
  uint32_t ccof    = ctx->cc_ch == LL_TIM_CHANNEL_CH1 ? TIM_SR_CC1OF : TIM_SR_CC2OF;
  if(!(sr & ccif)) {
    return;
  }
  uint32_t cap = ctx->cc_ch == LL_TIM_CHANNEL_CH1 ? tim->CCR1 : tim->CCR2;  // reading clears CCxIF
  if(sr & ccof) {
    tim->SR   = ~ccof;
    ctx->lost = 1;
  }
  ctx->steps = (ctx->steps + ctx->sign * (int32_t)(cap - ctx->last)) % ctx->e_res;
  ctx->last  = cap;
  ctx->sign  = -ctx->sign;
}

void TIM2_IRQHandler(void) {
  if(step_ctx) {
    take_dir_edge(step_ctx);
  } else {
    TIM2->SR = 0;
  }
}

// quadrature res clamped to the 16 bit timers (remap 1..3), ARR = res - 1
static int quad_res(struct enc_cmd_ctx_t *ctx, int r) {
  if(ctx->tim != CMD_ENC_TIM && r > 65536) {
    r = 65536;
  }
  return r;
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_cmd_ctx_t *ctx      = (struct enc_cmd_ctx_t *)ctx_ptr;
  struct enc_cmd_pin_ctx_t *pins = (struct enc_cmd_pin_ctx_t *)pin_ptr;
  ctx->e_res                     = 0;
  ctx->step                      = 0;
  ctx->lost                      = 0;
  PIN(res)                       = 4096.0;
  PIN(input_filter)              = 3;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_cmd_ctx_t *ctx      = (struct enc_cmd_ctx_t *)ctx_ptr;
  struct enc_cmd_pin_ctx_t *pins = (struct enc_cmd_pin_ctx_t *)pin_ptr;

  ctx->e_res = (int)PIN(res);
  if(ctx->e_res < 1) {
    ctx->e_res = 1;
  }

  //LL_AHB1_GRP1_EnableClock(FB1_|ENC0_B_IO_RCC);    //Enable needed Clocks for IOs
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;


  int mode = (int)PIN(mode);
  if(mode == 1 || mode == 2) {
    if((int)PIN(remap) != 0) {  // FB0/FB1 timers are 16 bit and their DMA belongs to the fb comps
      PIN(error) = 1.0;
      return;
    }
    ctx->step = 1;
  } else if(mode != 0) {  // up/down needs a second counter, the TIM2 DMA streams belong to the F3 link UART
    PIN(error) = 1.0;
    return;
  }

  switch((int)PIN(remap)) {
    case 0:
      ctx->a_pin        = CMD_A_PIN;
      ctx->b_pin        = CMD_B_PIN;
      ctx->c_pin        = CMD_C_PIN;
      ctx->c_en_pin     = CMD_C_EN_PIN;
      ctx->a_port       = CMD_A_PORT;
      ctx->b_port       = CMD_B_PORT;
      ctx->c_port       = CMD_C_PORT;
      ctx->c_en_port    = CMD_C_EN_PORT;
      ctx->tim_af       = CMD_ENC_TIM_AF;
      ctx->tim_rcc      = CMD_ENC_TIM_RCC;
      ctx->tim          = CMD_ENC_TIM;
      LL_APB1_GRP1_EnableClock(ctx->tim_rcc);
      break;
    case 1:
      ctx->a_pin        = FB0_A_PIN;
      ctx->b_pin        = FB0_B_PIN;
      ctx->c_pin        = FB0_Z_PIN;
      ctx->c_en_pin     = FB0_Z_TXEN_PIN;
      ctx->a_port       = FB0_A_PORT;
      ctx->b_port       = FB0_B_PORT;
      ctx->c_port       = FB0_Z_PORT;
      ctx->c_en_port    = FB0_Z_TXEN_PORT;
      ctx->tim_af       = FB0_ENC_TIM_AF;
      ctx->tim_rcc      = FB0_ENC_TIM_RCC;
      ctx->tim          = FB0_ENC_TIM;
      LL_APB1_GRP1_EnableClock(ctx->tim_rcc);
      break;
    case 2:
      ctx->a_pin        = FB1_A_PIN;
      ctx->b_pin        = FB1_B_PIN;
      ctx->c_pin        = FB1_Z_PIN;
      ctx->c_en_pin     = FB1_Z_TXEN_PIN;
      ctx->a_port       = FB1_A_PORT;
      ctx->b_port       = FB1_B_PORT;
      ctx->c_port       = FB1_Z_PORT;
      ctx->c_en_port    = FB1_Z_TXEN_PORT;
      ctx->tim_af       = FB1_ENC_TIM_AF;
      ctx->tim_rcc      = FB1_ENC_TIM_RCC;
      ctx->tim          = FB1_ENC_TIM;
      LL_APB2_GRP1_EnableClock(ctx->tim_rcc);
      break;
    case 3:
      ctx->a_pin        = LL_GPIO_PIN_8;
      ctx->b_pin        = LL_GPIO_PIN_9;
      ctx->c_pin        = LL_GPIO_PIN_8;
      ctx->c_en_pin     = LL_GPIO_PIN_2;
      ctx->a_port       = GPIOA;
      ctx->b_port       = GPIOA;
      ctx->c_port       = GPIOB;
      ctx->c_en_port    = GPIOB;
      ctx->tim_af       = FB1_ENC_TIM_AF;
      ctx->tim_rcc      = FB1_ENC_TIM_RCC;
      ctx->tim          = FB1_ENC_TIM;
      LL_APB2_GRP1_EnableClock(ctx->tim_rcc);
      break;
    default:
      return;
  }
  GPIO_InitStruct.Pin = ctx->a_pin;
  GPIO_InitStruct.Alternate = ctx->tim_af;
  LL_GPIO_Init(ctx->a_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ctx->b_pin;
  GPIO_InitStruct.Alternate = ctx->tim_af;
  LL_GPIO_Init(ctx->b_port, &GPIO_InitStruct);

  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Pin   = ctx->c_pin;
  LL_GPIO_Init(ctx->c_port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = ctx->c_en_pin;
  LL_GPIO_Init(ctx->c_en_port, &GPIO_InitStruct);

  LL_GPIO_SetOutputPin(ctx->c_en_port, ctx->c_en_pin);

  //Bind pins to Timer

  LL_TIM_DisableCounter(ctx->tim);
  uint32_t filter = (uint32_t)MAX(MIN(PIN(input_filter), 15), 0) << (TIM_CCMR1_IC1F_Pos + 16U);  // the LL_TIM_IC_FILTER_* encoding

  if(ctx->step) {
    // mode 1: step on A (TI1), dir on B (TI2); mode 2: dir on A, step on B.
    // Dir high counts up.
    uint32_t step_ch = mode == 1 ? LL_TIM_CHANNEL_CH1 : LL_TIM_CHANNEL_CH2;
    ctx->cc_ch       = mode == 1 ? LL_TIM_CHANNEL_CH2 : LL_TIM_CHANNEL_CH1;
    LL_TIM_SetPrescaler(ctx->tim, 0);
    LL_TIM_SetAutoReload(ctx->tim, 0xFFFFFFFF);  // free running 32 bit step count
    LL_TIM_IC_Config(ctx->tim, step_ch, LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | filter | LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_Config(ctx->tim, ctx->cc_ch, LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | filter | LL_TIM_IC_POLARITY_BOTHEDGE);
    LL_TIM_SetTriggerInput(ctx->tim, mode == 1 ? LL_TIM_TS_TI1FP1 : LL_TIM_TS_TI2FP2);
    LL_TIM_SetClockSource(ctx->tim, LL_TIM_CLOCKSOURCE_EXT_MODE1);
    LL_TIM_SetCounter(ctx->tim, 0);
    ctx->last  = 0;
    ctx->steps = 0;
    GPIO_TypeDef *dir_port = mode == 1 ? ctx->b_port : ctx->a_port;
    uint32_t dir_pin       = mode == 1 ? ctx->b_pin : ctx->a_pin;
    ctx->sign              = LL_GPIO_IsInputPinSet(dir_port, dir_pin) ? 1 : -1;
    ctx->tim->SR           = 0;
    LL_TIM_CC_EnableChannel(ctx->tim, ctx->cc_ch);
    step_ctx = ctx;
    if(mode == 1) {
      LL_TIM_EnableIT_CC2(ctx->tim);
    } else {
      LL_TIM_EnableIT_CC1(ctx->tim);
    }
    // preempts the rt (2), so the rt only sees a pending edge if the frt (0) held it off
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
    LL_TIM_EnableCounter(ctx->tim);
    return;
  }

  ctx->e_res = quad_res(ctx, ctx->e_res);
  LL_TIM_SetAutoReload(ctx->tim, ctx->e_res - 1);
  // quad
  // X4 on TI1/TI2, both direct, non-inverted, no prescaler, input_filter
  // (0..15) as the IC1F/IC2F code.
  LL_TIM_ENCODER_Init(ctx->tim, &(LL_TIM_ENCODER_InitTypeDef){
                                    .EncoderMode    = LL_TIM_ENCODERMODE_X4_TI12,
                                    .IC1Polarity    = LL_TIM_IC_POLARITY_RISING,
                                    .IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
                                    .IC1Prescaler   = LL_TIM_ICPSC_DIV1,
                                    .IC1Filter      = filter,
                                    .IC2Polarity    = LL_TIM_IC_POLARITY_RISING,
                                    .IC2ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
                                    .IC2Prescaler   = LL_TIM_ICPSC_DIV1,
                                    .IC2Filter      = filter,
                                });
  LL_TIM_EnableCounter(ctx->tim);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_cmd_ctx_t *ctx      = (struct enc_cmd_ctx_t *)ctx_ptr;
  struct enc_cmd_pin_ctx_t *pins = (struct enc_cmd_pin_ctx_t *)pin_ptr;

  if(PIN(error) > 0.0 && !ctx->step) {  // mode not supported here, timer not set up
    return;
  }

  int32_t tim;
  if(ctx->step) {
    __disable_irq();
    take_dir_edge(ctx);  // an edge the frt held off
    uint32_t cnt = LL_TIM_GetCounter(ctx->tim);
    take_dir_edge(ctx);  // an edge between the first take and the count: cnt is past it
    ctx->steps = (ctx->steps + ctx->sign * (int32_t)(cnt - ctx->last)) % ctx->e_res;
    ctx->last  = cnt;
    tim        = ctx->steps;
    __enable_irq();
    if(ctx->lost) {
      PIN(error) = 1.0;
    }
  } else {
    tim = LL_TIM_GetCounter(ctx->tim);
  }

  PIN(a) = LL_GPIO_IsInputPinSet(ctx->a_port, ctx->a_pin);
  PIN(b) = LL_GPIO_IsInputPinSet(ctx->b_port, ctx->b_pin);

  float p = 0.0;
  p       = mod(tim * 2.0f * M_PI / (float)ctx->e_res);

  PIN(pos) = p;

  int r = (int)PIN(res);
  if(r < 1) {
    r = 1;
  }
  if(!ctx->step) {
    r = quad_res(ctx, r);
  }

  if(ctx->e_res != r) {
    ctx->e_res = r;
    if(!ctx->step) {
      LL_TIM_SetAutoReload(ctx->tim, ctx->e_res - 1);
    }
  }

  if(PIN(fault) > 0.0) {
    LL_GPIO_SetOutputPin(ctx->c_port, ctx->c_pin);
  } else {
    LL_GPIO_ResetOutputPin(ctx->c_port, ctx->c_pin);
  }
}

const hal_comp_t enc_cmd_comp_struct = {
    .name      = "enc_cmd",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct enc_cmd_ctx_t),
    .pin_count = sizeof(struct enc_cmd_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
