#include "enc_cmd_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"

HAL_COMP(enc_cmd);

HAL_PIN(res);
HAL_PIN(pos);
HAL_PIN(a);
HAL_PIN(b);
HAL_PIN(fault);
HAL_PIN(mode);   // 0 = quad, 1 = step/dir, 2 = dir/step, 3 = up/down
HAL_PIN(remap);  // 0 = cmd, 1 = fb0, 2 = fb1, 3 = cmd bene style
HAL_PIN(input_filter);

struct enc_cmd_ctx_t {
  int e_res;
  uint32_t a_pin, b_pin, c_pin, c_en_pin, tim_af, tim_rcc;
  GPIO_TypeDef *a_port, *b_port, *c_port, *c_en_port;
  TIM_TypeDef *tim;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_cmd_ctx_t *ctx      = (struct enc_cmd_ctx_t *)ctx_ptr;
  struct enc_cmd_pin_ctx_t *pins = (struct enc_cmd_pin_ctx_t *)pin_ptr;
  ctx->e_res                     = 0;
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

  LL_TIM_SetAutoReload(ctx->tim, ctx->e_res * 2 - 1);
  // quad
  LL_TIM_DisableCounter(ctx->tim);
  // X4 on TI1/TI2, both direct, non-inverted, no prescaler, input_filter
  // (0..15) as the IC1F/IC2F code. The StdPeriph setup had TI1 on "both
  // edges", which RM0090 forbids in encoder mode, and its unshifted
  // "prescaler 1" had turned CC1S into TRC; neither reached the counting
  // path, so the count is the same.
  uint32_t filter = (uint32_t)MAX(MIN(PIN(input_filter), 15), 0) << (TIM_CCMR1_IC1F_Pos + 16U);  // the LL_TIM_IC_FILTER_* encoding
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

  int32_t tim = LL_TIM_GetCounter(ctx->tim);

  PIN(a) = LL_GPIO_IsInputPinSet(ctx->a_port, ctx->a_pin);
  PIN(b) = LL_GPIO_IsInputPinSet(ctx->b_port, ctx->b_pin);

  float p = 0.0;
  p       = mod(tim * 2.0f * M_PI / (float)ctx->e_res);

  PIN(pos) = p;

  int r = (int)PIN(res);
  if(r < 1) {
    r = 1;
  }

  if(ctx->e_res != r) {
    ctx->e_res = r;
    LL_TIM_SetAutoReload(ctx->tim, ctx->e_res * 2 - 1);
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
