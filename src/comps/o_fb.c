#include "o_fb_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"

/**
* ## Brief
* `o_fb` (F4 logic board, hardware v4) turns the A and B lines of the fb0 feedback connector into digital outputs, e.g. to send an encoder or step/dir style signal to another device. No template loads it.
*
* ## Component Explanation
*
* 1. **Setup (hw_init)**:
* - Enables the fb0 A and B RS485 transmitters (FB0_A_EN, FB0_B_EN high) and configures the A and B pins as push-pull outputs. The fb0 connector can then no longer be used as a feedback input.
*
* 2. **Output (rt)**:
* - Once per rt period, the A line is driven from `a` and the B line from `b`. The output rate is therefore limited to the rt frequency.
*/

HAL_COMP(o_fb);

HAL_PIN(a);  // *input*, fb0 line A output, > 0 = high
HAL_PIN(b);  // *input*, fb0 line B output, > 0 = high

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct o_fb_ctx_t * ctx = (struct o_fb_ctx_t *)ctx_ptr;
  // struct o_fb_pin_ctx_t * pins = (struct o_fb_pin_ctx_t *)pin_ptr;

  LL_GPIO_InitTypeDef GPIO_InitStructure;

  LL_GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;

  GPIO_InitStructure.Pin = FB0_A_EN_PIN;
  LL_GPIO_Init(FB0_A_EN_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = FB0_B_EN_PIN;
  LL_GPIO_Init(FB0_B_EN_PORT, &GPIO_InitStructure);

  LL_GPIO_SetOutputPin(FB0_A_EN_PORT, FB0_A_EN_PIN);
  LL_GPIO_SetOutputPin(FB0_B_EN_PORT, FB0_B_EN_PIN);

  GPIO_InitStructure.Pin = FB0_A_PIN;
  LL_GPIO_Init(FB0_A_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = FB0_B_PIN;
  LL_GPIO_Init(FB0_B_PORT, &GPIO_InitStructure);
}


static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct o_fb_ctx_t * ctx = (struct o_fb_ctx_t *)ctx_ptr;
  struct o_fb_pin_ctx_t *pins = (struct o_fb_pin_ctx_t *)pin_ptr;

  if(PIN(a) > 0) {
    LL_GPIO_SetOutputPin(FB0_A_PORT, FB0_A_PIN);
  } else {
    LL_GPIO_ResetOutputPin(FB0_A_PORT, FB0_A_PIN);
  }

  if(PIN(b) > 0) {
    LL_GPIO_SetOutputPin(FB0_B_PORT, FB0_B_PIN);
  } else {
    LL_GPIO_ResetOutputPin(FB0_B_PORT, FB0_B_PIN);
  }
}

const hal_comp_t o_fb_comp_struct = {
    .name      = "o_fb",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct o_fb_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
