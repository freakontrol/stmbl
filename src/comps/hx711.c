#include "hx711_comp.h"
/*
* This file is part of the stmbl project.
*
* Copyright (C) 2019 Rene Hopf <renehopf@mac.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"

/**
* ## Brief
* `hx` (file src/comps/hx711.c) reads up to two HX711 24 bit load cell ADCs connected to the fb1 connector of the F4 logic board by bit-banging their clock and data lines, and outputs the scaled readings as `out0`/`out1`. conf/template/hx.txt loads it (`hx0.gain = 100`); conf/move.txt uses `hx0.out0` as force input for `fmove0`.
*
* ## Component Explanation
*
* 1. **Wiring (hw_init)**:
* - Clock (PD_SCK of both chips): fb1 Z line, driven as an output with the Z transmitter enabled.
* - Data: DOUT of chip 0 on fb1 line A, DOUT of chip 1 on fb1 line B. `data_inv` and `clk_inv` are for inverting line drivers/receivers.
*
* 2. **Reading (rt)**:
* - `timer` counts up by the rt period. Once `timer > time` and chip 0 signals a finished conversion (data line low), the component clocks 24 bits out of both chips at the same time, MSB first, then one more clock pulse, which selects channel A with gain 128 for the next conversion. `timer` is then reset.
* - Each clock half period is a busy loop of `sleep` NOPs, so a read blocks `rt` for roughly 50 clock half periods.
* - Conversion to the outputs:
* ```c
* out = raw / 0x7fffff * gain + offset;
* ```
*
* {{% hint warning %}}
* The value is shifted once more after the last bit, so `raw` is twice the HX711 reading and the sign is taken from bit 22 of the reading instead of bit 23: readings are scaled by 2 (absorbed by `gain` when calibrating) and wrap around beyond half of the HX711 input range. Only chip 0 is checked for data ready; chip 1 is assumed to convert in sync. There is no plausibility or saturation check (see the TODO in the code).
* {{% /hint %}}
*/

HAL_COMP(hx);

HAL_PIN(out0);      // *output*, scaled reading of chip 0 (fb1 A)
HAL_PIN(out1);      // *output*, scaled reading of chip 1 (fb1 B)
HAL_PIN(gain);      // *parameter*, scale for full range (default 1), used for both outputs
HAL_PIN(offset);    // *parameter*, offset added after scaling (default 0), used for both outputs
HAL_PIN(sleep);     // *parameter*, NOPs per clock half period, default 20, clamped to 1..50
HAL_PIN(clk_inv);   // *parameter*, > 0 inverts the clock line
HAL_PIN(data_inv);  // *parameter*, > 0 inverts the data lines
HAL_PIN(timer);     // *output*, time since the last reading (s)
HAL_PIN(time);      // *parameter*, min. time between readings (s), default 0.01

struct hx_ctx_t {
  uint32_t error;
};

static void nopsleep(uint32_t t) {
  while(t > 0) {
    asm volatile("nop");
    t--;
  }
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct hx_ctx_t *ctx = (struct hx_ctx_t *)ctx_ptr;
  struct hx_pin_ctx_t *pins = (struct hx_pin_ctx_t *)pin_ptr;

  PIN(sleep) = 20;
  PIN(time)  = 0.01;
  PIN(gain)  = 1;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct hx_ctx_t *ctx = (struct hx_ctx_t *)ctx_ptr;
  //struct hx_pin_ctx_t * pins = (struct hx_pin_ctx_t *)pin_ptr;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);

  //TX enable Z
  GPIO_InitStruct.Pin   = FB1_Z_TXEN_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FB1_Z_TXEN_PORT, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(FB1_Z_TXEN_PORT, FB1_Z_TXEN_PIN);

  // output Z
  GPIO_InitStruct.Pin   = FB1_Z_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FB1_Z_PORT, &GPIO_InitStruct);
}

//TODO: plausibility, saturation, channel/gain config, 2 chips

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct hx_ctx_t *ctx      = (struct hx_ctx_t *)ctx_ptr;
  struct hx_pin_ctx_t *pins = (struct hx_pin_ctx_t *)pin_ptr;
  uint32_t value0           = 0;
  uint32_t value1           = 0;

  int sleep = CLAMP(PIN(sleep), 1, 50);

  if(PIN(timer) > PIN(time)) {
    if((PIN(data_inv) > 0.0) ? (LL_GPIO_IsInputPinSet(FB1_A_PORT, FB1_A_PIN)) : (!LL_GPIO_IsInputPinSet(FB1_A_PORT, FB1_A_PIN))) {  //data line low = conversion ready
      for(int i = 0; i < 24; i++) {
        if(PIN(clk_inv) > 0.0) {
          nopsleep(sleep);
          LL_GPIO_ResetOutputPin(FB1_Z_PORT, FB1_Z_PIN);
          nopsleep(sleep);
          LL_GPIO_SetOutputPin(FB1_Z_PORT, FB1_Z_PIN);
        } else {
          nopsleep(sleep);
          LL_GPIO_SetOutputPin(FB1_Z_PORT, FB1_Z_PIN);
          nopsleep(sleep);
          LL_GPIO_ResetOutputPin(FB1_Z_PORT, FB1_Z_PIN);
        }
        if((PIN(data_inv) > 0.0) ? (LL_GPIO_IsInputPinSet(FB1_A_PORT, FB1_A_PIN)) : (!LL_GPIO_IsInputPinSet(FB1_A_PORT, FB1_A_PIN))) {  //dout = 1
          value0++;
        }
        value0 = value0 << 1;
        if((PIN(data_inv) > 0.0) ? (LL_GPIO_IsInputPinSet(FB1_B_PORT, FB1_B_PIN)) : (!LL_GPIO_IsInputPinSet(FB1_B_PORT, FB1_B_PIN))) {  //dout = 1
          value1++;
        }
        value1 = value1 << 1;
      }
      //clock additional config bits
      if(PIN(clk_inv) > 0.0) {
        nopsleep(sleep);
        LL_GPIO_ResetOutputPin(FB1_Z_PORT, FB1_Z_PIN);
        nopsleep(sleep);
        LL_GPIO_SetOutputPin(FB1_Z_PORT, FB1_Z_PIN);
      } else {
        nopsleep(sleep);
        LL_GPIO_SetOutputPin(FB1_Z_PORT, FB1_Z_PIN);
        nopsleep(sleep);
        LL_GPIO_ResetOutputPin(FB1_Z_PORT, FB1_Z_PIN);
      }

      if(value0 & 0x800000) {  //if 24th bit is set, pad others, to get 2 complement number
        value0 |= 0xff000000;
      }
      int32_t sint = *((int32_t *)(&value0));
      PIN(out0)    = ((float)sint / (float)0x7fffff) * PIN(gain) + PIN(offset);

      if(value1 & 0x800000) {  //if 24th bit is set, pad others, to get 2 complement number
        value1 |= 0xff000000;
      }
      sint       = *((int32_t *)(&value1));
      PIN(out1)  = ((float)sint / (float)0x7fffff) * PIN(gain) + PIN(offset);
      PIN(timer) = 0.0;
    }
  }
  PIN(timer) += period;
}

hal_comp_t hx_comp_struct = {
    .name      = "hx",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct hx_ctx_t),
    .pin_count = sizeof(struct hx_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
