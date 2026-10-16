#include "adc_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "setup.h"
#include "usbd_cdc_if.h"

#define INPUT_REF (OP_REF * OP_R_OUT_LOW / (OP_R_OUT_HIGH + OP_R_OUT_LOW))
#define INPUT_GAIN (OP_R_FEEDBACK / OP_R_INPUT * OP_R_OUT_LOW / (OP_R_OUT_HIGH + OP_R_OUT_LOW))
#define V_DIFF(ADC, OVER) ((((float)(ADC)) / (float)(OVER) / ADC_RES * ADC_REF - INPUT_REF) / INPUT_GAIN)

#define TERM_NUM_WAVES 8

extern volatile uint32_t ADC_DMA_Buffer0[ADC_SAMPLES_IN_RT];  //240
extern volatile uint32_t ADC_DMA_Buffer1[ADC_SAMPLES_IN_RT];

/**
* ## Brief
* `adc` (F4 logic board) converts the raw sin/cos samples of the two feedback connectors, which the ADCs capture by DMA, into scaled sin/cos values for the feedback components, and can stream the raw samples to Servoterm as a scope. The feedback templates wire it, e.g. conf/template/res_fb0.txt (`res0.sin = adc0.sin0`, `res0.cos = adc0.cos0`, `res0.quad = adc0.quad`, `adc0.res_mode = res0.res_mode`) and conf/template/enc_fb0.txt (`enc_fb0.sin = adc0.sin0l`, `enc_fb0.cos = adc0.cos0l`, `enc_fb0.amp = adc0.amp0`).
*
* ## Component Explanation
*
* 1. **Sampling**:
* - ADC1 (sin) and ADC2 (cos) sample in dual mode into a DMA double buffer, 240 samples per rt period (1.2 MHz at 5 kHz rt). Each 32 bit sample holds sin in the lower and cos in the upper 16 bits. `rt` reads the half the DMA is not writing, i.e. the previous rt period.
* - The transfer-complete interrupt of this DMA (DMA2 stream 0) is what runs the hal rt on the F4. An ADC1/ADC2 overrun would stop the DMA, and with it the rt, for good, so the overrun interrupt (src/main.c `ADC_IRQHandler`) stops the hal and sets its state to `MISC_ERROR`.
* - The 240 samples form 24 groups of 10: 9 samples of fb0 followed by 1 sample of fb1.
* - Each raw value is converted into the differential input voltage (V) of the analog front end:
* ```c
* V_DIFF(adc, over) = (adc / over / ADC_RES * ADC_REF - INPUT_REF) / INPUT_GAIN
* ```
*
* 2. **Scaling and resolver demodulation (rt)**:
* - Per group the samples of each channel are summed and converted, then `sin_gain`/`cos_gain` and `sin_offset`/`cos_offset` are applied: `si = flip * sin_gain * V + sin_offset`.
* - `res_mode` = n > 0 flips the sign of fb0 groups in blocks of n (n groups +1, n groups -1, ...), which demodulates a resolver signal synchronous to its excitation (set by the `res` component).
*
* 3. **Outputs (rt)**:
* - `sin0`/`cos0` and `sin1`/`cos1` are the averages over all 24 groups.
* - `sin0l`/`cos0l` and `sin1l`/`cos1l` are the last group only, i.e. the newest value, used for sin/cos encoders.
* - `quad` is used by the encoder components for interpolation.
* - `amp0`/`amp1` are low pass filtered (0.9/0.1 per rt period) amplitudes `sqrt(s^2 + c^2)` of the last group of fb0/fb1 (the `sin0l`/`cos0l` and `sin1l`/`cos1l` values). uvw and encoder templates use them to detect a connected feedback.
*
* 4. **Scope stream to Servoterm (nrt)**:
* - `rt` copies one raw buffer; `nrt` waits `send_step` nrt calls, converts it and sends all 240 samples over USB as 8 waves: 0/1 = fb0 sin/cos (flipped by `res_mode`, 0 at fb1 positions), 2/3 = fb1 sin/cos (0 at fb0 positions), 4 = the flip sign, 5..7 unused.
* - Each wave byte is `(value + offset[i]) * gain[i] + 128`, clamped to 1..254; a frame starts with 255 and the buffer ends with 0xFE, which triggers Servoterm.
*
* {{% hint info %}}
* The scope stream blocks `nrt` for 240 USB writes and uses the same USB wave format as the `term` scope, so with both active their frames interleave in Servoterm. Waves 2/3 show fb1 only at every 10th position, so they look like spikes.
* {{% /hint %}}
*/

HAL_COMP(adc);

HAL_PIN(sin0);       // *output*, fb0 sin, average over the rt period (V)
HAL_PIN(cos0);       // *output*, fb0 cos, average over the rt period (V)
HAL_PIN(sin0l);      // *output*, fb0 sin, last group only (V)
HAL_PIN(cos0l);      // *output*, fb0 cos, last group only (V)
HAL_PIN(quad);       // *output*, quadrant (1..4) of the last fb0 sin/cos group
HAL_PIN(amp0);       // *output*, filtered fb0 sin/cos amplitude of the last group (V), after gain/offset

HAL_PIN(sin1);       // *output*, fb1 sin, average over the rt period (V)
HAL_PIN(cos1);       // *output*, fb1 cos, average over the rt period (V)
HAL_PIN(sin1l);      // *output*, fb1 sin, last group only (V)
HAL_PIN(cos1l);      // *output*, fb1 cos, last group only (V)
HAL_PIN(amp1);       // *output*, filtered fb1 sin/cos amplitude of the last group (V), after gain/offset

HAL_PIN(res_mode);   // *input*, resolver polarity flip every n groups (fb0 only), 0 = off, usually res0.res_mode

HAL_PIN(sin_gain);   // *parameter*, sin gain for fb0 and fb1, default 1
HAL_PIN(cos_gain);   // *parameter*, cos gain for fb0 and fb1, default 1

HAL_PIN(sin_offset); // *parameter*, sin offset (V) for fb0 and fb1, default 0
HAL_PIN(cos_offset); // *parameter*, cos offset (V) for fb0 and fb1, default 0

HAL_PIN(send_step);  // *parameter*, send one scope buffer every n nrt calls, 0 = scope off

HAL_PINA(offset, 8); // *parameter*, scope wave offsets, one per wave
HAL_PINA(gain, 8);   // *parameter*, scope wave gains, default 150 (0..3), 80 (4), 0 (5..7)

struct adc_ctx_t {
  volatile float txbuf[8][ADC_SAMPLES_IN_RT];
  volatile uint32_t txbuf_raw[ADC_SAMPLES_IN_RT];
  uint32_t txpos;
  uint32_t send_counter;   //send_step counter
  volatile uint32_t send;  //send buffer state 0=filling, 1=sending
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct adc_ctx_t *ctx      = (struct adc_ctx_t *)ctx_ptr;
  struct adc_pin_ctx_t *pins = (struct adc_pin_ctx_t *)pin_ptr;
  PINA(gain, 0)              = 150;
  PINA(gain, 1)              = 150;
  PINA(gain, 2)              = 150;
  PINA(gain, 3)              = 150;
  PINA(gain, 4)              = 80;
  PIN(sin_gain)              = 1.0;
  PIN(cos_gain)              = 1.0;
  ctx->txpos                 = 0;
  ctx->send_counter          = 0;
  ctx->send                  = 0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct adc_ctx_t *ctx      = (struct adc_ctx_t *)ctx_ptr;
  struct adc_pin_ctx_t *pins = (struct adc_pin_ctx_t *)pin_ptr;

  //scaled values for each group
  float si0[ADC_GROUPS];
  float co0[ADC_GROUPS];
  //integral per group
  uint32_t sii0;
  uint32_t coi0;
  //scaled, all groups
  float sin0all = 0.0;
  float cos0all = 0.0;
#ifdef FB1
  float co1[ADC_GROUPS];
  float si1[ADC_GROUPS];
  uint32_t sii1;
  uint32_t coi1;
  //scaled, all groups
  float sin1all = 0.0;
  float cos1all = 0.0;
#endif

  float s_o = PIN(sin_offset);
  float c_o = PIN(cos_offset);
  float s_g = PIN(sin_gain);
  float c_g = PIN(cos_gain);

  volatile uint32_t *ADC_DMA_Buffer;

  if(LL_DMA_GetCurrentTargetMem(DMA2, LL_DMA_STREAM_0) == LL_DMA_CURRENTTARGETMEM1) {
    ADC_DMA_Buffer = ADC_DMA_Buffer0;
  } else {
    ADC_DMA_Buffer = ADC_DMA_Buffer1;
  }
  int flip;
  int n = PIN(res_mode);


  for(int i = 0; i < ADC_GROUPS; i++) {  //each adc sampling group
    if(n > 0 && i % (2 * n) >= n) {
      flip = -1;
    } else {
      flip = 1;
    }
    sii0 = 0;
    coi0 = 0;
    for(int j = 0; j < ADC_OVER_FB0; j++) {  //each adc sample of fb0
      sii0 += ADC_DMA_Buffer[i * (ADC_OVER_FB0 + ADC_OVER_FB1) + j] & 0x0000ffff;
      coi0 += ADC_DMA_Buffer[i * (ADC_OVER_FB0 + ADC_OVER_FB1) + j] >> 16;
    }
    si0[i] = flip * s_g * V_DIFF(sii0, ADC_OVER_FB0) + s_o;
    co0[i] = flip * c_g * V_DIFF(coi0, ADC_OVER_FB0) + c_o;
    sin0all += si0[i];
    cos0all += co0[i];
#ifdef FB1
    sii1 = 0;
    coi1 = 0;
    for(int j = ADC_OVER_FB0; j < ADC_OVER_FB0 + ADC_OVER_FB1; j++) {  //each adc sample of fb1
      sii1 += ADC_DMA_Buffer[i * (ADC_OVER_FB0 + ADC_OVER_FB1) + j] & 0x0000ffff;
      coi1 += ADC_DMA_Buffer[i * (ADC_OVER_FB0 + ADC_OVER_FB1) + j] >> 16;
    }
    si1[i] = s_g * V_DIFF(sii1, ADC_OVER_FB1) + s_o;
    co1[i] = c_g * V_DIFF(coi1, ADC_OVER_FB1) + c_o;
    sin1all += si1[i];
    cos1all += co1[i];
#endif
  }
  if(ctx->send == 0) {
    memcpy((void *)(ctx->txbuf_raw), (void *)ADC_DMA_Buffer, ADC_SAMPLES_IN_RT * 4);
    ctx->send = 1;
  }

  PIN(sin0l) = si0[ADC_GROUPS - 1];
  PIN(cos0l) = co0[ADC_GROUPS - 1];
  PIN(sin0)  = sin0all / (float)ADC_GROUPS;
  PIN(cos0)  = cos0all / (float)ADC_GROUPS;
  // amplitude of the last group, after gain and offset like sin0/cos0, so
  // sin_gain/cos_gain also scale what enc_fb and uvw check against
  float s    = si0[ADC_GROUPS - 1];
  float c    = co0[ADC_GROUPS - 1];
  PIN(amp0)  = PIN(amp0) * 0.9 + sqrtf(s * s + c * c) * 0.1;
#ifdef FB1
  s          = si1[ADC_GROUPS - 1];
  c          = co1[ADC_GROUPS - 1];
  PIN(sin1l) = si1[ADC_GROUPS - 1];
  PIN(cos1l) = co1[ADC_GROUPS - 1];
  PIN(sin1)  = sin1all / (float)ADC_GROUPS;
  PIN(cos1)  = cos1all / (float)ADC_GROUPS;
  PIN(amp1)  = PIN(amp1) * 0.9 + sqrtf(s * s + c * c) * 0.1;
#endif

  // if(PIN(res_en) > 0.0) {
  //   s = (si0[3] - si0[2] + si0[1] - si0[0]) / 4.0;
  //   c = (co0[3] - co0[2] + co0[1] - co0[0]) / 4.0;
  // } else {
  //   s = (si0[3] + si0[2] + si0[1] + si0[0]) / 4.0;
  //   c = (co0[3] + co0[2] + co0[1] + co0[0]) / 4.0;
  // }

  //calculate quadrant for sin/cos interpolation
  if(si0[ADC_GROUPS - 1] >= 0) {
    if(co0[ADC_GROUPS - 1] > 0)
      PIN(quad) = 1;
    else
      PIN(quad) = 2;
  } else {
    if(co0[ADC_GROUPS - 1] > 0)
      PIN(quad) = 4;
    else
      PIN(quad) = 3;
  }
}


static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct adc_ctx_t *ctx      = (struct adc_ctx_t *)ctx_ptr;
  struct adc_pin_ctx_t *pins = (struct adc_pin_ctx_t *)pin_ptr;

  int tmp = 0;
  uint8_t buf[TERM_NUM_WAVES + 3];

  int n = PIN(res_mode);  //n gruppen pro halbwelle 1-12
  int flip;
  if(ctx->send == 1 && ctx->send_counter++ >= PIN(send_step) - 1 && PIN(send_step) > 0) {
    ctx->send_counter = 0;
    for(int i = 0; i < ADC_GROUPS; i++) {  //each adc sampling group
      if(n > 0 && i % (2 * n) >= n) {
        flip = -1;
      } else {
        flip = 1;
      }
      for(int j = 0; j < ADC_OVER_FB0; j++) {  //each adc sample of fb0
        ctx->txbuf[0][ctx->txpos] = flip * V_DIFF(ctx->txbuf_raw[i * (ADC_OVER_FB0 + ADC_OVER_FB1) + j] & 0x0000ffff, 1);
        ctx->txbuf[1][ctx->txpos] = flip * V_DIFF(ctx->txbuf_raw[i * (ADC_OVER_FB0 + ADC_OVER_FB1) + j] >> 16, 1);
        ctx->txbuf[2][ctx->txpos] = 0;
        ctx->txbuf[3][ctx->txpos] = 0;
        ctx->txbuf[4][ctx->txpos] = flip;
        ctx->txpos++;
      }
#ifdef FB1
      for(int j = ADC_OVER_FB0; j < ADC_OVER_FB0 + ADC_OVER_FB1; j++) {  //each adc sample of fb1
        ctx->txbuf[0][ctx->txpos] = 0;
        ctx->txbuf[1][ctx->txpos] = 0;
        ctx->txbuf[2][ctx->txpos] = V_DIFF(ctx->txbuf_raw[i * (ADC_OVER_FB0 + ADC_OVER_FB1) + j] & 0x0000ffff, 1);
        ctx->txbuf[3][ctx->txpos] = V_DIFF(ctx->txbuf_raw[i * (ADC_OVER_FB0 + ADC_OVER_FB1) + j] >> 16, 1);
        ctx->txbuf[4][ctx->txpos] = flip;
        ctx->txpos++;
      }
#endif
    }

    ctx->txpos = 0;
    buf[0]     = 255;                             //start of waves
    for(int k = 0; k < ADC_SAMPLES_IN_RT; k++) {  //each sample
      for(int i = 0; i < TERM_NUM_WAVES; i++) {   //each wave
        tmp        = (ctx->txbuf[i][k] + PINA(offset, i)) * PINA(gain, i) + 128;
        buf[i + 1] = CLAMP(tmp, 1, 254);
      }
      cdc_tx(buf, 9);
    }
    buf[0] = 0xfe;  //trigger servoterm
    cdc_tx(buf, 1);
    ctx->send = 0;
  }
}

hal_comp_t adc_comp_struct = {
    .name      = "adc",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct adc_ctx_t),
    .pin_count = sizeof(struct adc_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
