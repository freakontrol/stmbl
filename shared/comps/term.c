#include "term_comp.h"
#include "commands.h"
#include "hal.h"
#include "defines.h"
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "angle.h"

// Terminal and scope on the USB port. The frames are documented in
// docs/stmbldoc/content/docs/getting_started/servoterm_protocol.md; in short, a binary frame starts with a
// marker in 0xfb..0xff and every payload byte of a new frame is 1..250, so
// markers stay unique and text never contains one:
//   0xff  legacy sample, 8 bytes 1..253 (fmt 0, the default, unchanged)
//   0xfe  reset
//   0xfd  float sample: seq, 8 floats (fmt 1)
//   0xfc  header: proto, kind (0 live, 1 capture), period, channels, count, pre
//   0xfb  capture sample: index, 8 floats
// Numbers are base-250 digits, each stored + 1: a float is its 32 bits in 5
// digits (250^5 > 2^32), seq 1 digit, index 2.

#define TERM_NUM_WAVES 8
// rt fills this every send_step ticks and nrt drains it: 64 covers 12.8 ms
// of main loop at send_step 1. The f3 Makefile keeps 8, its HAL_MAX_CTX is
// 1024 bytes for all comps.
#ifndef TERM_BUF_SIZE
#define TERM_BUF_SIZE 64
#endif
// capture buffer, outside the HAL ctx: 1024 x 8 floats = 32 KB on the F4.
// None on the f3 (small TERM_BUF_SIZE), which has no RAM to spare.
#ifndef TERM_CAP_SIZE
#if TERM_BUF_SIZE >= 64
#define TERM_CAP_SIZE 1024
#else
#define TERM_CAP_SIZE 0
#endif
#endif
#define TERM_LINE 128

#define TERM_PROTO 1.0
#define MARK_RESET 0xfe
#define MARK_FLOAT 0xfd
#define MARK_HEADER 0xfc
#define MARK_CAP 0xfb

/**
* ## Brief
* `term` is the serial terminal of a board. It runs the command line (every received line goes to `hal_parse`, so HAL commands and pin settings can be typed) and streams the 8 `wave` pins as scope samples to the host (the Servoterm oscilloscope). On the F4 it can also take a triggered capture of 1024 samples and upload it. It is loaded by default as `term0` on both the F4 and the F3. On the F4 it talks over USB. On the F3 (HV board) there is no USB: its terminal bytes travel one per packet over the link to the F4 (`ls` on the F3, `hv` on the F4), and main.c sets `term0.send_step = 0` there, so no wave data is sent. Typical use on the F4 is a config line like `term0.wave0 = pid0.pos_error` with `term0.gain0` to scale it (see conf/template/obs_check.txt).
*
* ## Component Explanation
*
* 1. **Sampling** (rt):
* - Every `send_step` rt ticks the 8 `wave` pins are stored into a ring buffer, together with a sequence number `seq` that counts every live sample taken, dropped ones included. `TERM_BUF_SIZE` is 64 entries on the F4, which covers 12.8 ms of main loop at `send_step` 1, and 8 on the F3, set in its Makefile, because all F3 components share 1024 bytes of HAL memory. One entry is kept free, so 63 (or 7) samples fit.
* - When the buffer is full, the new sample is dropped instead of overwriting unsent ones and counted in `lost`; the host sees the gap in `seq`.
*
* 2. **Frames on the wire** (protocol v1, host side in qtservoterm term_v1):
* - The frames and their encoding are described on the [Servoterm protocol](/docs/getting_started/servoterm_protocol.md) page: 0xff legacy 8 bit sample (`fmt` 0), 0xfe reset, 0xfd float sample (`fmt` 1), 0xfc header, 0xfb capture sample.
* - The 0xff byte of a wave is `CLAMP((wave + offset) * gain + 128, 1, 253)`. A wave value of 0 is the middle of the scope, and with the default `gain` of 10 the visible range is about -12.7..+12.5. The float frames carry `(wave + offset) * gain`, not clamped.
* - In `fmt` 1 the live header is sent on connect and whenever `fmt` or the live sample period (rt period * `send_step`) changes. In `fmt` 0 no header is sent, because an old host would print it as text.
*
* 3. **Sending** (nrt):
* - All buffered samples are sent in the current `fmt`. On the F4 a frame goes into the USB transmit ring whole or not at all: if there is no room for it, it is dropped, so the host never sees a torn frame. Such drops are not counted in `lost`, only `seq` shows them.
* - Text output (command replies, `printf`) leaves 1 KB of the 4 KB USB transmit ring to the scope, so a long reply does not stall the samples.
* - Nothing is sampled while `send_step` is 0. While no terminal is connected, samples are taken from the buffer and dropped.
*
* 4. **Capture** (F4 only, needs `fmt` 1):
* - The capture buffer holds `cap_len` samples of all 8 waves (1024 x 8 floats = 32 KB, in the F4 CCM RAM outside the HAL memory). The F3 has no capture buffer: `cap_len` is 0 and the cap pins do nothing.
* - Writing `cap_arm` = 1 starts a capture (`cap_state` 1). With `fmt` 0 nrt prints "capture needs term0.fmt = 1 (float samples)" and clears `cap_arm`. At arming the sample period (rt period * `cap_step`) and the pre-trigger count (`cap_pre` * `cap_len`, `cap_pre` clamped to 0..0.95) are fixed.
* - Every `cap_step` rt ticks the 8 waves go into the capture ring. Once the pre-trigger samples are in, `cap_state` goes to 2 and the trigger is checked on wave `cap_ch` (clamped to 0..7): rising (`cap_edge` > 0) fires when the previous sample was below `cap_level` and this one is at or above it, falling (`cap_edge` < 0) the other way round, `cap_edge` 0 fires at once.
* - The trigger sample is the first one after the trigger. `cap_state` 3 takes the rest of the buffer, then `cap_state` 4 hands the buffer to nrt.
* - Upload (nrt): a capture header (kind 1, capture period, count, pre-trigger samples), then the samples oldest first as 0xfb frames, as many per pass as the transmit ring takes, then the live header again; then `cap_arm` and `cap_state` go back to 0. `offset` and `gain` are applied at upload time. Live samples taken during the upload are dropped. A finished capture waits while no host is connected, and a disconnect during the upload starts it over: the next host gets it whole.
* - Abort: writing `cap_arm` = 0, or setting `fmt` back to 0 (an old host cannot parse capture frames), sets `cap_arm` and `cap_state` to 0 in any capture state.
*
* 5. **Receiving** (nrt):
* - `con` is always 1 on the F3.
* - Each complete received line of up to 127 bytes is passed to `hal_parse`. A longer line is dropped whole and answered "line too long (max 127 bytes)".
* - A line `#<n> command` runs `command` and is answered `#<n> done` afterwards, so the host can pair its commands with their replies.
*
* {{% hint warning %}}
* The capture pins are evaluated on the F4 only, and a capture needs `fmt` 1.
* {{% /hint %}}
*/

HAL_COMP(term);

HAL_PINA(wave, 8);    // *input*, Signals shown on the scope, 8 channels
HAL_PINA(offset, 8);  // *parameter*, Offset added to each wave before scaling, default 0
HAL_PINA(gain, 8);    // *parameter*, Scale of each wave, fmt 0: scope steps per unit of the 1..253 range, fmt 1: factor on the float, default 10
HAL_PIN(send_step);   // *parameter*, Send a live sample every send_step rt ticks, 0 = off, default 50
HAL_PIN(con);         // *output*, 1 while a terminal is connected, always 1 on the F3
HAL_PIN(fmt);         // *parameter*, Sample format, 0 = 8 bit samples (old Servoterm), 1 = float samples with headers (protocol v1), default 0
HAL_PIN(lost);        // *output*, Count of live samples dropped because the ring buffer was full
HAL_PIN(cap_arm);     // *parameter*, Write 1 to arm a capture, back to 0 when it is uploaded, 0 aborts, default 0
HAL_PIN(cap_ch);      // *parameter*, Capture trigger channel 0..7, default 0
HAL_PIN(cap_level);   // *parameter*, Capture trigger level, in the wave's own units (before offset and gain), default 0
HAL_PIN(cap_edge);    // *parameter*, Capture trigger edge, 1 rising, -1 falling, 0 at once, default 1
HAL_PIN(cap_pre);     // *parameter*, Fraction of the capture before the trigger, 0..0.95, default 0.25
HAL_PIN(cap_step);    // *parameter*, rt ticks per capture sample, min 1, default 1
HAL_PIN(cap_state);   // *output*, Capture state, 0 idle, 1 pre-trigger, 2 waiting for the trigger, 3 post-trigger, 4 upload
HAL_PIN(cap_len);     // *output*, Capture buffer length in samples, 1024 on the F4, 0 on the F3

struct term_ctx_t {
  float wave_buf[TERM_BUF_SIZE][TERM_NUM_WAVES];
  uint32_t seq_buf[TERM_BUF_SIZE];
  uint32_t seq;  // live samples taken, dropped ones included
  uint32_t send_counter;
  uint32_t write_pos;
  uint32_t read_pos;
  float period;        // rt period seen by rt_func
  float hdr_period;    // live period last announced
  float hdr_fmt;       // fmt last announced
  uint32_t hdr_con;    // con when the header was last sent
  uint32_t skip_line;  // dropping the tail of an overlong line
  // capture: rt fills, nrt uploads
  uint32_t cap_w;      // next write index (ring)
  uint32_t cap_n;      // samples written since arming
  uint32_t cap_post;   // samples still to take after the trigger
  uint32_t cap_count;  // samples per capture, fixed at arming
  uint32_t cap_pre_n;  // pre-trigger samples, fixed at arming
  uint32_t cap_start;  // ring index of the first uploaded sample
  uint32_t cap_up;     // samples uploaded so far
  uint32_t cap_tick;
  float cap_last;      // trigger channel, last sample
  float cap_period;
};

#if TERM_CAP_SIZE > 0
static float cap_buf[TERM_CAP_SIZE][TERM_NUM_WAVES] __attribute__((section(".ccm")));  // CPU only
#endif

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct term_ctx_t *ctx      = (struct term_ctx_t *)ctx_ptr;
  struct term_pin_ctx_t *pins = (struct term_pin_ctx_t *)pin_ptr;

  PIN(send_step) = 50;
  for(int i = 0; i < TERM_NUM_WAVES; i++) {
    PINA(gain, i) = 10;
  }
  PIN(cap_edge)  = 1.0;
  PIN(cap_pre)   = 0.25;
  PIN(cap_step)  = 1.0;
  PIN(cap_len)   = TERM_CAP_SIZE;
  ctx->hdr_fmt   = -1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct term_ctx_t *ctx      = (struct term_ctx_t *)ctx_ptr;
  struct term_pin_ctx_t *pins = (struct term_pin_ctx_t *)pin_ptr;

  ctx->period = period;

  if(PIN(send_step) >= 1.0 && ++ctx->send_counter >= (uint32_t)PIN(send_step)) {
    ctx->send_counter = 0;
    ctx->seq++;
    // full: drop this sample instead of lapping the reader; seq shows the gap
    uint32_t next = (ctx->write_pos + 1) % TERM_BUF_SIZE;
    if(next != ctx->read_pos) {
      for(int i = 0; i < TERM_NUM_WAVES; i++) {
        ctx->wave_buf[ctx->write_pos][i] = PINA(wave, i);
      }
      ctx->seq_buf[ctx->write_pos] = ctx->seq;
      ctx->write_pos               = next;
    } else {
      PIN(lost) += 1.0;
    }
  }

#if TERM_CAP_SIZE > 0
  uint32_t state = (uint32_t)PIN(cap_state);
  if(state == 0) {
    if(PIN(cap_arm) > 0.0 && PIN(fmt) > 0.0) {
      ctx->cap_count  = TERM_CAP_SIZE;
      ctx->cap_pre_n  = (uint32_t)(CLAMP(PIN(cap_pre), 0.0, 0.95) * TERM_CAP_SIZE);
      ctx->cap_w      = 0;
      ctx->cap_n      = 0;
      ctx->cap_tick   = 0;
      ctx->cap_period = period * MAX(PIN(cap_step), 1.0);
      PIN(cap_state)  = 1.0;
    }
    return;
  }
  if(state > 3) {  // uploading
    return;
  }
  if(++ctx->cap_tick < (uint32_t)MAX(PIN(cap_step), 1.0)) {
    return;
  }
  ctx->cap_tick = 0;

  uint32_t ch = (uint32_t)CLAMP(PIN(cap_ch), 0.0, TERM_NUM_WAVES - 1);
  float v     = PINA(wave, ch);
  for(int i = 0; i < TERM_NUM_WAVES; i++) {
    cap_buf[ctx->cap_w][i] = PINA(wave, i);
  }
  ctx->cap_w = (ctx->cap_w + 1) % TERM_CAP_SIZE;
  ctx->cap_n++;

  if(state == 1 && ctx->cap_n >= ctx->cap_pre_n) {
    state          = 2;
    PIN(cap_state) = 2.0;
  }
  if(state == 2) {
    float lvl = PIN(cap_level);
    int trig  = 0;
    if(PIN(cap_edge) > 0.0) {
      trig = ctx->cap_n > 1 && ctx->cap_last < lvl && v >= lvl;
    } else if(PIN(cap_edge) < 0.0) {
      trig = ctx->cap_n > 1 && ctx->cap_last > lvl && v <= lvl;
    } else {
      trig = 1;
    }
    if(trig) {
      // this sample is the first after the trigger; pre samples sit before it
      ctx->cap_post  = ctx->cap_count - ctx->cap_pre_n - 1;
      ctx->cap_start = (ctx->cap_w + TERM_CAP_SIZE - 1 - ctx->cap_pre_n) % TERM_CAP_SIZE;
      state          = 3;
      PIN(cap_state) = 3.0;
      if(ctx->cap_post == 0) {
        ctx->cap_up    = 0;
        PIN(cap_state) = 4.0;
      }
    }
  } else if(state == 3) {
    if(--ctx->cap_post == 0) {
      ctx->cap_up    = 0;
      PIN(cap_state) = 4.0;  // nrt uploads, rt keeps its hands off cap_buf
    }
  }
  ctx->cap_last = v;
#endif
}

// n as base-250 digits, each + 1, most significant first
static void put_d250(unsigned char *p, uint32_t n, int digits) {
  for(int i = digits - 1; i >= 0; i--) {
    p[i] = (unsigned char)(n % 250 + 1);
    n /= 250;
  }
}

static void put_f32(unsigned char *p, float f) {
  union {
    float f;
    uint32_t u;
  } x;
  x.f = f;
  put_d250(p, x.u, 5);
}

static int send_header(float kind, float period, float count, float pre) {
  unsigned char buf[2 + 6 * 5];
  buf[0] = MARK_RESET;
  buf[1] = MARK_HEADER;
  put_f32(buf + 2, TERM_PROTO);
  put_f32(buf + 7, kind);
  put_f32(buf + 12, period);
  put_f32(buf + 17, TERM_NUM_WAVES);
  put_f32(buf + 22, count);
  put_f32(buf + 27, pre);
  return cdc_tx(buf, sizeof(buf));
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct term_ctx_t *ctx      = (struct term_ctx_t *)ctx_ptr;
  struct term_pin_ctx_t *pins = (struct term_pin_ctx_t *)pin_ptr;

  uint32_t con = cdc_is_connected() ? 1 : 0;
  PIN(con)     = con;

  // announce the live format on connect and on every change of it
  float live_period = ctx->period * MAX(PIN(send_step), 1.0);
  float fmt         = PIN(fmt) > 0.0 ? 1.0 : 0.0;
  if(con && PIN(cap_state) < 4.0 && (!ctx->hdr_con || fmt != ctx->hdr_fmt || live_period != ctx->hdr_period)) {
    if(fmt == 0.0) {
      // an old host may be listening: it would print a header as text.
      // Headers start when the host asks for fmt 1.
      ctx->hdr_con    = 1;
      ctx->hdr_fmt    = 0.0;
      ctx->hdr_period = live_period;
    } else if(send_header(0.0, live_period, 0.0, 0.0)) {
      ctx->hdr_con    = 1;
      ctx->hdr_fmt    = fmt;
      ctx->hdr_period = live_period;
    }
  }
  if(!con) {
    ctx->hdr_con = 0;
  }

#if TERM_CAP_SIZE > 0
  if(PIN(cap_arm) > 0.0 && fmt == 0.0 && PIN(cap_state) == 0.0) {
    printf("capture needs term0.fmt = 1 (float samples)\n");
    PIN(cap_arm) = 0.0;
  }
  // abort: cap_arm written 0, or the host went back to the legacy format,
  // which cannot parse capture frames
  if(PIN(cap_state) > 0.0 && (PIN(cap_arm) <= 0.0 || fmt == 0.0)) {
    PIN(cap_arm)   = 0.0;
    PIN(cap_state) = 0.0;
  }
  // no host: keep a finished capture and send it whole to the next one
  if(!con) {
    ctx->cap_up = 0;
  }
  if(PIN(cap_state) >= 4.0) {
    // upload: the header, the samples as room allows, then the live header
    if(ctx->cap_up == 0 && con) {
      if(!send_header(1.0, ctx->cap_period, ctx->cap_count, ctx->cap_pre_n)) {
        return;
      }
      ctx->cap_up = 1;
    }
    while(con && ctx->cap_up >= 1 && ctx->cap_up <= ctx->cap_count) {
      uint32_t i = ctx->cap_up - 1;
      uint32_t r = (ctx->cap_start + i) % TERM_CAP_SIZE;
      unsigned char buf[1 + 2 + TERM_NUM_WAVES * 5];
      buf[0] = MARK_CAP;
      put_d250(buf + 1, i, 2);
      for(int c = 0; c < TERM_NUM_WAVES; c++) {
        put_f32(buf + 3 + c * 5, (cap_buf[r][c] + PINA(offset, c)) * PINA(gain, c));
      }
      if(!cdc_tx(buf, sizeof(buf))) {
        break;  // ring full: carry on next pass
      }
      ctx->cap_up++;
    }
    if(con && ctx->cap_up > ctx->cap_count) {
      if(send_header(0.0, live_period, 0.0, 0.0)) {
        PIN(cap_arm)   = 0.0;
        PIN(cap_state) = 0.0;
      }
    }
    // live samples wait in the ring meanwhile; drop what is already stale
    ctx->read_pos = ctx->write_pos;
  }
#endif

  unsigned int wp = ctx->write_pos;
  while(ctx->read_pos != wp) {
    float *w = ctx->wave_buf[ctx->read_pos];
    if(fmt == 0.0) {
      unsigned char buf[TERM_NUM_WAVES + 1];
      buf[0] = 255;
      for(int i = 0; i < TERM_NUM_WAVES; i++) {
        int tmp    = (int)((w[i] + PINA(offset, i)) * PINA(gain, i) + 128);
        buf[i + 1] = CLAMP(tmp, 1, 253);  // 0xfe is the reset marker
      }
      cdc_tx(buf, sizeof(buf));
    } else {
      unsigned char buf[2 + TERM_NUM_WAVES * 5];
      buf[0] = MARK_FLOAT;
      put_d250(buf + 1, ctx->seq_buf[ctx->read_pos] % 250, 1);
      for(int i = 0; i < TERM_NUM_WAVES; i++) {
        put_f32(buf + 2 + i * 5, (w[i] + PINA(offset, i)) * PINA(gain, i));
      }
      cdc_tx(buf, sizeof(buf));
    }
    ctx->read_pos = (ctx->read_pos + 1) % TERM_BUF_SIZE;
  }

  if(con) {
    char rx_buf[TERM_LINE];
    int ret = cdc_getline(rx_buf, sizeof(rx_buf));
    if(ret < 0) {
      // overlong: the head is gone, drop the tail too
      if(!ctx->skip_line) {
        printf("line too long (max %d bytes)\n", TERM_LINE - 1);
      }
      ctx->skip_line = 1;
    } else if(ret > 0) {
      if(ctx->skip_line) {
        ctx->skip_line = 0;
      } else if(rx_buf[0] == '#' && rx_buf[1] >= '0' && rx_buf[1] <= '9') {
        // "#<n> command": run it, then "#<n> done" so the host can pair them
        char *cmd = rx_buf + 1;
        while(*cmd >= '0' && *cmd <= '9') {
          cmd++;
        }
        int n = cmd - rx_buf - 1;
        hal_parse(cmd);
        printf("#%.*s done\n", n, rx_buf + 1);
      } else {
        hal_parse(rx_buf);
      }
    }
  }
}

hal_comp_t term_comp_struct = {
    .name      = "term",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct term_ctx_t),
    .pin_count = sizeof(struct term_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
