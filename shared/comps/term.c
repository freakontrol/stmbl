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

HAL_COMP(term);

HAL_PINA(wave, 8);
HAL_PINA(offset, 8);
HAL_PINA(gain, 8);
HAL_PIN(send_step);
HAL_PIN(con);
HAL_PIN(fmt);        // 0 = 8 bit samples (old Servoterm), 1 = float samples
HAL_PIN(lost);       // live samples dropped because the ring was full
HAL_PIN(cap_arm);    // write 1 to arm a capture, back to 0 when it is uploaded; 0 aborts
HAL_PIN(cap_ch);     // trigger channel 0..7
HAL_PIN(cap_level);  // trigger level, in the wave's own units
HAL_PIN(cap_edge);   // 1 rising, -1 falling, 0 at once
HAL_PIN(cap_pre);    // fraction of the buffer before the trigger
HAL_PIN(cap_step);   // rt periods per capture sample
HAL_PIN(cap_state);  // 0 idle, 1 pre-trigger, 2 waiting, 3 post-trigger, 4 upload
HAL_PIN(cap_len);    // capture buffer length (samples)

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
