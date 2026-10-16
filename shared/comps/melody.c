/**
 * ## Overview
 *
 * Plays a tune through the motor: an audio-rate current added to the d-axis
 * command makes the stator hum the notes without torque (with the commutation
 * offset right). `link melody` (after `link pmsm`) inserts it between the
 * d-axis source and hv, as conf/template/melody.txt:
 *
 * ```
 * load melody
 * melody0.rt_prio = 6
 * melody0.in = fb_switch0.id
 * melody0.en = fault0.en_pid
 * hv0.d_cmd = melody0.out
 * ```
 *
 * `out` is `in` while nothing plays. The tone is a square (`wave = 1`) or a
 * quieter sine (`wave = 0`) of peak `amp` amps. The rt thread runs at 5 kHz,
 * so notes above ~2 kHz alias, and the current loop damps the high notes.
 *
 * Nothing plays unless `en`: linked to `fault0.en_pid` that is enabled and
 * past phasing, so the bridge is on anyway and the tone never switches it on.
 * `en` dropping stops the tune. With `chime = 1` a short rising arpeggio plays
 * on the first enable after power-up (`chimed` = 1 once it has), so the drive
 * says it booted and runs its config. `identify` (or a rising edge on `ident`)
 * beeps three times three, to find which drive is which.
 *
 * ## Commands
 *
 * | Command         | Action                                          |
 * |-----------------|-------------------------------------------------|
 * | `tune <notes>`  | load the notes and play them                    |
 * | `tune +<notes>` | add the notes to the loaded ones, do not play   |
 * | `tune ode`      | play Ode to Joy                                 |
 * | `tune`          | play the loaded notes again                     |
 * | `tunex`         | stop                                            |
 * | `identify`      | beep to find this drive                         |
 *
 * A note is `<letter>[#|b][octave][/len][.]`: `C4/4` quarter middle C,
 * `F#5/8` eighth, `Bb3/2.` dotted half, `R/4` quarter rest. Octave and
 * length carry over from the previous note (start: 4, /4). A quarter note is
 * one beat at `bpm`; each note sounds for `gate` of its length so repeated
 * notes stay apart. A terminal line holds 63 characters, so a longer tune is
 * loaded in parts with `tune +`; each part starts again at octave 4, /4. A
 * line with a bad note changes nothing.
 */
#include "melody_comp.h"
#include "commands.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

HAL_COMP(melody);

HAL_PIN(in);       // d-axis command passed through (A)
HAL_PIN(out);      // in + tone (A)
HAL_PIN(en);       // tunes play only while > 0 (fault0.en_pid)
HAL_PIN(amp);      // tone peak current (A)
HAL_PIN(wave);     // 0 = sine, 1 = square
HAL_PIN(bpm);      // quarter notes per minute
HAL_PIN(gate);     // sounding fraction of each note
HAL_PIN(loop);     // 1 = start over at the end
HAL_PIN(chime);    // 1 = chime on the first enable after power-up
HAL_PIN(ident);    // rising edge = identify
HAL_PIN(freq);     // note playing (Hz), 0 = rest or stopped
HAL_PIN(playing);  // 1 while a tune plays
HAL_PIN(chimed);   // 1 once the power-up chime has played

#define MELODY_MAX 128

struct tune_t {
  volatile float hz[MELODY_MAX];     // 0 = rest
  volatile float beats[MELODY_MAX];  // quarter notes
  volatile uint32_t n;
};

static struct tune_t user, chime, ident;
static struct tune_t *volatile start;  // set by the commands, taken by rt
static volatile uint32_t stop;
static volatile uint32_t enabled;  // rt's en, for the commands

// Ode to Joy (Beethoven, public domain), first two phrases.
static const char ode[] =
    "E4/4 E F G G F E D C C D E E/4. D/8 D/2 "
    "E/4 E F G G F E D C C D E D/4. C/8 C/2";
static const char chime_notes[] = "C4/16 E G C5/8";
static const char ident_notes[] =
    "A4/16 R A R A R/4 A/16 R A R A R/4 A/16 R A R A R/4";

struct melody_ctx_t {
  struct tune_t *cur;
  uint32_t idx;
  float t;      // time into the current note (s)
  float phase;  // tone phase (turns)
  uint32_t playing;
  uint32_t chimed;
  float ident;  // last ident pin
};

// Parses s into tune from note n on and returns the new note count, or -1 on
// a bad note. tune = 0 only checks s. The caller sets tune->n.
static int parse(const char *s, struct tune_t *tune, uint32_t n) {
  static const int8_t semi[7] = {9, 11, 0, 2, 4, 5, 7};  // A..G from C
  int oct                     = 4;
  float len                   = 4.0;

  while(*s) {
    while(*s == ' ') {
      s++;
    }
    if(!*s) {
      break;
    }
    if(n == MELODY_MAX) {
      printf("tune: more than %d notes\n", MELODY_MAX);
      return -1;
    }
    char c = *s++;
    if(c >= 'a' && c <= 'z') {
      c -= 'a' - 'A';
    }
    int rest = c == 'R';
    if(!rest && (c < 'A' || c > 'G')) {
      printf("tune: bad note at '%c%s'\n", c, s);
      return -1;
    }
    int k = rest ? 0 : semi[c - 'A'];
    if(*s == '#') {
      k++;
      s++;
    } else if(*s == 'b') {
      k--;
      s++;
    }
    if(*s >= '0' && *s <= '8') {
      oct = *s++ - '0';
    }
    if(*s == '/') {
      s++;
      int d = 0;
      while(*s >= '0' && *s <= '9') {
        d = d * 10 + *s++ - '0';
      }
      if(d < 1 || d > 64) {
        printf("tune: bad length /%d\n", d);
        return -1;
      }
      len = d;
    }
    float beats = 4.0 / len;
    if(*s == '.') {
      beats *= 1.5;
      s++;
    }
    if(*s && *s != ' ') {
      printf("tune: bad note at '%s'\n", s);
      return -1;
    }
    if(tune) {
      int midi       = 12 * (oct + 1) + k;
      tune->hz[n]    = rest ? 0.0 : 440.0 * powf(2.0, (midi - 69) / 12.0);
      tune->beats[n] = beats;
    }
    n++;
  }
  return n;
}

static void play(struct tune_t *tune) {
  stop  = 0;
  start = tune;
}

void tune_cmd(char *ptr) {
  if(ptr[0]) {
    uint32_t add  = ptr[0] == '+';
    const char *s = add ? ptr + 1 : ptr;
    if(!strcmp(s, "ode")) {
      s = ode;
    }
    uint32_t n0 = add ? user.n : 0;
    int n       = parse(s, 0, n0);  // check first, a bad line keeps the loaded notes
    if(n < 0) {
      return;
    }
    if(n == (int)n0) {
      printf("tune: no notes\n");
      return;
    }
    user.n = 0;  // rt stops reading before the arrays change
    parse(s, &user, n0);
    user.n = n;
    printf("tune: %d notes\n", n);
    if(add) {
      return;
    }
  }
  if(!user.n) {
    printf("tune: nothing loaded\n");
    return;
  }
  if(!enabled) {
    printf("tune: not enabled\n");
    return;
  }
  play(&user);
}

void tunex_cmd(char *ptr) {
  stop = 1;
}

void identify_cmd(char *ptr) {
  if(!enabled) {
    printf("identify: not enabled\n");
    return;
  }
  play(&ident);
}

COMMAND("tune", tune_cmd, "play notes, e.g. tune C4/4 E G C5/2 (tune +notes adds, tune ode, tune = again)");
COMMAND("tunex", tunex_cmd, "stop the tune");
COMMAND("identify", identify_cmd, "beep to find this drive (enabled only)");

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct melody_pin_ctx_t *pins = (struct melody_pin_ctx_t *)pin_ptr;

  PIN(en)    = 1.0;
  PIN(amp)   = 6.0;
  PIN(wave)  = 1.0;
  PIN(bpm)   = 120.0;
  PIN(gate)  = 0.9;
  PIN(chime) = 1.0;
  chime.n    = parse(chime_notes, &chime, 0);
  ident.n    = parse(ident_notes, &ident, 0);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct melody_ctx_t *ctx      = (struct melody_ctx_t *)ctx_ptr;
  struct melody_pin_ctx_t *pins = (struct melody_pin_ctx_t *)pin_ptr;

  enabled = PIN(en) > 0.0;
  if(!enabled) {
    start        = 0;  // a tune asked for while off does not start later
    ctx->playing = 0;
  } else {
    if(PIN(ident) > 0.0 && ctx->ident <= 0.0) {
      play(&ident);
    }
    if(!ctx->chimed) {
      ctx->chimed = 1;
      if(PIN(chime) > 0.0 && !start) {
        play(&chime);
      }
    }
  }
  ctx->ident = PIN(ident);

  struct tune_t *s = start;
  if(s) {
    start        = 0;
    ctx->cur     = s;
    ctx->idx     = 0;
    ctx->t       = 0.0;
    ctx->phase   = 0.0;
    ctx->playing = 1;
  }
  if(stop || !ctx->cur || ctx->idx >= ctx->cur->n) {
    ctx->playing = 0;
  }

  float tone = 0.0;
  float hz   = 0.0;
  if(ctx->playing) {
    struct tune_t *tune = ctx->cur;
    float beat          = 60.0 / MAX(PIN(bpm), 1.0);
    float len           = tune->beats[ctx->idx] * beat;
    if(ctx->t < len * CLAMP(PIN(gate), 0.0, 1.0)) {
      hz = tune->hz[ctx->idx];
    }
    if(hz > 0.0) {
      ctx->phase += hz * period;
      ctx->phase -= floorf(ctx->phase);
      float si, co;
      sincos_fast(ctx->phase * 2.0 * M_PI, &si, &co);
      if(PIN(wave) > 0.0) {
        si = si > 0.0 ? 1.0 : -1.0;
      }
      tone = PIN(amp) * si;
    } else {
      ctx->phase = 0.0;  // every note starts at zero current
    }
    ctx->t += period;
    if(ctx->t >= len) {
      ctx->t -= len;
      ctx->idx++;
      if(ctx->idx >= tune->n && PIN(loop) > 0.0 && tune == &user && !stop) {
        ctx->idx = 0;
      }
    }
  }

  PIN(freq)    = hz;
  PIN(playing) = ctx->playing;
  PIN(chimed)  = ctx->chimed && PIN(chime) > 0.0;
  PIN(out)     = PIN(in) + tone;
}

hal_comp_t melody_comp_struct = {
    .name      = "melody",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct melody_ctx_t),
    .pin_count = sizeof(struct melody_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
