// Host glue for one firmware side, compiled once per side with
// -DSIDE=f4_side or -DSIDE=f3_side and -DSIDE_TAG. Stubs the few board
// functions hal.c and term.c call, and exports the side's API.
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "hal.h"
#include "ringbuf.h"
#include "side.h"

#ifndef SIDE_TAG
#define SIDE_TAG ""
#endif
#ifndef SIDE_CONSOLE
#define SIDE_CONSOLE 0
#endif

uint64_t systime;

uint32_t hal_get_systick_value() {
  return 0;
}
uint32_t hal_get_systick_reload() {
  return 1;
}
uint32_t hal_get_systick_freq() {
  return 168000000;
}
void hal_init_watchdog(float time) {
}
void hal_reset_watchdog() {
}

// the console: on the F3 term0 reads command lines from the link (ls.c puts
// the F4's hv bytes in rx_buf); the F4 has none, the runner parses directly
struct ringbuf rx_buf = RINGBUF(512);
struct ringbuf tx_buf = RINGBUF(512);

int cdc_is_connected() {
  return SIDE_CONSOLE;
}
int cdc_getline(char *ptr, int len) {
  return rb_getline(&rx_buf, ptr, len);
}
int cdc_tx(void *data, uint32_t len) {
  return 0;
}
int cdc_tx_text(const char *data, int len) {
  return 0;
}
void cdc_poll() {
}

static int verbose;
static int errors;
static int at_line_start = 1;

// The firmware is written for 32-bit long: its "%li"/"%lu" go with int32_t and
// uint32_t. On the host long is 64 bits, so the 'l' is dropped from the
// format before it reaches libc (a "%li" into an int32_t would write past it).
static const char *fmt32(const char *fmt, char *out, size_t size) {
  size_t j = 0;
  for(const char *c = fmt; *c && j + 1 < size; c++) {
    out[j++] = *c;
    if(*c == '%' && c[1] == '%') {
      out[j++] = *++c;
    } else if(*c == '%') {
      const char *d = c + 1;
      while(*d && strchr("-+ #0123456789.*[]^", *d) && j + 1 < size) {  // flags, width, scan sets
        if(*d == '[') {
          while(*d && *d != ']' && j + 1 < size) {
            out[j++] = *d++;
          }
        }
        if(*d) {
          out[j++] = *d++;
        }
      }
      c = d - 1;
      if(d[0] == 'l' && d[1] != 'l' && strchr("diuxXo", d[1])) {
        c = d;  // skip the l
      }
    }
  }
  out[j] = 0;
  return out;
}

int side_sscanf(const char *str, const char *fmt, ...) {
  char f[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsscanf(str, fmt32(fmt, f, sizeof(f)), ap);
  va_end(ap);
  return n;
}

// every printf of this side's firmware lands here (-Dprintf=side_printf)
int side_printf(const char *fmt, ...) {
  char buf[512], f[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt32(fmt, f, sizeof(f)), ap);
  va_end(ap);
  // hal_parse ends every multi-line text (a template) with an empty line it
  // reports as "not found: ", on the board too: not an error
  int err = (strstr(buf, "not found") && strcmp(buf, "not found: \n")) || strstr(buf, "load_comp:");
  errors += err;
  if(verbose || err) {
    for(char *c = buf; *c; c++) {
      if(at_line_start) {
        fputs(SIDE_TAG, stdout);
      }
      fputc(*c, stdout);
      at_line_start = *c == '\n';
    }
  }
  return n;
}

static void side_init(float rt_period, float frt_period) {
  hal_init(rt_period, frt_period);
  hal_set_debug_level(1);
}

static void side_parse(const char *line) {
  char buf[256];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  buf[strcspn(buf, "\r\n")] = 0;
  if(buf[0]) {
    hal_parse(buf);
  }
}

static void *side_pin(const char *name) {
  char comp[64], pin[64];
  int32_t inst = 0;
  if(side_sscanf(name, " %63[a-zA-Z_]%li.%63[a-zA-Z0-9_]", comp, &inst, pin) != 3) {
    return 0;
  }
  return pin_inst_by_name(comp, inst, pin);
}

static float side_get(void *p) {
  return ((hal_pin_inst_t *)p)->source->value;
}

static void side_set(void *p, float v) {
  ((hal_pin_inst_t *)p)->value = v;
}

static int side_errors(void) {
  return errors;
}

static int side_hal_state(void) {
  return hal.hal_state;
}

static void side_set_systime(uint64_t ms) {
  systime = ms;
}

static void side_set_verbose(int v) {
  verbose = v;
  hal_set_debug_level(v ? 0 : 1);
}

static int side_console_getc(char *c) {
  return rb_getc(&tx_buf, c);
}

static void side_console_putc(char c) {
  rb_write(&rx_buf, &c, 1);
}

sim_side_t SIDE = {
    .name          = SIDE_TAG,
    .init          = side_init,
    .parse         = side_parse,
    .run_rt        = hal_run_rt,
    .run_frt       = hal_run_frt,
    .run_nrt       = hal_run_nrt,
    .pin           = side_pin,
    .get           = side_get,
    .set           = side_set,
    .errors        = side_errors,
    .hal_state     = side_hal_state,
    .set_systime   = side_set_systime,
    .set_verbose   = side_set_verbose,
    .console_getc  = side_console_getc,
    .console_putc  = side_console_putc,
};
