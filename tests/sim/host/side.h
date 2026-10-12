// One firmware side (F4 or F3) as the runner sees it. Each side is linked
// into a single object with only its sim_side_t global, so the two copies of
// hal.c, the comps and their tables never meet.
#pragma once
#include <stdint.h>

typedef struct {
  const char *name;
  void (*init)(float rt_period, float frt_period);
  void (*parse)(const char *line);  // one or more lines, as typed into Servoterm
  void (*run_rt)(void);
  void (*run_frt)(void);
  void (*run_nrt)(void);
  void *(*pin)(const char *name);  // "comp0.pin", 0 when not loaded
  float (*get)(void *pin);         // value at the pin's source
  void (*set)(void *pin, float v);  // the pin's own value (an output, or an unlinked input)
  int (*errors)(void);             // "not found" and load failures so far
  int (*hal_state)(void);          // hal.hal_state, 7 = HAL_OK2
  void (*set_systime)(uint64_t ms);
  void (*set_verbose)(int v);  // 0 errors only, 1 all firmware output
  int (*console_getc)(char *c);  // F3 only: the F4's hv command bytes in, 0 = none
  void (*console_putc)(char c);
} sim_side_t;

extern sim_side_t f4_side;
extern sim_side_t f3_side;
