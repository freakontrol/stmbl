#include "jog_comp.h"
#include "commands.h"
#include "hal.h"
#include "defines.h"

/**
* ## Brief
* `jog` provides terminal commands to jog an axis. F4 component, used in `conf/template/jog_cmd.txt` together with `stp` (`stp0.jog = jog0.jog`).
*
* ## Component Explanation
* 1. **Commands**:
* - `jogl` sets `jog` to -1, `jogr` to 1, `jogx` to 0.
*
* 2. **Timeout** (`rt`):
* - If no command is received for 0.75 s, `jog` returns to 0, so the command has to be repeated (e.g. by holding the key in the terminal) to keep moving.
* - The jog state is a global variable, all instances share it.
*/

HAL_COMP(jog);

HAL_PIN(jog);  // *output*, -1 = left, 0 = stop, 1 = right

static volatile float jog;
static volatile float jog_timeout;

void jog_left(char *ptr) {
  jog         = -1.0;
  jog_timeout = 0.0;
}

void jog_right(char *ptr) {
  jog         = 1.0;
  jog_timeout = 0.0;
}

void jog_stop(char *ptr) {
  jog         = 0.0;
  jog_timeout = 0.0;
}

COMMAND("jogl", jog_left, "Jog left");
COMMAND("jogr", jog_right, "Jog right");
COMMAND("jogx", jog_stop, "Stop jog");

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  jog_timeout = 0.0;
  jog         = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct jog_pin_ctx_t *pins = (struct jog_pin_ctx_t *)pin_ptr;

  if(jog_timeout < 0.75) {
    jog_timeout += period;
  } else {
    jog = 0.0;
  }
  PIN(jog) = jog;
}

hal_comp_t jog_comp_struct = {
    .name      = "jog",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct jog_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
