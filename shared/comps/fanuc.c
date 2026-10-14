#include "fanuc_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* The `fanuc` component decodes the four digital commutation tracks `C1`, `C2`, `C4` and `C8` of a Fanuc pulse coder into an absolute commutation angle. It is used on the F4 board via `conf/template/fanuc_io.txt`, which reads the tracks from `io0.C12`, `io0.CTX`, `io0.CRX` and `io0.C54` (with `io0.cmd_remap = 1`) and links `fanuc0.pos` to `fb_switch0.com_pos` / `fb_switch0.com_abs_pos`.
*
* ## Component Explanation
*
* 1. **Code word** (`rt`):
* - The 4-bit word is built as
* ```c
* t = C1 + 2 * C2 + 4 * C4 + 8 * C8;
* ```
*
* 2. **Lookup table**:
* - A fixed table maps the 16 possible words (a Gray-code sequence) to the sector numbers 0..15:
* ```c
* t:      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
* sector: 0 15  1  2 13 14 12 11  5  6  4  3  8  7  9 10
* ```
* - The output is `pos = mod(sector / 16 * 2 * pi)` (22.5 deg per sector). There is no error detection: every code is treated as valid.
*/

HAL_COMP(fanuc);

//fanuc encoder
HAL_PIN(C1);   // *input*, Commutation track C1 (bit 0, high when > 0)
HAL_PIN(C2);   // *input*, Commutation track C2 (bit 1, high when > 0)
HAL_PIN(C4);   // *input*, Commutation track C4 (bit 2, high when > 0)
HAL_PIN(C8);   // *input*, Commutation track C8 (bit 3, high when > 0)

//rotor position output
HAL_PIN(pos);  // *output*, Absolute commutation angle (rad, +-pi)

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct fanuc_ctx_t * ctx = (struct fanuc_ctx_t *)ctx_ptr;
  struct fanuc_pin_ctx_t *pins = (struct fanuc_pin_ctx_t *)pin_ptr;

  //TODO: const...
  uint8_t tab[16];
  tab[0]  = 0;
  tab[1]  = 15;
  tab[2]  = 1;
  tab[3]  = 2;
  tab[4]  = 13;
  tab[5]  = 14;
  tab[6]  = 12;
  tab[7]  = 11;
  tab[8]  = 5;
  tab[9]  = 6;
  tab[10] = 4;
  tab[11] = 3;
  tab[12] = 8;
  tab[13] = 7;
  tab[14] = 9;
  tab[15] = 10;

  uint32_t t = (PIN(C1) > 0.0) + (PIN(C2) > 0.0) * 2 + (PIN(C4) > 0.0) * 4 + (PIN(C8) > 0.0) * 8;
  // 16 Gray-code sectors per electrical turn, 1/16 turn each
  PIN(pos) = mod(tab[t] / 16.0 * 2.0 * M_PI);
}

hal_comp_t fanuc_comp_struct = {
    .name      = "fanuc",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct fanuc_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
