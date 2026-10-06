#include "ocdac_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
 * ocdac: sets the F3 overcurrent comparator threshold (hv0.dac) from a trip
 * current in A peak, using a linear dac map dac = dac0 + k * cur.
 *
 * The comparators see the raw shunt node (2.9 mV/A, about 3.6 counts/A in
 * theory); the measured slope and offset depend on the TIM8 break filter,
 * because the shorter filters trip on switching ringing on top of the
 * current. Measured with filter 0xC (the main_rework default), highest trip of the
 * 60/180/300 deg angles:
 *
 *   X, IKCM30F60GD, 1 Oct 2026     dac0 115, k 6.7   (dac 215..255, 15..21 A)
 *   spindle, IM06B50GC1, 6 Oct     dac0 128, k 6.48  (dac 200..340, 11.3..32.3 A
 *                                  measured id, 0.12 s d pulses)
 *   Y, IKCM30F60GD, filter 0xF     dac0 140, k 3.5   (old filter only)
 *
 * The defaults are X's. Fit dac0 and k per board from a few trips (id_dac
 * finds the dac for one current directly) and save them in the config.
 *
 * hv0.dac = ocdac0.dac
 */
HAL_COMP(ocdac);

HAL_PIN(cur);      // wanted trip current [A peak]
HAL_PIN(dac0);     // dac at 0 A from the fit
HAL_PIN(k);        // dac counts per A
HAL_PIN(dac_min);  // floor: lowest dac with no trip at 0 A (switching ringing)
HAL_PIN(dac_max);  // ceiling
HAL_PIN(dac);      // out, to hv0.dac
HAL_PIN(cur_out);  // out, trip current the clamped dac stands for [A]

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ocdac_pin_ctx_t *pins = (struct ocdac_pin_ctx_t *)pin_ptr;
  PIN(cur)     = 20.0;
  PIN(dac0)    = 115.0;
  PIN(k)       = 6.7;
  PIN(dac_min) = 150.0;
  PIN(dac_max) = 4095.0;
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ocdac_pin_ctx_t *pins = (struct ocdac_pin_ctx_t *)pin_ptr;

  float k      = MAX(PIN(k), 0.1);
  float dac    = CLAMP(roundf(PIN(dac0) + k * MAX(PIN(cur), 0.0)), PIN(dac_min), PIN(dac_max));
  PIN(dac)     = dac;
  PIN(cur_out) = (dac - PIN(dac0)) / k;
}

hal_comp_t ocdac_comp_struct = {
    .name      = "ocdac",
    .nrt       = nrt_func,
    .rt        = 0,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct ocdac_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
