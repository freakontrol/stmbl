#include "iit_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* `iit` is an I2t thermal model of the motor: it estimates the motor temperature from the phase current, for motors without a temperature sensor. It runs on the F4 board and is loaded by `conf/template/pid.txt`, `mpid.txt` and `uf.txt` with `iit0.cur = hv0.abs_cur`, `iit0.max_cur = conf0.max_ac_cur`, `iit0.high_temp = conf0.high_mot_temp`, `iit0.max_temp = conf0.max_mot_temp`, and `fault0.mot_temp = iit0.temp`, so the model derates the current (`fault0.scale`) and finally trips a motor over temperature fault.
*
* ## Component Explanation
* All work is done in `rt`. `max_cur` must be set; with 0 there is no model and `temp` stays at `amb_temp`.
*
* 1. **Model**:
* - Nominal (continuous) current: `cur_n = max_cur / MAX(cur_boost, 1)`.
* - An internal energy `e` is integrated: `e += (cur^2 - pout) * period`. `e` is a double: with `max_time` of several minutes a float `e` gets so large that one period's increment rounds away and the estimate stalls short of its final value.
* - Temperature: `temp = e / (max_cur^2 * max_time) * (max_temp - amb_temp) + amb_temp`.
* - Cooling: `pout = (temp - amb_temp) * cur_n^2 / (high_temp - amb_temp)`.
*
* 2. **Meaning of the parameters**:
* - At `cur_n` the temperature settles at `high_temp` (where `fault` starts derating).
* - Starting cold, `max_cur` reaches `max_temp` after roughly `max_time` seconds (a little longer because of the cooling term).
* - `high_temp` must be greater than `amb_temp`. `temp` starts at `amb_temp` after power up; the model state is not stored.
*/

HAL_COMP(iit);

// conf
HAL_PIN(amb_temp);   // *parameter*, Ambient temperature (degC, default 30)
HAL_PIN(high_temp);  // *parameter*, Steady state temperature at the nominal current (degC, default 80)
HAL_PIN(max_temp);   // *parameter*, Temperature reached after max_time at max_cur (degC, default 100)
HAL_PIN(max_cur);    // *parameter*, Maximum motor current (A), must be set
HAL_PIN(cur_boost);  // *parameter*, max_cur / nominal current (default 3)
HAL_PIN(max_time);   // *parameter*, Time at max_cur until max_temp (s, default 10, min 0.1)

// out
HAL_PIN(temp);  // *output*, Modelled motor temperature (degC)
// in
HAL_PIN(cur);  // *input*, Motor current magnitude (A)

struct iit_ctx_t {
  // double: at a motor-length time constant (max_time several minutes) a
  // float e is so large that one period's increment rounds away, and the
  // estimate stalls short of its final value
  double e;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct iit_ctx_t *ctx      = (struct iit_ctx_t *)ctx_ptr;
  struct iit_pin_ctx_t *pins = (struct iit_pin_ctx_t *)pin_ptr;
  PIN(amb_temp)              = 30.0;
  PIN(high_temp)             = 80.0;
  PIN(max_temp)              = 100.0;
  PIN(max_time)              = 10.0;
  PIN(cur_boost)             = 3.0;
  ctx->e                     = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct iit_ctx_t *ctx      = (struct iit_ctx_t *)ctx_ptr;
  struct iit_pin_ctx_t *pins = (struct iit_pin_ctx_t *)pin_ptr;

  float cur_n = PIN(max_cur) / MAX(PIN(cur_boost), 1.0);
  float max_e = PIN(max_cur) * PIN(max_cur) * MAX(PIN(max_time), 0.1);
  if(max_e <= 0.0) {  // max_cur 0: no model, keep temp a number so limits still compare
    ctx->e    = 0.0;
    PIN(temp) = PIN(amb_temp);
    return;
  }

  float temp = (float)ctx->e / max_e * (PIN(max_temp) - PIN(amb_temp)) + PIN(amb_temp);

  float pin  = PIN(cur) * PIN(cur);
  float pout = (temp - PIN(amb_temp)) * cur_n * cur_n / (PIN(high_temp) - PIN(amb_temp));

  ctx->e += (double)((pin - pout) * period);

  PIN(temp) = temp;
}

hal_comp_t iit_comp_struct = {
    .name      = "iit",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct iit_ctx_t),
    .pin_count = sizeof(struct iit_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
