#include "acim_fw_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* The `acim_fw` component is the induction motor's field weakening: it lowers
* the flux command (`scale`, 1 = rated flux) to hold the modulation `duty` at
* `duty_setpoint`, and gives a constant power torque limit `t_lim`.
*
* This is acim_ttc's regulator on its own. `duty` unlinked reads 0, so scale
* sits at 1 and there is no field weakening, which is stmbl's default; linking
* `acim_fw0.duty = hv0.duty` is the machine builder's choice.
*
* `t_lim` = p_max / mechanical speed, 0 = no limit (p_max 0).
*/

HAL_COMP(acim_fw);

HAL_PIN(en);             // *input*, 0 resets scale to 1
HAL_PIN(duty);           // *input*, modulation, hv0.duty
HAL_PIN(duty_setpoint);  // *parameter*, duty held in field weakening
HAL_PIN(ki);             // *parameter*, regulator gain [1/s]
HAL_PIN(scale_min);      // *parameter*, lowest flux fraction
HAL_PIN(vel);            // *input*, electrical speed [rad/s], angle0.vel
HAL_PIN(polecount);      // *parameter*, pole pairs
HAL_PIN(p_max);          // *parameter*, constant power limit [W], 0 = off

HAL_PIN(scale);          // *output*, flux command fraction
HAL_PIN(t_lim);          // *output*, torque limit [Nm], 0 = none

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_fw_pin_ctx_t *pins = (struct acim_fw_pin_ctx_t *)pin_ptr;

  PIN(duty_setpoint) = 0.9;
  PIN(ki)            = 50.0;
  PIN(scale_min)     = 0.1;
  PIN(polecount)     = 2.0;
  PIN(scale)         = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_fw_pin_ctx_t *pins = (struct acim_fw_pin_ctx_t *)pin_ptr;

  float scale = PIN(scale);
  if(PIN(en) > 0.0) {
    scale += (PIN(duty_setpoint) - PIN(duty)) * PIN(ki) * period;
  } else {
    scale = 1.0;
  }
  PIN(scale) = CLAMP(scale, CLAMP(PIN(scale_min), 0.01, 1.0), 1.0);

  float t_lim = 0.0;
  if(PIN(p_max) > 0.0) {
    t_lim = PIN(p_max) / MAX(ABS(PIN(vel)) / MAX(PIN(polecount), 1.0), 0.1);
  }
  PIN(t_lim) = t_lim;
}

hal_comp_t acim_fw_comp_struct = {
    .name      = "acim_fw",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct acim_fw_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
