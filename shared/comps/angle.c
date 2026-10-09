#include "angle_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* The `angle` component owns the commutation angle and the synchronous
* electrical speed sent to hv0 (`hv0.pos`, `hv0.vel`), for every motor type.
*
* ## Sources (`src`)
* - 0 feedback: `pos = pos_fb`, `vel = vel_fb`. A pure pass-through, used by
*   the PMSM templates (pos_fb = vel2.pos_out or pmsm_ttc0.pos_out, vel_fb =
*   vel2.vel), so it sends exactly what they linked to hv0 before.
* - 1 feedback plus slip: `vel = vel_m * polecount + slip`, `pos` integrates
*   vel. Indirect field orientation for an induction motor on an encoder:
*   vel_m is the mechanical rotor speed, slip comes from acim_flux.
* - 2 observer: `pos = pos_obs`, `vel = vel_obs`.
* - 3 open loop: `vel = vel_cmd` (electrical), `pos` integrates it. V/f and I/f.
*
* Switching into an integrating source (1, 3) starts from the current angle,
* so the switch itself does not move the field. Switching into 0 or 2 takes
* the source's angle as is; a sequencer that switches onto the observer is
* expected to do it once the observer has locked.
*
* `vel` is always synchronous electrical speed [rad/s], which is what the F3
* current loop wants for its decoupling and what it extrapolates the angle with.
*/

HAL_COMP(angle);

HAL_PIN(src);        // *parameter*, 0 feedback, 1 feedback + slip, 2 observer, 3 open loop
HAL_PIN(polecount);  // *parameter*, pole pairs, used by src 1

HAL_PIN(pos_fb);     // *input*, src 0, electrical angle [rad]
HAL_PIN(vel_fb);     // *input*, src 0, electrical speed [rad/s]
HAL_PIN(vel_m);      // *input*, src 1, mechanical rotor speed [rad/s]
HAL_PIN(slip);       // *input*, src 1, slip [rad/s electrical]
HAL_PIN(pos_obs);    // *input*, src 2, electrical angle [rad]
HAL_PIN(vel_obs);    // *input*, src 2, electrical speed [rad/s]
HAL_PIN(vel_cmd);    // *input*, src 3, electrical speed [rad/s]

HAL_PIN(pos);        // *output*, commutation angle [rad], to hv0.pos
HAL_PIN(vel);        // *output*, synchronous electrical speed [rad/s], to hv0.vel
HAL_PIN(v_lead);     // *parameter*, periods the voltage lands after the sample, 0 = none
HAL_PIN(pos_v);      // *output*, pos + vel * v_lead * period, the voltage angle (not wrapped)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct angle_pin_ctx_t *pins = (struct angle_pin_ctx_t *)pin_ptr;

  PIN(src)       = 0.0;
  PIN(polecount) = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct angle_pin_ctx_t *pins = (struct angle_pin_ctx_t *)pin_ptr;

  float pos = PIN(pos);
  float vel;

  switch((int)PIN(src)) {
    case 1:  // feedback plus slip
      vel = PIN(vel_m) * PIN(polecount) + PIN(slip);
      pos = mod(pos + vel * period);
      break;

    case 2:  // observer
      vel = PIN(vel_obs);
      pos = PIN(pos_obs);
      break;

    case 3:  // open loop
      vel = PIN(vel_cmd);
      pos = mod(pos + vel * period);
      break;

    default:  // feedback, pass-through
      vel = PIN(vel_fb);
      pos = PIN(pos_fb);
  }

  PIN(vel)   = vel;
  PIN(pos)   = pos;
  PIN(pos_v) = pos + vel * PIN(v_lead) * period;
}

hal_comp_t angle_comp_struct = {
    .name      = "angle",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct angle_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
