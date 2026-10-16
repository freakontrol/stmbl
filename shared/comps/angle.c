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
*   vel2.vel).
* - 1 feedback plus slip: `pos = pos_m * polecount + integral of slip`,
*   `vel = vel_m * polecount + slip`. Indirect field orientation for an
*   induction motor on an encoder: pos_m is the mechanical rotor angle,
*   vel_m the mechanical rotor speed, slip comes from acim_flux. The angle
*   is built from the encoder, not from vel_m, so the lag of a speed
*   estimate during a ramp does not show up as a lower slip.
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
HAL_PIN(pos_m);      // *input*, src 1, mechanical rotor angle [rad]
HAL_PIN(vel_m);      // *input*, src 1, mechanical rotor speed [rad/s]
HAL_PIN(slip);       // *input*, src 1, slip [rad/s electrical]
HAL_PIN(pos_obs);    // *input*, src 2, electrical angle [rad]
HAL_PIN(vel_obs);    // *input*, src 2, electrical speed [rad/s]
HAL_PIN(vel_cmd);    // *input*, src 3, electrical speed [rad/s]

HAL_PIN(pos);        // *output*, commutation angle [rad], to hv0.pos
HAL_PIN(vel);        // *output*, synchronous electrical speed [rad/s], to hv0.vel

struct angle_ctx_t {
  float slip_pos;  // src 1: field angle ahead of pos_m * polecount
  int last_src;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct angle_pin_ctx_t *pins = (struct angle_pin_ctx_t *)pin_ptr;

  PIN(src)       = 0.0;
  PIN(polecount) = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct angle_ctx_t *ctx      = (struct angle_ctx_t *)ctx_ptr;
  struct angle_pin_ctx_t *pins = (struct angle_pin_ctx_t *)pin_ptr;

  float pos = PIN(pos);
  float vel;
  int src   = (int)PIN(src);

  switch(src) {
    case 1:  // feedback plus slip
      if(ctx->last_src != 1) {  // start from the current angle
        ctx->slip_pos = minus(pos, mod(PIN(pos_m) * PIN(polecount)));
      }
      ctx->slip_pos = mod(ctx->slip_pos + PIN(slip) * period);
      vel           = PIN(vel_m) * PIN(polecount) + PIN(slip);
      pos           = mod(PIN(pos_m) * PIN(polecount) + ctx->slip_pos);
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

  ctx->last_src = src;
  PIN(vel)   = vel;
  PIN(pos)   = pos;
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
    .ctx_size  = sizeof(struct angle_ctx_t),
    .pin_count = sizeof(struct angle_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
