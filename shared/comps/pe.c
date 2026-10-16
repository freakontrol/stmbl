#include "pe_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `pe` (power and energy) computes power and energy values of the drive for display on the scope. It runs on the F4 board and is loaded by `conf/template/pid.txt` and `mpid.txt` with `pe0.udc = hv0.dc_volt`, `pe0.id/iq = hv0.id_fb/iq_fb`, `pe0.torque = pid0.torque_cmd`, `pe0.vel = vel1.vel`, `pe0.r = conf0.r`, `pe0.j = conf0.j`, `pe0.cap = 0.00054`. The templates set `idc`, `ud` and `uq` to 0. No component uses the outputs.
*
* ## Component Explanation
* 1. **Energies** (in `rt`): `e_el = cap * udc^2 / 2` (DC link capacitor), `e_kin = j * vel^2 / 2` (rotor).
*
* 2. **Powers**: `p_el_dc = udc * idc`, `p_m = torque * vel`, `p_el_ac` and `p_t` (copper losses) as below.
*
* `p_el_ac` is `1.5 * (ud * id + uq * iq)` and `p_t` is `1.5 * r * (id^2 + iq^2)`; `p_el_ac` is 0 in the templates anyway.
*/

HAL_COMP(pe);

HAL_PIN(udc);     // *input*, DC link voltage (V)
HAL_PIN(idc);     // *input*, DC link current (A)
HAL_PIN(ud);      // *input*, d-axis voltage (V)
HAL_PIN(uq);      // *input*, q-axis voltage (V), not used
HAL_PIN(id);      // *input*, d-axis current (A)
HAL_PIN(iq);      // *input*, q-axis current (A)
HAL_PIN(torque);  // *input*, Torque (Nm)
HAL_PIN(vel);     // *input*, Velocity (rad/s)
HAL_PIN(r);       // *parameter*, Phase resistance (Ohm)
HAL_PIN(j);       // *parameter*, Inertia (kgm^2)
HAL_PIN(cap);     // *parameter*, DC link capacitance (F)

HAL_PIN(e_el);     // *output*, Energy in the DC link capacitor (J)
HAL_PIN(e_kin);    // *output*, Kinetic energy (J)
HAL_PIN(p_el_dc);  // *output*, DC link power (W)
HAL_PIN(p_el_ac);  // *output*, AC power, buggy (W)
HAL_PIN(p_m);      // *output*, Mechanical power (W)
HAL_PIN(p_t);      // *output*, Copper losses, missing factor 3/2 (W)

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct pe_ctx_t * ctx = (struct pe_ctx_t *)ctx_ptr;
  struct pe_pin_ctx_t *pins = (struct pe_pin_ctx_t *)pin_ptr;

  float udc = PIN(udc);
  float vel = PIN(vel);
  float id  = PIN(id);
  float iq  = PIN(iq);
  float t   = PIN(torque);

  PIN(e_el)  = PIN(cap) * udc * udc / 2.0;
  PIN(e_kin) = PIN(j) * vel * vel / 2.0;

  PIN(p_el_dc) = udc * PIN(idc);
  PIN(p_el_ac) = 1.5 * (PIN(ud) * id + PIN(uq) * iq);
  PIN(p_m)     = t * vel;
  PIN(p_t)     = 1.5 * PIN(r) * (id * id + iq * iq);
}

hal_comp_t pe_comp_struct = {
    .name      = "pe",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct pe_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
