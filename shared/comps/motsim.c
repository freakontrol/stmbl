#include "motsim_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `motsim` is a simple software model of a PMSM with its own d/q current controller, for testing control components without hardware. It is built into the F4 firmware but no config template loads it; use `load motsim` and wire it by hand in place of `hv0`: it takes the same `d_cmd`/`q_cmd`/`cmd_mode`/`en` and commutation angle, and returns `id_fb`, `iq_fb`, `ud_fb`, `uq_fb` and a quantised rotor position `pos_fb`.
*
* ## Component Explanation
*
* Everything is done in `rt`, one integration step per rt period (explicit Euler). There are two sets of motor constants: `r`, `l`, `psi` are what the simulated controller believes (like `hv0.r/l/psi`), `mot_r`, `mot_l`, `mot_psi`, `mot_pp`, `mot_j`, `mot_f`, `mot_d` are the simulated motor.
*
* 1. **Current feedback**:
* - The motor currents `id`, `iq` (in the true rotor frame, angle `pos * mot_pp`) are turned into alpha/beta and back into d/q at the commutation angle `com_pos`. So `id_fb`/`iq_fb` are what a controller with a wrong commutation offset would see.
*
* 2. **Controller**, selected by `cmd_mode`:
* - 0, voltage mode: `ud = d_cmd`, `uq = q_cmd` (V).
* - 1, current mode: PI with feed forward, gains from the controller model:
* ```c
* kp = cur_bw * l;   ki = cur_bw * r;
* ud = r * id_fb - com_vel * l * iq_fb        + kp * (d_cmd - id_fb) + id_error_sum;
* uq = r * iq_fb + com_vel * (l * id_fb + psi) + kp * (q_cmd - iq_fb) + iq_error_sum;
* ```
* - `id_error_sum`/`iq_error_sum` integrate `ki * error`, limited to `pwm_volt - ud` (resp. `uq`); each axis is limited to +-`pwm_volt` on its own, not as a vector.
* - Both voltages are quantised to `pwm_res` steps of `pwm_volt` and written to `ud_fb`/`uq_fb`.
*
* 3. **Motor model**:
* - The voltages are rotated from `com_pos` into the true rotor frame. `en <= 0` sets them to 0 (only the voltage; the controller keeps running).
* ```c
* did/dt = (ud - mot_r * id + w * mot_l * iq) / mot_l;              // w = vel * mot_pp
* diq/dt = (uq - mot_r * iq - w * (mot_l * id + mot_psi)) / mot_l;
* torque = 3/2 * mot_psi * iq * mot_pp + load_torque - vel * mot_d - sign(vel) * mot_f;
* acc = torque / mot_j;
* ```
* - `pos` is integrated from `vel` and `acc` and wrapped to +-pi with `mod()`; `vel` in rad/s mechanical. `pos_fb` is `pos` truncated to `fb_res` steps per turn.
*
* {{% hint warning %}}
* This is an incomplete, experimental model. `deadtime`, `drop`, `thermal_r`, `thermal_mass`, `temp`, `cur_res`, `cur_noise`, `fb_noise`, `fb_delay`, `curpid_mult` and `motsim_mult` have defaults but are not used by the code: there is no PWM distortion, thermal model, noise, current quantisation, feedback delay or sub-stepping. The integration step is simply the rt period, which is coarse for small `mot_l / mot_r`.
* {{% /hint %}}
*/

HAL_COMP(motsim);


// sim inputs
HAL_PIN(d_cmd);         // *input*, d command (A in current mode, V in voltage mode)
HAL_PIN(q_cmd);         // *input*, q command (A in current mode, V in voltage mode)

HAL_PIN(cmd_mode);      // *input*, 0 = voltage mode, 1 = current mode
HAL_PIN(en);            // *input*, 0 = no voltage on the motor

HAL_PIN(com_pos);       // *input*, commutation angle of the controller (rad electrical)
HAL_PIN(com_vel);       // *input*, electrical velocity for the controller feed forward (rad/s)

HAL_PIN(r);             // *parameter*, controller model resistance (Ohm), default 1
HAL_PIN(l);             // *parameter*, controller model inductance (H), default 0.001
HAL_PIN(psi);           // *parameter*, controller model flux linkage (Vs), default 0.055

HAL_PIN(load_torque);   // *input*, external load torque (Nm)


// sim outputs
HAL_PIN(id_fb);         // *output*, d current seen at com_pos (A)
HAL_PIN(ud_fb);         // *output*, d voltage command after limit and quantisation (V)
HAL_PIN(iq_fb);         // *output*, q current seen at com_pos (A)
HAL_PIN(uq_fb);         // *output*, q voltage command after limit and quantisation (V)

HAL_PIN(pos_fb);        // *output*, rotor position quantised to fb_res (rad)

// sim state
HAL_PIN(pos);           // *input/output*, simulated rotor position, wrapped to +-pi (rad)
HAL_PIN(vel);           // *input/output*, simulated rotor velocity (rad/s)
HAL_PIN(acc);           // *output*, simulated rotor acceleration (rad/s^2)
HAL_PIN(temp);          // *output*, motor temperature, set to 20 in init, not used
HAL_PIN(id);            // *input/output*, simulated d current in the rotor frame (A)
HAL_PIN(iq);            // *input/output*, simulated q current in the rotor frame (A)

// sim config
HAL_PIN(mot_r);         // *parameter*, motor resistance (Ohm), default 0.75
HAL_PIN(mot_l);         // *parameter*, motor inductance (H), default 0.0015
HAL_PIN(mot_psi);       // *parameter*, motor flux linkage (Vs), default 0.05

HAL_PIN(mot_pp);        // *parameter*, motor pole pairs, default 3
HAL_PIN(mot_j);         // *parameter*, rotor inertia (kg m^2), default 0.000025
HAL_PIN(mot_f);         // *parameter*, coulomb friction (Nm), default 0.01
HAL_PIN(mot_d);         // *parameter*, viscous damping (Nm s/rad), default 0.003

HAL_PIN(thermal_r);     // *parameter*, thermal resistance, not used
HAL_PIN(thermal_mass);  // *parameter*, thermal mass, not used

HAL_PIN(deadtime);      // *parameter*, pwm dead time, not used
HAL_PIN(drop);          // *parameter*, pwm drop, not used

HAL_PIN(pwm_volt);      // *parameter*, dc link / voltage limit (V), default 320
HAL_PIN(pwm_res);       // *parameter*, pwm resolution in steps of pwm_volt, default 4800

HAL_PIN(cur_res);       // *parameter*, current resolution, not used
HAL_PIN(cur_noise);     // *parameter*, current noise, not used

HAL_PIN(fb_res);        // *parameter*, position feedback steps per turn, default 4096
HAL_PIN(fb_noise);      // *parameter*, feedback noise, not used
HAL_PIN(fb_delay);      // *parameter*, feedback delay, not used

HAL_PIN(curpid_mult);   // *parameter*, current loop iterations per cycle, not used
HAL_PIN(motsim_mult);   // *parameter*, model iterations per current loop cycle, not used


HAL_PIN(id_error_sum);  // *output*, d current PI integrator (V)
HAL_PIN(iq_error_sum);  // *output*, q current PI integrator (V)
HAL_PIN(cur_bw);        // *parameter*, current loop bandwidth (rad/s), default 500


static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct motsim_ctx_t * ctx = (struct motsim_ctx_t *)ctx_ptr;
  struct motsim_pin_ctx_t *pins = (struct motsim_pin_ctx_t *)pin_ptr;
  PIN(r)                        = 1.0;
  PIN(l)                        = 0.001;
  PIN(psi)                      = 0.055;

  PIN(mot_r)   = 0.75;
  PIN(mot_l)   = 0.0015;
  PIN(mot_psi) = 0.05;

  PIN(mot_pp) = 3;
  PIN(mot_j)  = 0.000025;
  PIN(mot_f)  = 0.01;
  PIN(mot_d)  = 0.003;

  PIN(thermal_r)    = 0.1;
  PIN(thermal_mass) = 0.1;

  PIN(deadtime) = 0.0;
  PIN(drop)     = 0.0;

  PIN(pwm_volt) = 320.0;
  PIN(pwm_res)  = 4800;

  PIN(cur_res)   = 0.01;
  PIN(cur_noise) = 0.0;

  PIN(fb_res)   = 4096;
  PIN(fb_noise) = 0.0;
  PIN(fb_delay) = 0.0;

  PIN(curpid_mult) = 3;
  PIN(motsim_mult) = 3;

  PIN(cur_bw) = 500.0;

  PIN(temp) = 20.0;
}


static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct motsim_ctx_t * ctx = (struct motsim_ctx_t *)ctx_ptr;
  struct motsim_pin_ctx_t *pins = (struct motsim_pin_ctx_t *)pin_ptr;

  float ud = 0.0;
  float uq = 0.0;

  float si = 0.0;
  float co = 0.0;
  sincos_fast(mod(PIN(pos) * PIN(mot_pp)), &si, &co);

  // inverse park transformation
  float ia = PIN(id) * co - PIN(iq) * si;
  float ib = PIN(id) * si + PIN(iq) * co;

  sincos_fast(PIN(com_pos), &si, &co);

  // park transformation
  PIN(id_fb) = ia * co + ib * si;
  PIN(iq_fb) = -ia * si + ib * co;

  float kp;
  float ki;

  float id_error;
  float iq_error;

  float psi_d;
  float psi_q;

  float ind_d;
  float ind_q;

  switch((int)PIN(cmd_mode)) {
    case 0:  // voltage mode
      ud = PIN(d_cmd);
      uq = PIN(q_cmd);
      break;

    case 1:  // current mode
      kp = PIN(cur_bw) * PIN(l);
      ki = PIN(cur_bw) * PIN(r);

      id_error = PIN(d_cmd) - PIN(id_fb);
      iq_error = PIN(q_cmd) - PIN(iq_fb);

      psi_d = PIN(l) * PIN(id_fb) + PIN(psi);
      psi_q = PIN(l) * PIN(iq_fb);

      ind_d = PIN(com_vel) * psi_q;
      ind_q = PIN(com_vel) * psi_d;

      ud = LIMIT(PIN(r) * PIN(id_fb) - ind_d + kp * id_error, PIN(pwm_volt));
      uq = LIMIT(PIN(r) * PIN(iq_fb) + ind_q + kp * iq_error, PIN(pwm_volt));

      PIN(id_error_sum) = LIMIT(PIN(id_error_sum) + ki * id_error * period, PIN(pwm_volt) - ud);
      PIN(iq_error_sum) = LIMIT(PIN(iq_error_sum) + ki * iq_error * period, PIN(pwm_volt) - uq);
      ud += PIN(id_error_sum);
      uq += PIN(iq_error_sum);
      break;
  }

  // pwm resolution, limit
  ud = LIMIT((int)(ud / PIN(pwm_volt) * PIN(pwm_res)) / PIN(pwm_res) * PIN(pwm_volt), PIN(pwm_volt));
  uq = LIMIT((int)(uq / PIN(pwm_volt) * PIN(pwm_res)) / PIN(pwm_res) * PIN(pwm_volt), PIN(pwm_volt));

  PIN(ud_fb) = ud;
  PIN(uq_fb) = uq;

  // inverse park transformation
  float ua = ud * co - uq * si;
  float ub = ud * si + uq * co;

  si = 0.0;
  co = 0.0;
  sincos_fast(mod(PIN(pos) * PIN(mot_pp)), &si, &co);

  // park transformation
  ud = ua * co + ub * si;
  uq = -ua * si + ub * co;

  // pwm distortion, deadtime, drop
  //

  if(PIN(en) <= 0.0) {
    ud = 0.0;
    uq = 0.0;
  }

  psi_d = PIN(mot_l) * PIN(id) + PIN(mot_psi);
  psi_q = PIN(mot_l) * PIN(iq);

  ind_d = PIN(vel) * PIN(mot_pp) * psi_q;
  ind_q = PIN(vel) * PIN(mot_pp) * psi_d;

  PIN(id) += (ud - PIN(mot_r) * PIN(id) + ind_d) / PIN(mot_l) * period;
  PIN(iq) += (uq - PIN(mot_r) * PIN(iq) - ind_q) / PIN(mot_l) * period;

  float e_torque = 3.0 / 2.0 * PIN(mot_psi) * PIN(iq) * PIN(mot_pp);
  float m_torque = e_torque + PIN(load_torque) - PIN(vel) * PIN(mot_d) - SIGN2(PIN(vel), 0.1) * PIN(mot_f);
  PIN(acc)       = m_torque / PIN(mot_j);
  PIN(pos) += PIN(vel) * period + PIN(acc) * period * period / 2.0;
  PIN(pos) = mod(PIN(pos));
  PIN(vel) += PIN(acc) * period;

  PIN(pos_fb) = ((int)(PIN(pos) / 2.0 * M_1_PI * PIN(fb_res))) * 2.0 * M_PI / PIN(fb_res);
}


hal_comp_t motsim_comp_struct = {
    .name      = "motsim",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,  //sizeof(struct motsim_ctx_t),
    .pin_count = sizeof(struct motsim_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};