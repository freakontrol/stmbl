#include "curpid_comp.h"
#include "commands.h"
#include "common.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `curpid` is the d/q current controller. From a d/q current command and the measured d/q currents it computes the d/q voltages with a PI controller plus resistance, back EMF and cross coupling feed forward and a predictor for the PWM delay. It also limits the current command to `max_cur`. In voltage mode it passes the command through as a voltage. It runs on the F3 (HV board) as `curpid0`, loaded by `stm32f303/src/main.c` (rt_prio 3). There all its inputs come from `ls0` (the F4's command and motor config) and `dq0` (measured currents), and `ud`/`uq` go to `idq0`. No F4 template loads it.
*
* ## Component Explanation
*
* 1. **Command limit** (rt):
* - Current mode (`cmd_mode` = 1): `scale = max_cur / |i_cmd|` when the command vector (`id_cmd`, `iq_cmd`) is longer than `max_cur`, else 1, so the command is scaled down to at most `max_cur` (A peak). The square root is only taken above the limit (`sqrtf` compiles to the FPU's vsqrt, the builds do not use -fno-builtin).
* - Voltage mode (`cmd_mode` = 0): `id_cmd`/`iq_cmd` are voltages. Two limits, the lower one wins: `scale = MIN(volt_scale, cur_scale)`.
* - `volt_scale = sqrt(pwm_volt^2 / |u_cmd|^2)`, clamped to 1, a plain clamp of the command to `pwm_volt` (commands below 0.1 `pwm_volt` count as 0.1 `pwm_volt`).
* - `cur_scale` (internal, clamped to 0..1) integrates the per unit overshoot of the measured current: `cur_scale += (1 - |i_fb|^2 / max_cur^2) * kci * period`. It drops while the current is above `max_cur` and recovers to 1 below it, so `kci` [1/s] means the same on a 2 A and a 27 A motor. It is reset to 1 in current mode.
* - The command is multiplied by `scale`.
*
* 2. **Predictor** (rt):
* - To cancel one tick of PWM delay, the measured current is moved one period ahead with the motor model, using the voltage output last tick:
* ```c
* id += (ud - r * id + vel * lq * iq) / ld * period * ksp;
* iq += (uq - r * iq - vel * (ld * id + psi)) / lq * period * ksp;
* ```
* - `1 / ld` and `1 / lq` are cached and only recomputed when the pins change.
*
* 3. **PI controller** (rt):
* - Proportional part plus feed forward, each axis first limited to +-`pwm_volt`:
* ```c
* ud = ff * r * id_cmd - kind * vel * lq * iq + cur_bw * ld * id_error;
* uq = ff * r * iq_cmd + kind * vel * (ld * id + psi) + cur_bw * lq * iq_error;
* ```
* - Integral part: `sum += cur_bw * r * error * period`, limited to +-(`pwm_volt` - proportional part), and added to the output. So the zero of the PI sits at `r / l`, and `cur_bw` is the closed loop bandwidth (rad/s).
* - `id_error`/`iq_error` show the error against the predicted current.
*
* 4. **Voltage vector limit** (rt):
* - The sum is then limited as one vector of length `pwm_volt`, d first: `ud` is clamped to +-`pwm_volt`, and `uq` to what is left of the circle, `sqrt(pwm_volt^2 - ud^2)`. So d keeps what it needs for the flux (and field weakening), q gets the rest.
* - Anti-windup by back-calculation: each integrator takes back what the limit cut from its axis, so it holds at the limit instead of winding up.
* - `ud`/`uq` are the limited vector, which is what the PWM stage can pass, so the predictor, the integrators and the F4 (via `ls0.ud_fb/uq_fb`) all see the voltage actually applied. On the F3 `pwm_volt` is `duty_max * udc / sqrt(3)`, inside the SVM hexagon.
*
* 5. **Modes and enable** (rt):
* - In voltage mode `ud`/`uq` are the scaled command (not passed through the vector limit), and the integrators and errors are 0.
* - With `en` <= 0, `ud`/`uq` are 0 and the integrators are reset.
*
* 6. **Parameters**:
* - nrt_init defaults: `kci` 2000 1/s (a 1.3x overshoot cuts the voltage in about 1 ms at 15 kHz), `scale` 1. `ff` and `kind` default to 0.
* - On the F3, `r`, `ld` (= `l`), `lq`, `psi`, `cur_bw`, `ff` (= `cur_ff`), `kind` (= `cur_ind`) and `max_cur` come from the F4 via `ls0`, and `pwm_volt` from `ls0` (DC link voltage, phase mode and `hv0.duty_max`).
*
* {{% hint warning %}}
* - The vector limit gives d priority: a large d demand (e.g. a d current step) can leave q with little or no voltage for that time.
* {{% /hint %}}
*/

HAL_COMP(curpid);

// enable
HAL_PIN(en);        // *input*, Enable, outputs 0 and integrators reset when <= 0
HAL_PIN(cmd_mode);  // *input*, Command mode, 0 = voltage (commands are V), 1 = current (commands are A)

// current command
HAL_PIN(id_cmd);  // *input*, D-axis command, current (A) or voltage (V) depending on cmd_mode
HAL_PIN(iq_cmd);  // *input*, Q-axis command, current (A) or voltage (V) depending on cmd_mode

// current feedback
HAL_PIN(id_fb);  // *input*, Measured d-axis current (A)
HAL_PIN(iq_fb);  // *input*, Measured q-axis current (A)

// HAL_PIN(ac_current);

// voltage output
HAL_PIN(ud);  // *output*, D-axis voltage (V), vector limited, also read back by the predictor
HAL_PIN(uq);  // *output*, Q-axis voltage (V), vector limited, also read back by the predictor

// maximum output current and voltage
HAL_PIN(max_cur);   // *input*, Maximum current (A), limits the command, min 0.01
HAL_PIN(pwm_volt);  // *input*, Maximum length of the output voltage vector (V)

// d, q resistance and inductance
HAL_PIN(r);   // *parameter*, Winding resistance (Ohm), min 0.1, default 0.5
HAL_PIN(ld);  // *parameter*, D-axis inductance (H), min 0.001, default 0.01
HAL_PIN(lq);  // *parameter*, Q-axis inductance (H), min 0.001, default 0.01

// torque constant
HAL_PIN(psi);  // *parameter*, Magnet flux linkage (Vs/rad), default 0.05

HAL_PIN(ff);      // *parameter*, Resistance feed forward factor, 0 = off, 1 = full
HAL_PIN(cur_bw);  // *parameter*, Current loop bandwidth (rad/s), default 250
HAL_PIN(ksp);     // *parameter*, Predictor gain for the PWM delay, 0 = off, default 1
HAL_PIN(kind);    // *parameter*, Back EMF and cross coupling feed forward factor, 0 = off, 1 = full
HAL_PIN(kci);     // *parameter*, Voltage mode current limit integrator gain, per unit (1/s), default 2000

HAL_PIN(scale);  // *output*, Factor applied to the command by the current or voltage limit (0..1)

HAL_PIN(vel);  // *input*, Electrical velocity (rad/s)

// current error outputs
HAL_PIN(id_error);  // *output*, D-axis current error (A), 0 in voltage mode
HAL_PIN(iq_error);  // *output*, Q-axis current error (A), 0 in voltage mode

struct curpid_ctx_t {
  float id_error_sum;
  float iq_error_sum;
  float cur_scale;  // volt mode current limit, per unit
  float ld_last, ld_inv;  // 1 / ld, 1 / lq recomputed only when the pin changes:
  float lq_last, lq_inv;  // two divisions a tick otherwise, ~14 cycles each
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct curpid_ctx_t *ctx      = (struct curpid_ctx_t *)ctx_ptr;
  struct curpid_pin_ctx_t *pins = (struct curpid_pin_ctx_t *)pin_ptr;

  ctx->cur_scale = 1.0;
  PIN(r)      = 0.5;
  PIN(ld)     = 0.01;
  PIN(lq)     = 0.01;
  PIN(psi)    = 0.05;
  PIN(cur_bw) = 250.0;
  PIN(kci)    = 2000.0;  // per unit [1/s]: 1.3x overshoot cuts the voltage in ~1 ms at 15 kHz
  PIN(ksp)    = 1.0;
  PIN(scale)  = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct curpid_ctx_t *ctx      = (struct curpid_ctx_t *)ctx_ptr;
  struct curpid_pin_ctx_t *pins = (struct curpid_pin_ctx_t *)pin_ptr;

  float r  = MAX(PIN(r), 0.1);
  float ld = MAX(PIN(ld), 0.001);
  float lq = MAX(PIN(lq), 0.001);
  if(ld != ctx->ld_last) {
    ctx->ld_last = ld;
    ctx->ld_inv  = 1.0 / ld;
  }
  if(lq != ctx->lq_last) {
    ctx->lq_last = lq;
    ctx->lq_inv  = 1.0 / lq;
  }

  float ff   = PIN(ff);
  float kind = PIN(kind);

  float max_cur = MAX(PIN(max_cur), 0.01);
  float idc     = PIN(id_cmd);
  float iqc     = PIN(iq_cmd);

  float max_volt = PIN(pwm_volt);

  float id = PIN(id_fb);
  float iq = PIN(iq_fb);

  // float ac_current = id * id + iq * iq;  //sqrtf(id * id + iq * iq);
  // PIN(ac_current)  = ac_current;

  float abscur;
  float absvolt;

  if(PIN(cmd_mode) == VOLT_MODE) {
    // Two limits, the lower one wins. The voltage one is a plain clamp of
    // the command. The current one integrates the feedback's overshoot in per
    // unit, so kci [1/s] means the same on a 2 A and a 27 A motor (an
    // integral in A^2 swings the voltage between 0 and full every tick on a
    // spindle).
    absvolt          = idc * idc + iqc * iqc;  // clamp cmd
    float volt_scale = sqrtf(CLAMP(max_volt * max_volt / MAX(absvolt, max_volt * max_volt * 0.01), 0.0, 1.0));

    abscur = id * id + iq * iq;  // clamp over fb
    ctx->cur_scale += (1.0 - abscur / (max_cur * max_cur)) * PIN(kci) * period;
    ctx->cur_scale = CLAMP(ctx->cur_scale, 0.0, 1.0);

    PIN(scale) = MIN(volt_scale, ctx->cur_scale);
  } else {
    ctx->cur_scale = 1.0;
    // clamp cmd
    abscur     = idc * idc + iqc * iqc;
    PIN(scale) = abscur > max_cur * max_cur ? max_cur / sqrtf(abscur) : 1.0;
  }
  PIN(scale) = CLAMP(PIN(scale), 0.0, 1.0);

  idc *= PIN(scale);
  iqc *= PIN(scale);

  float vel   = PIN(vel);
  float psi_d = ld * id + PIN(psi);
  float psi_q = lq * iq;
  float indd  = vel * psi_q;
  float indq  = vel * psi_d;

  // predictor to cancel pwm delay
  id += (PIN(ud) - r * id + indd) * ctx->ld_inv * period * PIN(ksp);
  iq += (PIN(uq) - r * iq - indq) * ctx->lq_inv * period * PIN(ksp);

  float id_error = idc - id;
  float iq_error = iqc - iq;

  float ud = LIMIT(ff * r * idc - kind * indd + PIN(cur_bw) * ld * id_error, max_volt);
  float uq = LIMIT(ff * r * iqc + kind * indq + PIN(cur_bw) * lq * iq_error, max_volt);

  if(PIN(cur_bw) * r > 0.0) {
    ctx->id_error_sum = LIMIT(ctx->id_error_sum + PIN(cur_bw) * r * id_error * period, max_volt - ud);
    ctx->iq_error_sum = LIMIT(ctx->iq_error_sum + PIN(cur_bw) * r * iq_error * period, max_volt - uq);
  } else {
    ctx->id_error_sum = 0.0;
    ctx->iq_error_sum = 0.0;
  }

  ud += ctx->id_error_sum;
  uq += ctx->iq_error_sum;

  // One voltage vector, limited as a vector with d first: per-axis clamps
  // would let |u| reach sqrt(2) pwm_volt, which hv0 would then scale down in
  // the phase domain behind the loop's back (the predictor, the integrators
  // and ud_fb/uq_fb would all see the unlimited vector). d keeps what it
  // needs for the flux, q gets the rest of the circle. Back-calculation: an integrator
  // takes whatever the limit cut from its axis, so it holds at the limit
  // instead of winding up.
  float ud_lim = LIMIT(ud, max_volt);
  float uq_max = sqrtf(MAX(max_volt * max_volt - ud_lim * ud_lim, 0.0));
  float uq_lim = LIMIT(uq, uq_max);
  ctx->id_error_sum += ud_lim - ud;
  ctx->iq_error_sum += uq_lim - uq;
  ud = ud_lim;
  uq = uq_lim;

  if(PIN(cmd_mode) == VOLT_MODE) {
    ud                = idc;
    uq                = iqc;
    ctx->id_error_sum = 0.0;
    ctx->iq_error_sum = 0.0;
    id_error          = 0.0;
    iq_error          = 0.0;
  }

  if(PIN(en) <= 0.0) {
    ud                = 0.0;
    uq                = 0.0;
    ctx->id_error_sum = 0.0;
    ctx->iq_error_sum = 0.0;
  }

  PIN(ud) = ud;
  PIN(uq) = uq;

  PIN(id_error) = id_error;
  PIN(iq_error) = iq_error;
}

hal_comp_t curpid_comp_struct = {
    .name      = "curpid",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct curpid_ctx_t),
    .pin_count = sizeof(struct curpid_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
