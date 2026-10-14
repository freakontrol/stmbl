#include "ypid_comp.h"
/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2017 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2017 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// #include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `ypid` is a simple cascaded position (P) / velocity (PI) controller whose output `out` has no fixed unit (for example a current or torque command). It is compiled into the F4 firmware but not loaded by any template or config in `conf/` (the F3 build has it commented out). For normal servo use see `pid`.
*
* ## Component Explanation
* All work is done in `rt`. There is no `nrt_init`: all pins start at 0. Negative gains are treated as 0.
*
* 1. **Position loop**:
* - `pos_error = minus(pos_ext_cmd, pos_fb)` is always computed, also when disabled.
* - `vel_cmd = pos_p * pos_error + vel_ff * vel_ext_cmd`.
* - `vel_sat` is set to +-1 if this exceeds +-`max_vel`.
* - `vel_cmd` is then clamped to `max_vel` and to `vel_fb +- max_acc * period`, so it cannot run away from the actual velocity by more than one period of `max_acc`.
*
* 2. **Velocity loop**:
* - `vel_error = vel_cmd - vel_fb`, set to 0 when `abs(vel_error) < vel_min` (dead band).
* - `out = LIMIT(vel_error * vel_p, max_out)`, plus the integrator `vel_error_sum * vel_i * vel_p`.
* - The integrator sums `vel_error` every rt cycle without multiplying by `period`, so the effective I gain depends on the rt frequency. It is clamped so the total stays within +-`max_out`, and cleared when `vel_i * vel_p` is 0.
* - `out_sat` is +-1 when `out` exceeds 99 % of `max_out`; `out` is finally limited to +-`max_out`.
*
* 3. **Saturation and enable**:
* - `saturated` counts the time (s) either `vel_sat` or `out_sat` is non zero and resets to 0 otherwise.
* - With `enable <= 0` all outputs except `pos_error` are 0 and the integrator is cleared.
*/

HAL_COMP(ypid);

HAL_PIN(pos_ext_cmd);  // *input*, Position command (rad)
HAL_PIN(pos_fb);       // *input*, Position feedback (rad)
HAL_PIN(pos_error);    // *output*, Position error wrapped to +-pi (rad)

HAL_PIN(vel_ext_cmd);  // *input*, Velocity feedforward command (rad/s)
HAL_PIN(vel_fb);       // *input*, Velocity feedback (rad/s)
HAL_PIN(vel_cmd);      // *output*, Clamped velocity command (rad/s)
HAL_PIN(vel_error);    // *output*, Velocity error after dead band (rad/s)
HAL_PIN(vel_min);      // *parameter*, Velocity error dead band (rad/s), e.g. 0.3

HAL_PIN(enable);  // *input*, Enable
HAL_PIN(out);     // *output*, Controller output, limited to max_out

HAL_PIN(pos_p);  // *parameter*, Position P gain (1/s), e.g. 10

HAL_PIN(vel_p);   // *parameter*, Velocity P gain, e.g. 0.5
HAL_PIN(vel_i);   // *parameter*, Velocity I gain per rt cycle, e.g. 0.005
HAL_PIN(vel_ff);  // *parameter*, Velocity feedforward gain, e.g. 1.0

// system limits
HAL_PIN(max_vel);  // *parameter*, Maximum velocity (rad/s)
HAL_PIN(max_acc);  // *parameter*, Maximum acceleration (rad/s^2)
HAL_PIN(max_out);  // *parameter*, Maximum output

HAL_PIN(vel_sat);    // *output*, Velocity command saturated (-1, 0, 1)
HAL_PIN(out_sat);    // *output*, Output saturated (-1, 0, 1)
HAL_PIN(saturated);  // *output*, Time in saturation (s)

struct ypid_ctx_t {
  float sat;
  float vel_error_sum;
};

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ypid_ctx_t *ctx      = (struct ypid_ctx_t *)ctx_ptr;
  struct ypid_pin_ctx_t *pins = (struct ypid_pin_ctx_t *)pin_ptr;

  float vel_cmd;
  float vel_fb = PIN(vel_fb);

  float vel_error;
  float vel_sat;
  float out_sat;

  float pos_error = minus(PIN(pos_ext_cmd), PIN(pos_fb));

  float pos_p = MAX(PIN(pos_p), 0.0);
  float vel_p = MAX(PIN(vel_p), 0.0);
  float vel_i = MAX(PIN(vel_i), 0.0);

  float velmin = MAX(-PIN(max_vel), vel_fb - PIN(max_acc) * period);
  float velmax = MIN(PIN(max_vel), vel_fb + PIN(max_acc) * period);

  float out;

  if(PIN(enable) > 0.0) {
    // pos -> vel
    vel_cmd = pos_p * pos_error + PIN(vel_ff) * PIN(vel_ext_cmd);
    vel_sat = SAT(vel_cmd, PIN(max_vel));
    vel_cmd = CLAMP(vel_cmd, velmin, velmax);  // min/max clamping

    // vel -> out
    vel_error = vel_cmd - vel_fb;
    if(ABS(vel_error) < PIN(vel_min)) {
      vel_error = 0;
    }

    ctx->vel_error_sum += vel_error;
    out = LIMIT(vel_error * vel_p, PIN(max_out));
    if(vel_i * vel_p > 0.0f) {
      ctx->vel_error_sum = CLAMP(ctx->vel_error_sum, (-PIN(max_out) - out) / (vel_i * vel_p), (PIN(max_out) - out) / (vel_i * vel_p));
    } else {
      ctx->vel_error_sum = 0;
    }
    out += ctx->vel_error_sum * vel_i * vel_p;
    out_sat = SAT(out, PIN(max_out) * 0.99);
    out     = LIMIT(out, PIN(max_out));

    // sat
    if(ABS(vel_sat) + ABS(out_sat) > 0.0) {
      ctx->sat += period;
    } else {
      ctx->sat = 0.0;
    }
  } else {
    ctx->vel_error_sum = 0.0;
    vel_cmd            = 0.0;
    vel_error          = 0.0;
    vel_sat            = 0.0;
    out_sat            = 0.0;
    ctx->sat           = 0.0;
    out                = 0.0;
  }

  PIN(pos_error) = pos_error;
  PIN(vel_cmd)   = vel_cmd;
  PIN(vel_error) = vel_error;
  PIN(out)       = out;

  PIN(vel_sat)   = vel_sat;
  PIN(out_sat)   = out_sat;
  PIN(saturated) = ctx->sat;
}


hal_comp_t ypid_comp_struct = {
    .name      = "ypid",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct ypid_ctx_t),
    .pin_count = sizeof(struct ypid_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
