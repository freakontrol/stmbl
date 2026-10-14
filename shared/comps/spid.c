#include "spid_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `spid` is a generic single loop PID controller with feedforward, output limits, dynamic anti windup and an output offset. It works on any unit. It is compiled into the F4 firmware but not loaded by any template or config in `conf/`.
*
* ## Component Explanation
* All work is done in `rt`. There is no `nrt_init`, so all gains and limits start at 0.
*
* 1. **Error**:
* - `error = cmd - fb` (not wrapped), limited to +-`max_error` when `max_error > 0`.
* - `cmd_d` and `error_d` are the derivatives of `cmd` and `error` over one `period`.
*
* 2. **Proportional / feedforward part**:
* ```c
* output = cmd * kff0 + cmd_d * kff1 + error * kp + error_d * kd;
* output = CLAMP(output, min_output - offset, max_output - offset);
* ```
*
* 3. **Integrator** (`error_sum`, internal):
* - `error_sum += error * ki * period + error_d * kdi * period`.
* - `ksdi` adds `error_d / abs(error) * ksdi * period`, only while the error is larger than 0.1 % of the output range divided by `ksdi`.
* - The integrator is clamped to the headroom left by the P output (dynamic anti windup) and then added to the output.
*
* 4. **Enable, saturation and offset**:
* - With `en <= 0` the output and integrator are 0, but `offset` is still added, so the output is `offset` while disabled.
* - `sat` counts up by `period` while the output is at 99 % of `min_output` or `max_output`, and is reset to 0 immediately otherwise.
* - `output = output + offset`.
*
* {{% hint warning %}}
* The "scaled differential" `ksd` term (`error_d / abs(error) * ksd`) is added to the integrator, not to the output, and without `period`, so it does not behave like a differential term. The 99 % saturation test assumes `min_output < 0 < max_output`. Treat `ksd`/`ksdi` as experimental.
* {{% /hint %}}
*/

HAL_COMP(spid);

// input
HAL_PIN(cmd);  // *input*, Command
HAL_PIN(fb);   // *input*, Feedback
HAL_PIN(en);   // *input*, Enable, 0 clears output and integrator

// gains
HAL_PIN(kp);      // *parameter*, Proportional gain
HAL_PIN(ki);      // *parameter*, Integral gain (1/s)
HAL_PIN(kd);      // *parameter*, Differential gain (s)
HAL_PIN(ksd);     // *parameter*, Scaled differential gain (experimental, feeds the integrator)
HAL_PIN(kdi);     // *parameter*, Differential integrator gain
HAL_PIN(ksdi);    // *parameter*, Scaled differential integrator gain (experimental)
HAL_PIN(kff0);    // *parameter*, Feedforward gain on cmd
HAL_PIN(kff1);    // *parameter*, Feedforward gain on the derivative of cmd (s)
HAL_PIN(offset);  // *parameter*, Constant added to the output

HAL_PIN(min_output);  // *parameter*, Minimum output
HAL_PIN(max_output);  // *parameter*, Maximum output
HAL_PIN(max_error);   // *parameter*, Error limit, 0 = no limit

// output
HAL_PIN(output);  // *output*, Controller output
HAL_PIN(error);   // *output*, Limited error cmd - fb
HAL_PIN(sat);     // *output*, Time the output is saturated (s)

struct spid_ctx_t {
  float error_sum;
  float last_error;
  float last_cmd;
};

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct spid_ctx_t *ctx      = (struct spid_ctx_t *)ctx_ptr;
  struct spid_pin_ctx_t *pins = (struct spid_pin_ctx_t *)pin_ptr;

  float offset    = PIN(offset);
  float min       = PIN(min_output) - offset;
  float max       = PIN(max_output) - offset;
  float max_error = PIN(max_error);

  float cmd   = PIN(cmd);
  float cmd_d = (cmd - ctx->last_cmd) / period;
  float error = cmd - PIN(fb);
  if(max_error > 0.0) {
    error = LIMIT(error, max_error);
  }
  float error_d = (error - ctx->last_error) / period;

  float output = 0.0;
  output += cmd * PIN(kff0);    // feedforward 0
  output += cmd_d * PIN(kff1);  // feedforward 1
  output += error * PIN(kp);    // porportional
  output += error_d * PIN(kd);  // differential
  if(PIN(ksd) != 0.0 && ABS(error) > (max - min) / PIN(ksd) * 0.001) {
    ctx->error_sum += error_d / ABS(error) * PIN(ksd);  // scalded differential
  }
  output = CLAMP(output, min, max);

  ctx->error_sum += error * PIN(ki) * period;     // integrator
  ctx->error_sum += error_d * PIN(kdi) * period;  // differential integrator
  if(PIN(ksdi) != 0.0 && ABS(error) > (max - min) / PIN(ksdi) * 0.001) {
    ctx->error_sum += error_d / ABS(error) * PIN(ksdi) * period;  // scalded differential integrator
  }
  ctx->error_sum = CLAMP(ctx->error_sum, min - output, max - output);  // dynamic anti windup

  output += ctx->error_sum;

  if(PIN(en) <= 0.0) {
    output         = 0.0;
    ctx->error_sum = 0.0;
  }

  if(output <= min * 0.99 || output >= max * 0.99) {
    PIN(sat) += period;
  } else {
    PIN(sat) = 0.0;
  }

  output += offset;

  PIN(output) = output;

  PIN(error)      = error;
  ctx->last_error = error;
  ctx->last_cmd   = cmd;
}

hal_comp_t spid_comp_struct = {
    .name      = "spid",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct spid_ctx_t),
    .pin_count = sizeof(struct spid_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
