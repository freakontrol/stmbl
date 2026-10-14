#include "stp_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `stp` is a simple point to point trajectory planner with velocity and acceleration limits, soft limits and a jog input. F4 component, used in `conf/template/jog_cmd.txt` (`stp0.jog = jog0.jog`, `rev0.in = stp0.mpos`) and `conf/festo.txt`.
*
* ## Component Explanation
* All work is done in `rt`. Positions are not wrapped, only `mpos` is.
*
* 1. **Target generation**:
* - `target` is moved by `vel_ext_cmd + jog * max_vel` (limited to 0.99 * `max_vel`) and `acc_ext_cmd` (limited to 0.99 * `max_acc`). `vel_ext_cmd` itself is integrated with `acc_ext_cmd` and written back.
* - `target` is clamped to `min_pos`..`max_pos`.
*
* 2. **Planning** (every period):
* - `dtg = target - pos`. The time to stop at the target from standstill with `max_acc` is `sqrt(2 |dtg| / max_acc)`, rounded up to whole periods (`ttg`).
* - The velocity for that time is limited to `max_vel`; the acceleration needed to reach it in one period is limited to `max_acc` and output as `acc_cmd`.
* - `pos` and `vel_cmd` are integrated with `acc_cmd`.
*
* 3. **Target reached**:
* - When less than one period to go and `|vel_cmd|` < `max_acc * period`, `pos` is set to `target` and `vel_cmd` to 0.
* - `at_target` = 1 when at most one period to go and `|vel_cmd|` < `max_vel / 10000`.
*/

HAL_COMP(stp);

HAL_PIN(target);       // *input/output*, Target position, clamped and moved by jog (rad)
HAL_PIN(vel_ext_cmd);  // *input/output*, External velocity, integrated from acc_ext_cmd (rad/s)
HAL_PIN(acc_ext_cmd);  // *input*, External acceleration (rad/s^2)
HAL_PIN(jog);          // *input*, Jog -1..1, multiplied with max_vel

HAL_PIN(pos);      // *output*, Planned position, not wrapped (rad)
HAL_PIN(mpos);     // *output*, Planned position, wrapped (rad, +-pi)
HAL_PIN(vel_cmd);  // *output*, Planned velocity (rad/s)
HAL_PIN(acc_cmd);  // *output*, Planned acceleration (rad/s^2)

HAL_PIN(max_pos);  // *parameter*, Upper soft limit (rad), default 20pi
HAL_PIN(min_pos);  // *parameter*, Lower soft limit (rad), default -20pi
HAL_PIN(max_vel);  // *parameter*, Maximum velocity (rad/s), default 2pi
HAL_PIN(max_acc);  // *parameter*, Maximum acceleration (rad/s^2), default 20pi

HAL_PIN(dtg);        // *output*, Distance to go (rad)
HAL_PIN(ttg);        // *output*, Time to go (s)
HAL_PIN(at_target);  // *output*, 1 = target reached

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct stp_ctx_t *ctx      = (struct stp_ctx_t *)ctx_ptr;
  struct stp_pin_ctx_t *pins = (struct stp_pin_ctx_t *)pin_ptr;
  PIN(target)                = 0.0;
  PIN(max_vel)               = 1.0 * 2.0 * M_PI;
  PIN(max_acc)               = 10.0 * 2.0 * M_PI;
  PIN(max_pos)               = 10.0 * 2.0 * M_PI;
  PIN(min_pos)               = -10.0 * 2.0 * M_PI;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct stp_ctx_t *ctx      = (struct stp_ctx_t *)ctx_ptr;
  struct stp_pin_ctx_t *pins = (struct stp_pin_ctx_t *)pin_ptr;

  float max_acc = MAX(PIN(max_acc), 0.01);
  float max_vel = MAX(PIN(max_vel), 0.01);

  // jog input
  float acc_ext_cmd = LIMIT(PIN(acc_ext_cmd), PIN(max_acc) * 0.99);
  float vel_ext_cmd = LIMIT(PIN(vel_ext_cmd) + PIN(jog) * max_vel, PIN(max_vel) * 0.99);
  PIN(target) += vel_ext_cmd * period + acc_ext_cmd * period * period / 2.0;
  PIN(vel_ext_cmd) += acc_ext_cmd * period;
  PIN(vel_ext_cmd) = LIMIT(PIN(vel_ext_cmd), max_vel * 0.99);

  // pos input
  float target = CLAMP(PIN(target), PIN(min_pos), PIN(max_pos));
  PIN(target)  = target;

  // update
  PIN(pos) += PIN(vel_cmd) * period + PIN(acc_cmd) * period * period / 2.0;
  PIN(vel_cmd) += PIN(acc_cmd) * period;

  // distance to go
  float to_go = target - PIN(pos);

  // time to go
  float time_to_go = sqrtf(2.0 * ABS(to_go) / max_acc);

  // real time to go
  int periods_to_go = ceilf(time_to_go / period);

  // calc new acc
  float acc = 0.0;
  if(periods_to_go) {
    acc = 2.0 * to_go / (periods_to_go * periods_to_go * period * period);
  }

  float vel = acc * periods_to_go * period;
  vel       = LIMIT(vel, max_vel);
  acc       = (vel - PIN(vel_cmd)) / period;
  acc       = LIMIT(acc, max_acc);

  if(time_to_go < period && ABS(PIN(vel_cmd)) < max_acc * period) {
    acc          = 0.0;
    PIN(vel_cmd) = 0.0;
    PIN(pos)     = PIN(target);
  }

  PIN(acc_cmd) = acc;
  PIN(dtg)     = to_go;
  PIN(ttg)     = periods_to_go * period;
  PIN(mpos)    = mod(PIN(pos));

  if((periods_to_go <= 1) & (ABS(PIN(vel_cmd)) < max_vel / 10000.0)) {
    PIN(at_target) = 1.0;
  } else {
    PIN(at_target) = 0.0;
  }
}

const hal_comp_t stp_comp_struct = {
    .name      = "stp",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct stp_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
