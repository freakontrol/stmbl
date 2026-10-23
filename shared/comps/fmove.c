#include "fmove_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `fmove` implements force guided movement (admittance control): a measured force moves an axis like a virtual mass with damping and friction, and after the force is released the axis moves automatically to a target. F4 component, used in `conf/move.txt` with a load cell (`fmove0.force_in = hx0.out0`, `fmove0.en = home0.en_out`, `home0.pos_in = fmove0.mpos`).
*
* ## Component Explanation
* Motion is calculated in `rt`, `nrt` prints the status. `pos`, `vel`, `acc` and the limits are in axis units (e.g. m), `scale` converts to motor angle. Default `scale` = 1000 / 25 * 2pi * 2.
*
* 1. **Force**:
* ```c
* force = force_in - gravity * real_mass - acc * real_mass - force_offset;
* force_offset += force * force_offset_lpf * period;   // slow zero drift compensation
* force -= SIGN(vel) * friction + vel * damping;
* ```
*
* 2. **Mode switching**:
* - `|force|` > `force_th` switches to `mode` 1 (force move).
* - The mode stays 1 while `|vel|` > 5% of `max_usr_vel`. After `force_time` seconds without force and movement it returns to `mode` 0 (auto move).
*
* 3. **Movement**:
* - Mode 0: moves to `target` (clamped to the limits) with a `sqrt(2 * max_acc * distance)` profile, limited by `max_acc` and `max_vel`.
* - Mode 1: `acc = force / virtual_mass`, limited by `max_usr_acc`, integrated to `vel`, limited by `max_usr_vel`.
* - In both modes the velocity is limited so that the axis can stop before `min_pos` and `max_pos`, and `pos` is clamped to them.
* - `mpos = mod(pos * scale)` is the motor angle.
*
* 4. **Disable**:
* - With `en` <= 0, `pos`, `mpos` and `vel` are 0 and `force_offset` adapts faster (rate 0.5 1/s). The position is therefore relative to where the axis was enabled; use `home` for an absolute reference.
*
* 5. **Status printing** (`nrt`):
* - Prints `force_move <pos>`, `on_target <pos>` or `moving <pos>`.
*/

HAL_COMP(fmove);

HAL_PIN(en);  // *input*, Enable, position is reset to 0 while <= 0

HAL_PIN(gravity);       // *parameter*, Gravity acceleration for weight compensation, default 0
HAL_PIN(real_mass);     // *parameter*, Moved mass for weight and inertia compensation, default 0
HAL_PIN(virtual_mass);  // *parameter*, Simulated mass in force mode, default 1
HAL_PIN(damping);       // *parameter*, Simulated viscous damping (force per velocity), default 1
HAL_PIN(friction);      // *parameter*, Simulated coulomb friction (force), default 0.01

HAL_PIN(min_pos);      // *parameter*, Lower position limit, default 0
HAL_PIN(max_pos);      // *parameter*, Upper position limit, default 0.5
HAL_PIN(max_vel);      // *parameter*, Maximum velocity in auto mode, default 0.1
HAL_PIN(max_acc);      // *parameter*, Maximum acceleration in auto mode, default 1
HAL_PIN(max_usr_vel);  // *parameter*, Maximum velocity in force mode, default 0.1
HAL_PIN(max_usr_acc);  // *parameter*, Maximum acceleration in force mode, default 2
HAL_PIN(scale);        // *parameter*, Motor angle per position unit (rad/unit)

HAL_PIN(force_in);          // *input*, Measured force
HAL_PIN(force);             // *output*, Compensated force
HAL_PIN(force_offset);      // *input/output*, Adapted force zero offset
HAL_PIN(force_offset_lpf);  // *parameter*, Adaption rate of force_offset (1/s), default 0.01
HAL_PIN(pos);               // *output*, Position (units)
HAL_PIN(mpos);              // *output*, Motor angle mod(pos * scale) (rad, +-pi)
HAL_PIN(vel);               // *output*, Velocity (units/s)
HAL_PIN(vel_old);           // *output*, Velocity of the last period (internal state)
HAL_PIN(acc);               // *output*, Acceleration (units/s^2)
HAL_PIN(target);            // *input/output*, Target for auto mode, clamped to the limits

HAL_PIN(force_time);   // *parameter*, Time without force before auto mode (s), default 5
HAL_PIN(force_timer);  // *output*, Time since the last force or movement (s)
HAL_PIN(force_th);     // *parameter*, Force threshold for force mode, default 0.1

HAL_PIN(print_freq);   // *parameter*, Status print frequency (Hz), 0 = off
HAL_PIN(print_timer);  // *output*, Print timer (internal state, s)

HAL_PIN(mode);  // *output*, 0 = auto move, 1 = force move

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fmove_pin_ctx_t *pins = (struct fmove_pin_ctx_t *)pin_ptr;
  PIN(force_time)              = 5.0;
  PIN(max_acc)                 = 1;
  PIN(max_vel)                 = 0.1;
  PIN(max_usr_acc)             = 2;
  PIN(max_usr_vel)             = 0.1;
  PIN(min_pos)                 = 0.0;
  PIN(max_pos)                 = 0.5;
  PIN(real_mass)               = 0.0;
  PIN(virtual_mass)            = 1.0;
  PIN(damping)                 = 1.0;
  PIN(friction)                = 0.01;
  PIN(scale)                   = 1000.0 / 25.0 * 2.0 * M_PI * 2.0;
  PIN(force_th)                = 0.1;
  PIN(gravity)                 = 0.0;
  PIN(force_offset_lpf)        = 0.01;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fmove_pin_ctx_t *pins = (struct fmove_pin_ctx_t *)pin_ptr;

  PIN(vel_old) = PIN(vel);

  PIN(force) = PIN(force_in) - PIN(gravity) * PIN(real_mass) - PIN(acc) * PIN(real_mass);
  PIN(force) -= PIN(force_offset);
  PIN(force_offset) += PIN(force) * PIN(force_offset_lpf) * period;
  PIN(force) -= SIGN(PIN(vel)) * PIN(friction) + PIN(vel) * PIN(damping);

  PIN(force_timer) += period;
  if(ABS(PIN(force)) > PIN(force_th)) {
    PIN(force_timer) = 0.0;
    PIN(mode)        = 1;
  } else if(PIN(mode) > 0 && ABS(PIN(vel)) > PIN(max_usr_vel) * 0.05) {
    PIN(force_timer) = 0.0;
  } else if(PIN(force_timer) > PIN(force_time)) {
    PIN(force_timer) = PIN(force_time);
    PIN(mode)        = 0;
  }

  PIN(target) = CLAMP(PIN(target), PIN(min_pos), PIN(max_pos));

  switch((int)PIN(mode)) {
    case 0:  // auto move
      PIN(vel) = SIGN(PIN(target) - PIN(pos)) * sqrtf(ABS(PIN(target) - PIN(pos)) * 2.0 * PIN(max_acc));
      PIN(vel) = CLAMP(PIN(vel), PIN(vel_old) - PIN(max_acc) * period, PIN(vel_old) + PIN(max_acc) * period);
      PIN(vel) = LIMIT(PIN(vel), PIN(max_vel));
      break;
    case 1:  // force move
      PIN(acc) = PIN(force) / PIN(virtual_mass);
      PIN(acc) = LIMIT(PIN(acc), PIN(max_usr_acc));
      PIN(vel) += PIN(acc) * period;
      PIN(vel) = CLAMP(PIN(vel), PIN(vel_old) - PIN(max_usr_acc) * period, PIN(vel_old) + PIN(max_usr_acc) * period);
      PIN(vel) = LIMIT(PIN(vel), PIN(max_usr_vel));
      break;
  }

  PIN(vel) = CLAMP(PIN(vel), -sqrtf(ABS(PIN(pos) - PIN(min_pos)) * 2.0 * MAX(PIN(max_acc), PIN(max_usr_acc))), sqrtf(ABS(PIN(pos) - PIN(max_pos)) * 2.0 * MAX(PIN(max_acc), PIN(max_usr_acc))));
  PIN(acc) = (PIN(vel) - PIN(vel_old)) / period;

  PIN(pos) += PIN(vel) * period;
  PIN(pos)  = CLAMP(PIN(pos), PIN(min_pos), PIN(max_pos));
  PIN(mpos) = mod(PIN(pos) * PIN(scale));
  PIN(print_timer) += period;

  if(PIN(en) <= 0.0) {
    PIN(force_offset) += PIN(force) * 0.5 * period;
    PIN(pos)  = 0.0;
    PIN(mpos) = 0.0;
    PIN(vel)  = 0.0;
  }
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fmove_pin_ctx_t *pins = (struct fmove_pin_ctx_t *)pin_ptr;
  if(PIN(print_freq) > 0.0) {
    if(PIN(print_timer) > 1.0 / PIN(print_freq)) {
      PIN(print_timer) = 0.0;
      if(PIN(mode) > 0.0) {
        printf("force_move %f\n", PIN(pos));
      } else if(ABS(PIN(target) - PIN(pos)) < 0.01 && ABS(PIN(vel)) < PIN(max_vel) * 0.1) {
        printf("on_target %f\n", PIN(pos));
      } else {
        printf("moving %f\n", PIN(pos));
      }
    }
  } else {
    PIN(print_timer) = 0.0;
  }
}

hal_comp_t fmove_comp_struct = {
    .name      = "fmove",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct fmove_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};