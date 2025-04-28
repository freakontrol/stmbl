#include "zv_ip_comp.h"
#include "hal.h"
#include "math.h"

/**
* ## Brief
* {{% hint danger %}}
* This is an experimental component, it's not fully tested. 
* {{% /hint %}} 
* The `zv_ip` component is responsible for applying a Zero Vibration (ZV) input shaper to the position and velocity commands of a motor. It reads the position and velocity commands, calculates the shaped commands, and outputs them for further processing.
*
* ## Component Explanation
*
* The `zv_ip` component applies a Zero Vibration (ZV) input shaper to the position and velocity commands of a motor. This component operates within a hardware abstraction layer (HAL) framework, providing real-time control and non-real-time initialization functions.
*
* 1. **Input Reading and Initialization**:
* - The component reads the position command (`cmd_pos`) and velocity command (`cmd_vel`) from input pins.
* - It also reads various parameters for the ZV input shaper, including `natural_frequency`, `damping_ratio`, `A1`, `A2`, and `T`.
*
* 2. **Parameter Calculation**:
* - The component calculates the ZV input shaper parameters (`A1`, `A2`, and `T`) based on the `natural_frequency` and `damping_ratio`.
*
* 3. **Input Shaping**:
* - The component applies the ZV input shaper to the position and velocity commands to generate the shaped position (`shaped_pos`) and shaped velocity (`shaped_vel`) outputs.
*
* 4. **Output Updates**:
* - The component updates the output pins with the calculated shaped position and velocity commands.
*/

HAL_COMP(zv_ip);

HAL_PIN(cmd_pos);       // *input*, Position command
HAL_PIN(cmd_vel);       // *input*, Velocity command

// ZV input shaper parameters
HAL_PIN(natural_frequency); // *parameter*, Natural frequency of the system (Hz)
HAL_PIN(damping_ratio);     // *parameter*, Damping ratio of the system
HAL_PIN(A1);               // *parameter*, Amplitude of the first impulse
HAL_PIN(A2);               // *parameter*, Amplitude of the second impulse
HAL_PIN(T);                // *parameter*, Time delay between impulses

// Intermediate values stored in pins
HAL_PIN(prev_cmd_pos);    // *internal*, Previous position command
HAL_PIN(prev_cmd_vel);    // *internal*, Previous velocity command
HAL_PIN(timer);           // *internal*, Timer to track time delay

// Shaped outputs
HAL_PIN(shaped_pos);      // *output*, Shaped position command
HAL_PIN(shaped_vel);      // *output*, Shaped velocity command

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct zv_ip_pin_ctx_t *pins = (struct zv_ip_pin_ctx_t *)pin_ptr;
  PIN(natural_frequency) = 50.0; // Default natural frequency
  PIN(damping_ratio)     = 0.05; // Default damping ratio
  PIN(A1)                = 0.0;  // Default amplitude of the first impulse
  PIN(A2)                = 0.0;  // Default amplitude of the second impulse
  PIN(T)                 = 0.0;  // Default time delay between impulses
  PIN(prev_cmd_pos)      = 0.0;  // Initialize previous position command
  PIN(prev_cmd_vel)      = 0.0;  // Initialize previous velocity command
  PIN(timer)             = 0.0;  // Initialize timer
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct zv_ip_pin_ctx_t *pins = (struct zv_ip_pin_ctx_t *)pin_ptr;

  // Calculate ZV input shaper parameters
  double omega_n = PIN(natural_frequency) * 2.0 * M_PI;
  double zeta = PIN(damping_ratio);
  double omega_d = omega_n * sqrt(1.0 - zeta * zeta);
  PIN(A1) = 1.0 / (1.0 + exp(-zeta * M_PI / sqrt(1.0 - zeta * zeta)));
  PIN(A2) = 1.0 - PIN(A1);
  PIN(T) = M_PI / omega_d;

  // Apply ZV input shaper to position and velocity commands
  double shaped_pos = PIN(A1) * PIN(cmd_pos);
  double shaped_vel = PIN(A1) * PIN(cmd_vel);

  // Increment the timer
  PIN(timer) += period;

  // Check if the time delay T has passed
  if (PIN(timer) >= PIN(T)) {
    shaped_pos += PIN(A2) * PIN(prev_cmd_pos);
    shaped_vel += PIN(A2) * PIN(prev_cmd_vel);
    PIN(timer) = 0.0; // Reset the timer
  }

  // Store the current commands for the next cycle
  PIN(prev_cmd_pos) = PIN(cmd_pos);
  PIN(prev_cmd_vel) = PIN(cmd_vel);

  // Update shaped outputs
  PIN(shaped_pos) = shaped_pos;
  PIN(shaped_vel) = shaped_vel;
}

hal_comp_t zv_ip_comp_struct = {
    .name      = "zv_ip",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct zv_ip_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};