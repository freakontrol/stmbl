#include "acim_ttc_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* The `acim_ttc` component is responsible for controlling an AC Induction Motor (ACIM) using Torque and Flux control strategies. It reads motor parameters and control settings, calculates derived parameters, updates the control algorithm in real-time, and supports different operating modes. The component ensures that the motor operates within safe and efficient boundaries by adjusting the velocity, position, and torque limits accordingly.
*
* ## Component Explanation
*
* The `acim_ttc` component is designed for controlling an AC Induction Motor (ACIM) using Torque and Flux control strategies. This component operates within a hardware abstraction layer (HAL) framework, providing real-time control and non-real-time initialization functions.
*
* 1. **Input Reading and Initialization**:
* - The component reads various motor parameters and control settings from input pins, including `mode`, `sensorless`, `torque_n`, `cur_n`, `slip_n`, `polecount`, `freq_n`, `vel_n`, `u_n`, `u_boost`, `t_boost`, and `s_boost`.
* - The torque command (`torque`) and measured velocity (`vel_m`) are also read from input pins.
*
* 2. **Parameter Calculation**:
* - The component calculates several derived parameters based on the input values, such as `poles`, `vel_n`, `t_n`, `freq_n`, `slip_n`, and `id_n`.
* - These parameters are used in the control algorithm to adjust the motor's operation.
*
* 3. **Scale Update**:
* - The `scale` parameter is updated based on the error between the duty cycle setpoint and the actual duty cycle. This helps in adjusting the control parameters dynamically.
* - The `scale` parameter is clamped to ensure it stays within a valid range.
*
* 4. **Operating Modes**:
* - The component supports different operating modes based on the `mode` input:
*   - **Slip Control**: Adjusts the d-axis and q-axis currents based on the slip and torque commands.
*   - **MTPA (Maximum Torque Per Ampere)**: Optimizes the torque for a given current.
*   - **V/f (Voltage over Frequency) Control**: Adjusts the voltage and frequency based on the slip and torque commands.
* - The appropriate control strategy is selected based on the `mode` input, and the corresponding commands are calculated.
*
* 5. **Torque Limits**:
* - The component calculates the minimum and maximum torque limits (`t_min` and `t_max`) based on the operating conditions and the `t_boost` parameter.
* - These limits ensure that the motor operates within safe and efficient boundaries.
*
* 6. **Slip and Velocity Updates**:
* - The slip and velocity are updated based on the sensorless control flag (`sensorless`).
* - If sensorless control is enabled, the velocity is adjusted by subtracting the slip. Otherwise, the velocity is adjusted by adding the slip.
*
* 7. **Output Updates**:
* - The component updates the output pins with the calculated values, including `cmd_mode`, `d_cmd`, `q_cmd`, `slip_n`, `slip`, `t_min`, `t_max`, and `pos`.
* - The estimated position (`pos`) is updated by integrating the estimated velocity over time and wrapping it using the `mod` function.
*/

HAL_COMP(acim_ttc);

HAL_PIN(mode);          // *parameter*, 0 = slip, 1 = mtpa, 2 = u/f
HAL_PIN(sensorless);    // *parameter*, Flag indicating whether sensorless control is enabled (1 = enabled, 0 = disabled)

// Motor values and parameters
HAL_PIN(torque_n);      // *parameter*, Nominal torque of the motor
HAL_PIN(cur_n);         // *parameter*, Nominal current of the motor
HAL_PIN(slip_n);        // *parameter*, Nominal slip frequency of the motor
HAL_PIN(polecount);     // *parameter*, Number of pole pairs in the motor
HAL_PIN(freq_n);        // *parameter*, Nominal frequency of the motor
HAL_PIN(vel_n);         // *parameter*, Nominal velocity of the motor
HAL_PIN(u_n);           // *parameter*, Nominal voltage of the motor
HAL_PIN(u_boost);       // *parameter*, Voltage boost factor for startup
HAL_PIN(t_boost);       // *parameter*, Torque boost factor for startup
HAL_PIN(s_boost);       // *parameter*, Slip boost factor for startup

// Torque command inputs
HAL_PIN(torque);        // *input*, Torque command input
HAL_PIN(vel_m);         // *input*, Measured velocity input

// Current command outputs
HAL_PIN(d_cmd);         // *output*, D-axis current command
HAL_PIN(q_cmd);         // *output*, Q-axis current command
HAL_PIN(cmd_mode);      // *output*, Command mode (0 = voltage control, 1 = current control)
HAL_PIN(pos);           // *output*, Estimated position of the motor
HAL_PIN(vel_e);         // *output*, Estimated velocity of the motor
HAL_PIN(slip);          // *output*, Estimated slip of the motor
HAL_PIN(t_min);         // Minimum torque limit
HAL_PIN(t_max);         // Maximum torque limit

// Control parameters
HAL_PIN(scale);         // Scaling factor for current commands
HAL_PIN(ki);            // Integral gain for scale adjustment
HAL_PIN(duty);          // Current duty cycle
HAL_PIN(duty_setpoint); // Desired duty cycle setpoint

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_ttc_pin_ctx_t *pins = (struct acim_ttc_pin_ctx_t *)pin_ptr;
  PIN(polecount)                  = 2;
  PIN(vel_n)                      = 1745.0 / 60.0 * 2.0 * M_PI;
  PIN(torque_n)                   = 23.0;
  PIN(cur_n)                      = 17.0;
  PIN(freq_n)                     = 60.0;
  PIN(u_n)                        = 80.0;
  PIN(u_boost)                    = 7.0;
  PIN(t_boost)                    = 1.3;
  PIN(s_boost)                    = 2.5;
  PIN(mode)                       = 0;
  PIN(sensorless)                 = 0;
  PIN(scale)                      = 1.0;
  PIN(ki)                         = 50.0;
  PIN(duty_setpoint)              = 0.9;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct acim_ttc_ctx_t * ctx = (struct acim_ttc_ctx_t *)ctx_ptr;
  struct acim_ttc_pin_ctx_t *pins = (struct acim_ttc_pin_ctx_t *)pin_ptr;

  float poles   = MAX(PIN(polecount), 1.0);
  float vel_n   = PIN(vel_n) * poles;
  float t_n     = MAX(PIN(torque_n), 0.001);
  float freq_n  = MAX(PIN(freq_n), 1.0);
  float slip_n  = freq_n * 2.0 * M_PI - vel_n;
  float cur_n   = PIN(cur_n);
  float u_n     = PIN(u_n);
  float u_boost = PIN(u_boost);
  float t_boost = PIN(t_boost);

  float torque = PIN(torque);
  float vel    = 0.0;
  if(PIN(sensorless) > 0.0) {
    vel = PIN(vel_e);
  } else {
    vel = PIN(vel_m) * poles;
  }

  float d_cmd    = 0.0;
  float q_cmd    = 0.0;
  float slip     = 0.0;
  float cmd_mode = 0;

  float id_n = cur_n / sqrtf(2.0);

  PIN(scale) += (PIN(duty_setpoint) - PIN(duty)) * PIN(ki) * period;
  PIN(scale) = CLAMP(PIN(scale), 0.01, 1);

  switch((int)PIN(mode)) {
    case 0:            // slip control
      cmd_mode = 1.0;  // cur cmd
      // d_cmd = MIN(id_n, id_n * freq_n * 2.0 * M_PI * v_boost / vel); // constant flux
      d_cmd = id_n * PIN(scale);
      q_cmd = id_n / t_n * torque / PIN(scale);
      slip  = slip_n * q_cmd / d_cmd;

      // id = id_n
      // slip = slip_n * iq / id
      // torque = 3/2 * p * K * iq * id

      // id = id_n * scale
      // // iq = toruqe / t_n * id_n * id_n / id
      // iq = toruqe / t_n * id_n / scale
      // slip = slip_n * iq / id

      // torque = 3/2 * p * K * iq * id
      // torque = t_n / id_n / id_n * id * iq
      // iq = torque / t_n * id_n * id_n / id
      break;

    case 1:            // mtpa
      cmd_mode = 1.0;  // cur cmd
      d_cmd    = 0.0;
      q_cmd    = cur_n / t_n * torque;
      slip     = slip_n * SIGN(torque);  // constant slip
      break;

    case 2:          // u/f slip
      cmd_mode = 0;  // volt cmd
      slip     = slip_n / t_n * torque;
      d_cmd    = MAX(u_n / freq_n * ABS(vel / 2.0 / M_PI), u_boost);
      q_cmd    = 0.0;
      break;

    default:
      cmd_mode = 1.0;  // cur cmd
      d_cmd    = 0;
      q_cmd    = 0;
      slip     = 0.0;
  }

  float t_min = 0;
  float t_max = 0;

  if(PIN(vel_m) > 0.0) {
    t_max = t_n * t_boost * PIN(scale);
    t_min = -t_max;
  } else {
    t_min = -t_n * t_boost * PIN(scale);
    t_max = -t_min;
  }

  slip = LIMIT(slip, slip_n * PIN(s_boost));

  if(PIN(sensorless) > 0.0) {
    vel -= slip;
    PIN(vel_m) = vel * poles;
  } else {
    vel += slip;
    PIN(vel_e) = vel;
  }

  PIN(t_min) = t_min;
  PIN(t_max) = t_max;

  PIN(cmd_mode) = cmd_mode;
  PIN(d_cmd)    = d_cmd;
  PIN(q_cmd)    = q_cmd;

  PIN(slip_n) = slip_n;
  PIN(slip)   = slip;
  PIN(pos)    = mod(PIN(pos) + vel * period);
}

hal_comp_t acim_ttc_comp_struct = {
    .name      = "acim_ttc",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct acim_ttc_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
