#include "sensorless_comp.h"
/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2016 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2016 Nico Stute <crinq@crinq.de>
* Copyright (C) 2025-2026 Thomas Deponte <deponte.thomas@gmail.com>
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


/**
* ## Brief
* The `sensorless` component is responsible for estimating the motor's velocity and position in real-time using electrical measurements. It applies low-pass filtering, back EMF calculation, velocity compensation, and position integration to achieve this.  
*
* ## Description  
* 
* 1. **Input Reading and Initialization**:
*    - `l` and `r` are the inductance and resistance of the motor, respectively. They are ensured to be positive.
*    - The voltage measurements `ud` and `uq` are updated using a delay line (`ud2`, `uq2`, `ud1`, `uq1`, `ud`, `uq`).
*    - The current measurements `id` and `iq` are read from the input pins.
* 
* 2. **Gain Clamping**:
*    - The gains `kb`, `ki`, and `kl` are clamped to ensure they stay within valid ranges.
* 
* 3. **Low-Pass Filtering of Current Changes**:
*    - The changes in currents `delta_id` and `delta_iq` are calculated using a low-pass filter. This helps to smooth out noise in the current measurements.
*    - The previous current values `old_id` and `old_iq` are updated to the current measurements.
* 
* 4. **Back EMF Calculation**:
*    - The back EMF (`ed` and `eq`) is estimated using the motor's electrical equations. The back EMF is the voltage generated by the motor due to its rotation.
*   - The equations used are:
*     ```c
*     ed = ud - r * id - PIN(delta_id) * l / period + vel * l * iq * kb;
*     eq = uq - r * iq - PIN(delta_iq) * l / period - vel * l * id * kb;
*     ```
*
* 5. **Velocity Compensation**:
*    - The estimated velocity `vel` is adjusted to compensate for the back EMF. The goal is to make the d-axis back EMF (`ed`) approach zero, which helps in stabilizing the control.
*    - The compensation is done using the following equation:
*      ```c
*      vel -= SIGN2(eq, 1.0) * ed * ki * period;
*      ```
*  
* 6. **Startup Boost**:
*    - If the absolute value of the estimated velocity is below a minimum threshold (`min_vel`), a boost is added to help the motor start up more reliably.
*    - The boost is applied using the following equation:
*      ```c
*      vel += SIGN2(id * iq, 0.1) * PIN(vel_boost);
*      ```
*  
* 7. **Velocity Limiting**:
*    - The estimated velocity is limited to ensure it does not exceed the maximum allowed value (`max_vel`).
*  
* 8. **Position Update**:
*    - The estimated position `pos` is updated by integrating the estimated velocity over time.
*    - The position is wrapped using the `mod` function to ensure it stays within a valid range (e.g., 0 to 2π for angular position).
*  
* 9. **Output Updates**:
*    - The estimated back EMF (`ed` and `eq`), velocity (`vel`), and position (`pos`) are updated on the output pins.  
*/

#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"


HAL_COMP(sensorless);

// Input pins
HAL_PIN(r);          // *parameter*, Resistance of the motor (R)
HAL_PIN(l);          // *parameter*, Inductance of the motor (L)

// Gain parameters for the control algorithm
HAL_PIN(ki);         // *parameter*, Integral gain for velocity compensation
HAL_PIN(kb);         // *parameter*, Back EMF gain
HAL_PIN(kl);         // *parameter*, Low-pass filter gain for current changes

// Velocity control parameters
HAL_PIN(min_vel);    // *parameter*, Minimum velocity threshold for startup boost
HAL_PIN(vel_boost);  // *parameter*, Velocity boost factor for startup
HAL_PIN(max_vel);    // *parameter*, Maximum allowed velocity

// Current and voltage measurements in the d-q reference frame
HAL_PIN(id);         // *input*, D-axis current
HAL_PIN(iq);         // *input*, Q-axis current
HAL_PIN(ud);         // *input*, D-axis voltage (current measurement)
HAL_PIN(uq);         // *input*, Q-axis voltage (current measurement)
HAL_PIN(ud1);        // *internal*, Delayed D-axis voltage (one sample delay)
HAL_PIN(uq1);        // *internal*, Delayed Q-axis voltage (one sample delay)
HAL_PIN(ud2);        // *internal*, Delayed D-axis voltage (two sample delays)
HAL_PIN(uq2);        // *internal*, Delayed Q-axis voltage (two sample delays)

// Output pins
HAL_PIN(vel);        // *output*, Estimated velocity of the motor
HAL_PIN(pos);        // *output*, Estimated position of the motor

// Estimated back EMF in the d-q reference frame
HAL_PIN(ed);         // *output*, D-axis back EMF
HAL_PIN(eq);         // *output*, Q-axis back EMF

// Previous current values for low-pass filtering
HAL_PIN(old_id);     // Previous D-axis current
HAL_PIN(old_iq);     // Previous Q-axis current

// Changes in current for back EMF calculation
HAL_PIN(delta_id);   // *output*, Change in D-axis current
HAL_PIN(delta_iq);   // *output*, Change in Q-axis current

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct sensorless_ctx_t *ctx      = (struct sensorless_ctx_t *)ctx_ptr;
  struct sensorless_pin_ctx_t *pins = (struct sensorless_pin_ctx_t *)pin_ptr;
  PIN(ki)                           = 1500.0;
  PIN(kb)                           = 1;
  PIN(kl)                           = 0.75;
  PIN(min_vel)                      = 3.0 * 2.0 * M_PI;
  PIN(vel_boost)                    = 0.2;
  PIN(max_vel)                      = 500.0 * 2.0 * M_PI * 1.1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct sensorless_ctx_t *ctx      = (struct sensorless_ctx_t *)ctx_ptr;
  struct sensorless_pin_ctx_t *pins = (struct sensorless_pin_ctx_t *)pin_ptr;

  float l = MAX(PIN(l), 0.00001);
  float r = MAX(PIN(r), 0.01);

  float ud = PIN(ud2);
  float uq = PIN(uq2);
  PIN(ud2) = PIN(ud1);
  PIN(uq2) = PIN(uq1);
  PIN(ud1) = PIN(ud);
  PIN(uq1) = PIN(uq);

  float id = PIN(id);
  float iq = PIN(iq);

  float kb = CLAMP(PIN(kb), 0, 1);
  float ki = CLAMP(PIN(ki), 0, 1.0 / period);
  float kl = CLAMP(PIN(kl), 0, 1);

  float vel = PIN(vel);
  float pos = PIN(pos);

  // lowpass current change
  PIN(delta_id) = PIN(delta_id) * kl + (id - PIN(old_id)) * (1 - kl);
  PIN(delta_iq) = PIN(delta_iq) * kl + (iq - PIN(old_iq)) * (1 - kl);

  PIN(old_id) = id;
  PIN(old_iq) = iq;

  // calc bemf
  float ed = ud - r * id - PIN(delta_id) * l / period + vel * l * iq * kb;
  float eq = uq - r * iq - PIN(delta_iq) * l / period - vel * l * id * kb;

  // velocity compensation, bemf_d -> 0
  vel -= SIGN2(eq, 1.0) * ed * ki * period;

  // startup boost
  if(ABS(vel) < PIN(min_vel)) {
    vel += SIGN2(id * iq, 0.1) * PIN(vel_boost);
  }

  vel = LIMIT(vel, PIN(max_vel));

  pos += vel * period;

  PIN(ed) = ed;
  PIN(eq) = eq;

  PIN(vel) = vel;
  PIN(pos) = mod(pos);
}

hal_comp_t sensorless_comp_struct = {
    .name      = "sensorless",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct sensorless_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};