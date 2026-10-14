#include "conf_comp.h"
#include <stdio.h>
#include "main.h"  // for Wait
#include "commands.h"
#include "hal.h"

/**
* ## Brief
* `conf` has no code: it is a set of named parameter pins that hold the motor, feedback, limit and tuning values of a drive in one place. A motor config sets them (conf/template/conf.txt shows the defaults a new config starts from, e.g. `conf0.r = 1`, `conf0.max_ac_cur = 6`), and the templates link them to the components that use them, e.g. `hv0.r = conf0.r`, `pid0.pos_bw = conf0.pos_bw`, `fault0.max_dc_volt = conf0.max_dc_volt`, `fb_switch0.mot_offset = conf0.mot_fb_offset`.
*
* ## Component Explanation
*
* 1. **Usage**:
* - Loaded with `load conf` (conf/template/conf.txt) and set with `conf0.<pin> = <value>` in the config. The values do nothing by themselves; only the links made by the loaded templates (pid.txt, mpid.txt, pmsm.txt, the feedback templates, ...) carry them to the components.
* - The component has no rt/nrt function and sets no defaults, so every pin not set in the config is 0.
* - Units follow stmbl conventions: currents are peak amps, angles rad, velocities rad/s.
*
* 2. **Groups**:
* - Motor model: `r`, `l`, `lq`, `psi`, `polecount`, `j`, `d`, `f`, `o`, `j_sys`, `j_lpf`, `out_rev`.
* - Feedback: `mot_fb_*`, `com_fb_*`, `joint_fb_*` (pole count, commutation offset, direction, resolution) and `cmd_rev`, `cmd_res` for the command input; `phase_time`/`phase_cur` for autophasing.
* - Limits and faults: `max_vel`, `max_acc`, `max_force`, `max_dc_cur`, `max_ac_cur`, `max_sat`, `max_pos_error`, the DC link voltage limits and the temperature and fan thresholds, all used by `fault0` and `pid0`. `high_ipm_temp`/`max_ipm_temp` (bridge junction derate start and trip, conf.txt sets 125/140 deg C) are linked to `fault0` only by conf/template/ipm.txt.
* - Loop tuning: `pos_bw`, `vel_bw`, `vel_d`, `vel_g`, `torque_g` for `pid0`, and `cur_bw`, `cur_ff`, `cur_ind` for the F3 current loop via `hv0`.
*
* {{% hint warning %}}
* `low_dc_volt` is 24 V in conf/template/conf.txt. The pid, mpid, uf and vf templates link `fault0.min_dc_volt = conf0.low_dc_volt`.
* {{% /hint %}}
*/

HAL_COMP(conf);

HAL_PIN(r);              // *parameter*, phase resistance (Ohm)
HAL_PIN(l);              // *parameter*, phase inductance (H), d axis when lq is set
HAL_PIN(lq);             // *parameter*, q axis inductance (H), 0 = same as l
HAL_PIN(j);              // *parameter*, motor inertia (kg*m^2)
HAL_PIN(d);              // *parameter*, motor damping (Nm/(rad/s))
HAL_PIN(f);              // *parameter*, motor friction (Nm)
HAL_PIN(o);              // *parameter*, motor torque offset (Nm)
HAL_PIN(j_sys);          // *parameter*, system (load) inertia (kg*m^2)
HAL_PIN(j_lpf);          // *parameter*, motor to load bandwidth (Hz)
HAL_PIN(psi);            // *parameter*, electrical torque constant / flux linkage (V*s/rad)
HAL_PIN(polecount);      // *parameter*, number of motor pole pairs
HAL_PIN(out_rev);        // *parameter*, motor direction, linked to hv0.rev
HAL_PIN(high_mot_temp);  // *parameter*, lower motor overtemperature limit (deg C), current is derated above
HAL_PIN(max_mot_temp);   // *parameter*, upper motor overtemperature limit (deg C)
HAL_PIN(phase_time);     // *parameter*, autophasing time (s)
HAL_PIN(phase_cur);      // *parameter*, autophasing current (A)

HAL_PIN(max_vel);        // *parameter*, max. velocity (rad/s)
HAL_PIN(max_acc);        // *parameter*, max. acceleration (rad/s^2)
HAL_PIN(max_force);      // *parameter*, max. torque (Nm)
HAL_PIN(max_dc_cur);     // *parameter*, max. DC link current (A)
HAL_PIN(max_ac_cur);     // *parameter*, max. phase current (A peak)

HAL_PIN(mot_fb_polecount);// *parameter*, motor feedback pole pairs
HAL_PIN(mot_fb_offset);  // *parameter*, motor feedback commutation offset (rad)
HAL_PIN(mot_fb_rev);     // *parameter*, motor feedback direction
HAL_PIN(mot_fb_res);     // *parameter*, motor feedback resolution (counts/rev)

HAL_PIN(joint_fb_polecount);// *parameter*, joint feedback pole pairs
HAL_PIN(joint_fb_offset);// *parameter*, joint feedback commutation offset (rad)
HAL_PIN(joint_fb_rev);   // *parameter*, joint feedback direction
HAL_PIN(joint_fb_res);   // *parameter*, joint feedback resolution (counts/rev), not linked by any template

HAL_PIN(com_fb_polecount);// *parameter*, commutation feedback pole pairs
HAL_PIN(com_fb_offset);  // *parameter*, commutation feedback commutation offset (rad)
HAL_PIN(com_fb_rev);     // *parameter*, commutation feedback direction
HAL_PIN(com_fb_res);     // *parameter*, commutation feedback resolution (counts/rev), not linked by any template

HAL_PIN(cmd_rev);        // *parameter*, command direction
HAL_PIN(cmd_res);        // *parameter*, command resolution (counts/rev)

HAL_PIN(max_dc_volt);    // *parameter*, upper DC link overvoltage limit (V), fault
HAL_PIN(max_hv_temp);    // *parameter*, upper driver overtemperature limit (deg C)
HAL_PIN(max_pos_error);  // *parameter*, max. position error (rad), 0 = disabled
HAL_PIN(high_dc_volt);   // *parameter*, lower DC link overvoltage limit (V), current is derated above
HAL_PIN(low_dc_volt);    // *parameter*, DC link undervoltage limit (V), to fault0.min_dc_volt
HAL_PIN(high_hv_temp);   // *parameter*, lower driver overtemperature limit (deg C), current is derated above
HAL_PIN(fan_hv_temp);    // *parameter*, driver fan switch on temperature (deg C)
HAL_PIN(fan_mot_temp);   // *parameter*, motor fan switch on temperature (deg C)

HAL_PIN(g);              // *parameter*, motor model limit scaling, not linked by any template
HAL_PIN(pos_bw);         // *parameter*, position loop bandwidth (rad/s/rad)
HAL_PIN(vel_bw);         // *parameter*, velocity loop bandwidth (rad/s^2/(rad/s))
HAL_PIN(vel_d);          // *parameter*, velocity loop damping
HAL_PIN(vel_g);          // *parameter*, velocity loop proportional limit scaling
HAL_PIN(torque_g);       // *parameter*, torque loop proportional limit scaling
HAL_PIN(cur_bw);         // *parameter*, current loop bandwidth (rad/s)
HAL_PIN(cur_ff);         // *parameter*, current loop resistance feed forward gain
HAL_PIN(cur_ind);        // *parameter*, current loop BEMF feed forward gain
HAL_PIN(max_sat);        // *parameter*, max. velocity, acceleration and torque saturation time (s)

HAL_PIN(high_ipm_temp);  // *parameter*, lower IPM junction overtemperature limit (deg C), current is derated above
HAL_PIN(max_ipm_temp);   // *parameter*, upper IPM junction overtemperature limit (deg C), fault

hal_comp_t conf_comp_struct = {
    .name      = "conf",
    .nrt       = 0,
    .rt        = 0,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct conf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
