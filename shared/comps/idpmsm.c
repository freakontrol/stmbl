#include "idpmsm_comp.h"
#include "common.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"
#include <math.h>

/**
* ## Brief
* `idpmsm` identifies a permanent magnet synchronous motor on the drive itself: winding resistance `r` together with the inverter dead time curve (`hv0.drop_k`, `hv0.drop_knee`), d and q inductance `ld`/`lq`, pole pairs `pp`, feedback direction `out_rev`, commutation offset `com_offset` (plus `com_fb_offset` for a commutation track such as `encf`'s) and flux linkage `psi` (plus a per pole back emf map from the F3's `emf0`). It runs on the F4 board and is loaded by the `id_pmsm` config template, which takes over `hv0` (enable, command mode, d/q commands, commutation angle, `r`, `l`, `lq`, current loop bandwidth) and prints the results as `conf0.*` and `hv0.*` lines to append to the config.
*
* ## How to run it
*
* 1. Have a working base config with the motor feedback set up (`fb_switch0.mot_abs_fb_no_offset` and `vel1.vel` must be valid), the load off the shaft and the drive disabled.
* 2. In the servoterm type `link id_pmsm`. The template does `load idpmsm` and wires:
* ```
* idpmsm0.en = fault0.en_out
* hv0.en = idpmsm0.en_out
* hv0.cur_bw = idpmsm0.cur_bw
* idpmsm0.loop_bw = conf0.cur_bw
* hv0.cmd_mode = idpmsm0.cmd_mode
* hv0.d_cmd = idpmsm0.d_cmd
* hv0.q_cmd = idpmsm0.q_cmd
* hv0.pos = idpmsm0.com_pos
* hv0.rev = idpmsm0.out_rev
* hv0.r = idpmsm0.r
* hv0.l = idpmsm0.l
* hv0.lq = idpmsm0.lq
* hv0.drop_k = 0
* hv0.emf_run = idpmsm0.emf_run
* hv0.emf_sel = idpmsm0.emf_sel
* hv0.emf_pp = idpmsm0.emf_pp
* idpmsm0.emf_val = hv0.emf_val
* idpmsm0.pos_fb = fb_switch0.mot_abs_fb_no_offset
* idpmsm0.mot_state = fb_switch0.mot_state_fb
* idpmsm0.com_fb = fb_switch0.com_fb_no_offset
* idpmsm0.com_rev = conf0.com_fb_rev
* idpmsm0.com_polecount = conf0.com_fb_polecount
* idpmsm0.vel_fb = vel1.vel
* relink
* ```
* - plus `ud_fb`, `uq_fb`, `id_fb`, `iq_fb`, `pwm_volt`, `dc_volt`, `pwm_freq`, `u_fb`, `v_fb` from `hv0`. It also sets `fault0.pos_error = 0`, `fault0.sat = 0` and `pid0.en = 0` so the position loop and its faults stay out of the way. The final `relink` flattens chained links (`a = b`, `b = c` becomes `a = c`): links typed at runtime are not flattened like the boot config, and without it `idpmsm0.com_polecount` would read `conf0.com_fb_polecount`'s own value instead of `conf0.polecount` through it. Dead time compensation is forced off: the r test fits the uncompensated loss.
* 3. Optionally change `idpmsm0.test_cur`, `idpmsm0.test_vel`, `idpmsm0.r_known` etc. (see below). `test_cur` must be below `conf0.max_ac_cur`.
* 4. Enable the drive. `en` going high moves `state` from 0 to 1.0 and the tests start. With the default `auto_step = 4.2` all three steps run back to back; with a lower `auto_step` the component stops at 1.1, 2.1 or 3.1, prints a prompt and waits for you to type `idpmsm0.state = 1.2` (or 2.2, 3.2). The r test alone takes about 50 s.
* 5. Copy the printed `conf0.r`, `conf0.l`, `conf0.lq`, `hv0.drop_k`, `hv0.drop_knee`, `conf0.polecount`, `conf0.mot_fb_offset`, `conf0.com_fb_offset` (only with a commutation track), `conf0.out_rev` and `conf0.psi` lines into the config. Lines printed in red, or after a "failed" message, are not measurements.
* 6. Disable the drive and reload the normal config. Disabling puts `state` back to 0, and `nrt` then resets most result pins to their init values, so the numbers are only kept in the printed text.
*
* Afterwards run `link id_tune` (component `idtune`) to find `hv0.adv` (it keeps the `hv0.drop_k` printed here when `hv0.drop_knee > 0`), then `id_mot`.
*
* ## Component Explanation
*
* `state` is a step number: `rt` does the timed measuring in states 1.2, 1.3, 2.2, 2.3 and 3.2; `nrt` does the setup, the r fit, the prompts and the printing in states 0, 1.0, 1.5, 1.4, 2.0, 2.4 to 2.6, 3.0 and 3.3. `en <= 0` forces `state = 0` in every rt cycle, which sets `en_out = 0`, clears the commands and `cur_bw = 1`. `rt_start` clears the internal context so a restarted run never continues half a measurement.
*
* Defaults from `nrt_init`: `l_freq = 0` (= `loop_bw / 2 pi`, 480 Hz at `cur_bw` 3000), `cur_bw = 1`.
*
* 1. **Resistance and dead time curve (state 1.2, current mode)**:
* - First 2 s at `cur_bw = 1`: `d_cmd = test_cur` at `com_pos = 0`. Because `hv0.r` is wired to `r`, `r` is bootstrapped as a filtered `ud_fb / id_fb` so current flows at all; the full current first also aligns the rotor at the strongest field.
* - The bootstrap `r` carries the dead time volts (about 2.3x the winding on X at 12 A), and the loop's integral gain is `cur_bw * r`, so the later steps to `test_cur` would overshoot into the F3's peak current trip. So at the first dwell `r` is replaced by `r_known` when given, else, after the first two dwells (1 and 0.67 `test_cur` at angle 0), by the slope `(u0 - u1) / (i0 - i1)` clamped to 0.01 .. the bootstrap value (only if the currents differ by > 0.1 A); the dead time loss is nearly flat there and cancels.
* - Then, at `cur_bw = 300`, 28 dwells: at each of the four angles `com_pos` = 0, pi, pi/2, -pi/2 (pairs half a turn apart give each current both signs through the same phases) the d current steps through 1, 0.67, 0.5, 0.33, 0.25, 0.17, 0.1 times `test_cur`. Each dwell settles 1 s (2 s for the first at each angle) and averages `id_fb` and `ud_fb` for 0.5 s. Each new angle after the first starts with 0.2 s at 0 A (inside its settle time), so the angle jump and the step to `test_cur` are apart.
* - With `hv0.drop_k = 0`, `ud_fb` carries the winding drop plus the dead time loss, modelled as hv.c's compensation curve: each phase loses `V0 * sign(i) * k(i)` with `k(i) = 1 - 1 / (1 + |i| / knee)^2`, and `dproj` is what those three losses put on d:
* ```c
* ud = r * id + V0 * dproj(id, com_pos, knee);
* ```
* - State 1.5 (nrt, the fit is too slow for a tick; bridge off): a grid over `knee` from 0.05 to 2.0 A in 0.025 A steps, at each knee a least squares fit of `r` and `V0` (of `V0` alone, with `r = r_known`, when `r_known > 0`); the knee with the smallest residual wins. Results: `dt_v0`, `dt_knee`, `dt_rms` (residual), `dt_top` (largest top dwell current) and `dt_k = dt_v0 / (dt_ideal * dc_volt)` (0 if `dc_volt` is not wired; `dt_ideal = 0` uses 2 us * `pwm_freq`).
* - Choosing `r_known`: with `r_known` > 0 the r stage runs with it from the first dwell (no slope bootstrap), the fit finds only the dead time curve (`V0` -> `drop_k`, and the knee) and the printed `conf0.r` is `r_known`. The drive's own IGBT and shunt slope (a few tens of mOhm) then ends up in the dead time curve, so no drive-fitted `r` is needed for the compensation. Use the per phase resistance of a Y winding, line to line / 2, measured at the drive end of the motor cable with the cable connected (the cable is in the current loop too), cold, with the temperature noted (copper +0.393 %/K; a 6 A run warms the winding about 3 %). A lead-nulled DMM reading is good enough; a four wire reading (bench supply 2-3 A through U-V, voltage sensed at the cable ends, `r = V_UV / (2 I)`, both polarities averaged, all three pairs within 1 %) only tightens the last 1-2 %. A fixed `r_known` is steadier than the fit, which reads high at higher `test_cur` as the winding heats (X: 0.659 at 6 A cold, 0.769 at 12 A), so use it for runs at 12 A.
* - `r_ok` needs `dt_top > test_cur / 2`, `r > 0.001` and `V0 >= 0`. If yes, `r` takes the fit and `avg_test_volt = r * test_cur + V0 * dproj(test_cur, 0, knee)` (limited to `pwm_volt / 2`) is kept as the bias voltage for the l test and the state goes to 1.3, otherwise straight to the report 1.4.
*
* 2. **Ld and Lq by sine injection (state 1.3, voltage mode, `cur_bw = 1`)**:
* - `d_cmd = avg_test_volt` holds about `test_cur` on d at `com_pos = 0`, which keeps every phase current on one side of zero so the dead time is a constant offset; a sine of `l_freq` (0: `loop_bw / 2 pi`, the current loop's crossover where the loop uses l; `loop_bw` 0 = 1000 rad/s; clamped to 20 Hz .. 0.2 / period and rounded to whole cycles of the measuring window, reported as `l_f`) is added first on `d_cmd`, then on `q_cmd`.
* - Per axis: 0.1 s settle, four 50 ms blocks that scale the amplitude (starting at 1 V; q starts from d's) until the current ripple is `l_ripple * test_cur` (amplitude kept within 0.2 V .. `pwm_volt / 4`), then about 0.4 s (whole cycles) of demodulation of voltage and current at the injection frequency, with the dc bias taken out.
* - The impedance `z = v1 / i1`, corrected for the zero order hold of the F3's 5 kHz command, gives
* ```c
* l = sqrt(z*z - r*r) / (2 * pi * f);   // 0 if z <= r
* ```
* - Results: `ld`, `lq` and the amplitudes `l_vd`, `l_id`, `l_vq`, `l_iq`. `l_ok = 1` if both inductances are > 0 and both current amplitudes landed within 0.5 .. 2 times the target ripple. Then `l = ld` (0 if not ok).
*
* 3. **Report (state 1.4, nrt)**:
* - Prints `conf0.r`, and if `l_ok` `conf0.l` (= Ld) and `conf0.lq`, else how much current the injection drew. Then `hv0.drop_k` and `hv0.drop_knee` to append, and the fit's `V0`, link voltage, knee and residual.
* - Goes to 2.0 if `r_ok`, else prints the top dwell current reached and goes to 9.0. State 9.0 has no rt case: the bridge stays off until the drive is disabled.
*
* 4. **Pole pairs and direction (state 2.2, current mode, `cur_bw = 100`, 4 s)**:
* - `d_cmd = test_cur` and the field angle `com_pos` is turned open loop, ramping from 0 to `test_vel` electrical rad/s in 1 s so the rotor can follow.
* - From 1.5 s to 4 s the field angle and the rotor angle (integral of `vel_fb`) are summed; `pp_raw = field / rotor`. A negative ratio sets `out_rev = 1` and the sign is dropped; `pp` is then rounded.
* - `pp_ok` needs the rotor turning (`|vel_fb| > 0.1`) in more than 90% of the window, a rounded value of 1 .. 24 and less than 0.15 away from an integer. On failure the state goes to 2.4 (message: lower `test_vel` or raise `test_cur`, then 9.0).
* - State 2.0 (before the rotor moves) prints a red warning if `mot_state` (`fb_switch0.mot_state_fb`) is not 3: the offsets are then relative to power-up.
* - Throughout the test the commutation track `com_fb` is followed tick by tick: its total movement and its largest single step (both in the track's own angle units, via `minus()`) are kept. This decides in step 5 whether a track is present and whether it is coarse.
*
* 5. **Commutation offset (state 2.3, 2 s)**:
* - `d_cmd = test_cur` at `com_pos = 0` so the rotor parks on the d axis. Over the second half of the dwell the unit vector of `pos_fb` is summed into `off_sin`/`off_cos` (`off_n` samples), a circular mean so the wrap at +-pi does not matter.
* - `com_offset = -atan2(off_sin, off_cos)`, folded into `[0, 2*pi/pp)` so the same axis always prints the same number. `off_mag` is the resultant length (1 = rotor held still).
* - `com_ok` needs `off_mag > 0.98` and `id_fb > test_cur / 2`. Success goes to 2.5 which prints `conf0.polecount`, `conf0.mot_fb_offset` and, if reversed, `conf0.out_rev = 1`; failure goes to 2.6 (message, then 9.0).
* - **Commutation track offset** (same dwell): the track is summed as an electrical angle, `com_fb * pp / cpp` with `cpp = com_polecount` (or `pp` if `com_polecount` < 1). At the end, if the track moved more than pi in the pp test:
* ```c
* com_fb_offset = -atan2(com_sin, com_cos) * cpp / pp;   // negated again if com_rev > 0
* ```
* - This is the `com_offset` for `fb_switch`, which commutates from `mod((com_abs_pos + com_offset) * polecount / com_polecount)` (both reversed under `com_rev`) while the motor feedback is not absolute. `com_fb_ok` = 1 for a fine track, 2 if a single step was over 0.5 rad (halls: the value can be up to 30 deg electrical off), 0 if the track did not move (no track; `com_fb_offset` = 0). With `com_fb_ok` > 0 state 2.5 also prints `conf0.com_fb_offset` (plus a note for a coarse track).
*
* 6. **Flux linkage psi (state 3.2, current mode, `cur_bw = 250`)**:
* - Commutation now closes on the feedback: `com_pos = mod((pos_fb + com_offset) * pp)`, `d_cmd = 0`. A speed loop drives `q_cmd` from the mechanical `vel_fb`; the clamp slews only the integrator, the P term acts on the unclamped error:
* ```c
* vel_error = LIMIT(v_t - vel_fb, ABS(test_vel) / 100);
* cur_sum   = LIMIT(cur_sum + ki * vel_error * period, test_cur);
* q_cmd     = LIMIT(vel_bw * period * (v_t - vel_fb) + cur_sum, test_cur);
* ```
* - Stages: spin up to `test_vel / 2` (mechanical rad/s), 2 s dwell, spin up to `test_vel`, 2 s dwell, then coast. A spin up is done when the filtered speed (50 ms) stays within 15% for 0.5 s; more than 8 s fails. A filtered speed under 10% of the target while `|cur_sum| > test_cur / 2` counts as a stall, which catches a wrong `pp` or offset. The clamp, the settle band, the stall check and the coast thresholds below use `abs(test_vel)`, so a negative `test_vel` runs the psi test in reverse.
* - The dwells record `uq - r * iq`, `iq` and speed. They are not used for psi (they contain the dead time); after the coast they give `udt_lo`/`udt_hi` = `uq - r iq - pp vel psi` (dead time volts on q) at the currents `idt_lo`/`idt_hi` and speeds `vel_lo`/`vel_hi`.
* - Coast: `en_out = 0`, the rotor runs down and the line voltage `u_fb - v_fb` is demodulated against the commutation angle, correcting the F3's `io.c` input filter (alpha = 750 / `pwm_freq`, 0.05 at 15 kHz) and the zero order hold of the F3 state block (`u_fb`/`v_fb` refresh only every `sizeof(f3_state_data_t) / 4` rt periods, which scales the emf by `sinc(w T / 2)`; the hold's delay stays in `emf_delay`). Sampling starts 50 ms after the bridge drops, so the freewheeling winding current has died. Samples are split into a fast band (> 0.6 `test_vel`) and a slow band (0.2 .. 0.6 `test_vel`, all on the speed magnitude); the coast ends below 0.2 `test_vel` or after 4 s. Each band needs 200 samples.
* - Results: `psi` (both bands, clamped to 0.001 .. 1 Vs), `psi_hi`, `psi_lo`, `emf_angle` (back emf phase against the commutation angle, extrapolated to standstill, electrical degrees) and `emf_delay` (s). The report compares `emf_angle` with the expected 120 deg (60 deg with `out_rev`) as an offset check with no current in it. `psi_ok = 1` on success.
* - Per pole map: during the coast `emf_run = 1` (it is -1, clear, while spinning up) makes the F3's `emf0` sum every pwm period's sample into `2 * pp` pole bins. After the coast `emf_run = 0` and `emf_sel` steps through emf0's results, one every 20 ms, read back on `emf_val`: `emf_psi` (whole coast), `emf_h5`/`emf_h7` (5th and 7th harmonic, % of the fundamental), `pole_min`, `pole_max` and `pole_spread` (% of the mean). A demagnetised pole reads low in its bin on every run; a uniformly weak rotor reads low in all bins (flat map). `emf_ok = 1` if the map came back with samples in every bin.
* - State 3.3 prints `conf0.psi` and the diagnostics: with `emf_ok` the printed `conf0.psi` is `emf_psi` (the F3 map sums every pwm sample with no filter or hold in it) and the F4 coast `psi` is shown as a cross-check, otherwise it is `psi`. On failure it prints the reason (stall, speed never settled, too few coast samples), then goes to 3.4 where it idles with the bridge off.
*
* 7. **Other states**:
* - `state = 10` only keeps `com_pos = mod((pos_fb + com_offset) * pp)` updated; it sets nothing else.
*
* {{% hint warning %}}
* - `test_vel` means two things: the pp test turns the field at `test_vel` electrical rad/s, while the psi test holds `test_vel / 2` and `test_vel` of mechanical speed (`vel_fb`).
* - The knee grid ends at 0.05 and 2.0 A; a fit on an edge prints a red warning, then keep the old `hv0.drop_k` and `hv0.drop_knee` if the residual is large. Without `dc_volt` wired, `hv0.drop_k` is not printed.
* - The `mot_state` check is made after the pp test (state 2.5), so an encoder that indexes during the test does not warn. If it still warns, the printed offsets are relative to power-up: index the encoder first.
* {{% /hint %}}
*/

HAL_COMP(idpmsm);

HAL_PIN(d_cmd);     // *output*, d command to hv0.d_cmd (A in current mode, V in voltage mode)
HAL_PIN(q_cmd);     // *output*, q command to hv0.q_cmd (A in current mode, V in voltage mode)
HAL_PIN(com_pos);   // *output*, commutation angle to hv0.pos (rad electrical)
HAL_PIN(cmd_mode);  // *output*, to hv0.cmd_mode, 0 = voltage, 1 = current
HAL_PIN(en);        // *input*, enable, usually fault0.en_out; low resets state to 0
HAL_PIN(en_out);    // *output*, bridge enable to hv0.en

HAL_PIN(id_fb);   // *input*, d current from hv0 (A)
HAL_PIN(iq_fb);   // *input*, q current from hv0 (A)
HAL_PIN(ud_fb);   // *input*, d voltage command from hv0 (V)
HAL_PIN(uq_fb);   // *input*, q voltage command from hv0 (V)
HAL_PIN(pos_fb);  // *input*, rotor position without commutation offset (rad), fb_switch0.mot_abs_fb_no_offset
HAL_PIN(vel_fb);  // *input*, mechanical rotor velocity (rad/s), vel1.vel

HAL_PIN(state);  // *input/output*, step number: 1.x r/l, 2.x pp/offset, 3.x psi, 9 = failed, 0 = off
HAL_PIN(timer);  // *output*, time in the current test step (s)

HAL_PIN(r);         // *output*, winding resistance (ohm), also wired to hv0.r, result for conf0.r
HAL_PIN(l);         // *output*, d axis inductance for conf0.l, 0 if not measured (H), also hv0.l
HAL_PIN(l_ok);      // *output*, 1 = ld/lq are valid measurements
HAL_PIN(l_freq);    // *parameter*, l test injection frequency (Hz), 0 (default) = loop_bw / 2 pi
HAL_PIN(loop_bw);   // *input*, the run's current loop bandwidth from conf0.cur_bw (rad/s), 0 = 1000
HAL_PIN(l_f);       // *output*, the injection frequency the l test used (Hz)
HAL_PIN(l_ripple);  // *parameter*, l test injected current amplitude, fraction of test_cur, default 0.15
HAL_PIN(ld);        // *output*, d axis inductance at test_cur on d (H)
HAL_PIN(lq);        // *output*, q axis inductance at the same bias (H), also hv0.lq, result for conf0.lq
HAL_PIN(l_vd);      // *output*, injected voltage amplitude on d (V)
HAL_PIN(l_id);      // *output*, resulting current amplitude on d (A)
HAL_PIN(l_vq);      // *output*, injected voltage amplitude on q (V)
HAL_PIN(l_iq);      // *output*, resulting current amplitude on q (A)
HAL_PIN(r_known);   // *parameter*, known winding resistance (ohm): fit only V0 and the knee, 0 = fit r too
HAL_PIN(dt_ideal);  // *parameter*, ideal dead time volts per link volt, 0 = 2 us * pwm_freq (0.03 at 15 kHz), default 0
HAL_PIN(pwm_freq);  // *input*, F3 PWM and rt rate (Hz), from hv0.pwm_freq, 0 = 15000
HAL_PIN(dt_v0);     // *output*, fitted per phase dead time volts at the link (V)
HAL_PIN(dt_knee);   // *output*, fitted knee of the dead time curve (A), result for hv0.drop_knee
HAL_PIN(dt_k);      // *output*, dt_v0 over the ideal dt_ideal * dc_volt, result for hv0.drop_k
HAL_PIN(dt_rms);    // *output*, residual of the r / dead time fit (V rms)
HAL_PIN(dt_top);    // *output*, largest top dwell current reached (A)
HAL_PIN(r_ok);      // *output*, 1 = the r test produced a resistance

HAL_PIN(pp);          // *output*, pole pairs, rounded, for conf0.polecount
HAL_PIN(pp_raw);      // *output*, field/rotor angle ratio before rounding, negative = reversed
HAL_PIN(pp_ok);       // *output*, 1 = rotor followed the field and ratio is near an integer
HAL_PIN(com_offset);  // *output*, commutation offset for conf0.mot_fb_offset (rad)
HAL_PIN(out_rev);     // *output*, 1 = feedback reversed, for conf0.out_rev, also hv0.rev

HAL_PIN(off_sin);  // *output*, sum of sin(pos_fb) over the offset dwell
HAL_PIN(off_cos);  // *output*, sum of cos(pos_fb) over the offset dwell
HAL_PIN(off_mag);  // *output*, resultant length of the offset dwell, 1 = rotor held still
HAL_PIN(off_n);    // *output*, samples in off_sin/off_cos
HAL_PIN(com_ok);   // *output*, 1 = offset dwell drew its current and the rotor held still

HAL_PIN(com_fb);         // *input*, commutation track (encf com_pos, fanuc_io, uvw halls) after com_rev (rad), fb_switch0.com_fb_no_offset; fb_switch commutates from it while mot_state is not absolute, e.g. an encf not yet indexed after a battery reset
HAL_PIN(com_rev);        // *parameter*, commutation track reversed if > 0, conf0.com_fb_rev
HAL_PIN(com_polecount);  // *parameter*, pole pairs of the commutation track, below 1 = pp, conf0.com_fb_polecount
HAL_PIN(com_fb_offset);  // *output*, commutation track offset for conf0.com_fb_offset (rad)
HAL_PIN(com_fb_ok);      // *output*, 1 = fine track, 2 = coarse (halls), 0 = none (did not move in the pp test)
HAL_PIN(mot_state);      // *input*, motor feedback state, 3 = absolute, fb_switch0.mot_state_fb

HAL_PIN(test_cur);  // *parameter*, test current (A peak), default 6
HAL_PIN(test_vel);  // *parameter*, test speed (rad/s), default 50: electrical in pp test, mechanical in psi test
HAL_PIN(ki);        // *parameter*, psi test speed loop integral gain (A/rad), default 1
HAL_PIN(vel_bw);    // *parameter*, psi test speed loop P gain, times period (A per rad/s), default 250

HAL_PIN(pwm_volt);  // *input*, usable phase voltage from hv0.pwm_volt (V)
HAL_PIN(dc_volt);   // *input*, dc link voltage from hv0.dc_volt (V), scales dt_k

HAL_PIN(psi);        // *output*, flux linkage from the F4 coast (Vs), printed as conf0.psi only when emf_ok is 0
HAL_PIN(psi_ok);     // *output*, 1 = the psi test produced a value
HAL_PIN(u_fb);       // *input*, phase u voltage to ground from hv0.u_fb (V), read while coasting
HAL_PIN(v_fb);       // *input*, phase v voltage to ground from hv0.v_fb (V)
HAL_PIN(psi_hi);     // *output*, psi from the fast half of the coast (Vs)
HAL_PIN(psi_lo);     // *output*, psi from the slow half of the coast (Vs)
HAL_PIN(emf_angle);  // *output*, back emf phase against the commutation angle at standstill (deg electrical)
HAL_PIN(emf_delay);  // *output*, delay of the phase voltage reading against the rotor angle (s)
HAL_PIN(vel_lo);     // *output*, mean speed of the low dwell (rad/s)
HAL_PIN(vel_hi);     // *output*, mean speed of the high dwell (rad/s)
HAL_PIN(udt_lo);     // *output*, dead time volts on q in the low dwell, uq - r iq - pp vel psi (V)
HAL_PIN(idt_lo);     // *output*, mean iq in the low dwell (A)
HAL_PIN(udt_hi);     // *output*, dead time volts on q in the high dwell (V)
HAL_PIN(idt_hi);     // *output*, mean iq in the high dwell (A)

HAL_PIN(emf_run);      // *output*, to hv0.emf_run, F3 emf0 map: 1 sum, 0 hold, -1 clear
HAL_PIN(emf_sel);      // *output*, to hv0.emf_sel, emf0 result to read back
HAL_PIN(emf_pp);       // *output*, to hv0.emf_pp, pole pairs for emf0's bins, abs(pp)
HAL_PIN(emf_val);      // *input*, emf0 result from hv0.emf_val
HAL_PIN(emf_ok);       // *output*, 1 = the F3 per pole map came back
HAL_PIN(emf_psi);      // *output*, psi over the whole coast from the F3's samples (Vs), printed as conf0.psi when emf_ok
HAL_PIN(emf_h5);       // *output*, 5th harmonic back emf, % of the fundamental
HAL_PIN(emf_h7);       // *output*, 7th harmonic back emf, % of the fundamental
HAL_PIN(pole_min);     // *output*, lowest pole bin's psi (Vs)
HAL_PIN(pole_max);     // *output*, highest pole bin's psi (Vs)
HAL_PIN(pole_spread);  // *output*, (pole_max - pole_min) / mean, %

HAL_PIN(cur_bw);     // *output*, current loop bandwidth to hv0.cur_bw: 1 or 300 in r test, 1 in l, 100 in pp/offset, 250 in psi
HAL_PIN(cur_sum);    // *output*, psi test speed loop integrator (A)
HAL_PIN(auto_step);  // *parameter*, run steps up to this number without a prompt, default 4.2 (all)

HAL_PIN(avg_test_volt);  // *output*, d bias voltage for the l test, r test_cur + V0 dproj (V)


// emf0 results: 9 totals, then psi re, im and samples for up to 16 pole bins
#define EMF_RB_BINS 16
#define EMF_RB_N (9 + 3 * EMF_RB_BINS)
// one config word out to the f3 and one state word back per result
#define EMF_RB_WAIT 0.02

// r test dwells: every current at each angle, full current first so the rotor
// aligns at the strongest field and stays; angles in pairs half a turn apart
// give each current both signs through the same phases
#define DW_ANGLES 4
#define DW_CURS 7
#define DW_N (DW_ANGLES * DW_CURS)
static const float dw_ang_tab[DW_ANGLES] = {0.0, M_PI, M_PI / 2.0, -M_PI / 2.0};
static const float dw_cur_tab[DW_CURS]   = {1.0, 0.67, 0.5, 0.33, 0.25, 0.17, 0.1};
#define DW_SETTLE 1.0    // per dwell before averaging [s]; the first at each angle gets twice
#define DW_ZERO 0.2      // 0 A at the start of each new angle [s], inside its settle
#define DW_AVG 0.5       // averaging [s]

// per phase share of the dead time loss, as hv.c's drop_knee curve shapes it
static float dt_shape(float i, float knee) {
  float a = 1.0 + ABS(i) / knee;
  float k = 1.0 - 1.0 / (a * a);
  return i >= 0.0 ? k : -k;
}
// what those three losses put on d when d carries i at electrical angle th
static float dt_dproj(float i, float th, float knee) {
  float c = cosf(th), s = sinf(th);
  float ia = i * c, ib = i * s;
  float su = dt_shape(ia, knee);
  float sv = dt_shape(-0.5 * ia + 0.5 * M_SQRT3 * ib, knee);
  float sw = dt_shape(-0.5 * ia - 0.5 * M_SQRT3 * ib, knee);
  float va = (2.0 * su - sv - sw) / 3.0, vb = (sv - sw) * M_SQRT1_3;
  return va * c + vb * s;
}

struct idpmsm_ctx_t {
  // l test: a sine injected on d, then on q, over a dc bias on d
  uint8_t l_axis;    // 0 d, 1 q
  uint8_t l_stage;   // 0 settle, 1 size the amplitude, 2 measure
  uint16_t l_block;  // sizing blocks done
  uint32_t l_n;      // samples in this block
  float l_t;         // time in this stage or block
  float l_th;        // injection phase
  float l_amp;       // injection voltage amplitude
  float v_re, v_im;  // voltage demodulated at the injection frequency
  float i_re, i_im;  // current, the same
  float v_dc, i_dc;  // plain sums of v and i, and of the reference sin/cos,
  float s_dc, c_dc;  // so the dc under the sine can be taken out

  // pp test
  uint32_t pp_n;     // ticks in the measure window where the rotor was turning
  uint32_t pp_w;     // ticks in the measure window
  float pp_field;    // field angle turned across the window [rad el]
  float pp_rotor;    // rotor angle turned across the window [rad]
  float com_last;    // com track in the pp test: last value,
  float com_travel;  // total movement [rad]
  float com_step;    // and largest single step [rad]
  float com_sin, com_cos;  // com track sums in the offset dwell

  // psi test: dwells at test_vel / 2 and test_vel, then a coast from the top
  uint8_t psi_stage;  // 0 spin up low, 1 dwell low, 2 spin up high, 3 dwell high, 4 coast
  uint8_t psi_fail;   // 0 none, 1 stalled, 2 never settled, 3 coast too short
  uint32_t psi_n;     // samples in this dwell
  float st_t;         // time in this stage
  float settle_t;     // time the filtered speed has been inside the band
  float vel_lp;       // filtered vel_fb, for settling and stall detection
  float sx;           // sum of pp * vel_fb across the dwell
  float sy;           // sum of uq_fb - r * iq_fb across the dwell
  float si;           // sum of iq_fb across the dwell
  float x_lo, y_lo, i_lo;
  float x_hi, y_hi, i_hi;
  // coast, per band (0 fast, 1 slow): the line to line voltage demodulated
  // against the commutation angle, see the psi case in rt_func
  uint32_t cn[2];
  float cz_re[2], cz_im[2];  // sum of (u - v) * g
  float cg_re[2], cg_im[2];  // sum of g, to take out the dc level of u - v
  float cu[2];               // sum of u - v
  float cw[2];               // sum of the electrical speed

  // dead time curve fit dwells
  uint8_t dw;            // dwell index
  float dw_t;            // time in this dwell
  uint32_t dw_n;         // samples averaged in it
  float dw_si, dw_su;    // sums of id_fb and ud_fb
  float dw_ang[DW_N];    // com_pos of each dwell [rad el]
  float dw_i[DW_N];      // mean id_fb
  float dw_u[DW_N];      // mean ud_fb

  // emf0 read back: one result per EMF_RB_WAIT, see the emf0 layout on the f3
  uint32_t rb_i;
  float rb_t;
  float rb[EMF_RB_N];
};

// hv0.u_fb/v_fb come through io.c's u = a * adc + (1 - a) * u per f3 tick,
// a = 750 / f (0.05 at 15 kHz, tau 1.33 ms), which the coast undoes
#define HV_IO_ALPHA(f) (750.0 / (f))
#define HV_DEADTIME 2.0e-6  // f3 PWM_DEADTIME [s]
// the f3 sends one word of its state block per packet, so u_fb and v_fb
// each refresh every HV_STATE_WORDS rt periods: a zero order hold that
// scales the back emf by sinc(w T / 2)
#define HV_STATE_WORDS (sizeof(f3_state_data_t) / 4)

#define L_SETTLE 0.1  // l test: after each axis starts [s]
#define L_BLOCK 0.05  // l test: one amplitude sizing block [s]
#define L_BLOCKS 4    // l test: sizing blocks per axis
#define L_MEASURE 0.4 // l test: demodulation window per axis [s]
#define PP_RAMP 1.0      // pp test: time to ramp the field up to test_vel [s]
#define PP_SETTLE 0.5    // pp test: wait after the ramp before measuring [s]
#define PP_TIME 4.0      // pp test: total [s]
#define PSI_SETTLE 0.5   // psi test: speed inside the band this long [s]
#define PSI_BAND 0.15    // psi test: settled band, fraction of the dwell speed
#define PSI_DWELL 2.0    // psi test: averaging time per dwell [s]
#define PSI_SPINUP 8.0   // psi test: give up reaching a dwell speed after [s]
#define PSI_STALL 0.5    // psi test: q current, as a fraction of test_cur, that has to turn the rotor
#define PSI_COAST_MIN 0.2   // psi coast: ignore speeds under this fraction of test_vel
#define PSI_COAST_SPLIT 0.6 // psi coast: the fast band is above this fraction of test_vel
#define PSI_COAST_TIME 4.0  // psi coast: longest it may run [s]
#define PSI_COAST_HOLDOFF 0.05 // psi coast: wait after the bridge drops, for the winding current to die [s]
#define PSI_COAST_N 200     // psi coast: fewest samples a band needs

static float hv_pwm_freq(struct idpmsm_pin_ctx_t *pins) {
  return PIN(pwm_freq) > 0.0 ? PIN(pwm_freq) : 15000.0;
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;
  PIN(r_known)  = 0.0;
  PIN(dt_ideal) = 0.0;
  PIN(test_cur) = 6.0;
  // 0: at the current loop's crossover, loop_bw / 2 pi (480 Hz at cur_bw
  // 3000), where the loop uses l; on Y that reads Ld/Lq ~5% under 250 Hz
  PIN(l_freq)   = 0.0;
  PIN(l_ripple) = 0.15;
  // the lowest speed where the coast's two halves agree on X; the pp test's
  // field runs at this many electrical rad/s
  PIN(test_vel)  = 50.0;
  PIN(ki)        = 1.0;
  PIN(vel_bw)    = 250.0;  // P = vel_bw * period, 0.05 A per rad/s at 5 kHz
  PIN(cur_bw)    = 1.0;
  PIN(auto_step) = 4.2;
}
// ctx survives a stop
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idpmsm_ctx_t *ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idpmsm_ctx_t));
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idpmsm_ctx_t *ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(r)          = 0.1;
      PIN(dt_v0)      = 0.0;
      PIN(dt_knee)    = 0.0;
      PIN(dt_k)       = 0.0;
      PIN(dt_rms)     = 0.0;
      PIN(l)          = 0.001;
      PIN(ld)         = 0.0;
      PIN(lq)         = 0.0;
      PIN(psi)        = 0.055;
      PIN(pp)         = 3.0;
      PIN(com_offset) = 0.0;
      PIN(out_rev)    = 0.0;
      PIN(cur_bw)     = 1.0;
      PIN(pp_ok)      = 0.0;
      PIN(com_ok)     = 0.0;
      PIN(psi_ok)     = 0.0;
      PIN(com_fb_offset) = 0.0;
      PIN(com_fb_ok)  = 0.0;
      break;

    case 10:  // r
      PIN(state)    = 1.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;

      if(PIN(auto_step) >= 1) {
        PIN(state) = 1.2;
      } else {
        printf("Measure r, l\n");
        printf("the motor can move a bit\n");
        printf("idpmsm0.state = 1.2 <font color='green'>to start</font>\n");
      }
      break;

    case 15: {  // r and dead time curve fit on the dwells rt took
      // grid over the knee, least squares for r and V0 at each (V0 alone with
      // r_known); the knee with the smallest residual wins
      float best = 1e9, br = 0.0, bv = 0.0, bk = 0.0;
      for(int kk = 0; kk <= 78; kk++) {
        float knee = 0.05 + 0.025 * kk;
        float sxx = 0.0, sxg = 0.0, sgg = 0.0, sxu = 0.0, sgu = 0.0;
        for(int n = 0; n < DW_N; n++) {
          float x = ctx->dw_i[n], g = dt_dproj(ctx->dw_i[n], ctx->dw_ang[n], knee), u = ctx->dw_u[n];
          sxx += x * x;
          sxg += x * g;
          sgg += g * g;
          sxu += x * u;
          sgu += g * u;
        }
        float r, v0;
        if(PIN(r_known) > 0.0) {  // r fixed, v0 alone: sum g (u - r i) / sum g^2
          if(sgg < 1e-9) {
            continue;
          }
          r  = PIN(r_known);
          v0 = (sgu - r * sxg) / sgg;
        } else {
          float det = sxx * sgg - sxg * sxg;
          if(ABS(det) < 1e-9) {
            continue;
          }
          r  = (sxu * sgg - sgu * sxg) / det;
          v0 = (sgu * sxx - sxu * sxg) / det;
        }
        float e  = 0.0;
        for(int n = 0; n < DW_N; n++) {
          float d = r * ctx->dw_i[n] + v0 * dt_dproj(ctx->dw_i[n], ctx->dw_ang[n], knee) - ctx->dw_u[n];
          e += d * d;
        }
        if(e < best) {
          best = e;
          br   = r;
          bv   = v0;
          bk   = knee;
        }
      }
      // the top dwell has to have got near its command
      float top = 0.0;
      for(int a = 0; a < DW_ANGLES; a++) {
        top = MAX(top, ctx->dw_i[a * DW_CURS]);
      }
      float r_ok     = (top > PIN(test_cur) * 0.5 && br > 0.001 && bv >= 0.0) ? 1.0 : 0.0;
      PIN(dt_top)    = top;
      PIN(dt_rms)    = sqrtf(best / DW_N);
      PIN(dt_v0)     = bv;
      PIN(dt_knee)   = bk;
      float dt_ideal = PIN(dt_ideal) > 0.0 ? PIN(dt_ideal) : HV_DEADTIME * hv_pwm_freq(pins);
      PIN(dt_k)      = PIN(dc_volt) > 1.0 ? bv / (dt_ideal * PIN(dc_volt)) : 0.0;
      PIN(r_ok)      = r_ok;
      if(r_ok > 0.0) {
        PIN(r)             = br;
        PIN(avg_test_volt) = LIMIT(br * PIN(test_cur) + bv * dt_dproj(PIN(test_cur), 0.0, bk), PIN(pwm_volt) / 2.0);
      } else {
        PIN(avg_test_volt) = 0.0;
      }
      // set up the l test, then hand back to rt
      ctx->l_axis  = 0;
      ctx->l_stage = 0;
      ctx->l_block = 0;
      ctx->l_n     = 0;
      ctx->l_t     = 0.0;
      ctx->l_th    = 0.0;
      ctx->l_amp   = 1.0;
      ctx->v_re = ctx->v_im = ctx->i_re = ctx->i_im = 0.0;
      PIN(timer) = 0.0;
      PIN(state) = r_ok > 0.0 ? 1.3 : 1.4;
    } break;

    case 14:
      if(PIN(r_ok) > 0.0) {
        printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
        if(PIN(l_ok) > 0.0) {
          printf("conf0.l = %f <font color='green'># append to config</font>\n", PIN(ld));
          printf("conf0.lq = %f <font color='green'># append to config</font>\n", PIN(lq));
          printf("<font color='green'># Ld and Lq at %f A on d, from a %f Hz injection\n", PIN(test_cur), PIN(l_f));
          printf("# (%f V -> %f A on d, %f V -> %f A on q).</font>\n", PIN(l_vd), PIN(l_id), PIN(l_vq), PIN(l_iq));
        } else {
          printf("<font color='red'>l not measured</font>: the injection drew %f A on d and %f A on q\n", PIN(l_id), PIN(l_iq));
          printf("of the %f A it aimed for. check idpmsm0.iq_fb/uq_fb are wired\n", PIN(l_ripple) * PIN(test_cur));
        }
        if(PIN(dc_volt) > 1.0) {
          printf("hv0.drop_k = %f <font color='green'># append to config</font>\n", PIN(dt_k));
        } else {
          printf("<font color='red'>hv0.drop_k not computed</font>: idpmsm0.dc_volt reads %f V, wire it to hv0.dc_volt\n", PIN(dc_volt));
        }
        printf("hv0.drop_knee = %f <font color='green'># append to config</font>\n", PIN(dt_knee));
        if(PIN(dt_knee) < 0.06 || PIN(dt_knee) > 1.99) {
          printf("<font color='red'>knee at the edge of the 0.05 to 2.0 A grid</font>: the dead time fit is unreliable,\n");
          printf("check the residual below and keep the old hv0.drop_k and hv0.drop_knee if it is large\n");
        }
        if(PIN(r_known) > 0.0) {
          printf("<font color='green'># dead time curve fitted over %i d dwells with r held at the %f you gave\n", DW_N, PIN(r_known));
        } else {
          printf("<font color='green'># r and the dead time curve fitted together over %i d dwells\n", DW_N);
        }
        printf("# (%i angles, %f to %f A): %f V per phase at %f V link, knee %f A,\n", DW_ANGLES, PIN(test_cur) * dw_cur_tab[DW_CURS - 1], PIN(test_cur), PIN(dt_v0), PIN(dc_volt), PIN(dt_knee));
        printf("# residual %f V rms.</font>\n", PIN(dt_rms));
      } else {
        printf("<font color='red'>r read failed</font>: the top dwell reached %f of %f A\n", PIN(dt_top), PIN(test_cur));
        printf("nothing below is measured, do not append it\n");
        printf("check that idpmsm0.test_cur (%f) is under conf0.max_ac_cur\n", PIN(test_cur));
      }
      // walk on only if r worked: hv0.r is this pin. 9.0 has no rt case, so
      // en_out stays off until the drive is disabled
      PIN(state) = PIN(r_ok) > 0.0 ? 2.0 : 9.0;
      break;

    case 20:  // pp, out_rev, com_offset
      PIN(state) = 2.1;
      PIN(timer) = 0.0;
      //PIN(d_cmd) = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(pp_ok)    = 0.0;
      ctx->pp_n     = 0;
      ctx->pp_w     = 0;
      ctx->pp_field = 0.0;
      ctx->pp_rotor = 0.0;
      ctx->com_last   = PIN(com_fb);
      ctx->com_travel = 0.0;
      ctx->com_step   = 0.0;
      ctx->com_sin    = 0.0;
      ctx->com_cos    = 0.0;
      PIN(com_fb_ok)  = 0.0;

      if(PIN(auto_step) >= 2) {
        PIN(state) = 2.2;
      } else {
        printf("Measure com_offset, polepairs, out_rev\n");
        printf("the motor will move\n");
        printf("idpmsm0.state = 2.2 <font color='green'>to start</font>\n");
      }
      break;

    case 24:  // pp failed, nothing downstream of it can run
      printf("<font color='red'>pp read failed</font>: ratio %f", PIN(pp_raw));
      printf(", the rotor turned for %lu of %lu ticks\n", (unsigned long)ctx->pp_n, (unsigned long)ctx->pp_w);
      printf("the rotor did not follow the field. lower idpmsm0.test_vel\n");
      printf("(%f el. rad/s) or raise idpmsm0.test_cur and rerun\n", PIN(test_vel));
      printf("nothing below is measured, do not append it\n");
      PIN(state) = 9.0;
      break;

    case 26:  // com_offset failed
      if(PIN(off_mag) <= 0.98) {
        printf("<font color='red'>mot_fb_offset not measured</font>: the rotor moved during the dwell\n");
        printf("(resultant %f, 1.0 means it held still). raise idpmsm0.test_cur.\n", PIN(off_mag));
      } else {
        printf("<font color='red'>mot_fb_offset not measured</font>: the dwell drew %f A of %f\n", PIN(id_fb), PIN(test_cur));
      }
      printf("nothing below is measured, do not append it\n");
      PIN(state) = 9.0;
      break;

    case 25:  // pp, out_rev, com_offset
      // checked after the move: an encoder that indexes during the pp test is fine
      if(PIN(mot_state) != 3.0) {
        printf("<font color='red'>motor feedback not absolute</font> (fb_switch0.mot_state %f):\n", PIN(mot_state));
        printf("the offsets below are relative to power-up. index the encoder first\n");
      }
      printf("conf0.polecount = %f <font color='green'># append to config</font>\n", PIN(pp));
      printf("conf0.mot_fb_offset = %f <font color='green'># append to config</font>\n", PIN(com_offset));
      if(PIN(com_fb_ok) > 0.0) {
        printf("conf0.com_fb_offset = %f <font color='green'># append to config</font>\n", PIN(com_fb_offset));
        if(PIN(com_fb_ok) > 1.0) {
          printf("# coarse commutation track (halls): com_fb_offset up to 30 deg el off\n");
        }
      }
      if(PIN(out_rev) > 0.0) {
        printf("conf0.out_rev = 1 <font color='green'># append to config</font>\n");
      }
      PIN(state) = 3.0;
      break;

    case 30:
      PIN(state)    = 3.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_sum)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(cur_bw)   = 250.0;
      PIN(psi_ok)    = 0.0;
      PIN(psi_hi)    = 0.0;
      PIN(psi_lo)    = 0.0;
      PIN(emf_angle) = 0.0;
      PIN(emf_delay) = 0.0;
      PIN(vel_lo)    = 0.0;
      PIN(vel_hi)    = 0.0;
      PIN(udt_lo)    = 0.0;
      PIN(idt_lo)    = 0.0;
      PIN(udt_hi)    = 0.0;
      PIN(idt_hi)    = 0.0;
      PIN(emf_ok)      = 0.0;
      PIN(emf_psi)     = 0.0;
      PIN(emf_h5)      = 0.0;
      PIN(emf_h7)      = 0.0;
      PIN(pole_min)    = 0.0;
      PIN(pole_max)    = 0.0;
      PIN(pole_spread) = 0.0;
      PIN(emf_run)     = -1.0;
      PIN(emf_sel)     = 0.0;
      ctx->rb_i        = 0;
      ctx->rb_t        = 0.0;
      ctx->psi_stage = 0;
      ctx->psi_fail  = 0;
      ctx->psi_n     = 0;
      ctx->st_t      = 0.0;
      ctx->settle_t  = 0.0;
      ctx->vel_lp    = 0.0;
      ctx->sx        = 0.0;
      ctx->sy        = 0.0;
      ctx->si        = 0.0;
      for(int b = 0; b < 2; b++) {
        ctx->cn[b]    = 0;
        ctx->cz_re[b] = 0.0;
        ctx->cz_im[b] = 0.0;
        ctx->cg_re[b] = 0.0;
        ctx->cg_im[b] = 0.0;
        ctx->cu[b]    = 0.0;
        ctx->cw[b]    = 0.0;
      }

      if(PIN(auto_step) >= 3) {
        PIN(state) = 3.2;
      } else {
        printf("Measure torque constant\n");
        printf("the motor will move\n");
        printf("idpmsm0.state = 3.2 <font color='green'>to start</font>\n");
      }
      break;

    case 33:
      if(PIN(psi_ok) > 0.0) {
        // the f3 map sums every pwm sample with no filter and no hold in it,
        // so it is the value to keep; the f4 coast is the cross-check
        if(PIN(emf_ok) > 0.0) {
          printf("conf0.psi = %f <font color='green'># append to config</font>\n", PIN(emf_psi));
          printf("<font color='green'># from the f3 back emf map; the f4 coast read %f.\n", PIN(psi));
        } else {
          printf("conf0.psi = %f <font color='green'># append to config</font>\n", PIN(psi));
          printf("<font color='green'>");
        }
        printf("# back emf with the bridge off, coasting down from %f rad/s:\n", PIN(vel_hi));
        printf("# %f in the fast half, %f in the slow half. no dead time or\n", PIN(psi_hi), PIN(psi_lo));
        printf("# current loop in it -- it is what a scope on the phases reads.</font>\n");
        printf("<font color='green'># back emf leads the commutation angle by %f deg el at standstill,\n", PIN(emf_angle));
        printf("# and reads %f ms later than the rotor angle. with conf0.mot_fb_offset\n", PIN(emf_delay) * 1000.0);
        printf("# right the angle repeats from run to run; a shift of it is an offset\n");
        printf("# error of that many electrical degrees.</font>\n");
        // +120 deg el expected at out_rev 0, +60 at out_rev 1: an offset check
        // with no current in it
        {
          float expect = PIN(out_rev) > 0.0 ? 60.0 : 120.0;
          printf("<font color='green'># emf_angle - %f = %f deg el: the offset check.</font>\n", expect, PIN(emf_angle) - expect);
        }
        printf("<font color='green'># dead time on q while turning, uq - r iq - pp vel psi:\n");
        printf("# %f V at %f A (%f rad/s), %f V at %f A (%f rad/s).</font>\n", PIN(udt_lo), PIN(idt_lo), PIN(vel_lo), PIN(udt_hi), PIN(idt_hi), PIN(vel_hi));
        if(PIN(emf_ok) > 0.0) {
          int bins = CLAMP((int)(ctx->rb[8] + 0.5), 2, EMF_RB_BINS);
          printf("<font color='green'># f3 back emf map, every pwm sample of the coast:\n");
          printf("# psi %f, 5th harmonic %f %%, 7th %f %%\n", PIN(emf_psi), PIN(emf_h5), PIN(emf_h7));
          printf("# per pole (bin 0 is arbitrary, the order is the rotor's):\n#");
          for(int b = 0; b < bins; b++) {
            float *r = &ctx->rb[9 + 3 * b];
            printf(" %f", sqrtf(r[0] * r[0] + r[1] * r[1]));
          }
          printf("\n# spread %f %% of the mean. a uniformly weak rotor reads flat;\n", PIN(pole_spread));
          printf("# a demagnetised pole reads low in the same place on every run.</font>\n");
        } else {
          printf("<font color='orange'># f3 back emf map did not come back: check the f3 image has emf0\n");
          printf("# and hv0.emf_run/emf_sel/emf_pp, idpmsm0.emf_val are linked</font>\n");
        }
        printf("done\n");
        printf("continue with id_mot\n");
      } else {
        if(ctx->psi_fail == 1) {
          printf("<font color='red'>psi read failed</font>: the rotor stalled\n");
          printf("check conf0.polecount, conf0.mot_fb_offset and out_rev above\n");
        } else if(ctx->psi_fail == 2) {
          printf("<font color='red'>psi read failed</font>: the speed never settled inside %i%%\n", (int)(PSI_BAND * 100.0));
          printf("of the dwell. check the load is off, or lower idpmsm0.test_vel\n");
        } else {
          printf("<font color='red'>psi read failed</font>: the coast gave %lu and %lu usable samples\n", (unsigned long)ctx->cn[0], (unsigned long)ctx->cn[1]);
          printf("check idpmsm0.u_fb/v_fb are wired to hv0.u_fb/v_fb, or raise idpmsm0.test_vel\n");
        }
        printf("nothing here is measured, do not append it\n");
      }

      PIN(state) = 3.4;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idpmsm_ctx_t *ctx        = (struct idpmsm_ctx_t *)ctx_ptr;
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  PIN(emf_pp) = ABS(PIN(pp));

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(en_out)   = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1.0;
      }
      break;

    case 12:  // r
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 1.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;

      // ud_fb, the loop's voltage command, with drop_k 0 (the template):
      //
      //   ud = r * id + V0 * dproj(id, th, knee)
      //
      // dproj is the d projection of the three phase losses sign(i) * k(i),
      // k = 1 - 1/(1 + |i|/knee)^2, the curve hv.c compensates with. Seven
      // currents from 0.1 to 1 x test_cur bend the curve enough to separate r
      // from the loss. The first 2 s at test_cur and cur_bw 1 bootstrap hv0.r
      // (the loop's plant model, this pin) so current flows at all.
      if(PIN(timer) < 2.0) {
        PIN(d_cmd) = PIN(test_cur);
        PIN(r)     = PIN(r) * 0.99 + PIN(ud_fb) / MAX(PIN(id_fb), 0.01) * 0.01;
        PIN(timer) += period;
        ctx->dw    = 0;
        ctx->dw_t  = 0.0;
        ctx->dw_n  = 0;
        ctx->dw_si = ctx->dw_su = 0.0;
        break;
      }
      PIN(cur_bw) = 300.0;
      // The bootstrap r is ud / id at test_cur with drop_k 0, so it carries
      // the dead time volts (about 2.3x the winding on X at 12 A). The loop's
      // integral gain is cur_bw * r, and with it the steps to test_cur at the
      // next angles overshoot into the f3's peak trip. Use r_known when it is
      // given, else the slope between the first two dwells below, where the
      // dead time loss is flat and cancels.
      if(PIN(r_known) > 0.0 && ctx->dw == 0) {
        PIN(r) = PIN(r_known);
      }
      {
        int a        = ctx->dw / DW_CURS, k = ctx->dw % DW_CURS;
        PIN(com_pos) = dw_ang_tab[a];
        PIN(d_cmd)   = PIN(test_cur) * dw_cur_tab[k];
        // a new angle starts from 0 A, not from the last (lowest) dwell of
        // the old one, so the angle jump and the step to test_cur are apart
        if(a > 0 && k == 0 && ctx->dw_t < DW_ZERO) {
          PIN(d_cmd) = 0.0;
        }
        float settle = k == 0 ? 2.0 * DW_SETTLE : DW_SETTLE;
        ctx->dw_t += period;
        if(ctx->dw_t > settle) {
          ctx->dw_si += PIN(id_fb);
          ctx->dw_su += PIN(ud_fb);
          ctx->dw_n++;
        }
        if(ctx->dw_t > settle + DW_AVG) {
          float n              = MAX((float)ctx->dw_n, 1.0);
          ctx->dw_ang[ctx->dw] = dw_ang_tab[a];
          ctx->dw_i[ctx->dw]   = ctx->dw_si / n;
          ctx->dw_u[ctx->dw]   = ctx->dw_su / n;
          ctx->dw++;
          if(ctx->dw == 2 && PIN(r_known) <= 0.0) {  // test_cur and 0.67 test_cur, angle 0
            float di = ctx->dw_i[0] - ctx->dw_i[1];
            if(di > 0.1) {
              PIN(r) = CLAMP((ctx->dw_u[0] - ctx->dw_u[1]) / di, 0.01, PIN(r));
            }
          }
          ctx->dw_t  = 0.0;
          ctx->dw_n  = 0;
          ctx->dw_si = ctx->dw_su = 0.0;
          if(ctx->dw >= DW_N) {  // the fit runs in nrt, it is too slow for a tick
            PIN(d_cmd)  = 0.0;
            PIN(en_out) = 0.0;
            PIN(timer)  = 0.0;
            PIN(state)  = 1.5;
          }
        }
      }
      break;

    case 15:  // r fit running in nrt
      PIN(en_out) = 0.0;
      PIN(d_cmd)  = 0.0;
      PIN(q_cmd)  = 0.0;
      break;

    case 13: {  // ld and lq, by injection
      // A sine on d, then on q, over the dc bias that holds test_cur on d,
      // demodulated at its own frequency: |z| = |r + j w l|. The bias keeps
      // every phase current on one side of zero, so the dead time is a
      // constant offset. ud_fb/id_fb come back in the same ls packet, so their
      // delay drops out; the f3's 5 kHz hold scales the fundamental by
      // sin(pi f T) / (pi f T), taken back out.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;  // volt cmd
      PIN(cur_bw)   = 1.0;
      PIN(com_pos)  = 0.0;

      float f = PIN(l_freq) > 0.0 ? PIN(l_freq) : (PIN(loop_bw) > 0.0 ? PIN(loop_bw) : 1000.0) / (2.0 * M_PI);
      f       = CLAMP(f, 20.0, 0.2 / period);
      // a multiple of 1 / L_MEASURE, so the window holds whole cycles
      f        = MAX(roundf(f * L_MEASURE), 1.0) / L_MEASURE;
      PIN(l_f) = f;
      float w = 2.0 * M_PI * f;
      ctx->l_th += w * period;
      if(ctx->l_th > 2.0 * M_PI) {
        ctx->l_th -= 2.0 * M_PI;
      }
      float sn, cs;
      sincos_fast(ctx->l_th, &sn, &cs);
      float inj  = ctx->l_amp * sn;
      PIN(d_cmd) = PIN(avg_test_volt) + (ctx->l_axis == 0 ? inj : 0.0);
      PIN(q_cmd) = ctx->l_axis == 1 ? inj : 0.0;

      float v = ctx->l_axis == 0 ? PIN(ud_fb) : PIN(uq_fb);
      float i = ctx->l_axis == 0 ? PIN(id_fb) : PIN(iq_fb);
      ctx->l_t += period;

      if(ctx->l_stage == 0) {  // let the bias and the new axis settle
        if(ctx->l_t >= L_SETTLE) {
          ctx->l_stage = 1;
          ctx->l_t     = 0.0;
          ctx->l_n     = 0;
          ctx->v_re = ctx->v_im = ctx->i_re = ctx->i_im = 0.0;
          ctx->v_dc = ctx->i_dc = ctx->s_dc = ctx->c_dc = 0.0;
        }
      } else {
        ctx->v_re += v * sn;
        ctx->v_im += v * cs;
        ctx->i_re += i * sn;
        ctx->i_im += i * cs;
        ctx->v_dc += v;
        ctx->i_dc += i;
        ctx->s_dc += sn;
        ctx->c_dc += cs;
        ctx->l_n++;
        float n = MAX((float)ctx->l_n, 1.0);
        // take the dc bias out with the mean times the reference sums, and
        // end the window on whole cycles
        float vm = ctx->v_dc / n, im = ctx->i_dc / n;
        float vr = ctx->v_re - vm * ctx->s_dc, vi = ctx->v_im - vm * ctx->c_dc;
        float ir = ctx->i_re - im * ctx->s_dc, ii = ctx->i_im - im * ctx->c_dc;
        if(ctx->l_stage == 1 && ctx->l_t >= L_BLOCK) {
          // size the amplitude to the ripple asked for: enough signal, and
          // never so much that a phase current crosses zero
          float i1     = 2.0 / n * sqrtf(ir * ir + ii * ii);
          float target = PIN(l_ripple) * PIN(test_cur);
          float k      = i1 > 0.001 ? target / i1 : 4.0;
          ctx->l_amp *= CLAMP(k, 0.25, 4.0);
          ctx->l_amp = CLAMP(ctx->l_amp, 0.2, PIN(pwm_volt) / 4.0);
          ctx->l_block++;
          ctx->l_t = 0.0;
          ctx->l_n = 0;
          ctx->v_re = ctx->v_im = ctx->i_re = ctx->i_im = 0.0;
          ctx->v_dc = ctx->i_dc = ctx->s_dc = ctx->c_dc = 0.0;
          if(ctx->l_block >= L_BLOCKS) {
            ctx->l_stage = 2;
          }
        } else if(ctx->l_stage == 2 && ctx->l_n >= (uint32_t)(MAX(roundf(L_MEASURE * f), 1.0) / f / period + 0.5)) {
          float v1   = 2.0 / n * sqrtf(vr * vr + vi * vi);
          float i1   = 2.0 / n * sqrtf(ir * ir + ii * ii);
          float x    = M_PI * f * period;
          float zoh  = sinf(x) / x;
          float z    = i1 > 0.001 ? v1 * zoh / i1 : 0.0;
          float lval = z > PIN(r) ? sqrtf(z * z - PIN(r) * PIN(r)) / w : 0.0;
          if(ctx->l_axis == 0) {
            PIN(ld)   = lval;
            PIN(l_vd) = v1;
            PIN(l_id) = i1;
          } else {
            PIN(lq)   = lval;
            PIN(l_vq) = v1;
            PIN(l_iq) = i1;
          }
          if(ctx->l_axis == 0) {  // on to q, starting from d's amplitude
            ctx->l_axis  = 1;
            ctx->l_stage = 0;
            ctx->l_block = 0;
            ctx->l_t     = 0.0;
            ctx->l_n     = 0;
          } else {
            float target = PIN(l_ripple) * PIN(test_cur);
            int ok_d     = PIN(ld) > 0.0 && PIN(l_id) > target * 0.5 && PIN(l_id) < target * 2.0;
            int ok_q     = PIN(lq) > 0.0 && PIN(l_iq) > target * 0.5 && PIN(l_iq) < target * 2.0;
            PIN(l_ok)    = ok_d && ok_q;
            // l is the d axis, as conf0.l is
            PIN(l)       = PIN(l_ok) > 0.0 ? PIN(ld) : 0.0;
            PIN(timer)   = 0.0;
            PIN(state)   = 1.4;
            PIN(d_cmd)   = 0.0;
            PIN(q_cmd)   = 0.0;
            PIN(en_out)  = 0.0;
          }
        }
      }
      break;
    }

    case 22:  // pp, out_rev
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      // ramp the field: a step to test_vel lets the rotor slip
      {
        float field_vel = PIN(test_vel) * MIN(PIN(timer) / PP_RAMP, 1.0);
        PIN(com_pos) += field_vel * period;
        PIN(com_pos) = mod(PIN(com_pos));

        // once locked on at full speed: the ratio of the angles turned over
        // the window, which cogging does not move
        if(PIN(timer) >= PP_RAMP + PP_SETTLE) {
          ctx->pp_w++;
          ctx->pp_field += field_vel * period;
          ctx->pp_rotor += PIN(vel_fb) * period;
          if(ABS(PIN(vel_fb)) > 0.1) {
            ctx->pp_n++;
          }
        }

        // does a commutation track follow the rotor, and in what steps
        // (in its own angle units: halls step 60 deg, encf about 0.35 deg)
        float step = ABS(minus(PIN(com_fb), ctx->com_last));
        ctx->com_last = PIN(com_fb);
        ctx->com_travel += step;
        ctx->com_step = MAX(ctx->com_step, step);
      }

      PIN(timer) += period;
      if(PIN(timer) >= PP_TIME) {
        PIN(timer)  = 0.0;
        PIN(pp)     = ABS(ctx->pp_rotor) > 0.01 ? ctx->pp_field / ctx->pp_rotor : 0.0;
        PIN(pp_raw) = PIN(pp);

        if(PIN(pp) < 0.0) {
          PIN(out_rev) = 1.0;
          PIN(pp) *= -1.0;
        }
        float pp_int = (int)(PIN(pp) + 0.5);

        // a rotor that followed the field turned the whole window and lands
        // near an integer; one that slipped does not
        PIN(pp_ok) = ctx->pp_n > ctx->pp_w * 9 / 10 && pp_int >= 1.0 && pp_int <= 24.0 && ABS(PIN(pp) - pp_int) < 0.15;
        PIN(pp)    = pp_int;

        PIN(off_sin) = 0.0;
        PIN(off_cos) = 0.0;
        PIN(off_n)   = 0.0;

        if(PIN(pp_ok) > 0.0) {
          PIN(state) = 2.3;
        } else {
          PIN(en_out) = 0.0;
          PIN(d_cmd)  = 0.0;
          PIN(state)  = 2.4;
        }
      }
      break;

    case 23:  // mot_fb_offset
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(com_pos) = 0.0;

      // back half of the dwell only, after the rotor stopped ringing
      PIN(timer) += period;
      if(PIN(timer) > 1.0) {
        PIN(off_sin) += sinf(PIN(pos_fb));
        PIN(off_cos) += cosf(PIN(pos_fb));
        PIN(off_n) += 1.0;
        // the commutation track at the same rotor angle, electrical
        float cpp = PIN(com_polecount) >= 1.0 ? PIN(com_polecount) : PIN(pp);
        ctx->com_sin += sinf(PIN(com_fb) * PIN(pp) / cpp);
        ctx->com_cos += cosf(PIN(com_fb) * PIN(pp) / cpp);
      }

      if(PIN(timer) >= 2.0) {
        float n       = MAX(PIN(off_n), 1.0);
        PIN(off_mag)  = sqrtf(PIN(off_sin) * PIN(off_sin) + PIN(off_cos) * PIN(off_cos)) / n;
        float com_off = -atan2f(PIN(off_sin), PIN(off_cos));

        // the rotor parks at any of pp alignments: fold into [0, 2 pi / pp)
        // so every run prints the same offset
        if(PIN(pp) >= 1.0) {
          float fold = 2.0 * M_PI / PIN(pp);
          com_off    = com_off - fold * floorf(com_off / fold);
        }
        PIN(com_offset) = com_off;

        // fb_switch commutates mod((com + com_fb_offset) * pp / cpp), both
        // negated under com_rev, and the rotor sits at electrical 0 here:
        // com_fb_offset = -com, com_fb_no_offset carrying com_rev's sign
        if(ctx->com_travel > M_PI && PIN(pp) >= 1.0) {
          float cpp          = PIN(com_polecount) >= 1.0 ? PIN(com_polecount) : PIN(pp);
          float com_el       = atan2f(ctx->com_sin, ctx->com_cos);
          PIN(com_fb_offset) = -com_el * cpp / PIN(pp) * (PIN(com_rev) > 0.0 ? -1.0 : 1.0);
          PIN(com_fb_ok)     = ctx->com_step > 0.5 ? 2.0 : 1.0;
        } else {
          PIN(com_fb_offset) = 0.0;
          PIN(com_fb_ok)     = 0.0;
        }
        PIN(com_ok)     = (PIN(off_mag) > 0.98 && PIN(id_fb) > PIN(test_cur) * 0.5) ? 1.0 : 0.0;

        PIN(d_cmd) = 0.0;
        PIN(timer) = 0.0;
        // the psi test commutates from this offset: stop on a bad one
        if(PIN(com_ok) > 0.0) {
          PIN(state) = 2.5;
        } else {
          PIN(en_out) = 0.0;
          PIN(state)  = 2.6;
        }
      }
      break;

    case 32:  // psi
      {
        float v_t = ctx->psi_stage < 2 ? PIN(test_vel) * 0.5 : PIN(test_vel);
        ctx->vel_lp += (PIN(vel_fb) - ctx->vel_lp) * period / 0.05;
        ctx->st_t += period;

        if(ctx->psi_stage < 4) {
          PIN(en_out)   = 1.0;
          PIN(cmd_mode) = 1.0;
          PIN(cur_bw)   = 250.0;
          PIN(d_cmd)    = 0.0;
          PIN(com_pos)  = mod((PIN(pos_fb) + PIN(com_offset)) * PIN(pp));
          PIN(emf_run)  = -1.0;

          // the clamp slews the integrator only: a clamped P limit-cycles
          float vel_raw   = v_t - PIN(vel_fb);
          float vel_error = LIMIT(vel_raw, ABS(PIN(test_vel)) / 100.0);
          PIN(cur_sum) = LIMIT(PIN(cur_sum) + PIN(ki) * vel_error * period, PIN(test_cur));
          PIN(q_cmd) = LIMIT(PIN(vel_bw) * period * vel_raw + PIN(cur_sum), PIN(test_cur));
        }

        // a bad pp or offset puts current in without torque: stall when the
        // integrator (not the P term, which spikes at spin up) asks for amps
        // and the rotor does not turn
        if(ctx->psi_stage < 4 && ABS(ctx->vel_lp) < ABS(v_t) * 0.1 && ABS(PIN(cur_sum)) > PIN(test_cur) * PSI_STALL) {
          ctx->psi_fail = 1;
        } else if(ctx->psi_stage == 0 || ctx->psi_stage == 2) {  // spin up
          if(ABS(ctx->vel_lp - v_t) < ABS(v_t) * PSI_BAND) {
            ctx->settle_t += period;
          } else {
            ctx->settle_t = 0.0;
          }
          if(ctx->settle_t >= PSI_SETTLE) {
            ctx->psi_stage++;
            ctx->st_t  = 0.0;
            ctx->psi_n = 0;
            ctx->sx    = 0.0;
            ctx->sy    = 0.0;
            ctx->si    = 0.0;
          } else if(ctx->st_t > PSI_SPINUP) {
            ctx->psi_fail = 2;
          }
        } else if(ctx->psi_stage == 1 || ctx->psi_stage == 3) {  // dwell
          // uq - r iq = pp vel psi + dead time on q. psi comes from the coast
          // below; the dwells keep the dead time on q for the report
          ctx->sx += PIN(vel_fb) * PIN(pp);
          ctx->sy += PIN(uq_fb) - PIN(iq_fb) * PIN(r);
          ctx->si += PIN(iq_fb);
          ctx->psi_n++;
          if(ctx->st_t >= PSI_DWELL) {
            float n = (float)ctx->psi_n;
            if(ctx->psi_stage == 1) {
              ctx->x_lo = ctx->sx / n;
              ctx->y_lo = ctx->sy / n;
              ctx->i_lo = ctx->si / n;
            } else {
              ctx->x_hi = ctx->sx / n;
              ctx->y_hi = ctx->sy / n;
              ctx->i_hi = ctx->si / n;
            }
            ctx->settle_t = 0.0;
            ctx->psi_stage++;
            ctx->st_t = 0.0;
          }
        } else if(ctx->psi_stage == 4) {  // coast
          // Bridge off: the terminals carry the back emf alone,
          //
          //   u - v = sqrt3 w psi cos(th + phi)       w = pp vel, th = commutation angle
          //
          // seen through io.c's filter H(w) and the state block's hold. Multiplying
          // by g = exp(-j th) / (sqrt3 w H(w) sinc(w T / 2)) and averaging leaves
          // psi exp(j phi) / 2, plus the dc level of u - v times the mean of g,
          // taken back out.
          PIN(en_out)   = 0.0;
          PIN(d_cmd)    = 0.0;
          PIN(q_cmd)    = 0.0;
          PIN(cur_sum)  = 0.0;
          PIN(cmd_mode) = 0.0;
          // keep sending the rotor angle: emf0 demodulates against it
          PIN(com_pos) = mod((PIN(pos_fb) + PIN(com_offset)) * PIN(pp));

          float vel  = PIN(vel_fb);
          float avel = ABS(vel);
          PIN(emf_run) = avel > ABS(PIN(test_vel)) * PSI_COAST_MIN ? 1.0 : 0.0;
          // the f3 opens the bridge a few packets later and the winding current
          // then freewheels through the diodes for about L / R, the terminals on
          // the rails; io.c's filter and the hold carry that on. emf0 waits
          // its own EMF_HOLDOFF for the same reason
          if(avel > ABS(PIN(test_vel)) * PSI_COAST_MIN && ctx->st_t > PSI_COAST_HOLDOFF) {
            int b   = avel > ABS(PIN(test_vel)) * PSI_COAST_SPLIT ? 0 : 1;
            float w = vel * PIN(pp);
            float s_th, c_th, s_wt, c_wt;
            sincos_fast((PIN(pos_fb) + PIN(com_offset)) * PIN(pp), &s_th, &c_th);
            float f_io = hv_pwm_freq(pins);
            float a_io = HV_IO_ALPHA(f_io);
            sincos_fast(w / f_io, &s_wt, &c_wt);
            // 1 / H = (1 - (1 - a) exp(-j w T)) / a
            float hi_re = (1.0 - (1.0 - a_io) * c_wt) / a_io;
            float hi_im = ((1.0 - a_io) * s_wt) / a_io;
            // the hold's sinc(w T / 2): its delay stays in, emf_delay reports it
            float x    = w * (float)HV_STATE_WORDS * period * 0.5;
            float zoh  = ABS(x) > 1e-3 ? sinf(x) / x : 1.0;
            // exp(-j th) / H, over sqrt3 w (signed, so turning backwards does
            // not add half a turn to phi) and the hold
            float k    = 1.0 / (1.7320508 * w * zoh);
            float g_re = (c_th * hi_re + s_th * hi_im) * k;
            float g_im = (c_th * hi_im - s_th * hi_re) * k;
            float uv   = PIN(u_fb) - PIN(v_fb);
            ctx->cz_re[b] += uv * g_re;
            ctx->cz_im[b] += uv * g_im;
            ctx->cg_re[b] += g_re;
            ctx->cg_im[b] += g_im;
            ctx->cu[b] += uv;
            ctx->cw[b] += w;
            ctx->cn[b]++;
          }

          if(avel < ABS(PIN(test_vel)) * PSI_COAST_MIN || ctx->st_t > PSI_COAST_TIME) {
            float z_re[3], z_im[3], w_mean[2];
            uint32_t n_all = ctx->cn[0] + ctx->cn[1];
            for(int b = 0; b < 3; b++) {  // 0 fast, 1 slow, 2 both
              float n  = b < 2 ? (float)ctx->cn[b] : (float)n_all;
              float zr = b < 2 ? ctx->cz_re[b] : ctx->cz_re[0] + ctx->cz_re[1];
              float zi = b < 2 ? ctx->cz_im[b] : ctx->cz_im[0] + ctx->cz_im[1];
              float gr = b < 2 ? ctx->cg_re[b] : ctx->cg_re[0] + ctx->cg_re[1];
              float gi = b < 2 ? ctx->cg_im[b] : ctx->cg_im[0] + ctx->cg_im[1];
              float u  = b < 2 ? ctx->cu[b] : ctx->cu[0] + ctx->cu[1];
              n        = MAX(n, 1.0);
              z_re[b]  = zr / n - (u / n) * (gr / n);
              z_im[b]  = zi / n - (u / n) * (gi / n);
              if(b < 2) {
                w_mean[b] = ctx->cw[b] / n;
              }
            }
            if(ctx->cn[0] >= PSI_COAST_N && ctx->cn[1] >= PSI_COAST_N) {
              PIN(psi)    = CLAMP(2.0 * sqrtf(z_re[2] * z_re[2] + z_im[2] * z_im[2]), 0.001, 1.0);
              PIN(psi_hi) = 2.0 * sqrtf(z_re[0] * z_re[0] + z_im[0] * z_im[0]);
              PIN(psi_lo) = 2.0 * sqrtf(z_re[1] * z_re[1] + z_im[1] * z_im[1]);
              // the angle drifts with speed by the reading delay between the
              // two paths; two bands give the slope and the standstill value
              float ph_hi = atan2f(z_im[0], z_re[0]);
              float ph_lo = atan2f(z_im[1], z_re[1]);
              float slope = ABS(w_mean[0] - w_mean[1]) > 1.0 ? mod(ph_hi - ph_lo) / (w_mean[0] - w_mean[1]) : 0.0;
              PIN(emf_angle) = mod(ph_lo - slope * w_mean[1]) * 180.0 / M_PI;
              PIN(emf_delay) = -slope;
              // what the dwells saw beyond back emf and r iq
              PIN(vel_lo) = ctx->x_lo / PIN(pp);
              PIN(vel_hi) = ctx->x_hi / PIN(pp);
              PIN(udt_lo) = ctx->y_lo - ctx->x_lo * PIN(psi);
              PIN(idt_lo) = ctx->i_lo;
              PIN(udt_hi) = ctx->y_hi - ctx->x_hi * PIN(psi);
              PIN(idt_hi) = ctx->i_hi;
              PIN(psi_ok) = 1.0;
            } else {
              ctx->psi_fail = 3;
            }
            ctx->psi_stage = 5;
            ctx->rb_i      = 1;
            ctx->rb_t      = 0.0;
          }
        } else if(ctx->psi_stage == 5) {  // read emf0's map back from the f3
          PIN(emf_run) = 0.0;
          PIN(emf_sel) = ctx->rb_i;
          ctx->rb_t += period;
          if(ctx->rb_t >= EMF_RB_WAIT) {
            ctx->rb[ctx->rb_i] = PIN(emf_val);
            ctx->rb_t          = 0.0;
            ctx->rb_i++;
            int bins = CLAMP((int)(ctx->rb[8] + 0.5), 2, EMF_RB_BINS);
            if(ctx->rb_i >= 9 && ctx->rb_i >= 9 + 3 * bins) {
              float z1  = sqrtf(ctx->rb[2] * ctx->rb[2] + ctx->rb[3] * ctx->rb[3]);
              float mn  = 1e9, mx = 0.0, sum = 0.0;
              int good  = ctx->rb[1] >= PSI_COAST_N && z1 > 0.0;
              for(int b = 0; b < bins; b++) {
                float *r = &ctx->rb[9 + 3 * b];
                float p  = sqrtf(r[0] * r[0] + r[1] * r[1]);
                good     = good && r[2] > 0.0;
                mn       = MIN(mn, p);
                mx       = MAX(mx, p);
                sum += p;
              }
              if(good) {
                PIN(emf_psi)     = z1;
                PIN(emf_h5)      = sqrtf(ctx->rb[4] * ctx->rb[4] + ctx->rb[5] * ctx->rb[5]) / z1 * 100.0;
                PIN(emf_h7)      = sqrtf(ctx->rb[6] * ctx->rb[6] + ctx->rb[7] * ctx->rb[7]) / z1 * 100.0;
                PIN(pole_min)    = mn;
                PIN(pole_max)    = mx;
                PIN(pole_spread) = (mx - mn) / (sum / bins) * 100.0;
                PIN(emf_ok)      = 1.0;
              }
              ctx->psi_stage = 6;
            }
          }
        }
      }

      if(ctx->psi_fail || ctx->psi_stage >= 6) {
        PIN(timer)    = 0.0;
        PIN(en_out)   = 0.0;
        PIN(d_cmd)    = 0.0;
        PIN(q_cmd)    = 0.0;
        PIN(cur_sum)  = 0.0;
        PIN(cmd_mode) = 0.0;

        PIN(state) = 3.3;
      }
      break;

    case 100:
      PIN(com_pos) = mod((PIN(pos_fb) + PIN(com_offset)) * PIN(pp));
  }
}


hal_comp_t idpmsm_comp_struct = {
    .name      = "idpmsm",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idpmsm_ctx_t),
    .pin_count = sizeof(struct idpmsm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};