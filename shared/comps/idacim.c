

#include "idacim_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `idacim` identifies an AC induction motor. A standstill test measures the stator resistance `r` (with the inverter dead time voltage `drop` separated from it), the leakage inductance `l` (sigma*Ls) by injection at two frequencies around the current loop's crossover, and the rotor time constant `tr` and rotor side magnetizing inductance `lmr` (Lm^2/Lr) by d current steps; an optional current ladder (`knee`) fits how tr changes with flux. From these and the motor plate it computes a `vf` parameter set. An optional rotating test spins the rotor open loop; with an encoder it first reads `tr` and the inertia from up and down speed ramps at two currents, can check `tr` with the rotor turning under FOC (`rot_tr_vel`), then sweeps the flux, each level at the ramps' top emf and at half that speed, and fits the magnetizing curve (`acim_flux0.lmr`, `lmr_sat`, `i_knee`, `tr_sat`, `i_dip`, `lmr_dip`, `tr_dip`, `acim_foc0.id_n`). A separate pole pair test reads `conf0.polecount` and the direction. It runs on the F4 board, drives `hv0` directly and is loaded by the `id_acim` template.
*
* ## Component Explanation
*
* 1. **Before you start**:
* - `test_cur` must be under `conf0.max_ac_cur`. The r chord's dead time correction needs every phase above about 2 A in the lower dwell (`test_cur / 2` on d is `-test_cur / 4` on v and w), and the rotor test steps between the same two levels, so `lmr` and `ls` are read over `test_cur / 2 .. test_cur`. No test level should go under `i_min`, where the dead time distorts the voltage: the console warns if `test_cur < 2 * i_min`. For the V/f set, a `test_cur` near the resulting `id_n` is best (the report says when to rerun).
* - If you have a four wire measurement of the winding resistance, set `idacim0.r_known` to it: r is then taken as given and only the dead time drop is read.
* - For the V/f set give the plate: `n_volt`, `n_freq`, `n_cur` and `n_pp`. Without all four the V/f set is skipped; the rotating test needs at least `n_freq` and `n_pp`, and the tr knee fit needs `id_n` from the V/f set.
* - `spin = 1` goes on to the rotating test after the standstill test. That test turns the rotor, so the shaft must be free and unloaded.
* - `knee = 1` adds the tr ladder to the rotor test (see 5.). It needs `test_cur >= 1.5 * i_min / 0.7` (about 8.6 A with the default `i_min`), otherwise the console says there is no range and no ladder runs.
* - The template sets `hv0.drop_k = 0` (no dead time compensation) for the run: the r fit and the dead time drop are read from `ud_fb`.
* - While the state is 0 the nrt function resets `r` 0.1 ohm, `l` 1 mH, `drop` 0, `r_ok`, `l_ok`, `tr_ok` and `out_rev` 0, `cur_bw` 1. The parameters keep their values.
*
* 2. **How to run it**:
* - At the console, `link id_acim`. The template loads `idacim` and wires `idacim0.en = fault0.en_out`, sets `fault0.pos_error = 0` and `pid0.en = 0`, connects `hv0.en`, `cur_bw`, `cmd_mode`, `d_cmd`, `q_cmd`, `pos`, `vel` (from `com_vel`), `rev`, `r`, `l`, `psi` to this component (`hv0.lq = 0`) and `hv0.ud_fb`, `id_fb`, `uq_fb`, `iq_fb`, `pwm_volt`, `dc_volt`, `conf0.cur_bw` (as `loop_bw`), `vel1.vel` and `fb_switch0.vel_fb` (as `pos_fb`, the encoder position without offset) back into it. `hv0.r` / `hv0.l` are this component's `r` / `l`, so they are the current loop's plant model during the tests.
* - Enable the drive. The rotor may stay free for the standstill test: a dc field and a single pulsating axis make no torque on a cage. The state goes `0` -> `1.0` -> `1.1` and the console asks for `idacim0.state = 1.2`. `en` low returns to `0` from any state.
* - The standstill test runs r (`1.2`, 4 s), leakage (`1.3`, a few s) and the rotor test (`1.4`, about `(1 + 2 * rot_cycles) * rot_half` at most, shorter once tr is known, plus each ladder rung). State `1.5` prints the results and goes to `3.0` (done), to `4.0` if `spin` is set, or back to `0` if r failed. Append the printed lines to the config and save.
* - Rotating test: `4.0` (set it by hand after a standstill test in the same enable, or via `spin`) checks that `tr_ok`, `n_freq` and `n_pp` are there, prints the top speeds of the ramps, the flux sweep and the tr check, and asks for `idacim0.state = 4.2`. It ends in `5.0`.
* - Pole pair test: set `idacim0.state = 2` by hand; it asks to unblock the rotor and for `idacim0.state = 2.2`, and ends in `3.0`.
*
* 3. **Resistance (`state` 1.2)**:
* - Current mode, `cur_bw` 1, current on the d axis at `com_pos = 0`: 2 s at `test_cur`, then 2 s at `test_cur / 2`. `id_fb` / `ud_fb` of each dwell are low-passed (0.001 per tick) into `tmp2` / `tmp3` (top) and `tmp0` / `tmp1` (half). Meanwhile `r` tracks `ud_fb / id_fb` so the current loop can build up voltage at all.
* - `ud_fb` contains `r * id + 4/3 * drop` (dead time; at angle 0 the phase currents are id, -id/2, -id/2). After 4 s `fit_di = tmp2 - tmp0` and the chord `r_2p = (tmp3 - tmp1) / fit_di` (only if `fit_di > 0.01` A) are computed, and `r` is taken from one of two modes:
* - `r_known > 0`: `r = r_known`, `drop = 0.75 * (tmp3 - r * tmp2)`. Needs the top dwell to reach half of `test_cur`.
* - default: `r = r_2p - r_bias` with `r_bias = drop_slope * dc_volt / test_cur` (0 if `dc_volt` is not wired), `drop` as above. The dead time drop rises roughly as ln(i), which biases the chord; `drop_slope` was fitted on one bridge.
* - `r_ok` is set if the chosen mode produced a value. Then `avg_test_volt = r * test_cur + 4/3 * drop` (limited to `pwm_volt / 2`) and the state goes to `1.3`, or straight to `1.5` if `r_ok` is 0.
*
* 4. **Leakage inductance (`state` 1.3)**:
* - Voltage mode, `cur_bw` 1: `d_cmd = avg_test_volt + amp * sin(w t)`. The dc bias holds about `test_cur` on d, so every phase current stays on one side of zero and the dead time is only an offset. The first settle is 1 s (the bias settles on the slow rotor pole), after a frequency change 0.2 s.
* - The pair of frequencies sits on either side of the current loop's crossover `f_c = loop_bw / 2 pi`. At a `cur_bw` of 1000 the default pair is 120 / 240 Hz. Both are rounded to multiples of 20 Hz, so every window holds whole cycles, and kept within 20 Hz .. `0.2 / period`. The leakage of a cage rotor has no plateau over frequency, so this is the band `conf0.l` has to describe.
* - At each frequency 4 blocks of 50 ms size the amplitude so the injected current is `l_ripple * test_cur`, then about 0.4 s (a whole number of cycles, counted in samples) demodulate `ud_fb` and `id_fb`, with the dc bias taken out of both (the mean times the reference sums, as in `idpmsm`), into the impedance `l_za` / `l_zb` (corrected for the zero order hold of the command) and the current reached, `l_ia` / `l_ib`. No resistance goes in:
* ```c
* l * l = (zb * zb - za * za) / (wb * wb - wa * wa);
* ```
* - `l_res` is the resistance (stator plus cage) the pair implies. `l_ok` needs a positive l^2 and both currents within 0.5..2 times the target. If the reactance at the upper frequency is under `2 * l_res` (small leakage, as on high speed spindles), the pair moves up an octave, at most 3 times and while the upper one stays under `0.2 / period`; `l_fa` / `l_fb` are the pair used.
* - `l = sqrt(l^2)` if `l_ok`, else 1 mH (then measure line to line near 150 Hz with an LCR meter and halve it). The state goes to `1.4`.
*
* 5. **Rotor time constant and magnetizing inductance (`state` 1.4)**:
* - Current mode at `cur_bw = rot_bw`, `com_pos = 0`: `d_cmd` steps between `test_cur` and `test_cur / 2`, `1 + 2 * rot_cycles` levels. The first level only magnetizes. Each level lasts `rot_half` until an edge has given a tr, then `16 * tr` of the last fit (at least 0.1 s).
* - The rotor flux follows the stator current with `tr = Lr/Rr`, and its emf `lmr * d(i_mr)/dt` appears in `ud_fb` on top of `r * id`, the dead time and `l * d(id)/dt`. Integrated from the edge, with `c` the offset `ud - r id` where the edge settles:
* ```c
* lam(t) = integral(ud - r * id - c) - l * (id - i0) = lmr * (i_mr - i0);
* integral(lam) = lmr * integral(id - i0) - tr * lam;
* ```
* - This is fitted by least squares on the current that actually flowed, from `rot_t0` (skips the current loop's settling) to the end of the edge, with `lmr`, `tr` and the edge's own offset `c` as unknowns (Gauss-Newton in nrt, started from the median of 8 block means over the last quarter). `r_2p` (or `r`) is used for r, because the chord takes the dead time out at exactly these two levels. A median of three filters single bad samples; the sums take every 8th tick. An edge counts if it moves the current by more than 10 % of the level; nrt prints one `# edge` line per edge.
* - At the end the rising and falling edges are reduced apart: `tr_rise` / `tr_fall` are their medians, `tr` their mean (this cancels the asymmetry a step into saturation causes), `tr_spread` the worse direction's (max - min) / median, `tr_min` / `tr_max` the extremes. `lmr` is the median of the falling edges only (rising ones read low near saturation). `tr_ok` if `tr` is over `5 * period` and under the start of the last edge's settled tail (0.75 of its length); then `slip_n = 1 / tr` and `ls = l + lmr`, otherwise these are 0 (`tr` is reported either way).
* - Offset ladder (`lad_top > 0`): after the main test, rung k of `lad_n` repeats the steps between its top and `lad_ratio` times it, and stores its top current, `lmr` slope, `tr` and spread in `lad_i`, `lad_lm`, `lad_tr`, `lad_sp`.
* - `knee = 1` (with `lad_top` 0) writes the ladder itself at the start of a run: `lad_ratio` 0.7, `lad_bot = max(0.4 * test_cur, i_min / 0.7)`, `lad_top = test_cur`, `lad_n` 4, and records it in `lad_auto`. A ladder the knee wrote is replaced or dropped on the next run; one set by hand stays.
*
* 6. **Report and V/f set (`state` 1.5, nrt)**:
* - Waits until nrt has reduced the last rung. If `r_ok`: prints `conf0.r`, `conf0.l` (or why l failed), and if `tr_ok` also `acim_flux0.tr`, `acim_flux0.lmr` and the slip for `acim_ttc` (`vel_n = (2 pi freq_n - slip_n) / polecount`), with warnings for `tr_spread > 0.25` and `rot_dip > 0.1` (raise `rot_bw`).
* - With `tr_ok` and all four plate pins the V/f set: `u = n_volt * sqrt(2/3)`, `w = 2 pi n_freq`, `id_n = u / sqrt(r^2 + w^2 ls^2)`, `iq_n = sqrt((sqrt(2) n_cur)^2 - id_n^2)`. It prints `conf0.polecount = n_pp` and `vf0.u_n = u`, `vf0.vel_n = w / n_pp`, `vf0.u_boost = r * id_n + 4/pi * drop` (V/f runs in volt mode with no dead time compensation, and a rotating vector loses the square wave's fundamental, 4/pi of the per phase drop; on the spindle `r * id_n` alone, 3.7 V, stalled and 12 V started), `vf0.boost_vel` (where `w ls = 3 r`, clamped to 2..20 % of `w`, per pole pair), `vf0.slip_n = iq_n / (tr * id_n) / n_pp`, `vf0.cur_n = iq_n`, and `acim_foc0.id_n` as a start value. The same values are on the `vf_*`, `id_n` and `iq_n` pins.
* - With a ladder it prints a table per rung (range, lmr slope, tr, spread; with `lad_bot` 0 and `lad_ratio` 0.5 also psi_r and the secant lmr, integrated from the slopes), then the tr knee fit: acim_flux's model `tr * (1 + tr_sat * x)`, `x = (id_n - i) / (id_n - i_knee)` clamped 0..1, fitted over the rungs (only those with an lmr, a lower current of at least `i_min` and spread up to 0.3, each at the middle of its range) and the main test (at `0.75 * test_cur`), weighted by 1 / spread^2, for every `i_knee` from 0 to 0.9 `id_n` in 1 % steps. It needs `id_n` and three levels. It is rejected if `tr_sat` is outside 0..1 or tr at `id_n` is more than 30 % off the main test's tr; otherwise `knee_i`, `knee_tr_sat`, `knee_tr` are set and it prints `acim_flux0.tr` (tr at `id_n`, replacing the one above), `acim_flux0.i_knee` and `acim_flux0.tr_sat`.
* - Then the dead time `drop`, which is also part of `vf0.u_boost`.
* - If `r_ok` is 0 it prints why the r test failed and returns to state 0 (with `en` still high, straight back to the 1.1 prompt).
*
* 7. **Rotating test (`state` 4.2 to 4.7), rotor free, no load**:
* - Current mode at `cur_bw = rot_bw`, `d_cmd = id_n` (or `test_cur` without the V/f set), and `com_pos` turns open loop. `4.2`: the field ramps at `rot_acc` to `rot_vel` of plate speed and holds 1 s. If `vel_fb` is within 20 % of the field, `rot_enc = 1`; if the rotor turns but lags more, it pulled out and the test ends (`4.6`). The order is then `4.4` ramps, `4.5` tr check (if `rot_tr_vel > 0`), `4.3` flux sweep, `4.6`, `4.7`; without an encoder `4.2` goes straight to the sweep.
* - In every state but `4.5` a slow integrator (20 /s) trims `d_cmd` (within 25 % of its target) and `q_cmd` (within 10 % of the d target) until `id_fb` is on target and `iq_fb` is 0: the current loop leaves an error at speed that differs between an up and a down ramp (simulated: id 19.2 / 19.7 A for 20.4, iq 0 / 0.2 A), and with the trim both ramps see the same flux and current.
* - `psi` (to `hv0.psi`, curpid's emf feed forward and predictor) is the rotor flux from the emf, `abs(e) / w` with `e = u - r i - j w l i`, low passed over 20 ms while the field turns faster than 0.1 of plate speed, starting from `lmr * id_n`. It does not come from `acim_flux0`, whose low flux shape is what this test is after (on the spindle that read 0.075 for 0.107 Vs at 6.9 A, and the predicted iq ran 4 A off at 780 rad/s el). `com_vel` gives `hv0.vel` the frame's own speed: the rotor's speed has the wrong sign on a motor that turns against the field (`out_rev = 1`), and hv0 then advanced the frame the wrong way and the F3 decoupled against the emf (the spindle ran iq -3..-12 A for 0 commanded).
* - `4.4` ramps, only with `rot_enc`: two pairs, the first at `id_n`, the second at 0.65 `id_n` (at least `i_min`). Each pair holds until the rotor flux has settled (`max(6 tr, 0.5 s)`), ramps up from `rot_vel` to `w_hi = min(2 rot_vel, 0.9)` of plate speed, holds 0.5 s and ramps back down. Over the middle half of each ramp it averages the air gap torque (stator power less copper loss, over the field speed: no tr in it), the slip against `vel_fb`, the slip against the encoder position (mean field speed less `pp` times the position travelled over the time), the rotor flux on d (`(uq - r iq) / w - l id`), `id_fb` and the acceleration. A slip x tr over 1 ends the ramps (pulled out).
* - tr from the ramps: with the current on d the rotor flux lags it by `atan(ws tr)`, so the torque is `1.5 pp psi_d id ws tr`. Up minus down, the air gap torque holds no friction, iron loss, r or dead time error (same speeds, flux and current on both): `dT = psi_d id tr dws`. The measured slip, though, is the field's commanded speed against the encoder's, which comes back a delay D later, so each pair's slip difference also carries `pp (alpha_up - alpha_down) D` (simulated: about 1 ms, 23 % of `dws` at `id_n`). That part does not scale with the flux, so the two pairs give both:
* ```c
* dws_m_k = dT_k / (psi_d_k * id_k * tr_k) + pp * dalpha_k * D;   // tr_k = tr * lmr_k / lmr_0
* ```
* - The 2x2 solve needs the pairs to differ enough in flux (determinant over 0.2 of `a0 b1`); D goes to `rot_delay` and a tr within 0.3..3 times the standstill `tr` to `rot_tr_ramp`, with the first pair's lmr times `1 + (ws tr)^2`. From then on the sweep uses this tr and lmr in place of the standstill ones. (The standstill steps read the spindle's tr as 0.078-0.093 s, where its orientation needs about 0.12 at every flux.)
* - `4.5` rotating tr check, after the ramps when `rot_tr_vel > 0` (off by default; the rotor runs up to `rot_tr_vel`): FOC on the encoder with the test's own rotor flux model at the standstill tr (`i_mr` low passed from `id_fb` with tr, `slip = iq / (tr i_mr)`). The frame turns with the encoder position plus that slip, not with `vel_fb`, whose lag on a ramp is a frame speed error as large as the one looked for (vel1: 1.8 ms; simulated k 0.99 from the true angles for 1.06). The d current is held at what gives 0.8 of the plate emf at `rot_tr_vel` (at least 1.5 `i_min`, at most `id_n`; a flux that follows the speed tilts the angle with the ramp's sign, simulated k 0.87 for 1), and `q_cmd = +-ia` (`min(0.6 id_n, sqrt(imax^2 - id^2))`, no speed loop) ramps the rotor up to `rot_tr_vel` and down to 0.7 of it. After one unmeasured pair, 3 pairs are measured where the speed is between 0.76 and 0.94 of `rot_tr_vel` and at least `3 tr` into the ramp.
* - There the emf is large against the dead time and r errors, so `e = u - r i - j w l i = j w psi_r` gives the flux angle in the frame (the dead time loses 4/pi of `drop` along the current and is taken out; simulated k 0.975 for 1.056 without it). With the model's tr off by `k = tr_true / tr` the frame settles where `tan(phi - delta) = k tan(phi)` (phi the current's angle, delta the flux's). An offset `b` on the angle (dead time, voltage timing) is the same on the up and the down ramp, where phi changes sign, so `b` is found by bisection where both ramps give the same `k`; this needs at least 100 samples on each ramp and phi of opposite sign. A `k` in 0.3..3 gives `rot_tr = k tr`, `rot_tr_k`, `rot_tr_imr`, `rot_tr_lmr` (`abs(e) / w / i_mr`) and `rot_tr_b`; they are printed as comments only. The report also gives the same without the dead time correction and a torque balance `(T_up - T_down) = J (a_up - a_down)` with `T = 1.5 pp psi abs(i) sin(phi - delta)`, whose J against `rot_j` tells which flux angle is right.
* - Then `q_cmd = -ia` brakes the rotor (d current rising to keep 0.8 of the plate emf) until it is under `rot_vel` of plate speed, where the open loop field takes over at the rotor's speed and the sweep starts. A ramp that does not reach its end within 15 s goes on to the next phase; 30 s without getting under `rot_vel` switches the bridge off, the rotor coasts and the state goes to `4.7`.
* - `4.3` flux sweep: d current at 1.15, 1.0, 0.9, 0.75, 0.6, 0.45, 0.35, 0.25 of `id_n`, from the top down, limited to `i_min` .. `0.85 * sqrt(2) * n_cur` (levels that would repeat a limit are skipped; without an encoder it stops at 0.45). With an encoder each level runs where its emf is the ramps' top one, at `w_hi * id_n / i`, at most `rot_vmax` of plate speed (but not under `w_hi`): the field weakening range, where acim_flux uses these levels. An inverter voltage error is a larger part of a small emf; at one speed for all levels (0.4 of plate speed) the spindle read lmr at 5.5 A as 6.6 mH and 9.4 at 0.9, against about 12 from a flux observer at 838 rad/s. Without an encoder every level runs at `rot_vel`.
* - Each level settles `max(6 tr, 0.3 s)` and averages 0.5 s. At no load `uq = r iq + w psi_s`, so the rotor flux is `(uq - r iq) / w - l id`; with an encoder it is corrected for the slip angle (with tr scaled by the point's own lmr against the lmr that tr belongs to, the ramps' or the standstill one). A point with slip x tr over 0.35 (0.6 with the ramps' tr, which holds further) is dropped and ends the pass; a filtered slip x tr over 1 (near pull out) ends it too.
* - With an encoder the same levels are read again, each at half its speed: an offset du on uq reads as du / w in the flux, so `lmr = (w1 psi1 - w2 psi2) / (w1 i1 - w2 i2)` takes it out. While the field changes speed the d current is `id_n` (`id_n * w_hi / w` above `w_hi`, so the emf stays at the ramps' top), the flux settles first, and with an encoder the field waits while the slip x tr is over 0.35. Results in rising order in `sw_i` / `sw_psi` (`sw_n` points, at most 8).
* - Per point the report also lists tr from the voltages: `x_v`, the tangent of the flux angle behind the current from `e = u - r i - j w l i` (slip x tr with no tr in it), over the encoder slip at each speed, and the two speed value `(x1 - 2 x2) / (ws1 - 2 ws2)`, which takes out the `w lmr / R_fe` iron loss puts on `x_v`. These are diagnostics only.
* - `4.6` ramps the field down to 0, `4.7` (nrt) reports and goes to `5.0`: `rot_id_n`, the d current where the stator flux reaches `n_volt` at `n_freq` (interpolated from the sweep, or `id_n` without `n_volt`), and `rot_lmr`, the secant there, printed as `acim_foc0.id_n`, `acim_flux0.i_n` and `acim_flux0.lmr`. With `rot_tr_ramp` it prints `acim_flux0.tr`, the ramps' tr moved to `rot_id_n` with the secant lmr (tr follows lmr, Rr does not saturate), which replaces the standstill tr, and the field to position delay; with an encoder but no tr from the ramps it says to keep the standstill tr.
* - Magnetizing curve: with at least 3 points under 0.95 `id_n` the secant lmr is fitted to acim_flux's shape `lmr * (1 + lmr_sat * x) * (1 - lmr_dip * y)`, `x = (id_n - i) / (id_n - i_knee)`, `y = (i_dip - i) / i_dip`, both clamped 0..1 (growth as the iron desaturates down to the knee, flat, then a fall under `i_dip` at low induction), over a grid of `i_knee` and `i_dip <= i_knee` starting at the lowest point; terms that come out negative are dropped. The fit is used only with an encoder, `lmr_sat <= 1`, `lmr_dip <= 0.5` and its lmr at `id_n` within 5 % of `rot_lmr`: then it prints `acim_flux0.lmr_sat`, `i_knee`, `tr_sat` (= `lmr_sat`), `i_dip`, `lmr_dip` and `tr_dip` (= `lmr_dip`: tr = Lr / Rr follows the secant lmr below the dip as above the knee; on the spindle the rotating checks agree with this to a few %). Otherwise `lmr_sat` comes from a straight line through the lowest point at or above 0.4 `id_n` and no knee or dip is set. Without an encoder that `lmr_sat` is printed to append (at least 0); with an encoder it is only printed as a comment, because the line runs through the dip and does not fit the config's knee and dip, which stay.
* - With the ramps also `rot_j` (inertia from the first pair's air gap torque, printed as `conf0.j`), `rot_j_s` (the same from the position slip and the ramps' tr, equal by construction), the friction torque `rot_tf` and `rot_lag`, the lag of `vel_fb` behind the position from the difference of their slips.
*
* 8. **Pole pairs and direction (`state` 2.2), rotor free**:
* - `2.0` (nrt) clears `pp` and asks for `idacim0.state = 2.2`. For 3 s: current mode, `cur_bw` 100, `d_cmd = test_cur`, and `com_pos` advances open loop at `test_vel`, dragging the rotor along. While `abs(vel_fb) > 0.1` rad/s, `pp` is low-passed (0.005 per tick, starting from the first reading) from `test_vel / vel_fb`. At the end a negative `pp` sets `out_rev = 1`, and `pp` is rounded to an integer.
* - `2.4` (nrt) prints `conf0.polecount` and, if reversed, `conf0.out_rev = 1`, if `pp` is at least 1, the ratio before rounding is within 0.15 of it and the rotor turned in more than 80 % of the ticks; otherwise it prints the ratio and that fraction as a failure (a slipping rotor, a load, or no `vel_fb`). Then it goes to `3.0` (done).
*
* {{% hint warning %}}
* `drop` is a voltage per phase; it goes into `vf0.u_boost` but is not a value for `hv0.drop_k` (a fraction). `conf0.l` here is the leakage, right for `acim_foc` (which carries the rotor flux in `hv0.psi`); `acim_ttc` has no flux term, so with it iq falls short at speed. `tr` is the rotor's at the test's temperature and flux: a hot cage reads shorter. The rotating test uses `r`, `l`, `tr` and `lmr` from the pins and needs `tr_ok`; `en` low resets `r` and `l` to 0.1 ohm / 1 mH and clears `tr_ok`, so run it in the same enable as the standstill test. Without an encoder the sweep is not corrected for slip, so drag reads as saturation at the low points. In the pole pair test the rotor slips behind the field, so `test_vel / vel_fb` reads slightly high before rounding.
* {{% /hint %}}
*/

HAL_COMP(idacim);

HAL_PIN(d_cmd);  // *output*, d axis command to hv0.d_cmd, current (A) or voltage (V) depending on cmd_mode
HAL_PIN(q_cmd);  // *output*, q axis command to hv0.q_cmd (A), 0 in the standstill and pole pair tests, the slow iq trim in the rotating test and +-ia in the tr check
HAL_PIN(com_pos);  // *output*, commutation angle to hv0.pos (rad), 0 in the standstill tests, turns in the pole pair and rotating tests
HAL_PIN(com_vel);  // *output*, rate of com_pos (rad/s electrical) to hv0.vel, for hv0's angle extrapolation and the F3 decoupling
HAL_PIN(cmd_mode);  // *output*, to hv0.cmd_mode, 0 = voltage, 1 = current
HAL_PIN(en);  // *input*, enable, from fault0.en_out; low aborts (state -> 0)
HAL_PIN(en_out);  // *output*, enables hv0 while a test runs

HAL_PIN(id_fb);  // *input*, d axis current from hv0.id_fb (A)
HAL_PIN(ud_fb);  // *input*, d axis voltage from hv0.ud_fb (V)

HAL_PIN(state);  // *input/output*, 0 off, 1.1 wait, 1.2 r, 1.3 leakage, 1.4 rotor, 1.5 report, 2.1 wait, 2.2 pp test, 2.4 print, 3 done, 4.1 wait, 4.2 spin up, 4.4 ramps (tr, inertia), 4.5 rotating tr check, 4.3 flux sweep, 4.6 spin down, 4.7 report, 5 done
HAL_PIN(timer);  // *output*, time in the r and pole pair tests (s)

HAL_PIN(r);  // *output*, measured resistance (ohm), to hv0.r, 0.1 while idle, result for conf0.r
HAL_PIN(l);  // *output*, measured leakage inductance sigma*Ls (H), to hv0.l, 1 mH while idle or if not measured, result for conf0.l
HAL_PIN(l_ok);  // *output*, 1 = l is a measurement, 0 = it is not
HAL_PIN(l_freq_a);  // *parameter*, leakage test, lower injection frequency (Hz), 0 = 0.75 loop_bw / 2 pi, default 0
HAL_PIN(l_freq_b);  // *parameter*, leakage test, upper injection frequency (Hz), 0 = 1.5 loop_bw / 2 pi, default 0
HAL_PIN(loop_bw);  // *input*, the run's current loop bandwidth from conf0.cur_bw (rad/s), sets the leakage test frequencies, 0 = 1000
HAL_PIN(l_ripple);  // *parameter*, leakage test, injected current as a fraction of test_cur, default 0.15
HAL_PIN(l_za);  // *output*, impedance magnitude at l_fa (ohm)
HAL_PIN(l_zb);  // *output*, impedance magnitude at l_fb (ohm)
HAL_PIN(l_ia);  // *output*, injected current amplitude reached at l_fa (A)
HAL_PIN(l_ib);  // *output*, injected current amplitude reached at l_fb (A)
HAL_PIN(l_res);  // *output*, resistance the two frequencies imply, stator plus cage (ohm)
HAL_PIN(l_fa);  // *output*, lower injection frequency used (Hz)
HAL_PIN(l_fb);  // *output*, upper injection frequency used (Hz)

HAL_PIN(rot_half);  // *parameter*, rotor test, longest time at each current level (s), default 1.5
HAL_PIN(rot_cycles);  // *parameter*, rotor test, measured cycles of two edges each, default 4, at most 16
HAL_PIN(rot_bw);  // *parameter*, current loop bandwidth of the rotor and rotating tests (rad/s), default 1500
HAL_PIN(rot_t0);  // *parameter*, rotor test, fit starts this long after each edge (s), default 0.015, kept under tr / 5
HAL_PIN(tr);  // *output*, rotor time constant Lr/Rr, mean of tr_rise and tr_fall (s), result for acim_flux0.tr
HAL_PIN(slip_n);  // *output*, 1/tr, slip constant (rad/s electrical), 0 = not measured
HAL_PIN(lmr);  // *output*, rotor side magnetizing inductance Lm^2/Lr, median of the falling edges (H), result for acim_flux0.lmr
HAL_PIN(ls);  // *output*, stator inductance l + lmr (H)
HAL_PIN(rot_n);  // *output*, edges that went into tr and lmr
HAL_PIN(rot_dip);  // *output*, largest current error at rot_t0, fraction of the step
HAL_PIN(tr_ok);  // *output*, 1 = tr, slip_n and lmr are measurements
HAL_PIN(tr_spread);  // *output*, (max - min) / median of tr, the worse of the two directions
HAL_PIN(tr_rise);  // *output*, median tr of the edges up to test_cur (s)
HAL_PIN(tr_fall);  // *output*, median tr of the edges down to test_cur/2 (s)
HAL_PIN(tr_min);  // *output*, shortest tr of all edges (s)
HAL_PIN(tr_max);  // *output*, longest tr of all edges (s)

HAL_PIN(lad_top);  // *parameter*, offset ladder, top current (A), 0 = no ladder, default 0
HAL_PIN(lad_n);  // *parameter*, offset ladder, rungs (at most 4), default 4
HAL_PIN(lad_bot);  // *parameter*, offset ladder, lowest rung's top (A), rungs evenly from here to lad_top, 0 = lad_top k/n
HAL_PIN(lad_ratio);  // *parameter*, offset ladder, rung's lower current over its top, 0 = 0.5, else clamped 0.3 to 0.9
HAL_PINA(lad_i, 4);  // *output*, ladder rung upper current (A)
HAL_PINA(lad_lm, 4);  // *output*, ladder rung lmr, the slope of rotor flux over the rung (H), 0 = no fit
HAL_PINA(lad_tr, 4);  // *output*, ladder rung tr (s)
HAL_PINA(lad_sp, 4);  // *output*, ladder rung tr spread
HAL_PIN(i_min);  // *parameter*, lowest current any test level may use, under it the dead time distorts the voltage (A), default 4
HAL_PIN(knee);  // *parameter*, 1 = run the ladder for the tr knee fit, from 0.4 test_cur to test_cur (unless lad_top is set), default 0
HAL_PIN(lad_auto);  // *output*, lad_top the knee wrote, cleared on the next run, 0 = none
HAL_PIN(knee_i);  // *output*, i_knee for acim_flux0 from the ladder's tr (A), 0 = not fitted
HAL_PIN(knee_tr_sat);  // *output*, tr_sat for acim_flux0 from the ladder's tr
HAL_PIN(knee_tr);  // *output*, tr at id_n from the knee fit (s), result for acim_flux0.tr when fitted
HAL_PIN(drop);  // *output*, dead time voltage per phase at the top dwell (V), 4/pi of it goes into vf_boost
HAL_PIN(r_known);  // *parameter*, known winding resistance (ohm), 0 = fit it, default 0
HAL_PIN(fit_di);  // *output*, current difference of the two r dwells (A)
HAL_PIN(r_2p);  // *output*, two dwell chord slope (ohm), 0 = the fit did not run, also r of the rotor test
HAL_PIN(drop_slope);  // *parameter*, chord dead time bias (ohm A per volt of dc link), default 0.0039
HAL_PIN(r_bias);  // *output*, bias subtracted from the chord to get r (ohm)
HAL_PIN(r_ok);  // *output*, 1 = this run produced a resistance

HAL_PIN(pp);  // *output*, measured pole pairs from the pole pair test, result for conf0.polecount
HAL_PIN(out_rev);  // *output*, 1 = motor turns backwards, to hv0.rev, result for conf0.out_rev
HAL_PIN(spin);  // *parameter*, 1 = go on to the rotating test after the standstill test (turns the rotor), default 0

HAL_PIN(n_volt);  // *parameter*, plate voltage, line to line rms (V), 0 = not given
HAL_PIN(n_freq);  // *parameter*, plate frequency at that voltage (Hz), 0 = not given
HAL_PIN(n_cur);  // *parameter*, plate current, rms (A), 0 = not given
HAL_PIN(n_pp);  // *parameter*, plate pole pairs (poles / 2), 0 = not given
HAL_PIN(id_n);  // *output*, magnetizing current at plate voltage and frequency (A peak), V/f set
HAL_PIN(iq_n);  // *output*, active current at plate current (A peak), V/f set
HAL_PIN(vf_u_n);  // *output*, for vf0.u_n, phase peak voltage at vf_vel_n (V)
HAL_PIN(vf_vel_n);  // *output*, for vf0.vel_n (rad/s mechanical)
HAL_PIN(vf_boost);  // *output*, for vf0.u_boost, r id_n + 4/pi drop (V)
HAL_PIN(vf_boost_vel);  // *output*, for vf0.boost_vel (rad/s mechanical)
HAL_PIN(vf_slip_n);  // *output*, for vf0.slip_n, slip at vf_cur_n (rad/s mechanical)
HAL_PIN(vf_cur_n);  // *output*, for vf0.cur_n, active current at plate current (A peak)

HAL_PIN(test_cur);  // *parameter*, highest current of the standstill test (A peak), default 8
HAL_PIN(test_vel);  // *parameter*, field speed of the pole pair test (rad/s electrical), default 50

HAL_PIN(vel_fb);  // *input*, mechanical velocity from vel1.vel (rad/s)
HAL_PIN(pos_fb);  // *input*, rotor position without offset from fb_switch0.vel_fb (rad mechanical), slip of the ramps and frame of the rotating tr check
HAL_PIN(psi);  // *output*, rotor flux to hv0.psi for curpid's emf feed forward and predictor (Vs), abs(emf) / w low passed in the rotating test, set to 0 when the standstill or the rotating test starts
HAL_PIN(uq_fb);  // *input*, q axis voltage from hv0.uq_fb (V), rotating test
HAL_PIN(iq_fb);  // *input*, q axis current from hv0.iq_fb (A), rotating test

HAL_PIN(rot_vel);  // *parameter*, rotating test speed, fraction of plate speed (0.05 to 0.45), default 0.4
HAL_PIN(rot_acc);  // *parameter*, rotating test ramp (rad/s^2 electrical), 0 = plate speed in 5 s, default 0
HAL_PIN(sw_n);  // *output*, flux sweep points taken
HAL_PINA(sw_i, 8);  // *output*, flux sweep magnetizing current, rising (A)
HAL_PINA(sw_psi, 8);  // *output*, flux sweep rotor flux (Vs peak)
HAL_PIN(rot_id_n);  // *output*, d current for plate flux at plate voltage and frequency (A), result for acim_foc0.id_n and acim_flux0.i_n
HAL_PIN(rot_lmr);  // *output*, secant lmr at rot_id_n (H), result for acim_flux0.lmr
HAL_PIN(rot_lmr_sat);  // *output*, saturation from the sweep, result for acim_flux0.lmr_sat (and tr_sat with a knee)
HAL_PIN(rot_i_knee);  // *output*, knee current from the sweep (A), result for acim_flux0.i_knee, 0 = not fitted
HAL_PIN(rot_i_dip);  // *output*, dip current from the sweep (A), result for acim_flux0.i_dip, 0 = no dip
HAL_PIN(rot_lmr_dip);  // *output*, dip depth from the sweep, result for acim_flux0.lmr_dip
HAL_PIN(rot_j);  // *output*, inertia from the ramps via the air gap torque (kg m^2), 0 = no encoder, result for conf0.j
HAL_PIN(rot_j_s);  // *output*, inertia from the ramps via the position slip and the ramps' tr (kg m^2)
HAL_PIN(rot_tf);  // *output*, friction torque in the ramps speed range (Nm)
HAL_PIN(rot_enc);  // *output*, 1 = vel_fb followed the field, so slip can be read (ramps, tr check, sweep correction, second speed)
HAL_PIN(rot_tr_vel);  // *parameter*, top speed of the rotating tr check (rad/s mechanical), needs an encoder, 0 = skip the check, default 0
HAL_PIN(rot_tr);  // *output*, tr from the rotating check at rot_tr_imr, rot_tr_k x tr (s), 0 = not run or no solution
HAL_PIN(rot_tr_k);  // *output*, rot_tr over the standstill tr
HAL_PIN(rot_tr_imr);  // *output*, magnetizing current the check ran at (A)
HAL_PIN(rot_tr_lmr);  // *output*, lmr at rot_tr_imr from the emf (H)
HAL_PIN(rot_tr_b);  // *output*, flux angle offset shared by the up and down ramps (rad)
HAL_PIN(rot_tr_ramp);  // *output*, tr at the ramps' current (id_n) from the open loop ramps (s), 0 = not measured, moved to rot_id_n for acim_flux0.tr
HAL_PIN(rot_lag);  // *output*, lag of vel_fb behind pos_fb on the ramps (s)
HAL_PIN(rot_delay);  // *output*, delay from field command to position feedback found on the ramps (s)
HAL_PIN(rot_vmax);  // *parameter*, highest field speed of the flux sweep with an encoder, fraction of the plate speed, default 1

HAL_PIN(pwm_volt);  // *input*, usable voltage from hv0.pwm_volt (V), limits avg_test_volt and the injection
HAL_PIN(dc_volt);  // *input*, dc link voltage from hv0.dc_volt (V), scales r_bias

HAL_PIN(cur_bw);  // *output*, current loop bandwidth to hv0.cur_bw (rad/s)

HAL_PIN(tmp0);  // *output*, filtered id_fb of the half current dwell (A)
HAL_PIN(tmp1);  // *output*, filtered ud_fb of the half current dwell (V)
HAL_PIN(tmp2);  // *output*, filtered id_fb of the full current dwell (A)
HAL_PIN(tmp3);  // *output*, filtered ud_fb of the full current dwell (V)
HAL_PIN(avg_test_volt);  // *output*, voltage that holds test_cur incl. dead time, dc bias of the leakage test (V)

#define ROT_BLK 8     // rotor test: the settled tail is read as the median of this many blocks
#define ROT_EDGES 32  // rotor test: edges kept per rung, so rot_cycles is at most 16
#define ROT_DEC 8     // rotor test: the fit sums take every this many ticks

// State of the leakage and rotor tests. They integrate over thousands of
// ticks, so none of this can be pins without making the state machine's
// scratch pins mean two different things at once.
#define SW_N 8             // rotating test: flux sweep levels
#define FO_CYC 3           // rotating tr check: measured up and down ramp pairs

struct idacim_ctx_t {
  // leakage injection
  uint8_t l_fi;     // 0 = l_freq_a, 1 = l_freq_b
  uint8_t l_up;     // the pair runs at l_freq_a/b times 2^l_up
  uint8_t l_stage;  // 0 settle, 1 size the amplitude, 2 measure
  uint16_t l_block; // sizing blocks done
  uint32_t l_n;     // samples in this block or window
  float l_t;        // time in this stage or block
  float l_th;       // injection phase
  float l_amp;      // injection voltage amplitude
  float v_re, v_im, i_re, i_im;
  float v_dc, i_dc, s_dc, c_dc;  // leakage window: sums for the dc removal
  float pp_raw;     // pole pair test: the ratio before rounding
  uint32_t pp_n;    // pole pair test: samples, and those the rotor turned in
  uint32_t pp_turn;
  // rotor step
  uint16_t r_edge;      // edges seen in this rung, the first comes from another level
  uint16_t r_n;         // edges that produced a fit (nrt)
  uint16_t r_seen;      // edges handed to nrt, fitted or not
  float half_e;         // this edge's length [s]
  float t0_e;           // this edge's rot_t0 [s]
  uint16_t dec;         // ticks since the fit sums were last fed
  uint8_t rung;         // 0 = the test at test_cur, 1.. = ladder rungs
  uint8_t act;          // this edge is being fitted
  uint8_t have_prev;    // the previous edge's settled values are known
  uint8_t have_t0;      // i_t0 captured
  float r_t;            // time since the last commanded edge
  float prev_u, prev_i; // where the previous edge settled
  float c0;             // offset taken out while integrating, ud - R id where the last edge settled
  float i0;             // settled current before the step
  float i_t0;           // id_fb at rot_t0
  float p;              // integral of ud - R id - c0 since the edge
  float q;              // integral of id - i0 since the edge
  float jp;             // integral of p
  float u1, u2, i1, i2; // the two samples before this one, for a median of three
  float s[14];          // fit sums, kept as a polynomial in this edge's own offset
  float blk_u[ROT_BLK], blk_i[ROT_BLK];  // settled tail, sums per block
  uint16_t blk_n[ROT_BLK];
  // rt hands each finished edge and rung to nrt, which does the solving and
  // the medians: in rt they cost the f4 more than its slack (bench, 5 Oct)
  float ps[14];         // the edge's sums
  float pd;             // its offset correction c - c0, from the tail
  float pdip;           // its |current error at rot_t0| / step
  uint8_t pup;          // 1 = it went up to the upper level
  float pdi;            // its step, settled to settled [A]
  float ptail;          // max - min of its tail's block means of ud [V]
  volatile uint8_t pend;   // an edge waits for nrt
  volatile uint8_t rdone;  // a rung waits for nrt
  uint8_t rd_rung;      // which rung
  float rd_hi;          // its upper current
  float rd_t1;          // where its settled tail starts, longest tr it can see
  float rd_per;         // rt period
  // nrt's
  float tr_e[ROT_EDGES], lm_e[ROT_EDGES];  // per edge fits
  uint8_t up_e[ROT_EDGES];                 // 1 = the edge went up to the upper level
  float dip;            // largest |current error at rot_t0| / step
  // rotating test
  float w_e;            // field speed [rad/s electrical]
  float w_t;            // test speed [rad/s electrical]
  float sw_t;           // time at this sweep level or ramp phase
  uint8_t sw_k;         // sweep level, or ramp phase in the inertia test
  uint8_t sw_k0;        // lowest sweep level used: lower ones would repeat i_min
  uint8_t sw_top;       // highest sweep level used: higher ones would repeat imax
  float sw_slip;        // flux sweep: slip, low passed, to stop before the rotor pulls out
  uint8_t sw_pass;      // flux sweep: 0 at the test speed, 1 at half of it, 2 back up
  uint8_t sw_np;        // flux sweep: points taken in pass 0
  float sw_hold;        // flux sweep: time at id_n before a speed change
  uint8_t sw_fi[SW_N];  // flux sweep: level of each point
  float sw_i1[SW_N], sw_p1[SW_N];  // pass 0: magnetizing current [A], rotor flux [Vs]
  float sw_i2[SW_N], sw_p2[SW_N];  // pass 1, 0 = not taken
  float sw_x2[SW_N];    // pass 1: slip x tr
  float sw_du[SW_N];    // q voltage offset the two speeds put on each point [V]
  uint8_t stall;        // the rotor fell behind the field
  float fo_imr;         // rotating tr check: the model's magnetizing current [A]
  float fo_pos;         // last pos_fb
  float psi_lp;         // |emf| / w, low passed, for PIN(psi)
  float com_last;       // com_pos a tick ago
  float fo_t;           // time in this ramp [s]
  uint8_t fo_ph;        // 0 up, 1 down, 2 down and out
  uint8_t fo_cyc;       // window crossings down
  float fo_s[2][6];     // up / down: sums of flux angle, current angle, i_mr, emf / w / i_mr; angle and emf / w / i_mr without the dead time
  uint32_t fo_n[2];
  float fo_a[2][4];     // up / down: sums of acceleration [rad/s^2 mech], |i|, emf / w [Vs], iq
  float fo_wl;          // last w_r
  uint32_t sw_cnt;      // samples summed
  float s_uq, s_iq, s_id, s_slip;  // sums
  float s_ud;                      // sum of ud, for the flux angle
  float sw_v1[SW_N], sw_w1[SW_N];  // pass 0: flux angle from the voltages (tan), encoder slip [rad/s el]
  float sw_v2[SW_N], sw_w2[SW_N];  // pass 1
  float s_tv;                      // ramp window: air gap power / field speed, summed [Nm / (1.5 pp)]
  float v_a, t_a, v_b, t_b;        // the ramp window's first and last speed and time
  // the ramps, up and down at id_n, then up and down at RAMP_I2 of it:
  float j_slip[4], j_alpha[4];     // mean slip from vel_fb [rad/s el], acceleration [rad/s^2 mech]
  float j_tv[4];                   // mean air gap torque / (1.5 pp) from the stator voltages
  float sw_x[SW_N];     // flux sweep: slip x tr per level
  float sw_w[SW_N];     // flux sweep: pass 0 field speed per point [rad/s el], pass 1 runs at half
  float pos_u, pos_l;   // rotor position unwrapped, and pos_fb last tick [rad mech]
  float p_a, p_b;       // ramp window: position at its first and last sample
  float s_we, s_pd, s_ir;          // ramp window sums: field speed, rotor flux on d, d current
  float j_ws[4], j_pd[4], j_ir[4]; // slip from position [rad/s el], rotor flux on d [Vs], d current [A]
  float tr_rot, lm_rot;            // tr and secant lmr at the ramps' current, from the ramps
  float d_tr, q_tr;                // current trims: d_cmd and q_cmd on top of the targets [A]
};

#define SW_MEASURE 0.5     // rotating test: averaging window per level [s]
#define SW_X_MAX 0.35      // flux sweep: largest slip x tr a point may have, standstill tr
#define SW_X_MAX_ROT 0.6   // the same with the ramps' tr
#define SW_X_STOP 1.0      // flux sweep: slip x tr where the drag nears pull out
#define RAMP_I2 0.65       // rotating test: d current of the second ramp pair, fraction of id_n
// of the d current at plate flux; the low ones reach into field weakening,
// where acim_flux's knee sits, and stop at i_min
static const float sw_frac[SW_N] = {0.25, 0.35, 0.45, 0.6, 0.75, 0.9, 1.0, 1.15};

#define L_BIAS_SETTLE 1.0  // leakage test: first settle, the dc bias rides the slow rotor pole [s]
#define L_SETTLE 0.2       // leakage test: settle after changing frequency [s]
#define L_BLOCK 0.05       // leakage test: one amplitude sizing block [s]
#define L_BLOCKS 4         // leakage test: sizing blocks per frequency
#define L_MEASURE 0.4      // leakage test: demodulation window per frequency [s]
#define ROT_TAIL 0.75      // rotor test: the settled tail starts here, fraction of rot_half

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;
  PIN(r_known)                  = 0.0;
  PIN(drop_slope)               = 0.0039;
  // The r chord's dead time correction needs every phase above about 2 A in
  // its lower dwell (test_cur/2 on d is -test_cur/4 on v and w), and the
  // rotor test reuses the same two levels.
  PIN(test_cur)                 = 8.0;
  PIN(i_min)                    = 4.0;
  PIN(test_vel)                 = 50.0;
  // 0: a pair either side of the current loop's crossover f_c = loop_bw /
  // 2 pi, at 0.75 and 1.5 f_c (120/240 Hz at cur_bw 1000, 360/720 at 3000).
  // The leakage has no plateau on a cage rotor (spindle, 6 Oct: 0.93 mH at
  // 120/240 Hz, 0.75 at 360/720), so this is the band conf0.l has to
  // describe, not the kHz an LCR meter defaults to.
  PIN(l_freq_a)                 = 0.0;
  PIN(l_freq_b)                 = 0.0;
  PIN(l_ripple)                 = 0.15;
  // tr is 50 to 150 ms on a few kW motor. Each level's settled value is
  // read over its last quarter, so 1.5 s puts that 7.6 tr out even at
  // 150 ms; at 1.0 s the leftover tail reads tr 2% short there (simulated).
  PIN(rot_half)                 = 1.5;
  PIN(rot_cycles)               = 4.0;
  PIN(rot_bw)                   = 1500.0;
  PIN(rot_t0)                   = 0.015;
  PIN(lad_top)                  = 0.0;
  PIN(lad_n)                    = 4.0;
  PIN(rot_vel)                  = 0.4;
  PIN(rot_vmax)                 = 1.0;
  PIN(rot_acc)                  = 0.0;
  PIN(cur_bw)                   = 1.0;
}

static float median(float *v, int n) {  // sorts v
  for(int i = 1; i < n; i++) {
    float x = v[i];
    int j   = i - 1;
    while(j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  if(n <= 0) {
    return 0.0;
  }
  return n & 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

static float med3(float a, float b, float c) {
  return MAX(MIN(a, b), MIN(MAX(a, b), c));
}

// ctx survives a stop
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx = (struct idacim_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idacim_ctx_t));
}

// x' G y over the basis (a, bk, t, sg, yk) the fit sums are kept in
static double rot_dot(const double g[5][5], const double *x, const double *y) {
  double r = 0.0;
  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      r += x[i] * g[i][j] * y[j];
    }
  }
  return r;
}

// The rotor test's arithmetic, out of rt. Each edge's fit is solved here in
// double, from the sums rt collected; at the end of a rung the edges are
// reduced to medians.
//
// The edge's offset c is fitted with Lmr and tr, not read off the tail: an
// error e in c grows as e t in lam, and at 16 A one mV of it moved lmr 2% and
// tr with it (bench, 5 Oct; the tail's median carried a few mV). With
// y = yk + d sg and b = bk + d t the model y = Lmr a + tr b is linear in Lmr
// and tr but not in d, so Gauss-Newton from the tail's d, which is close.
// The settled part pins d down through the curvature d t^2/2 it leaves in the
// integral, over the whole edge instead of its last quarter.
static void rot_nrt(struct idacim_ctx_t *ctx, struct idacim_pin_ctx_t *pins) {
  if(ctx->pend) {
    float tr = 0.0, lm = 0.0, dfit = 0.0;
    int ok   = 0;
    if(ctx->r_n < ROT_EDGES) {
      float *s       = ctx->ps;
      double g[5][5] = {
          {s[0], s[1], s[2], s[7], s[6]},
          {s[1], s[3], s[4], s[9], s[8]},
          {s[2], s[4], s[5], s[11], s[10]},
          {s[7], s[9], s[11], s[12], s[13]},
          {s[6], s[8], s[10], s[13], 0.0},  // yk yk is not needed
      };
      // start: Lmr and tr at the tail's d
      double d   = ctx->pd;
      double ea[5] = {1, 0, 0, 0, 0};
      double eb[5] = {0, 1, d, 0, 0};
      double ey[5] = {0, 0, 0, d, 1};
      double aa  = rot_dot(g, ea, ea);
      double ab  = rot_dot(g, ea, eb);
      double bb  = rot_dot(g, eb, eb);
      double ay  = rot_dot(g, ea, ey);
      double by  = rot_dot(g, eb, ey);
      double det = aa * bb - ab * ab;
      ok         = det > 0.0;
      double x0  = ok ? (ay * bb - by * ab) / det : 0.0;
      double x1  = ok ? (by * aa - ay * ab) / det : 0.0;
      for(int it = 0; it < 6 && ok; it++) {
        // residual y - model and the model's derivatives in (Lmr, tr, d)
        double r[5]    = {-x0, -x1, -x1 * d, d, 1};
        double j[3][5] = {
            {1, 0, 0, 0, 0},
            {0, 1, d, 0, 0},
            {0, 0, x1, -1, 0},
        };
        double m[3][3], v[3];
        for(int k = 0; k < 3; k++) {
          v[k] = rot_dot(g, j[k], r);
          for(int l = 0; l < 3; l++) {
            m[k][l] = rot_dot(g, j[k], j[l]);
          }
        }
        double dm = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if(!(dm > 0.0)) {
          ok = 0;
          break;
        }
        double s0 = (v[0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (v[1] * m[2][2] - m[1][2] * v[2]) + m[0][2] * (v[1] * m[2][1] - m[1][1] * v[2])) / dm;
        double s1 = (m[0][0] * (v[1] * m[2][2] - m[1][2] * v[2]) - v[0] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * v[2] - v[1] * m[2][0])) / dm;
        double s2 = (m[0][0] * (m[1][1] * v[2] - v[1] * m[2][1]) - m[0][1] * (m[1][0] * v[2] - v[1] * m[2][0]) + v[0] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / dm;
        x0 += s0;
        x1 += s1;
        d += s2;
      }
      if(ok) {
        lm   = (float)x0;
        tr   = (float)x1;
        dfit = (float)d;
        ok = tr > 0.0 && lm > 0.0;
        if(ok) {
          ctx->tr_e[ctx->r_n] = tr;
          ctx->lm_e[ctx->r_n] = lm;
          ctx->up_e[ctx->r_n] = ctx->pup;
          ctx->dip            = MAX(ctx->dip, ctx->pdip);
          ctx->r_n++;
        }
      }
    }
    // one line per edge, to see whether the scatter between edges is noise
    // or follows something: direction, step, offset correction, tail noise
    ctx->r_seen++;
    printf("# edge %f %s tr %f ms lmr %f mH d %f V (tail %f) di %f A dip %f tail %f V%s\n", (float)ctx->r_seen, ctx->pup ? "up" : "dn", tr * 1000.0, lm * 1000.0, dfit, ctx->pd, ctx->pdi, ctx->pdip, ctx->ptail, ok ? "" : " rejected");
    ctx->pend = 0;
  }
  if(ctx->rdone && !ctx->pend) {
    int n    = ctx->r_n;
    float tr = 0.0, lm = 0.0, sp = 0.0, tmin = 0.0, tmax = 0.0, trise = 0.0, tfall = 0.0;
    int ok   = 0;
    if(n >= 2) {
      // Rising and falling edges are taken apart: over a step that
      // reaches into saturation the flux is not linear in i_mr, and the
      // fit then reads one direction long and the other short by about
      // the same amount (simulated, 8 <-> 16 A on a knee at 12 A: 82 and
      // 123 ms on 100). The mean of the two medians cancels that, and the
      // spread is the worse of the two directions' own, so it says how
      // repeatable the edges are, not how saturated the step is.
      float v[ROT_EDGES];
      float med[2] = {0.0, 0.0};
      int cnt[2]   = {0, 0};
      for(int up = 0; up < 2; up++) {
        int m = 0;
        for(int k = 0; k < n; k++) {
          if(ctx->up_e[k] == up) {
            v[m++] = ctx->tr_e[k];
          }
        }
        med[up] = median(v, m);  // sorted now
        cnt[up] = m;
        if(m > 0) {
          tmin = (cnt[0] + cnt[1] == m) ? v[0] : MIN(tmin, v[0]);
          tmax = (cnt[0] + cnt[1] == m) ? v[m - 1] : MAX(tmax, v[m - 1]);
          if(med[up] > 0.0) {
            sp = MAX(sp, (v[m - 1] - v[0]) / med[up]);
          }
        }
      }
      tfall = med[0];
      trise = med[1];
      tr    = cnt[0] && cnt[1] ? 0.5 * (trise + tfall) : (cnt[1] ? trise : tfall);
      // lmr from the falling edges. Rising ones read it 20% under them at
      // 12 and 16 A, and with 3 s edges instead of 1.5 their lmr spread
      // 7.5-11.6 mH and two of four fits failed, while falling ones held
      // 13.2-13.8, the at-speed 13.2 (bench, 5 Oct): something slow at the
      // upper level (heating, or flux creeping near saturation) bends the
      // edges that end there. A falling edge ends at the lower level, where
      // both are smallest. Rising edges only when no falling one fitted.
      int lup = cnt[0] ? 0 : 1;
      int m   = 0;
      for(int k = 0; k < n; k++) {
        if(ctx->up_e[k] == lup) {
          v[m++] = ctx->lm_e[k];
        }
      }
      lm = median(v, m);
      // a fit that lands outside the window it was taken over is not an
      // exponential this test can see
      ok = tr > 5.0 * ctx->rd_per && tr < ctx->rd_t1;
    }
    if(ctx->rd_rung == 0) {
      PIN(rot_n)     = n;
      PIN(rot_dip)   = ctx->dip;
      PIN(tr)        = tr;  // reported either way, so a rejected fit still says what it was
      PIN(tr_rise)   = trise;
      PIN(tr_fall)   = tfall;
      PIN(tr_min)    = tmin;
      PIN(tr_max)    = tmax;
      PIN(tr_spread) = sp;
      PIN(slip_n)    = ok ? 1.0 / tr : 0.0;
      PIN(lmr)       = ok ? lm : 0.0;
      PIN(ls)        = ok ? PIN(l) + lm : 0.0;
      PIN(tr_ok)     = ok ? 1.0 : 0.0;
      PIN(knee_i)      = 0.0;
      PIN(knee_tr_sat) = 0.0;
      PIN(knee_tr)     = 0.0;
      for(int k = 0; k < 4; k++) {
        PINA(lad_i, k)  = 0.0;
        PINA(lad_lm, k) = 0.0;
        PINA(lad_tr, k) = 0.0;
        PINA(lad_sp, k) = 0.0;
      }
    } else {
      PINA(lad_i, ctx->rd_rung - 1)  = ctx->rd_hi;
      PINA(lad_lm, ctx->rd_rung - 1) = ok ? lm : 0.0;
      PINA(lad_tr, ctx->rd_rung - 1) = tr;
      PINA(lad_sp, ctx->rd_rung - 1) = sp;
    }

    ctx->r_n    = 0;
    ctx->r_seen = 0;
    ctx->dip    = 0.0;
    ctx->rdone = 0;
  }
}

// ladder rung k (1..n): its top, and its lower current as a fraction of it
static float lad_hi(struct idacim_pin_ctx_t *pins, int k, int n) {
  if(PIN(lad_bot) > 0.0 && n > 1) {
    return PIN(lad_bot) + (PIN(lad_top) - PIN(lad_bot)) * (float)(k - 1) / (float)(n - 1);
  }
  return PIN(lad_top) * (float)k / (float)MAX(n, 1);
}
static float lad_lo(struct idacim_pin_ctx_t *pins) {
  return PIN(lad_ratio) > 0.0 ? CLAMP(PIN(lad_ratio), 0.3, 0.9) : 0.5;
}

// acim_flux's tr model, tr (1 + tr_sat x) with x = (i_n - i) / (i_n - i_knee)
// clamped 0..1, fitted to the ladder rungs' tr and the main test's, each at
// its range's middle (0.75 of its top). For every knee on a grid a weighted
// straight line in x gives tr at i_n and tr_sat; the knee with the smallest
// residual wins. Rungs are weighted by their spread.
#define KNEE_SP_MAX 0.3   // largest edge spread a level may have to enter the fit
#define KNEE_SAT_MAX 1.0  // largest tr_sat accepted
#define KNEE_TR_DEV 0.3   // tr at id_n within this fraction of the main test's

static void knee_fit(struct idacim_pin_ctx_t *pins) {
  float pi[5], pt[5], pw[5];
  int n    = 0;
  float in = PIN(id_n);
  PIN(knee_i)      = 0.0;
  PIN(knee_tr_sat) = 0.0;
  PIN(knee_tr)     = 0.0;
  for(int k = 0; k < (int)MIN(PIN(lad_n), 4.0); k++) {
    // under i_min the r chord no longer describes the dead time, and the
    // step's flux reads off (simulated: tr +14% at 2.4..4.8 A): such rungs
    // are listed, not fitted
    // a rung whose edges spread over KNEE_SP_MAX says less than the effect
    // looked for (spindle, 6 Oct: 0.32-1.03 on 1.9-4.8 A steps, where the
    // main test's 8 A step held 0.15), so it is listed, not fitted
    float lo = lad_lo(pins) * PINA(lad_i, k);
    if(PINA(lad_lm, k) > 0.0 && PINA(lad_tr, k) > 0.0 && lo >= 0.999 * PIN(i_min) && PINA(lad_sp, k) <= KNEE_SP_MAX) {
      float sp = MAX(PINA(lad_sp, k), 0.05);
      pi[n]    = 0.5 * (1.0 + lad_lo(pins)) * PINA(lad_i, k);
      pt[n]    = PINA(lad_tr, k);
      pw[n++]  = 1.0 / (sp * sp);
    }
  }
  if(PIN(tr_ok) > 0.0) {
    float sp = MAX(PIN(tr_spread), 0.05);
    pi[n]    = 0.75 * PIN(test_cur);
    pt[n]    = PIN(tr);
    pw[n++]  = 1.0 / (sp * sp);
  }
  if(n < 3 || in <= 0.0) {
    printf("<font color='red'># tr knee: needs id_n (plate pins) and three levels with spread under %f (ladder rungs or the main test), have %f: keep the config's i_knee and tr_sat</font>\n", KNEE_SP_MAX, (float)n);
    return;
  }
  float best = -1.0, ba = 0.0, bb = 0.0, bk = 0.0;
  for(int g = 0; g <= 90; g++) {
    float ik = in * 0.01 * (float)g;
    float sw = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for(int j = 0; j < n; j++) {
      float x = CLAMP((in - pi[j]) / (in - ik), 0.0, 1.0);
      sw += pw[j];
      sx += pw[j] * x;
      sy += pw[j] * pt[j];
      sxx += pw[j] * x * x;
      sxy += pw[j] * x * pt[j];
    }
    float det = sw * sxx - sx * sx;
    if(det <= 1e-9 * sw * sw) {  // every point on the same x: no slope to fit
      continue;
    }
    float b = (sw * sxy - sx * sy) / det;
    float a = (sy - b * sx) / sw;
    float e = 0.0;
    for(int j = 0; j < n; j++) {
      float x = CLAMP((in - pi[j]) / (in - ik), 0.0, 1.0);
      float d = pt[j] - a - b * x;
      e += pw[j] * d * d;
    }
    if(a > 0.0 && (best < 0.0 || e < best)) {
      best = e;
      ba   = a;
      bb   = b;
      bk   = ik;
    }
  }
  if(best < 0.0) {
    printf("<font color='red'># tr knee: no fit</font>\n");
    return;
  }
  float sat = bb / ba;
  printf("<font color='green'># tr knee fit over %f levels: id [A], tr [ms], model [ms]\n", (float)n);
  for(int j = 0; j < n; j++) {
    float x = CLAMP((in - pi[j]) / (in - bk), 0.0, 1.0);
    printf("# %f %f %f\n", pi[j], pt[j] * 1000.0, ba * (1.0 + sat * x) * 1000.0);
  }
  printf("</font>");
  // A fit that bends tr by more than its own size, or puts tr at id_n far
  // from the main test's, is extrapolating noise (spindle, 6 Oct: tr_sat 7
  // through one rung, tr 12 ms at id_n against 97 measured), not a knee.
  // tr falling at low flux is not what saturation does either.
  float trm = PIN(tr_ok) > 0.0 ? PIN(tr) : 0.0;
  if(sat < 0.0 || sat > KNEE_SAT_MAX || (trm > 0.0 && (ba < (1.0 - KNEE_TR_DEV) * trm || ba > (1.0 + KNEE_TR_DEV) * trm))) {
    printf("<font color='red'># tr knee: fit rejected (i_knee %f, tr_sat %f, tr at id_n %f ms against %f measured): keep the config's i_knee and tr_sat</font>\n", bk, sat, ba * 1000.0, trm * 1000.0);
    return;
  }
  PIN(knee_i)      = bk;
  PIN(knee_tr_sat) = sat;
  PIN(knee_tr)     = ba;
  printf("acim_flux0.tr = %f <font color='green'># append to config, tr at id_n %f A (replaces the tr above, taken at %f A)</font>\n", ba, in, 0.75 * PIN(test_cur));
  printf("acim_flux0.i_knee = %f <font color='green'># append to config</font>\n", bk);
  printf("acim_flux0.tr_sat = %f <font color='green'># append to config</font>\n", PIN(knee_tr_sat));
}

// The V/f set from the standstill measurements and the plate: what a VFD's
// standstill autotune gives, enough to run a spindle without feedback.
// The plate voltage at the plate frequency sets the flux; at no load the
// stator sees r id_n + j w ls id_n, so that voltage gives id_n with the ls
// this test read. ls is lmr's chord over test_cur/2..test_cur, so id_n is
// best when test_cur is near it. Slip at active current iq is iq / (tr i_mr)
// electrical, and the boost covers r id_n where the emf is still small,
// plus the dead time loss: V/f runs in volt mode with no compensation, and a
// rotating vector loses the square wave's fundamental, 4/pi of the per phase drop
// (spindle, 9 Oct: r id_n alone, 3.7 V, stalled; 12 V started).
static void vf_set(struct idacim_pin_ctx_t *pins) {
  float u  = PIN(n_volt) * 0.8164966;  // line to line rms to phase peak
  float we = 2.0 * M_PI * PIN(n_freq);
  float pp = PIN(n_pp);
  float r  = PIN(r);
  float ls = PIN(ls);
  float in = PIN(n_cur) * 1.4142136;
  float id = u / sqrtf(r * r + we * we * ls * ls);
  float iq = in > id ? sqrtf(in * in - id * id) : 0.0;

  PIN(id_n)         = id;
  PIN(iq_n)         = iq;
  PIN(vf_u_n)       = u;
  PIN(vf_vel_n)     = we / pp;
  PIN(vf_boost)     = r * id + 4.0 / M_PI * PIN(drop);  // square wave loss, fundamental
  PIN(vf_boost_vel) = CLAMP(3.0 * r / ls, 0.02 * we, 0.2 * we) / pp;  // where w ls is 3 r
  PIN(vf_slip_n)    = iq / (PIN(tr) * id) / pp;
  PIN(vf_cur_n)     = iq;

  printf("<font color='green'># V/f from the standstill test and the plate (%f V, %f Hz, %f A, %f pole pairs):</font>\n", PIN(n_volt), PIN(n_freq), PIN(n_cur), pp);
  printf("conf0.polecount = %f <font color='green'># append to config</font>\n", pp);
  printf("vf0.u_n = %f <font color='green'># append to config</font>\n", PIN(vf_u_n));
  printf("vf0.vel_n = %f <font color='green'># append to config</font>\n", PIN(vf_vel_n));
  printf("vf0.u_boost = %f <font color='green'># append to config</font>\n", PIN(vf_boost));
  printf("vf0.boost_vel = %f <font color='green'># append to config</font>\n", PIN(vf_boost_vel));
  printf("vf0.slip_n = %f <font color='green'># append to config</font>\n", PIN(vf_slip_n));
  printf("vf0.cur_n = %f <font color='green'># append to config</font>\n", PIN(vf_cur_n));
  printf("acim_foc0.id_n = %f <font color='green'># FOC start value, the rotating test refines it</font>\n", id);
  printf("<font color='green'># id_n %f A and iq_n %f A peak at the plate current.\n", id, iq);
  if(in <= id) {
    printf("</font><font color='red'># id_n is over the plate current: check n_volt, n_freq and n_cur.</font>\n");
  } else if(id > 1.2 * PIN(test_cur) || id < 0.6 * PIN(test_cur)) {
    printf("# lmr was read at %f..%f A: for an id_n that saturates the iron the\n", PIN(test_cur) * 0.5, PIN(test_cur));
    printf("# way it will run, rerun with idacim0.test_cur near %f.</font>\n", id);
  } else {
    printf("# lmr was read at %f..%f A, close enough to id_n.</font>\n", PIN(test_cur) * 0.5, PIN(test_cur));
  }
}

// The rotating test's results. Flux sweep: at no load the rotor runs at the
// field's speed, so in the field's frame uq = r iq + w psi_s,d and the rotor
// flux is psi_s,d - l id. Its secant over id is acim_flux's lmr; the point
// where the stator flux reaches plate voltage over plate frequency is the
// rated magnetizing current. The uq read is clean of the dead time: that sits
// in phase with the current, on d.
//
// Inertia, with an encoder: on a ramp the rotor lags the field by the slip
// ws, and a current fed cage then gives T = 1.5 pp lmr i^2 ws tr / (1 +
// (ws tr)^2). The same ramp up and down cancels friction:
// J = (T_up - T_down) / (alpha_up - alpha_down), friction = (T_up + T_down) / 2.
static void rot_report(struct idacim_ctx_t *ctx, struct idacim_pin_ctx_t *pins) {
  int n    = (int)PIN(sw_n);
  float pp = PIN(n_pp);
  float wn = 2.0 * M_PI * PIN(n_freq);
  float idr = PIN(id_n) > 0.0 ? PIN(id_n) : PIN(test_cur);
  printf("<font color='green'># flux sweep: magnetizing current [A], rotor flux [Vs], secant lmr [mH]; speed [rad/s mech], lmr [mH] and slip x tr there, and at half that speed; q voltage offset [V]\n");
  for(int k = 0; k < n; k++) {
    printf("# %f %f %f; %f %f %f %f %f %f\n", PINA(sw_i, k), PINA(sw_psi, k), PINA(sw_i, k) > 0.0 ? PINA(sw_psi, k) / PINA(sw_i, k) * 1000.0 : 0.0, ctx->sw_w[k] / pp,
           ctx->sw_i1[k] > 0.0 ? ctx->sw_p1[k] / ctx->sw_i1[k] * 1000.0 : 0.0, ctx->sw_x[k], ctx->sw_i2[k] > 0.0 ? ctx->sw_p2[k] / ctx->sw_i2[k] * 1000.0 : 0.0, ctx->sw_x2[k], ctx->sw_du[k]);
  }
  printf("</font>");
  // rotating tr: the flux angle from the voltages over the encoder's slip,
  // per point and speed. Iron loss puts w lmr / R_fe on x_v at any slip;
  // the two speeds take it out: tr2 = (x1 - 2 x2) / (ws1 - 2 ws2)
  printf("<font color='green'># rotating tr per point: i_mr [A]; x_v, slip [rad/s el], x_v / slip [ms] at each speed; two speed tr [ms]\n");
  for(int k = 0; k < n; k++) {
    float w1 = ctx->sw_w1[k], w2 = ctx->sw_w2[k];
    float den = w1 - 2.0 * w2;
    printf("# %f; %f %f %f; %f %f %f; %f\n", PINA(sw_i, k), ctx->sw_v1[k], w1, ABS(w1) > 0.05 ? ctx->sw_v1[k] / w1 * 1000.0 : 0.0,
           ctx->sw_v2[k], w2, ABS(w2) > 0.05 ? ctx->sw_v2[k] / w2 * 1000.0 : 0.0, ABS(den) > 0.05 ? (ctx->sw_v1[k] - 2.0 * ctx->sw_v2[k]) / den * 1000.0 : 0.0);
  }
  printf("</font>");
  if(ctx->stall && n == 0) {
    printf("<font color='red'>the rotor did not follow the field up to speed: lower idacim0.rot_acc, or unload the shaft</font>\n");
    return;
  }
  if(n < 2) {
    printf("<font color='red'>flux sweep incomplete</font>\n");
    return;
  }
  // d current at plate flux: stator flux psi_r + l id against u_n / w_n, by
  // linear interpolation (or extension from the nearest two)
  float id_n = idr;
  if(PIN(n_volt) > 0.0) {
    float psn = PIN(n_volt) * 0.8164966 / wn;
    int k     = 1;
    while(k < n - 1 && PINA(sw_psi, k) + PIN(l) * PINA(sw_i, k) < psn) {
      k++;
    }
    float ia = PINA(sw_i, k - 1), ib = PINA(sw_i, k);
    float pa = PINA(sw_psi, k - 1) + PIN(l) * ia, pb = PINA(sw_psi, k) + PIN(l) * ib;
    if(pb > pa) {
      id_n = ia + (psn - pa) * (ib - ia) / (pb - pa);
    }
  }
  // secant lmr there, interpolated the same way
  int k = 1;
  while(k < n - 1 && PINA(sw_i, k) < id_n) {
    k++;
  }
  float ia = PINA(sw_i, k - 1), ib = PINA(sw_i, k);
  float la = PINA(sw_psi, k - 1) / ia, lb = PINA(sw_psi, k) / ib;
  float lmr = ib > ia ? la + (id_n - ia) * (lb - la) / (ib - ia) : lb;
  // acim_flux's shape of the secant lmr under id_n: lmr (1 + lmr_sat x)
  // (1 - lmr_dip y), x = (id_n - i) / (id_n - i_knee) and y = (i_dip - i) /
  // i_dip clamped 0..1. Growth from id_n down to the knee as the iron
  // desaturates, flat, then a fall under i_dip at low induction (spindle, 6
  // Oct: flat 9-19 A, 11% down at 6.7 A). For every i_knee and i_dip <=
  // i_knee on a grid, lmr, lmr_sat and lmr_dip by least squares (a term that
  // comes out negative is dropped and the rest refitted); the smallest
  // residual wins. The points repeat to about 0.5%, against 10-25% for
  // standstill steps at these currents. Rotor resistance does not saturate,
  // so tr = Lr / Rr follows the same curve: tr_sat = lmr_sat, and acim_flux
  // puts the dip on tr itself.
  float sat = 0.0, ik = 0.0, fa = 0.0, dip = 0.0, idp = 0.0;
  int nlow  = 0;
  for(int j = 0; j < n; j++) {
    nlow += PINA(sw_i, j) < 0.95 * id_n;
  }
  if(nlow >= 3 && lmr > 0.0) {
    float best = -1.0;
    // a knee or a dip edge under the lowest point is a plateau nobody saw
    // (simulated: tr 10% off and 3 ms of speed lag put a knee at 0.7 A with
    // tr_sat 0.27 on a 8.5 A, 0.14 motor), so the grids start there
    float i_lo = PINA(sw_i, 0);
    float g0   = CLAMP(i_lo / id_n, 0.0, 0.9);
    for(int g = 0; g <= 30; g++) {
      float kg = id_n * (g0 + (0.9 - g0) * 0.0333333 * (float)g);
      for(int h = 0; h <= 30; h++) {
        float dg = i_lo + (kg - i_lo) * 0.0333333 * (float)h;
        // columns 1, x, -y; a fit that turns a term negative is redone
        // without it: both, growth only, dip only, flat
        for(int vr = 0; vr < 4; vr++) {
          int use_x = vr == 0 || vr == 1, use_y = vr == 0 || vr == 2;
          // one point under the dip's edge fits any depth with some edge:
          // a dip needs two
          int under = 0;
          for(int j = 0; j < n; j++) {
            under += PINA(sw_i, j) < dg;
          }
          if(use_y && under < 2) {
            continue;
          }
          int m     = 1 + use_x + use_y;
          float A[3][3] = {{0.0}}, B[3] = {0.0};
          int cnt = 0;
          for(int j = 0; j < n; j++) {
            // acim_flux holds lmr above i_n, where the iron still
            // saturates: points over id_n would only pull the line off
            // the ones under it
            float ij = PINA(sw_i, j);
            if(ij > 1.02 * id_n) {
              continue;
            }
            float vx   = CLAMP((id_n - ij) / (id_n - kg), 0.0, 1.0);
            float vy   = dg > 0.0 ? -CLAMP((dg - ij) / dg, 0.0, 1.0) : 0.0;
            float v[3] = {1.0, use_x ? vx : vy, vy};
            float yj = PINA(sw_psi, j) / ij;
            for(int r = 0; r < m; r++) {
              for(int c = 0; c < m; c++) {
                A[r][c] += v[r] * v[c];
              }
              B[r] += v[r] * yj;
            }
            cnt++;
          }
          if(cnt < m + 1) {
            continue;
          }
          // Gauss elimination, m <= 3
          int sing = 0;
          for(int c = 0; c < m && !sing; c++) {
            if(ABS(A[c][c]) < 1e-9 * (A[0][0] + 1e-9)) {
              sing = 1;
              break;
            }
            for(int r = c + 1; r < m; r++) {
              float f = A[r][c] / A[c][c];
              for(int k = c; k < m; k++) {
                A[r][k] -= f * A[c][k];
              }
              B[r] -= f * B[c];
            }
          }
          if(sing) {
            continue;
          }
          float X[3] = {0.0, 0.0, 0.0};
          for(int r = m - 1; r >= 0; r--) {
            float t = B[r];
            for(int k = r + 1; k < m; k++) {
              t -= A[r][k] * X[k];
            }
            X[r] = t / A[r][r];
          }
          float a0 = X[0];
          float s0 = use_x ? X[1] : 0.0;
          float d0 = use_y ? X[m - 1] : 0.0;
          if(a0 <= 0.0 || s0 < 0.0 || d0 < 0.0) {
            continue;  // try with fewer terms
          }
          float e = 0.0;
          for(int j = 0; j < n; j++) {
            float ij = PINA(sw_i, j);
            if(ij > 1.02 * id_n) {
              continue;
            }
            float x = CLAMP((id_n - ij) / (id_n - kg), 0.0, 1.0);
            float y = dg > 0.0 ? CLAMP((dg - ij) / dg, 0.0, 1.0) : 0.0;
            float d = PINA(sw_psi, j) / ij - (a0 + s0 * x - d0 * y);
            e += d * d;
          }
          if(best < 0.0 || e < best * 0.999) {
            best = e;
            fa   = a0;
            sat  = s0 / a0;
            ik   = kg;
            dip  = d0 / a0;
            idp  = dg;
          }
          break;
        }
      }
    }
  }
  if(sat < 0.005) {  // no growth: no knee to set
    sat = 0.0;
    ik  = 0.0;
  }
  if(dip < 0.005) {
    dip = 0.0;
    idp = 0.0;
  }
  // terms past double or a dip near total, or an lmr at id_n off the
  // interpolated one, are noise or a rotor that slipped
  int knee_ok = PIN(rot_enc) > 0.0 && fa > 0.0 && sat <= KNEE_SAT_MAX && dip <= 0.5 && ABS(fa - lmr) < 0.05 * lmr;
  float sat_f = sat, dip_f = dip, fa_f = fa;
  if(!knee_ok) {
    // fallback: a straight line through the lowest point at or over 0.4
    // id_n (lower ones sit in the dip), no knee
    int j0 = 0;
    while(j0 < n - 1 && PINA(sw_i, j0) < 0.4 * id_n) {
      j0++;
    }
    float i0 = PINA(sw_i, j0);
    sat      = (i0 < id_n && lmr > 0.0) ? (PINA(sw_psi, j0) / i0 / lmr - 1.0) / (1.0 - i0 / id_n) : 0.0;
    ik       = 0.0;
    dip      = 0.0;
    idp      = 0.0;
  }
  // without an encoder a pulled out rotor shows as flux far under what the
  // standstill test read: slip shorts it
  if(lmr < 0.7 * PIN(lmr)) {
    printf("<font color='red'># lmr %f mH is far under the standstill %f mH: the rotor may not have followed the field (load, or rot_acc too high)</font>\n", lmr * 1000.0, PIN(lmr) * 1000.0);
  }
  if(PIN(rot_enc) <= 0.0) {
    printf("<font color='green'># no encoder: the sweep is not corrected for slip, drag reads as saturation at the low points (lmr_sat low)</font>\n");
  }
  PIN(rot_id_n)    = id_n;
  PIN(rot_lmr)     = lmr;
  PIN(rot_lmr_sat) = sat;
  PIN(rot_i_knee)  = ik;
  PIN(rot_i_dip)   = idp;
  PIN(rot_lmr_dip) = dip;
  printf("acim_foc0.id_n = %f <font color='green'># append to config</font>\n", id_n);
  printf("acim_flux0.i_n = %f <font color='green'># append to config</font>\n", id_n);
  printf("acim_flux0.lmr = %f <font color='green'># append to config, secant at id_n</font>\n", lmr);
  // the ramps' lmr from the sweep at their current: tr follows lmr from
  // there to id_n (Rr does not saturate)
  float lmj = lmr;
  {
    int m = 1;
    while(m < n - 1 && PINA(sw_i, m) < idr) {
      m++;
    }
    float ja = PINA(sw_i, m - 1), jb = PINA(sw_i, m);
    float lma = PINA(sw_psi, m - 1) / ja, lmb = PINA(sw_psi, m) / jb;
    lmj = jb > ja ? lma + (idr - ja) * (lmb - lma) / (jb - ja) : lmb;
  }
  if(PIN(rot_tr_ramp) > 0.0 && lmj > 0.0) {
    printf("acim_flux0.tr = %f <font color='green'># append to config, from the ramps (%f at %f A), replaces the standstill tr %f</font>\n", PIN(rot_tr_ramp) * lmr / lmj, PIN(rot_tr_ramp), idr, PIN(tr));
    printf("<font color='green'># ramps at %f and %f A: field to position delay %f ms</font>\n", 0.5 * (ctx->j_ir[0] + ctx->j_ir[1]), 0.5 * (ctx->j_ir[2] + ctx->j_ir[3]), PIN(rot_delay) * 1000.0);
  } else if(PIN(rot_enc) > 0.0 && !ctx->stall) {
    printf("<font color='red'># no tr from the ramps (the torque or the slip did not differ between them): keep the standstill tr %f</font>\n", PIN(tr));
  }
  if(knee_ok || PIN(rot_enc) <= 0.0) {
    printf("acim_flux0.lmr_sat = %f <font color='green'># append to config</font>\n", MAX(sat, 0.0));
  } else {
    // the fallback line runs through the dip and does not fit the config's
    // knee and dip, which stay
    printf("<font color='green'># acim_flux0.lmr_sat %f from the fallback line, not to append</font>\n", sat);
  }
  if(knee_ok) {
    printf("acim_flux0.i_knee = %f <font color='green'># append to config, lmr and tr flat below it</font>\n", ik);
    printf("acim_flux0.tr_sat = %f <font color='green'># append to config, tr follows the secant lmr</font>\n", sat);
    printf("acim_flux0.i_dip = %f <font color='green'># append to config, lmr and tr fall below it</font>\n", idp);
    printf("acim_flux0.lmr_dip = %f <font color='green'># append to config</font>\n", dip);
    // tr = Lr / Rr follows the secant lmr below the dip as it does above the
    // knee (tr_sat): printed, so a clean config needs no hand step (13 Oct:
    // the rotating checks agree with this to a few % on the spindle)
    printf("acim_flux0.tr_dip = %f <font color='green'># append to config, tr follows the secant lmr</font>\n", dip);
    printf("<font color='green'># fitted lmr at id_n %f mH; model against the sweep:", fa * 1000.0);
    for(int j = 0; j < n; j++) {
      float ij = PINA(sw_i, j);
      float x  = ik > 0.0 || sat > 0.0 ? CLAMP((id_n - ij) / (id_n - ik), 0.0, 1.0) : 0.0;
      float y  = idp > 0.0 ? CLAMP((idp - ij) / idp, 0.0, 1.0) : 0.0;
      printf(" %f %f/%f", ij, PINA(sw_psi, j) / ij * 1000.0, ij > 1.02 * id_n ? fa * 1000.0 : fa * (1.0 + sat * x) * (1.0 - dip * y) * 1000.0);
    }
    printf("</font>\n");
  } else if(PIN(rot_enc) > 0.0 && nlow < 3) {
    printf("<font color='red'># saturation knee: %f sweep points under id_n with slip x tr up to %f, 3 needed (the drag stopped the sweep); keep the config's i_knee, tr_sat, i_dip and lmr_dip</font>\n", (float)nlow, ctx->tr_rot > 0.0 ? SW_X_MAX_ROT : SW_X_MAX);
  } else if(PIN(rot_enc) > 0.0) {
    printf("<font color='red'># saturation knee: fit rejected (lmr_sat %f, lmr_dip %f, lmr at id_n %f mH against %f), lmr_sat from the lowest point over 0.4 id_n; keep the config's i_knee, tr_sat, i_dip and lmr_dip</font>\n", sat_f, dip_f, fa_f * 1000.0, lmr * 1000.0);
  } else {
    printf("<font color='green'># no encoder: no saturation knee, keep the config's i_knee and tr_sat</font>\n");
  }
  if(PIN(n_volt) <= 0.0) {
    printf("<font color='green'># no idacim0.n_volt: id_n is the current the sweep was centred on</font>\n");
  } else if(id_n > PINA(sw_i, n - 1) || id_n < PINA(sw_i, 0)) {
    printf("<font color='red'># id_n is outside the sweep: extended, rerun with id_n there</font>\n");
  }

  PIN(rot_j)   = 0.0;
  PIN(rot_j_s) = 0.0;
  PIN(rot_tf) = 0.0;
  if(PIN(rot_enc) <= 0.0) {
    printf("<font color='green'># no encoder (vel_fb did not follow the field): no inertia</font>\n");
  } else if(ctx->stall) {
    printf("<font color='red'># the rotor fell behind the field on a ramp: lower idacim0.rot_acc for inertia</font>\n");
  } else {
    // slip torque at the ramps' current, with the slip from position
    float tr = PIN(rot_tr_ramp) > 0.0 ? PIN(rot_tr_ramp) : PIN(tr);
    float t[2];
    for(int u = 0; u < 2; u++) {
      float ws = ctx->j_ws[u];
      t[u]     = 1.5 * pp * lmj * idr * idr * ws * tr / (1.0 + ws * ws * tr * tr);
    }
    float w_hi = wn * MIN(2.0 * PIN(rot_vel), 0.9);
    float da = ctx->j_alpha[0] - ctx->j_alpha[1];
    if(da > 0.0) {
      // J from the stator voltages (air gap power, no tr). With the ramps'
      // tr the slip gives the same J by construction; it gives friction,
      // and vel_fb's slip against position's gives its lag.
      float tv0 = 1.5 * pp * ctx->j_tv[0], tv1 = 1.5 * pp * ctx->j_tv[1];
      float j_s = (t[0] - t[1]) / da;
      float j_v = (tv0 - tv1) / da;
      PIN(rot_j)   = j_v;
      PIN(rot_tf)  = 0.5 * (t[0] + t[1]);
      PIN(rot_j_s) = j_s;
      printf("conf0.j = %f <font color='green'># append to config, kg m^2</font>\n", j_v);
      printf("<font color='green'># friction %f Nm at %f-%f rad/s mech; ramps %f / %f rad/s^2 mech, slip %f / %f rad/s el (vel_fb %f / %f)</font>\n", PIN(rot_tf), (ctx->w_t + 0.25 * (w_hi - ctx->w_t)) / pp, (ctx->w_t + 0.75 * (w_hi - ctx->w_t)) / pp, ctx->j_alpha[0], ctx->j_alpha[1], ctx->j_ws[0], ctx->j_ws[1], ctx->j_slip[0], ctx->j_slip[1]);
      PIN(rot_lag) = ((ctx->j_slip[0] - ctx->j_slip[1]) - (ctx->j_ws[0] - ctx->j_ws[1])) / (pp * da);
      printf("<font color='green'># vel_fb lags the rotor position by ~%f ms (slip J %f)</font>\n", PIN(rot_lag) * 1000.0, j_s);
      if(j_v <= 0.0) {
        printf("<font color='red'># no inertia: the air gap torque did not rise with acceleration; check the ud_fb/uq_fb links, rerun</font>\n");
      }
    }
  }
  PIN(rot_tr) = PIN(rot_tr_k) = PIN(rot_tr_imr) = PIN(rot_tr_lmr) = PIN(rot_tr_b) = 0.0;
  if(PIN(rot_tr_vel) > 0.0) {
    if(ctx->fo_n[0] < 100 || ctx->fo_n[1] < 100) {
      printf("<font color='red'># rotating tr check: too few samples (%f up, %f down), the speed window %f..%f rad/s mech was not ramped through</font>\n", (float)ctx->fo_n[0], (float)ctx->fo_n[1], 0.7 * PIN(rot_tr_vel), PIN(rot_tr_vel));
    } else {
      float d[2], p[2];
      for(int u = 0; u < 2; u++) {
        d[u] = ctx->fo_s[u][0] / (float)ctx->fo_n[u];
        p[u] = ctx->fo_s[u][1] / (float)ctx->fo_n[u];
      }
      // tan(p - d + b) / tan(p) the same on both ramps: b by bisection
      // both tangents stay off their poles over the bracket
      float lo = MAX(-0.6, -1.52 - (p[1] - d[1])), hi = MIN(0.6, 1.52 - (p[0] - d[0])), b = 0.0, k = 0.0;
      int ok = ABS(tanf(p[0])) > 0.05 && ABS(tanf(p[1])) > 0.05 && p[0] * p[1] < 0.0;
      for(int it = 0; ok && it < 40; it++) {
        b = 0.5 * (lo + hi);
        float f  = tanf(p[0] - d[0] + b) / tanf(p[0]) - tanf(p[1] - d[1] + b) / tanf(p[1]);
        float fl = tanf(p[0] - d[0] + lo) / tanf(p[0]) - tanf(p[1] - d[1] + lo) / tanf(p[1]);
        if((f > 0.0) == (fl > 0.0)) {
          lo = b;
        } else {
          hi = b;
        }
      }
      k = ok ? tanf(p[0] - d[0] + b) / tanf(p[0]) : 0.0;
      float imr = 0.5 * (ctx->fo_s[0][2] / (float)ctx->fo_n[0] + ctx->fo_s[1][2] / (float)ctx->fo_n[1]);
      float lm  = 0.5 * (ctx->fo_s[0][3] / (float)ctx->fo_n[0] + ctx->fo_s[1][3] / (float)ctx->fo_n[1]);
      printf("<font color='green'># rotating tr check %f..%f rad/s mech: flux angle %f / %f, current angle %f / %f rad (up / down), offset %f rad, i_mr %f A, lmr from the emf %f mH</font>\n",
             0.7 * PIN(rot_tr_vel), PIN(rot_tr_vel), d[0], d[1], p[0], p[1], b, imr, lm * 1000.0);
      {
        // the same without the dead time correction, for comparison
        float d0 = ctx->fo_s[0][4] / (float)ctx->fo_n[0], d1 = ctx->fo_s[1][4] / (float)ctx->fo_n[1];
        float l2 = MAX(-0.6, -1.52 - (p[1] - d1)), h2 = MIN(0.6, 1.52 - (p[0] - d0)), b2 = 0.0;
        for(int it = 0; it < 40; it++) {
          b2 = 0.5 * (l2 + h2);
          float f  = tanf(p[0] - d0 + b2) / tanf(p[0]) - tanf(p[1] - d1 + b2) / tanf(p[1]);
          float fl = tanf(p[0] - d0 + l2) / tanf(p[0]) - tanf(p[1] - d1 + l2) / tanf(p[1]);
          if((f > 0.0) == (fl > 0.0)) {
            l2 = b2;
          } else {
            h2 = b2;
          }
        }
        float lm0 = 0.5 * (ctx->fo_s[0][5] / (float)ctx->fo_n[0] + ctx->fo_s[1][5] / (float)ctx->fo_n[1]);
        printf("<font color='green'># without the dead time (drop %f V): flux angle %f / %f, k %f, offset %f, lmr from the emf %f mH</font>\n", PIN(drop), d0, d1, tanf(p[0] - d0 + b2) / tanf(p[0]), b2, lm0 * 1000.0);
        // torque balance: (T_up - T_down) = J (a_up - a_down), T = 1.5 pp psi
        // |i| sin(current angle - flux angle); J from the inertia ramps says
        // which flux angle is right
        float acc_[2], im_[2], ps_[2];
        for(int u = 0; u < 2; u++) {
          float nn = (float)ctx->fo_n[u];
          acc_[u]  = ctx->fo_a[u][0] / nn;
          im_[u]   = ctx->fo_a[u][1] / nn;
          ps_[u]   = ctx->fo_a[u][2] / nn;
        }
        float da_ = acc_[0] - acc_[1];
        if(ABS(da_) > 1.0) {
          float tf_ = 1.5 * pp;
          float j_c = tf_ * (ps_[0] * im_[0] * sinf(p[0] - d[0] + b) - ps_[1] * im_[1] * sinf(p[1] - d[1] + b)) / da_;
          float j_0 = tf_ * (ps_[0] * im_[0] * sinf(p[0] - d0 + b2) - ps_[1] * im_[1] * sinf(p[1] - d1 + b2)) / da_;
          float j_f = tf_ * (ps_[0] * im_[0] * sinf(p[0]) - ps_[1] * im_[1] * sinf(p[1])) / da_;
          printf("<font color='green'># torque balance: acc %f / %f rad/s^2 mech, J with the corrected angle %f, uncorrected %f, frame taken as right %f kg m^2 (inertia ramps %f)</font>\n", acc_[0], acc_[1], j_c, j_0, j_f, PIN(rot_j));
        }
      }
      if(ok && k > 0.3 && k < 3.0) {
        PIN(rot_tr)     = PIN(tr) * k;
        PIN(rot_tr_k)   = k;
        PIN(rot_tr_imr) = imr;
        PIN(rot_tr_lmr) = lm;
        PIN(rot_tr_b)   = b;
        printf("<font color='green'># rotating tr %f s at i_mr %f A = %f x the standstill tr</font>\n", PIN(rot_tr), imr, k);
      } else {
        printf("<font color='red'># rotating tr check: no solution (k %f)</font>\n", k);
      }
    }
  }
  printf("rotating test done\n");
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx       = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  rot_nrt(ctx, pins);

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      // hv0.r and l go back to safe defaults: nothing after this may use
      // them as measured
      PIN(r)       = 0.1;
      PIN(l)       = 0.001;
      PIN(drop)    = 0.0;
      PIN(r_ok)    = 0.0;
      PIN(l_ok)    = 0.0;
      PIN(tr_ok)   = 0.0;
      PIN(out_rev) = 0.0;
      PIN(cur_bw)  = 1.0;
      break;

    case 10:  // r, l
      // test_cur is the highest current of the standstill test, as in
      // idpmsm. The knee ladder runs under it, from 0.4 test_cur, in 0.7
      // steps; no level goes under i_min, where the dead time distorts the
      // voltage (the rotor test's lower level is half of test_cur, a rung's
      // is lad_ratio of its top)
      if(PIN(test_cur) < 2.0 * PIN(i_min)) {
        printf("<font color='red'># idacim0.test_cur %f A puts the rotor test's lower step under idacim0.i_min %f A</font>\n", PIN(test_cur), PIN(i_min));
      }
      // a ladder the knee wrote is redone from this run's test_cur, or
      // dropped with knee 0; one set by hand (lad_top changed) stays
      if(PIN(lad_auto) > 0.0 && PIN(lad_top) == PIN(lad_auto)) {
        PIN(lad_top) = 0.0;
        PIN(lad_bot) = 0.0;
      }
      PIN(lad_auto) = 0.0;
      if(PIN(knee) > 0.0 && PIN(lad_top) <= 0.0) {
        PIN(lad_ratio) = 0.7;
        PIN(lad_bot)   = MAX(0.4 * PIN(test_cur), PIN(i_min) / 0.7);
        PIN(lad_top)   = PIN(test_cur);
        PIN(lad_n)     = 4.0;
        if(PIN(lad_top) < 1.5 * PIN(lad_bot)) {  // too little range above i_min for a knee
          printf("<font color='red'># knee: idacim0.test_cur %f A leaves no range above idacim0.i_min %f A, no ladder</font>\n", PIN(test_cur), PIN(i_min));
          PIN(lad_top) = 0.0;
          PIN(lad_bot) = 0.0;
        }
        PIN(lad_auto) = PIN(lad_top);
      }
      PIN(state)    = 1.1;
      PIN(timer)    = 0.0;
      PIN(psi)      = 0.0;
      ctx->psi_lp   = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(tmp0)     = 0.0;
      PIN(tmp1)     = 0.0;
      PIN(tmp2)     = 0.0;
      PIN(tmp3)     = 0.0;

      printf("Measure r, leakage l, rotor time constant\n");
      printf("<font color='green'>the rotor may stay free: a dc field makes no torque on a cage</font>\n");
      printf("idacim0.state = 1.2 <font color='green'>to start</font>\n");
      break;

    case 15:
      if(ctx->pend || ctx->rdone) {  // the last rung is still being reduced
        break;
      }
      if(PIN(r_ok) > 0.0) {
        printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
        if(PIN(l_ok) > 0.0) {
          printf("conf0.l = %f <font color='green'># leakage, sigma*Ls</font>\n", PIN(l));
          printf("<font color='green'># from |Z| %f / %f ohm at %f / %f Hz, which\n", PIN(l_za), PIN(l_zb), PIN(l_fa), PIN(l_fb));
          printf("# also imply %f ohm of stator plus cage resistance there.\n", PIN(l_res));
          printf("# right for acim_foc (hv0.psi carries the rotor flux); acim_ttc\n");
          printf("# has no flux term, so there it makes iq fall short at speed.</font>\n");
        } else {
          printf("<font color='red'>l not measured</font>: injected %f / %f A of %f A asked\n", PIN(l_ia), PIN(l_ib), PIN(l_ripple) * PIN(test_cur));
          printf("use an LCR meter: line to line near 150 Hz, halved\n");
        }
        if(PIN(tr_ok) > 0.0) {
          printf("acim_flux0.tr = %f <font color='green'># append to config</font>\n", PIN(tr));
          printf("acim_flux0.lmr = %f <font color='green'># append to config</font>\n", PIN(lmr));
          printf("<font color='green'># rotor time constant tr = %f ms from %f edges\n", PIN(tr) * 1000.0, PIN(rot_n));
          printf("# slip_n = 1/tr = %f rad/s. for acim_ttc, which derives it from vel_n:\n", PIN(slip_n));
          printf("# acim_ttc0.vel_n = (2 pi acim_ttc0.freq_n - %f) / conf0.polecount\n", PIN(slip_n));
          printf("# from %f edges: up %f ms, down %f ms (median each), min %f max %f ms,\n", PIN(rot_n), PIN(tr_rise) * 1000.0, PIN(tr_fall) * 1000.0, PIN(tr_min) * 1000.0, PIN(tr_max) * 1000.0);
          printf("# spread %f of tr within a direction. up and down apart means\n", PIN(tr_spread));
          printf("# the step reaches into saturation; tr is their mean.</font>\n");
          if(PIN(tr_spread) > 0.25) {
            printf("<font color='red'># edges disagree: check ud_fb noise, or rerun with more rot_cycles.</font>\n");
          }
          printf("<font color='green'>");
          printf("# lmr = Lm^2/Lr = %f mH, ls = %f mH, at %f..%f A on d.\n", PIN(lmr) * 1000.0, PIN(ls) * 1000.0, PIN(test_cur) * 0.5, PIN(test_cur));
          printf("# tr is the rotor's at this temperature and flux: a hot cage\n");
          printf("# reads shorter, and rated flux saturates it a little shorter.</font>\n");
          if(PIN(n_volt) > 0.0 && PIN(n_freq) > 0.0 && PIN(n_cur) > 0.0 && PIN(n_pp) >= 1.0) {
            vf_set(pins);
          } else {
            printf("<font color='green'># for the V/f set give the plate: idacim0.n_volt, n_freq, n_cur, n_pp</font>\n");
          }
          if(PIN(rot_dip) > 0.1) {
            printf("<font color='red'># the current was %f of the step off at rot_t0:\n", PIN(rot_dip));
            printf("# the loop is slow. the fit uses the current that flowed, but\n");
            printf("# raise idacim0.rot_bw and rerun to confirm.</font>\n");
          }
        } else if(PIN(rot_n) >= 2.0) {
          printf("<font color='red'>tr not measured</font>: the fit gave %f ms, outside what this test can see\n", PIN(tr) * 1000.0);
        } else {
          printf("<font color='red'>tr not measured</font>: fewer than two edges fitted\n");
        }
        if(PIN(lad_top) > 0.0 && PIN(lad_n) >= 1.0) {
          // each rung's lmr is the slope of psi_r over its range; taken at the
          // range's middle and integrated from zero, it gives psi_r and the
          // secant psi_r / id that acim_flux's lmr is. Below the first rung
          // the slope is extended along the line through the first two.
          printf("<font color='green'># offset ladder: rung, id range [A], lmr slope [mH], tr [ms], spread, psi_r [Vs], secant lmr [mH]\n");
          int nr   = (int)MIN(PIN(lad_n), 4.0);
          float l0 = 0.0;
          for(int k = 0, j = -1; k < nr; k++) {
            if(PINA(lad_lm, k) > 0.0 && PINA(lad_i, k) > 0.0) {
              if(j < 0) {
                j  = k;
                l0 = PINA(lad_lm, k);
              } else {
                float ma = 0.75 * PINA(lad_i, j), mb = 0.75 * PINA(lad_i, k);
                l0       = PINA(lad_lm, j) + (PINA(lad_lm, j) - PINA(lad_lm, k)) * ma / (mb - ma);
                l0       = MAX(l0, PINA(lad_lm, j));
                break;
              }
            }
          }
          float psi = 0.0, mp = 0.0, lp = l0;
          for(int k = 0; k < nr; k++) {
            float lm = PINA(lad_lm, k);
            float m  = 0.75 * PINA(lad_i, k);
            float lo = lad_lo(pins) * PINA(lad_i, k);
            if(lm <= 0.0 || m <= 0.0) {
              printf("# %f %f..%f no fit (tr %f ms, spread %f)\n", (float)(k + 1), lo, PINA(lad_i, k), PINA(lad_tr, k) * 1000.0, PINA(lad_sp, k));
              continue;
            }
            if(PIN(lad_bot) > 0.0 || lad_lo(pins) != 0.5) {
              // rungs that do not tile 0..top: no psi_r from integrating them
              printf("# %f %f..%f %f %f %f\n", (float)(k + 1), lo, PINA(lad_i, k), lm * 1000.0, PINA(lad_tr, k) * 1000.0, PINA(lad_sp, k));
              continue;
            }
            psi += 0.5 * (lp + lm) * (m - mp);
            mp = m;
            lp = lm;
            printf("# %f %f..%f %f %f %f %f %f\n", (float)(k + 1), PINA(lad_i, k) * 0.5, PINA(lad_i, k), lm * 1000.0, PINA(lad_tr, k) * 1000.0, PINA(lad_sp, k), psi, psi / m * 1000.0);
          }
          printf("# psi_r and secant are at each range's middle, 0.75 of its top.</font>\n");
          knee_fit(pins);
        }
        printf("<font color='green'># dead time %f V per phase at the %f A dwell, %f V link\n", PIN(drop), PIN(test_cur), PIN(dc_volt));
        if(PIN(r_known) > 0.0) {
          printf("# read against the r you gave.</font>\n");
        } else {
          printf("# r is the chord %f less its dead time bias %f (idacim0.drop_slope).\n", PIN(r_2p), PIN(r_bias));
          printf("# below about 5 A the bias is understated; a four wire r in\n");
          printf("# idacim0.r_known is better.</font>\n");
        }
      } else {
        if(PIN(r_known) > 0.0) {
          printf("<font color='red'>r read failed</font>: the top dwell did not reach half of %f A\n", PIN(test_cur));
        } else {
          printf("<font color='red'>r fit failed</font>: the two dwells differ by %f A\n", PIN(fit_di));
        }
        printf("nothing below is measured, do not append it\n");
        printf("check that idacim0.test_cur (%f) is under conf0.max_ac_cur\n", PIN(test_cur));
      }
      // the standstill test ends here; spin goes on to the rotating test
      // (4.0), the pole pair test (2.0) runs only when set by hand. walk on
      // only if r worked: hv0.r is this pin
      if(PIN(r_ok) <= 0.0) {
        PIN(state) = 0.0;
      } else if(PIN(spin) > 0.0) {
        PIN(state) = 4.0;
      } else {
        printf("standstill test done\n");
        PIN(state) = 3.0;
      }
      break;

    case 20:  // pp
      PIN(state)    = 2.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(pp)       = 0.0;  // the filter starts from this run's first reading
      ctx->pp_n     = 0;
      ctx->pp_turn  = 0;

      printf("Measure polepairs\n");
      printf("<font color='green'>unblock the rotor, it will move</font>\n");
      printf("idacim0.state = 2.2 <font color='green'>to start</font>\n");
      break;

    case 40:  // rotating test
      if(PIN(tr_ok) <= 0.0 || PIN(n_freq) <= 0.0 || PIN(n_pp) < 1.0) {
        printf("<font color='red'>the rotating test needs this session's standstill test (tr, lmr) and idacim0.n_freq, n_pp</font>\n");
        PIN(state) = 3.0;
        break;
      }
      memset(ctx, 0, sizeof(struct idacim_ctx_t));
      PIN(sw_n)    = 0.0;  // a stall before the sweep reports no points, not the last run's
      PIN(rot_tr_ramp) = 0.0;
      PIN(rot_lag)   = 0.0;
      PIN(rot_delay) = 0.0;
      PIN(psi)       = 0.0;  // from this run's emf, not the last test's
      PIN(state) = 4.1;
      {
        float wn   = 2.0 * M_PI * PIN(n_freq);
        float w_hi = wn * MIN(2.0 * PIN(rot_vel), 0.9);
        printf("Rotating test: open loop field at %f A d, ramps up to %f rad/s mech, flux sweep up to %f\n", PIN(id_n) > 0.0 ? PIN(id_n) : PIN(test_cur), w_hi / PIN(n_pp), MAX(wn * PIN(rot_vmax), w_hi) / PIN(n_pp));
        if(PIN(rot_tr_vel) > 0.0) {
          printf("tr check up to %f rad/s mech\n", PIN(rot_tr_vel));
        }
      }
      printf("<font color='green'>the rotor turns: unblock it, no load on the shaft</font>\n");
      printf("idacim0.state = 4.2 <font color='green'>to start</font>\n");
      break;

    case 47:
      rot_report(ctx, pins);
      PIN(state) = 5.0;
      break;

    case 24:
      // a slipping rotor, or no vel_fb, reads a ratio off any integer
      if(PIN(pp) >= 1.0 && ABS(ctx->pp_raw - PIN(pp)) < 0.15 && ctx->pp_turn > 0.8 * ctx->pp_n) {
        printf("conf0.polecount = %f <font color='green'># append to config</font>\n", PIN(pp));
        if(PIN(out_rev) > 0.0) {
          printf("conf0.out_rev = 1 <font color='green'># append to config</font>\n");
        }
      } else {
        printf("<font color='red'>pole pair read failed</font>: ratio %f, rotor turned for %f of the time (check vel_fb, or a load on the shaft)\n", ctx->pp_raw, ctx->pp_n > 0 ? (float)ctx->pp_turn / (float)ctx->pp_n : 0.0);
      }
      printf("done\n");
      PIN(state) = 3.0;
      break;
  }
}

// tr from the ramps. With the current on d the rotor flux lags it by
// atan(ws tr) at slip ws, so psi_q = -ws tr psi_d and the torque is 1.5 pp
// psi_d id ws tr. The air gap torque's difference between an up and a down
// ramp holds no friction, iron loss, r or dead time error (the same speeds,
// flux and current on both): dT = psi_d id tr dws, in j_tv's units. The
// slip, though, is the field's commanded speed against the encoder's: the
// field reaches the motor and the position comes back some time D later, so
// each ramp reads its slip off by its acceleration times D, and the pair by
// dws_m = dws + pp (alpha_up - alpha_down) D (simulated: about 1 ms, 23 %
// of dws at id_n). That part does not scale with the flux and the true one
// does, so a second pair at a lower current gives both:
//   dws_m,k = dT_k / (psi_d,k id_k tr_k) + pp dalpha_k D,  tr_k = tr lmr_k / lmr_0
// The standstill steps read the spindle's tr 0.078-0.093 s, where its
// orientation needs about 0.12 at every flux (13 Oct).
static void rot_tr_fit(struct idacim_ctx_t *ctx, struct idacim_pin_ctx_t *pins) {
  float pp = PIN(n_pp);
  float a[2], b[2], m[2], lm[2];
  for(int k = 0; k < 2; k++) {
    float ir = 0.5 * (ctx->j_ir[2 * k] + ctx->j_ir[2 * k + 1]);
    float pd = 0.5 * (ctx->j_pd[2 * k] + ctx->j_pd[2 * k + 1]);
    lm[k]    = ir > 0.0 ? pd / ir : 0.0;
    float kp = 0.5 * (ctx->j_pd[2 * k] * ctx->j_ir[2 * k] + ctx->j_pd[2 * k + 1] * ctx->j_ir[2 * k + 1]);
    a[k]     = kp > 0.0 && lm[0] > 0.0 ? (ctx->j_tv[2 * k] - ctx->j_tv[2 * k + 1]) / kp * lm[0] / lm[k] : 0.0;
    b[k]     = pp * (ctx->j_alpha[2 * k] - ctx->j_alpha[2 * k + 1]);
    m[k]     = ctx->j_ws[2 * k] - ctx->j_ws[2 * k + 1];
  }
  float det = a[0] * b[1] - a[1] * b[0];
  float g   = 0.0, d = 0.0;
  // the pairs must differ in flux enough to tell the two apart
  if(ABS(det) > 0.2 * ABS(a[0] * b[1])) {
    g = (m[0] * b[1] - m[1] * b[0]) / det;
    d = (a[0] * m[1] - a[1] * m[0]) / det;
  }
  PIN(rot_delay) = d;
  if(g > 0.0 && 1.0 / g > 0.3 * PIN(tr) && 1.0 / g < 3.0 * PIN(tr)) {
    float tr     = 1.0 / g;
    float x      = 0.5 * (ctx->j_ws[0] + ctx->j_ws[1]) * tr;
    ctx->tr_rot  = tr;
    ctx->lm_rot  = lm[0] * (1.0 + x * x);
    PIN(rot_tr_ramp) = tr;
  }
}

// The flux sweep's start, after the ramps (or straight after the run up
// without an encoder). The sweep runs down from the top: at low flux the
// drag needs more slip (pull out at slip x tr = 1, where 0.8 Nm of drag on
// the spindle sits near 4.6 A), so the sweep stops at the first level that
// slips too far, while the rotor is still at speed. Without an encoder the
// slip is unseen: no level under 0.45 of id_n.
static void sw_start(struct idacim_ctx_t *ctx, struct idacim_pin_ctx_t *pins, float idr, float imax) {
  PIN(state)  = 4.3;
  PIN(sw_n)   = 0.0;
  ctx->sw_k   = 0;
  ctx->sw_k0  = PIN(rot_enc) > 0.0 ? 0 : 2;
  while(ctx->sw_k0 < SW_N - 2 && sw_frac[ctx->sw_k0 + 1] * idr <= PIN(i_min)) {
    ctx->sw_k0++;
  }
  ctx->sw_top = SW_N - 1;
  while(ctx->sw_top > ctx->sw_k0 && sw_frac[ctx->sw_top - 1] * idr >= imax) {
    ctx->sw_top--;
  }
  ctx->sw_slip = 0.0;
  ctx->sw_pass = 0;
  ctx->sw_np   = 0;
  ctx->sw_hold = 0.0;
  ctx->sw_t    = 0.0;
  ctx->sw_cnt  = 0;
  ctx->s_uq = ctx->s_iq = ctx->s_id = ctx->s_slip = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx        = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

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

    case 12:  // r -- rotor blocked, current control: dwell at two currents and
              // fit the voltage the current loop needs to sustain them.
              // com_pos is held at 0 throughout -- an induction motor has no rotor
              // flux to align to, so the injection axis is arbitrary.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 1.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;

      // ud = r * id + 4/3 * drop: at angle 0 the phase currents are id,
      // -id/2, -id/2, and the three per phase dead time drops land 4/3 on d.
      // Two dwells split r (the slope) from the drop. Full current first, so
      // the first dwell aligns what the second holds.
      if(PIN(timer) < 2.0) {
        PIN(d_cmd) = PIN(test_cur);
        PIN(tmp2)  = PIN(tmp2) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp3)  = PIN(tmp3) * 0.999 + PIN(ud_fb) * 0.001;
      } else {
        PIN(d_cmd) = PIN(test_cur) * 0.5;
        PIN(tmp0)  = PIN(tmp0) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp1)  = PIN(tmp1) * 0.999 + PIN(ud_fb) * 0.001;
      }

      // hv0.r is this pin, the loop's plant model: the ud/id ratio bootstraps
      // it so current flows at cur_bw 1; the fit replaces it
      PIN(r) = PIN(r) * 0.99 + PIN(ud_fb) / MAX(PIN(id_fb), 0.01) * 0.01;

      PIN(timer) += period;
      if(PIN(timer) >= 4.0) {
        // the filters are linear and alike, so the fit holds on the filtered
        // pair even where neither dwell reaches its command
        float di    = PIN(tmp2) - PIN(tmp0);
        PIN(fit_di) = di;
        PIN(r_2p) = di > 0.01 ? MAX((PIN(tmp3) - PIN(tmp1)) / di, 0.001) : 0.0;

        float r_ok = 0.0;

        if(PIN(r_known) > 0.0) {
          // the drop is read at the top dwell, which has to be near its command
          if(PIN(tmp2) > PIN(test_cur) * 0.5) {
            PIN(r)      = PIN(r_known);
            PIN(r_bias) = 0.0;
            PIN(drop) = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
            r_ok      = 1.0;
          }
        } else if(di > 0.01) {
          // The dead time drop rises as K ln(i), which biases the chord by
          // 8/3 K ln2 / test_cur. drop_slope = 8/3 ln2 K / Vdc, 0.0039 ohm A
          // per volt on the bridge it was fitted on. Skipped without dc_volt.
          PIN(r_bias) = PIN(dc_volt) > 1.0 ? PIN(drop_slope) * PIN(dc_volt) / MAX(PIN(test_cur), 0.1) : 0.0;
          PIN(r)      = MAX(PIN(r_2p) - PIN(r_bias), 0.001);
          PIN(drop)   = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
          r_ok        = 1.0;
        }

        PIN(r_ok) = r_ok;

        // the l test needs the voltage that holds test_cur, dead time
        // included, and never one built from a failed measurement
        if(r_ok > 0.0) {
          PIN(avg_test_volt) = PIN(r) * PIN(test_cur) + 4.0 / 3.0 * PIN(drop);
          PIN(avg_test_volt) = LIMIT(PIN(avg_test_volt), PIN(pwm_volt) / 2.0);
        } else {
          PIN(avg_test_volt) = 0.0;
        }

        memset(ctx, 0, sizeof(struct idacim_ctx_t));
        ctx->l_amp  = 1.0;
        PIN(timer)  = 0.0;
        PIN(state)  = r_ok > 0.0 ? 1.3 : 1.5;
        PIN(d_cmd)  = 0.0;
        PIN(en_out) = 0.0;
        PIN(tmp0)   = 0.0;
        PIN(tmp1)   = 0.0;
        PIN(tmp2)   = 0.0;
        PIN(tmp3)   = 0.0;
      }
      break;

    case 13: {  // leakage l, by injection at two frequencies
      // A time constant test (l = tau * r) does not work here: on an induction
      // motor a voltage step's current has two poles, about 2 ms and 100-250 ms,
      // and the area method returns the slow one's Ls / r: the full stator
      // inductance, some twenty times the leakage the current loop wants. So
      // inject instead, as idpmsm does, but on d only (a cage rotor is round)
      // and at two frequencies:
      //
      //   |Z|^2 = R^2 + w^2 l^2   at both  =>  l^2 = (|Zb|^2 - |Za|^2) / (wb^2 - wa^2)
      //
      // so no resistance goes in. That matters here: the cage adds a
      // frequency dependent resistance on top of the stator's (0.2 ohm at
      // 100 Hz on the spindle, 0.8 at 1 kHz), and subtracting the dc r from
      // one |Z| gets l 10-13% wrong near the loop's crossover. The pair
      // assumes R and l are equal at both frequencies; the cage's rise between
      // 120 and 240 Hz costs a few percent of l, not more.
      //
      // The dc bias holds test_cur on d, so every phase current stays on one
      // side of zero and the dead time is an offset, not a nonlinearity. In
      // volt mode that bias settles on the slow rotor pole, hence the longer
      // first settle. A single pulsating axis makes no torque at standstill.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;  // volt cmd
      PIN(cur_bw)   = 1.0;
      PIN(com_pos)  = 0.0;
      PIN(q_cmd)    = 0.0;

      float f_c = (PIN(loop_bw) > 0.0 ? PIN(loop_bw) : 1000.0) / (2.0 * M_PI);
      float fa  = PIN(l_freq_a) > 0.0 ? PIN(l_freq_a) : 0.75 * f_c;
      float fb  = PIN(l_freq_b) > 0.0 ? PIN(l_freq_b) : 1.5 * f_c;
      // every window (L_BLOCK and the settles are multiples of it) has to
      // hold whole cycles, or the dc bias leaks into the demodulation
      // (simulated: 119.4/238.7 Hz read l 10% high): multiples of 1/L_BLOCK
      fa        = MAX(roundf(fa * L_BLOCK), 1.0) / L_BLOCK;
      fb        = MAX(roundf(fb * L_BLOCK), 1.0) / L_BLOCK;
      fa        = CLAMP(fa * (float)(1 << ctx->l_up), 20.0, 0.2 / period);
      fb        = CLAMP(fb * (float)(1 << ctx->l_up), 20.0, 0.2 / period);
      float f  = ctx->l_fi == 0 ? fa : fb;
      float w  = 2.0 * M_PI * f;
      ctx->l_th += w * period;
      if(ctx->l_th > 2.0 * M_PI) {
        ctx->l_th -= 2.0 * M_PI;
      }
      float sn, cs;
      sincos_fast(ctx->l_th, &sn, &cs);
      PIN(d_cmd) = PIN(avg_test_volt) + ctx->l_amp * sn;

      float v = PIN(ud_fb);
      float i = PIN(id_fb);
      ctx->l_t += period;
      float target = PIN(l_ripple) * PIN(test_cur);

      if(ctx->l_stage == 0) {
        if(ctx->l_t >= (ctx->l_fi == 0 ? L_BIAS_SETTLE : L_SETTLE)) {
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
        // take the dc bias out with the mean times the reference sums (as
        // idpmsm), and end the window on a sample count, not float time
        float vm = ctx->v_dc / n, im = ctx->i_dc / n;
        float vr = ctx->v_re - vm * ctx->s_dc, vi = ctx->v_im - vm * ctx->c_dc;
        float ir = ctx->i_re - im * ctx->s_dc, ii = ctx->i_im - im * ctx->c_dc;
        if(ctx->l_stage == 1 && ctx->l_t >= L_BLOCK) {
          // size the amplitude to the ripple asked for: enough signal, and
          // never so much that a phase current crosses zero
          float i1 = 2.0 / n * sqrtf(ir * ir + ii * ii);
          float k  = i1 > 0.001 ? target / i1 : 4.0;
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
          // the f3 holds each 5 kHz command for a whole tick, which scales the
          // applied fundamental by sin(pi f T) / (pi f T); take it back out
          float v1  = 2.0 / n * sqrtf(vr * vr + vi * vi);
          float i1  = 2.0 / n * sqrtf(ir * ir + ii * ii);
          float x   = M_PI * f * period;
          float zoh = sinf(x) / x;
          float z   = i1 > 0.001 ? v1 * zoh / i1 : 0.0;
          if(ctx->l_fi == 0) {  // on to the upper frequency, from this amplitude
            PIN(l_za)    = z;
            PIN(l_ia)    = i1;
            ctx->l_fi    = 1;
            ctx->l_stage = 0;
            ctx->l_block = 0;
            ctx->l_t     = 0.0;
            ctx->l_n     = 0;
          } else {
            PIN(l_zb)  = z;
            PIN(l_ib)  = i1;
            float wa   = 2.0 * M_PI * fa;
            float wb   = 2.0 * M_PI * fb;
            float za   = PIN(l_za);
            float den  = wb * wb - wa * wa;
            float l2   = den > 0.0 ? (z * z - za * za) / den : 0.0;
            float r2   = den > 0.0 ? (wb * wb * za * za - wa * wa * z * z) / den : 0.0;
            int ok     = l2 > 0.0 && PIN(l_ia) > target * 0.5 && PIN(l_ia) < target * 2.0 && i1 > target * 0.5 && i1 < target * 2.0;
            PIN(l_ok)  = ok ? 1.0 : 0.0;
            PIN(l_res) = r2 > 0.0 ? sqrtf(r2) : 0.0;
            PIN(l_fa)  = fa;
            PIN(l_fb)  = fb;
            // a small leakage, as on high speed spindles, leaves |Z| mostly
            // resistance at these frequencies, and the cage's rise between
            // them then weighs on l^2. Go up an octave until the reactance at
            // the upper one is twice the resistance, as far as rt can draw it.
            if(ok && sqrtf(l2) * wb < 2.0 * PIN(l_res) && fb * 2.0 <= 0.2 / period && ctx->l_up < 3) {
              ctx->l_up++;
              ctx->l_fi    = 0;
              ctx->l_stage = 0;
              ctx->l_block = 0;
              ctx->l_t     = 0.0;
              ctx->l_n     = 0;
              break;
            }
            // hv0.l = idacim0.l, so the rotor test's current loop runs on this.
            // A failed read leaves the 1 mH init, a sane leakage for a few kW.
            PIN(l) = ok ? sqrtf(l2) : 0.001;

            memset(ctx, 0, sizeof(struct idacim_ctx_t));
            PIN(timer)  = 0.0;
            PIN(state)  = 1.4;
            PIN(d_cmd)  = 0.0;
            PIN(en_out) = 0.0;
          }
        }
      }
      break;
    }

    case 14: {  // rotor time constant and magnetizing inductance, by d current steps
      // Hold com_pos and step the d current between test_cur and test_cur/2
      // with a fast current loop. The rotor flux follows the stator current
      // with tr = Lr/Rr, and while it moves the stator sees its emf
      //
      //   e = Lmr * d(i_mr)/dt,   tr * d(i_mr)/dt = id - i_mr,   Lmr = Lm^2/Lr
      //
      // on top of r*id, the dead time and l*d(id)/dt. The first two settle with
      // the current; the chord from the r test is their dc slope over exactly
      // these two levels, so it takes them out of the part that has not.
      //
      // The current is not a clean step: the tail pushes against the loop and
      // id sags for as long as the integrator takes to catch it, and that sag
      // moves the flux too. Fitting an exponential to ud alone came out 3% short
      // on tr with a 1500 rad/s loop and 10% at 300 (simulated). So fit the
      // model to the current that actually flowed. With everything integrated
      // from the edge, where i_mr still sits at the old level i0, and c the
      // offset ud - R id where this edge settles:
      //
      //   lam(t) = integral(ud - R id - c) - l (id - i0) = Lmr (i_mr - i0)
      //   integral(lam) = Lmr * integral(id - i0) - tr * lam
      //
      // which is linear in Lmr and tr, fitted by least squares from rot_t0 to
      // the end of the edge.
      //
      // c is this edge's own, fitted with them (rot_nrt). Taken from an
      // earlier visit to the level, one bad sample or a cage warming between
      // visits put an error in c that grows as t in lam and
      // t^2 in its integral, and edges spread 100-300% on the spindle (2 Oct);
      // read off the edge's last quarter, a few mV of it still spread them
      // 30% (5 Oct). The integration takes out c0, the previous edge's offset
      // (the chord makes it the same at both levels), and the sums are kept as
      // a polynomial in the correction d = c - c0. The median of ROT_BLK block
      // means over the last quarter starts the fit and settles the level for
      // the next edge. A median of three
      // past rot_t0 keeps single bad samples out of the integrals. Each edge
      // is fitted on its own and the median is reported.
      //
      // Ladder rungs (lad_n > 0, lad_top > 0) repeat the same steps between
      // lad_top k/n and half of it after the main test: each rung's lmr is the
      // slope of rotor flux over its range, so together they give psi_r(id).
      // slip_n in acim_ttc is 1/tr in electrical rad/s.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = MAX(PIN(rot_bw), 1.0);
      PIN(com_pos)  = 0.0;
      PIN(q_cmd)    = 0.0;

      int nrung  = (int)CLAMP(PIN(lad_n), 0.0, 4.0);
      float hi   = ctx->rung ? lad_hi(pins, ctx->rung, nrung) : PIN(test_cur);
      float lo   = ctx->rung ? lad_lo(pins) * hi : 0.5 * hi;
      if(ctx->r_t == 0.0) {
        // rot_half is the longest edge. Once an edge has given a tr, the
        // next ones last 16 tr: past that the flux has settled and a longer
        // edge only lets an offset error grow (as e t in lam). On a short tr,
        // as on high speed spindles, 1.5 s edges spread 40-55% and 16 tr
        // ones 7-9% (simulated, tr 20 ms). rot_t0 skips the current loop's
        // settling; there it would skip most of the flux's move too, so it
        // stays under a fifth of tr.
        float tr_last = ctx->r_n > 0 ? ctx->tr_e[ctx->r_n - 1] : 0.0;
        ctx->half_e   = MAX(PIN(rot_half), 0.1);
        ctx->t0_e     = PIN(rot_t0);
        if(tr_last > 0.0) {
          ctx->half_e = CLAMP(16.0 * tr_last, 0.1, ctx->half_e);
          ctx->t0_e   = MIN(ctx->t0_e, 0.2 * tr_last);
        }
      }
      float half = ctx->half_e;
      float t0   = CLAMP(ctx->t0_e, period, half * 0.25);
      float t1   = half * ROT_TAIL;
      int lv     = ctx->r_edge & 1;  // 0 at hi, 1 at half of it
      float rin  = PIN(r_2p) > 0.0 ? PIN(r_2p) : PIN(r);

      PIN(d_cmd) = lv ? lo : hi;

      float ud = PIN(ud_fb);
      float id = PIN(id_fb);

      if(ctx->r_t == 0.0) {  // first tick of this level
        // the first edge of a rung comes from somewhere else: magnetizes only
        ctx->act = ctx->r_edge >= 1 && ctx->have_prev;
        if(ctx->act) {
          ctx->i0      = ctx->prev_i;
          ctx->c0      = ctx->prev_u - rin * ctx->prev_i;
          ctx->have_t0 = 0;
          ctx->p = ctx->q = ctx->jp = 0.0;
          ctx->dec = 0;
          for(int k = 0; k < 14; k++) {
            ctx->s[k] = 0.0;
          }
        }
        for(int k = 0; k < ROT_BLK; k++) {
          ctx->blk_u[k] = ctx->blk_i[k] = 0.0;
          ctx->blk_n[k] = 0;
        }
      }
      ctx->r_t += period;
      float t = ctx->r_t;

      // single bad samples: a median of three once the edge's fast part is over
      float um = ud;
      float im = id;
      if(t > t0) {
        um = med3(ud, ctx->u1, ctx->u2);
        im = med3(id, ctx->i1, ctx->i2);
      }
      ctx->u2 = ctx->u1;
      ctx->u1 = ud;
      ctx->i2 = ctx->i1;
      ctx->i1 = id;

      if(ctx->act) {
        ctx->p += (um - rin * im - ctx->c0) * period;
        ctx->q += (im - ctx->i0) * period;
        ctx->jp += ctx->p * period;
        if(t >= t0) {
          if(!ctx->have_t0) {
            ctx->i_t0    = im;
            ctx->have_t0 = 1;
          }
          if(++ctx->dec >= ROT_DEC) {
            ctx->dec = 0;
            // y = Lmr a + tr b, with b = bk + d t and y = yk - d t^2/2
            float a  = ctx->q;
            float bk = -(ctx->p - PIN(l) * (im - ctx->i0));
            float yk = ctx->jp - PIN(l) * ctx->q;
            float tt = t;
            float sg = -0.5 * t * t;
            ctx->s[0] += a * a;
            ctx->s[1] += a * bk;
            ctx->s[2] += a * tt;
            ctx->s[3] += bk * bk;
            ctx->s[4] += bk * tt;
            ctx->s[5] += tt * tt;
            ctx->s[6] += a * yk;
            ctx->s[7] += a * sg;
            ctx->s[8] += bk * yk;
            ctx->s[9] += bk * sg;
            ctx->s[10] += tt * yk;
            ctx->s[11] += tt * sg;
            ctx->s[12] += sg * sg;
            ctx->s[13] += sg * yk;
          }
        }
      }
      if(t >= t1) {
        int k = (int)((t - t1) / (half - t1) * ROT_BLK);
        k     = CLAMP(k, 0, ROT_BLK - 1);
        ctx->blk_u[k] += um;
        ctx->blk_i[k] += im;
        ctx->blk_n[k]++;
      }

      if(t >= half) {  // end of this level
        float bu[ROT_BLK], bi[ROT_BLK];
        int nb = 0;
        for(int k = 0; k < ROT_BLK; k++) {
          if(ctx->blk_n[k] > 0) {
            bu[nb] = ctx->blk_u[k] / (float)ctx->blk_n[k];
            bi[nb] = ctx->blk_i[k] / (float)ctx->blk_n[k];
            nb++;
          }
        }
        if(nb > 0) {
          float uinf = median(bu, nb);
          float iinf = median(bi, nb);
          float di   = iinf - ctx->i0;
          if(ctx->act && ABS(di) > 0.1 * hi && !ctx->pend) {
            for(int k = 0; k < 14; k++) {
              ctx->ps[k] = ctx->s[k];
            }
            ctx->pd   = uinf - rin * iinf - ctx->c0;
            ctx->pdip = ABS((ctx->i_t0 - iinf) / di);
            ctx->pup  = lv == 0;
            ctx->pdi  = di;
            ctx->ptail = bu[nb - 1] - bu[0];  // median() sorted bu
            ctx->pend = 1;
          }
          ctx->prev_u    = uinf;
          ctx->prev_i    = iinf;
          ctx->have_prev = 1;
        }
        ctx->r_edge++;
        ctx->r_t = 0.0;
      }

      if(ctx->r_edge >= 1 + 2 * (int)CLAMP(PIN(rot_cycles), 1.0, ROT_EDGES / 2)) {
        ctx->rd_rung = ctx->rung;
        ctx->rd_hi   = hi;
        ctx->rd_t1   = t1;
        ctx->rd_per  = period;
        ctx->rdone   = 1;

        if(PIN(lad_top) > 0.0 && ctx->rung < nrung) {  // next rung
          ctx->rung++;
          ctx->r_edge = 0;
          ctx->r_t    = 0.0;
        } else {
          PIN(timer)  = 0.0;
          PIN(state)  = 1.5;
          PIN(d_cmd)  = 0.0;
          PIN(en_out) = 0.0;
        }
      }
      break;
    }

    case 22:  // pp -- rotor free to turn: hold q_cmd at 0 and ramp com_pos open-loop
              // at test_vel while injecting d_cmd, forcing the rotor to follow the
              // rotating stator field (same technique as a sensorless PMSM I/F
              // startup). Comparing the commanded electrical rate against the
              // measured mechanical rate gives pole pairs directly.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(com_pos) += PIN(test_vel) * period;
      PIN(com_pos) = mod(PIN(com_pos));

      ctx->pp_n++;
      if(ABS(PIN(vel_fb)) > 0.1) {
        ctx->pp_turn++;
        float pp_m = PIN(test_vel) / PIN(vel_fb);
        PIN(pp)    = PIN(pp) != 0.0 ? PIN(pp) * 0.995 + pp_m * 0.005 : pp_m;
      }

      PIN(timer) += period;
      if(PIN(timer) >= 3.0) {
        PIN(timer) = 0.0;

        if(PIN(pp) < 0.0) {
          PIN(out_rev) = 1.0;
          PIN(pp) *= -1.0;
        }
        ctx->pp_raw = PIN(pp);
        PIN(pp)     = (int)(PIN(pp) + 0.5);

        PIN(en_out)   = 0.0;
        PIN(d_cmd)    = 0.0;
        PIN(cmd_mode) = 0.0;

        PIN(state) = 2.4;
      }
      break;

    case 42:  // rotating test: field up to speed, open loop at the plate flux current
    case 43:  // flux sweep
    case 44:  // inertia ramps
    case 45:  // rotating tr check (FOC on the encoder)
    case 46: {  // back down
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = MAX(PIN(rot_bw), 1.0);
      PIN(q_cmd)    = 0.0;
      int st        = (int)(PIN(state) * 10.0 + 0.5);
      float pp      = PIN(n_pp);
      float wn      = 2.0 * M_PI * PIN(n_freq);
      float acc     = PIN(rot_acc) > 0.0 ? PIN(rot_acc) : 0.2 * wn;
      float idr     = PIN(id_n) > 0.0 ? PIN(id_n) : PIN(test_cur);
      float imax    = PIN(n_cur) > 0.0 ? 0.85 * 1.4142136 * PIN(n_cur) : 1.15 * idr;
      float w_hi    = wn * MIN(2.0 * PIN(rot_vel), 0.9);
      ctx->w_t      = wn * CLAMP(PIN(rot_vel), 0.05, 0.45);
      float vel     = PIN(vel_fb);
      // electrical slip, with the field's sign; only meaningful with an encoder
      float slip = ctx->w_e - pp * ABS(vel);
      ctx->pos_u += minus(PIN(pos_fb), ctx->pos_l);
      ctx->pos_l = PIN(pos_fb);
      // the ramps' tr once they have run, else the standstill one
      float tr_u  = ctx->tr_rot > 0.0 ? ctx->tr_rot : PIN(tr);
      float lm_u  = ctx->tr_rot > 0.0 ? ctx->lm_rot : PIN(lmr);
      float w_max = MAX(wn * PIN(rot_vmax), w_hi);
      ctx->sw_t += period;
      PIN(d_cmd) = idr;
      // curpid's predictor and emf feed forward use hv0.psi: from the emf
      // here, not acim_flux0 (its low flux shape is what this test is
      // after; on the spindle it read 0.075 for 0.107 Vs at 6.9 A, and the
      // predicted iq ran 4 A off at 780 rad/s el: 1.5 A for 12.6)
      if(ctx->psi_lp <= 0.0) {
        ctx->psi_lp = MAX(PIN(lmr), 0.0) * idr;
      }
      if(ABS(ctx->w_e) > 0.1 * wn) {
        float e_d = PIN(ud_fb) - PIN(r) * PIN(id_fb) + ctx->w_e * PIN(l) * PIN(iq_fb);
        float e_q = PIN(uq_fb) - PIN(r) * PIN(iq_fb) - ctx->w_e * PIN(l) * PIN(id_fb);
        ctx->psi_lp += (sqrtf(e_d * e_d + e_q * e_q) / ABS(ctx->w_e) - ctx->psi_lp) * MIN(period / 0.02, 1.0);
      }
      PIN(psi) = ctx->psi_lp;

      if(st == 42) {
        ctx->w_e = MIN(ctx->w_e + acc * period, ctx->w_t);
        if(ctx->w_e >= ctx->w_t && ctx->sw_t > ctx->w_t / acc + 1.0) {
          // the encoder follows the field within a fifth: inertia can be
          // read. Turning but well behind it: the rotor pulled out on the
          // ramp, and nothing after this would mean anything.
          float lag    = ABS(pp * ABS(vel) - ctx->w_e);
          PIN(rot_enc) = lag < 0.2 * ctx->w_e ? 1.0 : 0.0;
          if(lag >= 0.2 * ctx->w_e && pp * ABS(vel) > 0.2 * ctx->w_e) {
            ctx->stall = 1;
            PIN(state) = 4.6;
            break;
          }
          // with an encoder the ramps come first: their tr corrects the
          // sweep's points for slip
          if(PIN(rot_enc) > 0.0) {
            PIN(state)  = 4.4;
            ctx->sw_k   = 0;
            ctx->sw_t   = 0.0;
            ctx->sw_cnt = 0;
          } else {
            sw_start(ctx, pins, idr, imax);
          }
        }
      } else if(st == 43) {
        // Two passes over the same levels, each level at its own speed and at
        // half of it. The rotor flux read from uq carries any offset du on uq
        // as du / w (on the spindle the low levels read 4% higher at half
        // speed with the same slip, 6 Oct): with psi_k = lmr i_k + du / w_k
        // at both speeds, lmr = (w1 psi1 - w2 psi2) / (w1 i1 - w2 i2).
        // That offset is volts of inverter error against an emf that falls
        // with the flux: at one speed for all levels (0.4 of plate speed)
        // the spindle read lmr at 5.5 A as 6.6 mH, 9.4 at 0.9, against about
        // 12 from a flux observer at 838 rad/s (13 Oct). So with an encoder
        // each level runs where its emf is the ramps' top one, w_hi id_n / i
        // up to rot_vmax: the field weakening range, where acim_flux uses
        // these levels.
        int fi = ctx->sw_pass == 0 ? ctx->sw_top - ctx->sw_k : ctx->sw_fi[MIN(ctx->sw_k, SW_N - 1)];
        float w_l = ctx->w_t;
        if(PIN(rot_enc) > 0.0) {
          float idl = CLAMP(sw_frac[fi] * idr, PIN(i_min), imax);
          w_l       = MIN(w_hi * idr / idl, w_max);
        }
        float w_s = ctx->sw_pass == 2 ? ctx->w_e : (ctx->sw_pass == 1 ? 0.5 * w_l : w_l);
        if(ctx->w_e != w_s) {
          // the last level was the lowest flux: back to id_n and let the
          // flux build before the field changes speed, or the rotor falls
          // behind and pulls out (simulated). Over w_hi the flux falls with
          // the speed, the emf stays at the ramps' top, and the field waits
          // for the rotor while it slips more than a third of pull out.
          PIN(d_cmd) = idr * MIN(w_hi / MAX(ctx->w_e, 1.0), 1.0);
          ctx->sw_hold += period;
          if(ctx->sw_hold > MAX(6.0 * tr_u, 0.3) && (PIN(rot_enc) <= 0.0 || ABS(slip) * tr_u < 0.35)) {
            ctx->w_e = ctx->w_e < w_s ? MIN(ctx->w_e + acc * period, w_s) : MAX(ctx->w_e - acc * period, w_s);
          }
          ctx->sw_t    = 0.0;
          ctx->sw_slip = 0.0;
        } else if(ctx->sw_pass == 2) {
          PIN(d_cmd) = idr;
          {
            int n = ctx->sw_np;
            for(int k = 0; k < n; k++) {
              float lm = ctx->sw_i1[k] > 0.0 ? ctx->sw_p1[k] / ctx->sw_i1[k] : 0.0;
              float du = 0.0;
              float w1  = ctx->sw_w[k];
              float den = w1 * ctx->sw_i1[k] - 0.5 * w1 * ctx->sw_i2[k];
              if(ctx->sw_i2[k] > 0.0 && den > 0.0) {
                lm = (w1 * ctx->sw_p1[k] - 0.5 * w1 * ctx->sw_p2[k]) / den;
                du = w1 * (ctx->sw_p1[k] - lm * ctx->sw_i1[k]);
              }
              PINA(sw_i, k)   = ctx->sw_i1[k];
              PINA(sw_psi, k) = lm * ctx->sw_i1[k];
              ctx->sw_du[k]   = du;
            }
            // taken from the top: put them in rising order, as the report
            // and the inertia test read them
            for(int a = 0, b = n - 1; a < b; a++, b--) {
              float t;
              t = PINA(sw_i, a), PINA(sw_i, a) = PINA(sw_i, b), PINA(sw_i, b) = t;
              t = PINA(sw_psi, a), PINA(sw_psi, a) = PINA(sw_psi, b), PINA(sw_psi, b) = t;
              t = ctx->sw_x[a], ctx->sw_x[a] = ctx->sw_x[b], ctx->sw_x[b] = t;
              t = ctx->sw_x2[a], ctx->sw_x2[a] = ctx->sw_x2[b], ctx->sw_x2[b] = t;
              t = ctx->sw_du[a], ctx->sw_du[a] = ctx->sw_du[b], ctx->sw_du[b] = t;
              t = ctx->sw_i1[a], ctx->sw_i1[a] = ctx->sw_i1[b], ctx->sw_i1[b] = t;
              t = ctx->sw_p1[a], ctx->sw_p1[a] = ctx->sw_p1[b], ctx->sw_p1[b] = t;
              t = ctx->sw_i2[a], ctx->sw_i2[a] = ctx->sw_i2[b], ctx->sw_i2[b] = t;
              t = ctx->sw_p2[a], ctx->sw_p2[a] = ctx->sw_p2[b], ctx->sw_p2[b] = t;
              t = ctx->sw_v1[a], ctx->sw_v1[a] = ctx->sw_v1[b], ctx->sw_v1[b] = t;
              t = ctx->sw_w1[a], ctx->sw_w1[a] = ctx->sw_w1[b], ctx->sw_w1[b] = t;
              t = ctx->sw_v2[a], ctx->sw_v2[a] = ctx->sw_v2[b], ctx->sw_v2[b] = t;
              t = ctx->sw_w2[a], ctx->sw_w2[a] = ctx->sw_w2[b], ctx->sw_w2[b] = t;
              t = ctx->sw_w[a], ctx->sw_w[a] = ctx->sw_w[b], ctx->sw_w[b] = t;
            }
            PIN(sw_n)  = n;
            ctx->sw_k  = 0;
            ctx->sw_t  = 0.0;
            PIN(state) = 4.6;
          }
        } else {
          ctx->sw_hold = 0.0;
          float id   = CLAMP(sw_frac[fi] * idr, PIN(i_min), imax);
          PIN(d_cmd) = id;
          ctx->sw_slip += (slip - ctx->sw_slip) * MIN(period / 0.05, 1.0);
          int last = ctx->sw_pass == 0 ? fi <= ctx->sw_k0 : ctx->sw_k + 1 >= ctx->sw_np;
          // the rotor nears pull out: back to id_n before it falls behind
          if(PIN(rot_enc) > 0.0 && ctx->sw_t > 0.1 && ABS(ctx->sw_slip) * tr_u > SW_X_STOP) {
            ctx->s_uq = ctx->s_iq = ctx->s_id = ctx->s_slip = ctx->s_ud = 0.0;
            ctx->sw_cnt = 0;
            last        = 1;
            ctx->sw_t   = MAX(6.0 * tr_u, 0.3) + SW_MEASURE + 1.0;
          }
          if(ctx->sw_t > MAX(6.0 * tr_u, 0.3)) {
            ctx->s_uq += PIN(uq_fb);
            ctx->s_ud += PIN(ud_fb);
            ctx->s_iq += PIN(iq_fb);
            ctx->s_id += PIN(id_fb);
            ctx->s_slip += slip;
            ctx->sw_cnt++;
          }
          if(ctx->sw_t > MAX(6.0 * tr_u, 0.3) + SW_MEASURE) {
            if(ctx->sw_cnt > 0) {
              float n   = (float)ctx->sw_cnt;
              float idm = ctx->s_id / n;
              float psd = (ctx->s_uq / n - PIN(r) * ctx->s_iq / n) / ctx->w_e;
              // the rotor flux's angle behind the current, from the voltages
              // alone: e = u - r i - j w l i = j w psi_r, so x_v = tan of that
              // angle = slip tr with no tr in it (and no slip: the encoder's
              // slip gives tr = x_v / ws, reported per point)
              float iqm = ctx->s_iq / n;
              float e_d = ctx->s_ud / n - PIN(r) * idm + ctx->w_e * PIN(l) * iqm;
              float e_q = ctx->s_uq / n - PIN(r) * iqm - ctx->w_e * PIN(l) * idm;
              float th  = atan2f(iqm, idm) - atan2f(-e_d, e_q);
              float x_v = tanf(th);
              float ws_m = ctx->s_slip / n;
              // with load (drag) the rotor slips and its flux turns off the
              // current axis by atan(slip tr): uq only sees the projection.
              // Back to the flux magnitude and the magnetizing current, from
              // the encoder's slip; without an encoder the slip is unknown.
              // tr is Lr / Rr and grows with lmr at low flux: x with one tr
              // alone read lmr 6% low at x 0.6 (simulated, 14% saturation),
              // so tr follows this point's own lmr against the one tr was
              // taken at (the ramps', else the standstill test's), a few rounds
              float ws  = PIN(rot_enc) > 0.0 ? ctx->s_slip / n : 0.0;
              float pr  = psd - PIN(l) * idm;
              float trp = tr_u;
              float x   = 0.0, c = 1.0;
              for(int it = 0; it < 4; it++) {
                x = ws * trp;
                c = sqrtf(1.0 + x * x);
                if(lm_u > 0.0 && idm > 0.0) {
                  trp = tr_u * CLAMP(pr * c * c / idm / lm_u, 0.5, 2.0);
                }
              }
              // past SW_X_MAX the slip correction leans on tr more than the
              // point can carry (spindle, 6 Oct: 5.4 and 8.5 mH at x 0.57 and
              // 0.50 against 11-12 mH above): dropped, and lower levels only
              // slip more
              int k = ctx->sw_k;
              // with the ramps' tr the correction holds further (simulated)
              if(ABS(x) <= (ctx->tr_rot > 0.0 ? SW_X_MAX_ROT : SW_X_MAX)) {
                if(ctx->sw_pass == 0) {
                  ctx->sw_fi[k] = fi;
                  ctx->sw_w[k]  = ctx->w_e;
                  ctx->sw_x[k]  = x;
                  ctx->sw_i1[k] = idm / c;
                  ctx->sw_p1[k] = pr * c;
                  ctx->sw_i2[k] = 0.0;
                  ctx->sw_p2[k] = 0.0;
                  ctx->sw_x2[k] = 0.0;
                  ctx->sw_v1[k] = x_v;
                  ctx->sw_w1[k] = ws_m;
                  ctx->sw_v2[k] = 0.0;
                  ctx->sw_w2[k] = 0.0;
                  ctx->sw_np    = k + 1;
                } else {
                  ctx->sw_x2[k] = x;
                  ctx->sw_v2[k] = x_v;
                  ctx->sw_w2[k] = ws_m;
                  ctx->sw_i2[k] = idm / c;
                  ctx->sw_p2[k] = pr * c;
                }
              } else {
                last = 1;
              }
            }
            ctx->s_uq = ctx->s_iq = ctx->s_id = ctx->s_slip = ctx->s_ud = 0.0;
            ctx->sw_cnt = 0;
            ctx->sw_t   = 0.0;
            ctx->sw_k++;
            if(last) {
              ctx->sw_k    = 0;
              ctx->sw_slip = 0.0;
              // the second speed needs the slip, so an encoder
              ctx->sw_pass = ctx->sw_pass == 0 && PIN(rot_enc) > 0.0 && ctx->sw_np > 0 ? 1 : 2;
            }
          }
        }
      } else if(st == 44) {
        // two pairs of ramps, at id_n and at RAMP_I2 of it: phase 0 hold
        // until the rotor flux has settled, 1 up to w_hi, 2 hold, 3 down to
        // w_t; the middle half of each ramp is averaged
        int ph   = ctx->sw_k & 3;
        int pr   = ctx->sw_k >> 2;
        float wa = ctx->w_t, wb = w_hi;
        PIN(d_cmd) = pr ? MAX(RAMP_I2 * idr, PIN(i_min)) : idr;
        if(ph == 1) {
          ctx->w_e = MIN(ctx->w_e + acc * period, wb);
        } else if(ph == 3) {
          ctx->w_e = MAX(ctx->w_e - acc * period, wa);
        }
        if(ph == 1 || ph == 3) {
          float x = (ctx->w_e - wa) / (wb - wa);
          if(x > 0.25 && x < 0.75) {
            if(ctx->sw_cnt == 0) {
              ctx->v_a = ABS(vel);
              ctx->t_a = ctx->sw_t;
              ctx->p_a = ctx->pos_u;
            }
            ctx->s_slip += slip;
            ctx->s_we += ctx->w_e;
            ctx->p_b = ctx->pos_u;
            // rotor flux on the current's axis, as the sweep reads it
            ctx->s_pd += (PIN(uq_fb) - PIN(r) * PIN(iq_fb)) / MAX(ctx->w_e, 1.0) - PIN(l) * PIN(id_fb);
            ctx->s_ir += PIN(id_fb);
            // air gap power over the field speed: copper loss off the
            // stator power. Independent of tr; the r and dead time errors
            // are the same on both ramps and drop out of the difference.
            float id_f = PIN(id_fb), iq_f = PIN(iq_fb);
            ctx->s_tv += (PIN(ud_fb) * id_f + PIN(uq_fb) * iq_f - PIN(r) * (id_f * id_f + iq_f * iq_f)) / MAX(ctx->w_e, 1.0);
            ctx->v_b = ABS(vel);
            ctx->t_b = ctx->sw_t;
            ctx->sw_cnt++;
          }
          if(ABS(slip) * PIN(tr) > 1.0) {  // past the torque peak: pulled out
            ctx->stall = 1;
            PIN(state) = 4.6;
          }
        }
        int done = (ph == 0 && ctx->sw_t > MAX(6.0 * PIN(tr), 0.5)) || (ph == 1 && ctx->w_e >= wb) || (ph == 2 && ctx->sw_t > 0.5) || (ph == 3 && ctx->w_e <= wa);
        if(done && PIN(state) < 4.55) {
          if(ph == 1 || ph == 3) {
            int u            = (ph == 1 ? 0 : 1) + 2 * pr;
            float dt         = ctx->t_b - ctx->t_a;
            float n          = (float)MAX(ctx->sw_cnt, 1);
            ctx->j_slip[u]   = ctx->s_slip / n;
            ctx->j_tv[u]     = ctx->s_tv / n;
            ctx->j_alpha[u]  = dt > 0.0 ? (ctx->v_b - ctx->v_a) / dt : 0.0;
            // slip from the mean speed over the window, position over time
            ctx->j_ws[u]     = dt > 0.0 ? ctx->s_we / n - pp * ABS(ctx->p_b - ctx->p_a) / dt : 0.0;
            ctx->j_pd[u]     = ctx->s_pd / n;
            ctx->j_ir[u]     = ctx->s_ir / n;
            ctx->s_slip = ctx->s_tv = ctx->s_we = ctx->s_pd = ctx->s_ir = 0.0;
            ctx->sw_cnt      = 0;
          }
          ctx->sw_k++;
          ctx->sw_t = 0.0;
          if(ctx->sw_k > 7) {
            rot_tr_fit(ctx, pins);
            if(PIN(state) < 4.55 && PIN(rot_tr_vel) > 0.0) {
              PIN(state)  = 4.5;
              ctx->d_tr = ctx->q_tr = 0.0;
              ctx->fo_imr = idr;
              ctx->fo_pos = PIN(pos_fb);
              ctx->fo_t   = 0.0;
              ctx->fo_ph  = 0;
              ctx->fo_cyc = 0;
              ctx->fo_wl = pp * ABS(vel);
              for(int u = 0; u < 2; u++) {
                ctx->fo_n[u] = 0;
                for(int q = 0; q < 4; q++) {
                  ctx->fo_a[u][q] = 0.0;
                }
                for(int q = 0; q < 6; q++) {
                  ctx->fo_s[u][q] = 0.0;
                }
              }
            } else if(PIN(state) < 4.55) {
              sw_start(ctx, pins, idr, imax);
            }
          }
        }
      } else if(st == 45) {
        // Rotating tr check. FOC on the encoder with this test's own rotor
        // flux model at the standstill tr (slip = iq / (tr i_mr)), d current
        // for 0.8 of the plate frequency's emf at the top speed, and torque
        // only ramps (iq +-ia, no speed loop) between 0.7 and 1 of
        // rot_tr_vel. At that speed the emf is large against the dead time
        // and r errors, so the voltages give the rotor flux's angle in the
        // frame: e = u - r i - j w l i = j w psi_r. With the model's tr off
        // by k = tr_true / tr, the frame settles where tan(phi - delta) = k
        // tan(phi), phi the current's angle, delta the flux's. Offsets on
        // the angle (dead time, voltage timing) are the same on the up and
        // the down ramp, where phi changes sign: the pair gives k and the
        // offset. (Spindle, 13 Oct: standstill 78-93 ms, the running motor
        // needed about 120 ms at every flux, flux_obs and torque balance.)
        float w_r  = pp * ABS(vel);
        float whm  = PIN(rot_tr_vel) * pp;
        float wlm  = 0.7 * whm;
        // d current: fixed at the window's top speed value while ramping
        // through it (a flux that follows the speed changes on a ramp and
        // tilts the angle with the ramp's sign, as the slip error does:
        // simulated, k read 0.87 for 1), back up under the plate speed on
        // the way out
        float idw  = CLAMP(idr * 0.8 * wn / MAX(whm, 1.0), 1.5 * PIN(i_min), idr);
        float idc  = ctx->fo_ph == 2 ? CLAMP(idr * 0.8 * wn / MAX(w_r, 1.0), idw, idr) : idw;
        float ia   = MIN(0.6 * idr, sqrtf(MAX(imax * imax - idc * idc, 0.0)));
        float trm  = MAX(PIN(tr), 0.005);
        PIN(d_cmd) = idc;
        PIN(q_cmd) = ctx->fo_ph == 0 ? ia : -ia;
        ctx->fo_imr += (PIN(id_fb) - ctx->fo_imr) * period / (trm + period);
        float sl = PIN(iq_fb) / (trm * MAX(ctx->fo_imr, MAX(PIN(i_min), 0.5)));
        // the frame follows the encoder's position, not vel_fb: a speed
        // estimate trails on a ramp (vel1, 1.8 ms), which is a frame speed
        // error of acceleration x lag, as large as the slip error looked for
        // and of the ramp's sign (simulated: k 0.99 from the true angles for 1.06)
        float dp    = minus(PIN(pos_fb), ctx->fo_pos);
        ctx->fo_pos = PIN(pos_fb);
        ctx->w_e    = pp * dp * (vel >= 0.0 ? 1.0 : -1.0) / period + sl;
        float w_f   = w_r + sl;  // smooth frame speed, for the emf
        ctx->fo_t += period;
        float wm = 0.2 * (whm - wlm);
        float w_prev = ctx->fo_wl;
        ctx->fo_wl   = w_r;
        if(ctx->fo_ph < 2 && ctx->fo_cyc >= 1 && ctx->fo_t > 3.0 * trm && w_r > wlm + wm && w_r < whm - wm) {
          float idf = PIN(id_fb), iqf = PIN(iq_fb);
          // the dead time loses 4/pi of the drop along the current (the
          // template turns its compensation off): it scales e_q by +-sin
          // phi between the ramps and reads as a slip error (simulated,
          // k 0.975 for 1.056 without this)
          float ia_ = MAX(sqrtf(idf * idf + iqf * iqf), 0.1);
          float dt_ = 4.0 / M_PI * MAX(PIN(drop), 0.0) / ia_;
          float e_d = PIN(ud_fb) - dt_ * idf - PIN(r) * idf + w_f * PIN(l) * iqf;
          float e_q = PIN(uq_fb) - dt_ * iqf - PIN(r) * iqf - w_f * PIN(l) * idf;
          int u     = ctx->fo_ph;
          ctx->fo_s[u][0] += atan2f(-e_d, e_q);
          ctx->fo_s[u][1] += atan2f(iqf, idf);
          ctx->fo_s[u][2] += ctx->fo_imr;
          ctx->fo_s[u][3] += sqrtf(e_d * e_d + e_q * e_q) / MAX(w_f, 1.0) / MAX(ctx->fo_imr, 0.5);
          float e_d0 = e_d + dt_ * idf, e_q0 = e_q + dt_ * iqf;
          ctx->fo_s[u][4] += atan2f(-e_d0, e_q0);
          ctx->fo_a[u][0] += (w_r - w_prev) / pp / period;
          ctx->fo_a[u][1] += sqrtf(idf * idf + iqf * iqf);
          ctx->fo_a[u][2] += sqrtf(e_d * e_d + e_q * e_q) / MAX(w_f, 1.0);
          ctx->fo_a[u][3] += iqf;
          ctx->fo_s[u][5] += sqrtf(e_d0 * e_d0 + e_q0 * e_q0) / MAX(w_f, 1.0) / MAX(ctx->fo_imr, 0.5);
          ctx->fo_n[u]++;
        }
        if(ctx->fo_ph == 0 && (w_r >= whm || ctx->fo_t > 15.0)) {
          ctx->fo_ph = 1;
          ctx->fo_t  = 0.0;
        } else if(ctx->fo_ph == 1 && w_r <= wlm) {
          ctx->fo_cyc++;
          ctx->fo_t  = 0.0;
          ctx->fo_ph = ctx->fo_cyc > FO_CYC ? 2 : 0;
        } else if(ctx->fo_ph == 2 && w_r <= ctx->w_t) {
          // back under the plate frequency: the open loop field takes over
          // at the rotor's speed and ramps down
          // and the flux sweep follows
          ctx->w_e   = w_r;
          PIN(q_cmd) = 0.0;
          PIN(d_cmd) = idr;
          sw_start(ctx, pins, idr, imax);
        } else if(ctx->fo_ph < 2 && ctx->fo_t > 15.0) {
          ctx->fo_ph = 2;  // the window is not reached: out
          ctx->fo_t  = 0.0;
        } else if(ctx->fo_ph == 2 && ctx->fo_t > 30.0) {
          PIN(en_out)   = 0.0;  // no deceleration: let it coast, the field off
          PIN(d_cmd)    = 0.0;
          PIN(q_cmd)    = 0.0;
          PIN(cmd_mode) = 0.0;
          PIN(state)    = 4.7;
        }
      } else {
        ctx->w_e = MAX(ctx->w_e - acc * period, 0.0);
        if(ctx->w_e <= 0.0) {
          PIN(en_out)   = 0.0;
          PIN(d_cmd)    = 0.0;
          PIN(cmd_mode) = 0.0;
          PIN(state)    = 4.7;
        }
      }
      // The current loop leaves an error at speed that differs between an
      // up and a down ramp (simulated: id 19.2 / 19.7 A for 20.4, iq 0 /
      // 0.2 A): a slow integrator on top holds id and iq on their targets,
      // so both ramps see the same flux and current and their offsets
      // cancel.
      // (not in the tr check, which runs its own FOC)
      if(PIN(en_out) > 0.0 && st != 45) {
        float d_t = PIN(d_cmd);
        ctx->d_tr = CLAMP(ctx->d_tr + (d_t - PIN(id_fb)) * 20.0 * period, -0.25 * d_t, 0.25 * d_t);
        ctx->q_tr = CLAMP(ctx->q_tr - PIN(iq_fb) * 20.0 * period, -0.1 * d_t, 0.1 * d_t);
        PIN(d_cmd) = d_t + ctx->d_tr;
        PIN(q_cmd) = ctx->q_tr;
      }
      PIN(com_pos) = mod(PIN(com_pos) + ctx->w_e * period);
      break;
    }
  }
  // hv0.vel must be this frame's speed. It was angle0.vel, the rotor's: on
  // a motor that turns against the field's sign (out_rev 1, the spindle) that
  // has the wrong sign, and hv0 advanced the frame by -2 w adv and the f3
  // decoupled against the emf; the rotating test ran with iq -3..-12 A for 0
  // commanded (13 Oct)
  PIN(com_vel)  = minus(PIN(com_pos), ctx->com_last) / period;
  ctx->com_last = PIN(com_pos);
}

hal_comp_t idacim_comp_struct = {
    .name      = "idacim",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idacim_ctx_t),
    .pin_count = sizeof(struct idacim_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
