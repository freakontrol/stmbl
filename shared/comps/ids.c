#include "ids_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `ids` tunes the gains of the `pid` position/velocity loop (`pos_bw`, `vel_bw`, `vel_d`) by moving the axis back and forth on a trapezoid profile between `min_pos` and `max_pos` and raising one gain at a time while each raise still cuts the tracking-error cost. In speed mode (`pos_en = 0`, a spindle) it instead tunes only `vel_bw`, from the speed dip under a load torque step at +-`max_vel`. It runs on the F4 board and is loaded by the `id_pid` template, normally as the last identification step after `id_mot` (and `id_sys` if a load is coupled). With `lpf = 1` (template `id_lpf`) it instead measures the coupling resonance and computes `conf0.j_lpf`.
*
* ## Component Explanation
*
* 1. **Before you start**:
* - `id_mot` (and `id_sys`) should be done and their results (`conf0.j`, `conf0.j_sys`, `conf0.d`, `conf0.f`, `conf0.o`) in the config, since the pid feedforward and gain scaling use them.
* - `conf0.cur_bw` should be set: `vel_bw` is capped at `conf0.cur_bw / bw_ratio` (`bw_ratio` default 7.5, so 400 at a `cur_bw` of 3000). If either is 0 there is no cap and the console says so.
* - `conf0.max_force` should be the drive's real torque: it sets the noise limit (`fb_max` x `max_force`) and the torque saturation test. With 0 the noise limit is off and the console says so.
* - The axis starts where it stands (`pos_fb`), so there is no step at the start, and drives back to the start position when the search is done.
*
* 2. **How to run it**:
* - At the console, `link id_pid`. The template loads `ids` and wires `ids0.en = fault0.en_out`, the trajectory into `pid0.pos_ext_cmd` / `vel_ext_cmd` / `acc_ext_cmd`, `pid0.pos_bw` / `vel_bw` / `vel_d` from this component, `pid0.pos_error` / `vel_error` / `torque_cmd` / `fb_torque_cmd` / `sat` back into it, `ids0.pos_en = pid0.pos_en` and `pid0.torque_ext_cmd = ids0.torque_dist` for speed mode, plus `ids0.cur_bw = conf0.cur_bw`, `ids0.pos_fb = fb_switch0.pos_fb`, `ids0.max_torque = conf0.max_force`, the start values from `conf0.pos_bw` / `vel_bw` / `vel_d` and the limits from `conf0.max_acc` / `max_vel`. It sets `conf0.max_pos_error = 0`, `conf0.max_sat = 10` and `conf0.vel_g = 1`, and puts `pos_cmd`, `vel_cmd`, `min_cost` and the three gains on the scope waves.
* - Enable the drive. With `auto_step >= 1` (default) the search starts at once; with `auto_step = 0` the console asks for `ids0.state = 1.2` first. In speed mode it always starts at once.
* - Watch `min_cost` and the gains on the scope. When it is done the console prints `conf0.pos_bw`, `conf0.vel_bw` and `conf0.vel_d`, the last noise and torque peak (in speed mode the last speed noise and load step dip), and the number of saturated cycles that were not scored: append the gains to the config and save. If the torque peak went over `conf0.max_force` it says so: raise `conf0.max_force` to the drive's real torque or lower `ids0.max_acc`.
* - For the coupling resonance, `link id_lpf` instead (it links `id_pid` and sets `ids0.lpf = 1`, `ids0.j_mot = conf0.j`, `ids0.j_sys = conf0.j_sys` and puts `ring_sig` on scope wave 1). Run it before `id_pid`: `j_lpf` changes the plant the gains are tuned against.
*
* 3. **Gain search (`state` 1.x)**:
* - `0`: idle. The trajectory follows `pos_fb`, which becomes the start position, `target` is set to the start position + `max_pos`, the gains are reset to the start values, `max_params[0] = cur_bw / bw_ratio` (1e6 when either is 0), `max_params[1] = 1`, and the cost, noise and saturation counters are cleared. When `en` goes high it goes to `1.0` (or `2.0` with `lpf > 0`); `en` low returns to `0` from any state.
* - `1.0` -> `1.1` (nrt): prints the `vel_bw` cap and the noise limit. With `pos_en <= 0` it goes to the speed mode search `1.5` (section 4), else with `auto_step >= 1` straight to `1.2`, otherwise it waits in `1.1`.
* - `1.2` (rt): a trapezoidal trajectory (`pos`, `vel`, `acc`) moves to the start position + `max_pos` and back to the start position + `min_pos` in cycles of `2 * (|max_pos - min_pos| / vel + 2 * vel / acc)`, with `vel` and `acc` the capped profile limits (1.2 s with the defaults). `pos_cmd` is `pos` wrapped to +-pi, `vel_cmd` / `acc_cmd` are scaled by `ff`. Over each cycle it integrates the cost, the rms of `fb_torque` above 50 Hz (`noise`) and the peak of `|torque|` (`peak`):
* ```c
* cost += (kp * |pos_error| + ks * pos_error^2 + kv * vel_error^2) * period;
* ```
* - A cycle counts as saturated when `sat` (pid0.sat) is high or `|torque|` reaches 0.99 x `max_torque`. The first cycle after enable is a warm-up and is not scored; it runs on while pid saturates, up to 25 cycles. After that a saturated cycle is not scored either and is repeated with the same gains (`skipped` counts them); 5 saturated cycles in a row count as a failed step.
* - A score averages the cost of `rep` cycles. A step is noisy when it failed by saturation or `noise` is above `fb_max` x `max_torque`. The gain selected by `param` is changed (coordinate search):
* - if the gain is above its limit it is clamped and the search moves to the next gain. Limits: `vel_bw <= cur_bw / bw_ratio`, `1 / vel_d <= 1` (so `vel_d >= 1`), `pos_bw <= 2 * vel_bw`.
* - else at the gain's start value: a saturation failure cuts it by `kd` and retries, up to 5 times; a noisy score cuts it by `kd` and moves to the next gain; otherwise the score becomes `min_cost` and the gain is multiplied by `1 + step` (`step`).
* - else after a raise: if the score is noisy or `cost > min_cost * (1 - kg)` the raise is taken back (divided by `1 + step`) and the search moves to the next gain; otherwise `min_cost = cost` and the gain is raised again.
* - Down search for `vel_bw`: when already its first raise fails, the start value is past where gain pays, and `vel_bw` is cut by `1 + step` instead, as long as the cut stays at or above 4 x `pos_bw`. Each cut is kept while the score is not noisy and `cost <= cost0 * (1 + kg)`, with `cost0` the start value's score; the first cut that fails is taken back, and the search stops at the 4 x `pos_bw` floor, then moves on to `vel_d`. More gain there only buys feedback noise (spindle, magnetic encoder, 13 Oct: the default 250 had 2.5 to 3 times the current noise of a hand set 40 at the same speed noise). A `vel_bw` that climbs (a servo axis) never takes this branch, and `vel_d` and `pos_bw` search as before.
* - When all three gains are done the trajectory drives back to the start position without scoring, so repeated runs do not walk the axis toward a stroke end. There the feedforward outputs are zeroed and the state goes to `1.3`; the nrt function prints the results and goes to `1.4` (done). The trajectory stays at the start position until `en` goes low.
* - With good feedforward the tracking cost gets very small, and below the noise limit it keeps falling a little with each raise of `vel_bw`, so in practice the `cur_bw / bw_ratio` cap is what stops `vel_bw`. The torque peak is only reported: it is set by the profile (J x acceleration at the reversals), not by the gains.
*
* 4. **Speed mode (`state` 1.5)**:
* - With `pos_en = 0` (from `pid0.pos_en`, a spindle) there is no position to track, so `1.5` replaces the profile and only `vel_bw` is searched; `vel_d` and `pos_bw` stay at their start values. `min_pos` / `max_pos` do not apply: the axis turns freely.
* - Each cycle ramps at the capped `max_acc` to +`max_vel` or -`max_vel` (alternating), settles 0.5 s, integrates the speed noise (rms of `vel_error`, output `sp_noise`) over 2 s, then puts a load torque step of `dist` x `max_torque` against the motion on `torque_dist` (`pid0.torque_ext_cmd`) for 0.5 s, and releases it for 0.5 s. The largest `vel_error` in the motion's direction while the step is on is the dip (`dip`). A cycle is saturated when, after the ramp, `sat` is high or `abs(torque)` reaches 0.99 x `max_torque`.
* - A score covers 4 cycles (2 per direction; the bench spindle read +-10 % from 2 x 1 s). It is bad when a cycle saturated or the dip is above `dip_max` x `max_vel`.
* - Saturation means too much gain (the loop oscillates) or a profile the drive cannot follow, never a soft loop: it lowers `vel_bw` or stops a raise. Only a dip over `dip_max` raises it.
* - A saturated start value is no usable reference: it is cut by `1 + step` and scored again, until a start value scores without saturation (down to `vel_bw` 5, which then is the result).
* - The first unsaturated score is the noise reference. If it is good, `vel_bw` is divided by `1 + step` per score while the score stays good and the speed noise stays within `3 x kg` of the reference; the first that fails, or a `vel_bw` under 5, ends the search at the last kept value.
* - If it is bad (too soft already), `vel_bw` is raised by `1 + step` until the first good score, which is the result. A raise stops at the `vel_bw` with the smallest dip so far once a score saturates or its dip grows above that smallest dip x `(1 + 3 x kg)`: more gain then makes the loop oscillate (on an observer, whose lag sets the margin), it is not soft. A raise past the `cur_bw / bw_ratio` cap is scored once at the cap; still soft there, the cap is the result.
* - The torque peak (`peak`) is tracked per score as in the gain search, so the console's check of the peak against `conf0.max_force` works in speed mode too. When the kept `vel_bw` still misses `dip_max` or saturates (a raise stopped at the smallest dip or at the cap, or a start value cut down to 5), the console flags it in red: lower `dist` or `max_acc`, or check the feedback.
* - The result is the lowest gain that still takes the load step. The speed noise is only a coarse guard and does not steer it: on the spindle (magnetic encoder, belt, 13 Oct) it was the same from `vel_bw` 40 to 250 and read +-10 % from score to score even over 4 x 2 s, while the current noise grew 3 x with the gain. On that spindle it found `vel_bw` 79.66 with encoder FOC and 59.85 sensorless.
* - When done the speed ramps down to 0 where the axis is (it does not drive back), the feedforward is zeroed and the state goes to `1.3` / `1.4` as above.
*
* 5. **Coupling resonance for `pid0.j_lpf` (`state` 2.x)**:
* - pid models a compliant coupling by driving `j_mot + j_sys` below `j_lpf` and only `j_mot` above it. The corner belongs at the two-mass anti-resonance. `ids` excites the axis, times the ring in `vel_error` and sets `j_lpf = f_ring`, so the coupling stiffness does not need to be known. The ring is timed in closed loop at the configured gains, where the speed loop holds the motor, so it sits at the anti-resonance and not at the free resonance above it. The gains are not changed: pid runs with the start values (the conf0 gains).
* - `2.0` -> `2.1` (nrt): prints what will happen; with `auto_step >= 1` it goes to `2.2`, otherwise it waits for `ids0.state = 2.2`.
* - `2.2` (rt): with `ring_vel > 0` the axis goes back and forth from where it stands (`pos_fb`), `ring_reps` legs in all. Each leg accelerates at `ring_acc` to its cruise speed, the ring is timed while it cruises for `ring_dwell` seconds, then the axis turns round. The legs alternate in pairs between `ring_vel` and 1.5 x `ring_vel` (both cruise speeds are lowered together so that the faster one stays within the capped `max_vel`), so the axis travels up to about 1.5 x `ring_vel` x `ring_dwell` (15 rad with the defaults) plus the acceleration distance from the start. Kicked at the end of an acceleration while cruising, friction is a constant force and the ring runs free; kicked from standstill, Coulomb friction can stop the load before the ring is timed. With `ring_vel = 0` the axis instead moves `ring_pos` alternately to either side of the start point from standstill, and each dwell starts when it arrives.
* - During each dwell `vel_error` high passed at `ring_hp_hz` (output on `ring_sig`) goes through a Schmitt trigger whose band follows the previous half cycle's peak. Accepted half periods must lie between `max(2 * ring_hp_hz, 3 / ring_dwell)` and `min(rt rate / 20, 500)` Hz, agree with the running mean of the same speed once 4 are in, and come from peaks above `ring_min_amp` and the decay gate. After the last dwell the axis drives back to the start position.
* - At the end, `f_ring_a` and `f_ring_b` are the mean ring frequencies at `ring_vel` and at 1.5 x `ring_vel` (each from at least 3 half periods), `f_ring` the mean of all half periods (at least 6) and `zeta_ring` the mean log decrement damping. With `ring_vel > 0`, `f_ring` is set to 0 unless `f_ring_a` and `f_ring_b` agree within 15 % of their mean: a mechanical mode stays put when the speed changes, while speed-locked ripple (cogging, the screw) moves with it. With `f_ring` in range, `j_lpf = f_ring` and `ring_ok = 1`. State `2.3` (nrt) prints `conf0.j_lpf` (and `zv_ip0` values, and a warning when `conf0.j_sys` is 0, since `j_lpf` then does nothing), or why nothing usable was found (the two speeds disagree, well damped ring, no ring), and goes to `2.4` (done).
* - `j_lpf` only pays if the velocity loop crosses near that frequency; with `conf0.vel_bw` well under `2 pi j_lpf` set `conf0.j_lpf = 0` (conf/template/conf.txt sets 100 Hz).
*
* {{% hint warning %}}
* The pid gains follow this component's outputs for as long as the template is linked, also in state 0 and 1.4.
* {{% /hint %}}
*/
HAL_COMP(ids);

HAL_PIN(en);  // *input*, enable from fault0.en_out; high starts the gain search (or the ring measurement with lpf), low aborts (state -> 0)

HAL_PIN(state);  // *input/output*, 0 off, 1.0/1.1 start/wait, 1.2 gain search, 1.5 speed mode search, 1.3 print, 1.4 done, 2.0/2.1/2.2/2.3/2.4 ring measurement start/wait/run/print/done
HAL_PIN(param);  // *output*, gain currently searched: 0 vel_bw, 1 1/vel_d, 2 pos_bw
HAL_PIN(step);  // *parameter*, relative gain increase per raise, default 0.1
HAL_PIN(rep);  // *parameter*, cycles averaged per score, default 2

HAL_PIN(min_pos);  // *parameter*, lower end of the travel relative to the start position (rad), default -10
HAL_PIN(max_pos);  // *parameter*, upper end of the travel relative to the start position (rad), default 10
HAL_PIN(max_vel);  // *parameter*, test velocity (rad/s), default 100, capped at vel_lim
HAL_PIN(max_acc);  // *parameter*, test acceleration (rad/s^2), default 500, capped at acc_lim
HAL_PIN(acc_lim);  // *input*, conf0.max_acc, caps max_acc and ring_acc when > 0 (rad/s^2)
HAL_PIN(vel_lim);  // *input*, conf0.max_vel, caps max_vel when > 0 (rad/s)

HAL_PIN(pos);  // *output*, unwrapped trajectory position (rad)
HAL_PIN(pos_fb);  // *input*, feedback position from fb_switch0.pos_fb, the profile starts here (rad)
HAL_PIN(vel);  // *output*, trajectory velocity (rad/s)
HAL_PIN(acc);  // *output*, trajectory acceleration (rad/s^2)
HAL_PIN(pos_cmd);  // *output*, position command, pos wrapped to +-pi (rad), to pid0.pos_ext_cmd
HAL_PIN(vel_cmd);  // *output*, velocity feedforward, velocity * ff (rad/s), to pid0.vel_ext_cmd
HAL_PIN(acc_cmd);  // *output*, acceleration feedforward acc * ff (rad/s^2), to pid0.acc_ext_cmd

HAL_PIN(pos_error);  // *input*, position error from pid0.pos_error (rad)
HAL_PIN(vel_error);  // *input*, velocity error from pid0.vel_error (rad/s)

HAL_PIN(pos_bw);  // *output*, position bandwidth under test, to pid0.pos_bw, result for conf0.pos_bw
HAL_PIN(vel_bw);  // *output*, velocity bandwidth under test, to pid0.vel_bw, result for conf0.vel_bw
HAL_PIN(vel_d);  // *output*, velocity loop damping under test, to pid0.vel_d, result for conf0.vel_d
HAL_PIN(cur_bw);  // *input*, current loop bandwidth, conf0.cur_bw, vel_bw is capped at cur_bw / bw_ratio
HAL_PIN(bw_ratio);  // *parameter*, vel_bw cap is cur_bw / bw_ratio, 0 = no cap, default 7.5

HAL_PIN(ff);  // *parameter*, feedforward scale for vel_cmd and acc_cmd, default 1
HAL_PIN(kp);  // *parameter*, cost weight of abs(pos_error), default 1
HAL_PIN(ks);  // *parameter*, cost weight of pos_error^2, default 0
HAL_PIN(kv);  // *parameter*, cost weight of vel_error^2, default 1
HAL_PIN(kg);  // *parameter*, least relative cost cut a raise has to give to be kept, default 0.05
HAL_PIN(kd);  // *parameter*, factor applied to a start value that is noisy or saturates, default 0.7

HAL_PINA(params, 3);  // *output*, gains searched: 0 vel_bw, 1 1/vel_d, 2 pos_bw
HAL_PINA(max_params, 3);  // *output*, limits: cur_bw / bw_ratio (1e6 without cap), 1.0, 2 * vel_bw

HAL_PIN(torque);  // *input*, total torque command from pid0.torque_cmd, for peak and saturation (Nm)
HAL_PIN(fb_torque);  // *input*, feedback torque from pid0.fb_torque_cmd, for the noise measurement (Nm)
HAL_PIN(max_torque);  // *input*, conf0.max_force, scales the noise limit and the saturation test, 0 = noise limit off (Nm)
HAL_PIN(fb_max);  // *parameter*, noise limit as fraction of max_torque, default 0.05
HAL_PIN(noise);  // *output*, last score: rms of fb_torque above 50 Hz (Nm)
HAL_PIN(peak);  // *output*, last score: peak abs(torque) (Nm)

HAL_PIN(target);  // *output*, end point the trajectory is moving to (rad)
HAL_PIN(cost);  // *output*, cost accumulated for the current score, averaged per cycle when scored
HAL_PIN(min_cost);  // *output*, cost to beat: score of the last kept gain
HAL_PIN(auto_step);  // *parameter*, >= 1 starts without waiting for state = 1.2 or 2.2, default 1

HAL_PIN(timer);  // *output*, time in the current cycle (s)
HAL_PIN(sat);  // *input*, pid0.sat, a cycle with it high counts as saturated
HAL_PIN(pos_bw0);  // *input*, start value of pos_bw from conf0.pos_bw, 10 when 0
HAL_PIN(vel_bw0);  // *input*, start value of vel_bw from conf0.vel_bw, 100 when 0
HAL_PIN(vel_d0);  // *input*, start value of vel_d from conf0.vel_d, 10 when 0
HAL_PIN(skipped);  // *output*, saturated cycles not scored in this run

#define NOISE_HZ 50.0  // fb_torque above this counts as noise [Hz]
#define SAT_MAX 5      // saturated cycles in a row before a step fails
#define SP_SETTLE 0.5  // speed mode: settle at speed before the noise window [s]
#define SP_NOISE 2.0   // speed mode: speed noise window [s]
#define SP_REP 4       // speed mode: cycles per score (the bench spindle read +-10 % from 2 x 1 s)
#define SP_STEP 0.5    // speed mode: load step on, then off [s]
#define SP_MIN_BW 5.0  // speed mode: lowest vel_bw [rad/s]
#define WARM_MAX 25    // warm-up cycles at most while pid saturates
#define CUT_MAX 5      // kd cuts of a saturating start value before moving on
#define RING_VEL_B 1.5 // second cruise speed, x ring_vel
#define RING_AGREE 0.15 // the two speeds' rings agree within this (Y: ~10 % run to run, ripple hits 20-25 %)


// ---------------------------------------------------------------------------
// ring down estimator for pid0.j_lpf
//
// pid.c models a compliant coupling by believing it drives j_mot + j_sys below
// j_lpf and only j_mot above it. Collapsing its acc_cmd_lp / acc_cmd_hp split,
// the acc_cmd -> fb_torque transfer is
//
//   j_mot + j_sys / (1 + s / wc)
//
// a lag network with its pole at wc and its zero at wc * (1 + j_sys / j_mot).
// The corner belongs at the two mass anti resonance, sqrt(K / j_sys): that is
// where the load stops following and the inertia the motor actually sees starts
// falling toward j_mot.
//
// K is the one quantity the id sequence never measures, and searching for j_lpf
// against the cost below does not work -- the cost is dominated by the low
// frequency tracking error, which is exactly the band j_lpf does not act in. So
// measure it instead. Kick the axis and time the ring. It is timed in closed
// loop at the configured gains, where the speed loop holds the motor, so the
// ring sits at the anti resonance, not at the free resonance
// sqrt(1 + j_sys / j_mot) above it: a two mass sim of Y (pid.c, vel.c, ids.c,
// vel_bw 300-500) puts it at 0.79-0.91 of the anti resonance. j_lpf = f_ring.
//
// Frequency is the right thing to measure here because it survives a messy
// excitation: several torque edges superposed at the same mode are still that
// mode's period, even though the amplitude and phase are a mess.
//
// This lives on its own branch of the state machine (2.x), entered on enable
// with ids0.lpf = 1 (link id_lpf), and touches nothing the gain search uses, so
// it is safe to run on a machine that is already tuned.
HAL_PIN(lpf);  // *parameter*, 1 = measure the coupling ring (state 2.x) instead of tuning (1.x), set by id_lpf
HAL_PIN(pos_en);  // *input*, pid0.pos_en, 0 = speed mode (vel_bw search of state 1.5), default 1
HAL_PIN(torque_dist);  // *output*, speed mode load torque step for pid0.torque_ext_cmd (Nm)
HAL_PIN(dist);  // *parameter*, speed mode load step, fraction of max_torque, default 0.2
HAL_PIN(dip_max);  // *parameter*, speed mode largest speed dip allowed on the load step, fraction of max_vel, default 0.05
HAL_PIN(sp_noise);  // *output*, speed mode last speed noise, vel_error rms (rad/s)
HAL_PIN(dip);  // *output*, speed mode last speed dip on the load step (rad/s)
HAL_PIN(j_mot);  // *input*, motor inertia, conf0.j (kgm^2), not used by the ring measurement
HAL_PIN(j_sys);  // *input*, load inertia, conf0.j_sys (kgm^2), only for the warning when it is 0

HAL_PIN(ring_pos);  // *parameter*, size of each excitation move with ring_vel 0 (rad), default 0.2
HAL_PIN(ring_vel);  // *parameter*, cruise speed the ring is timed at (rad/s), 0 = twitch ring_pos from standstill, default 10
HAL_PIN(ring_acc);  // *parameter*, excitation acceleration, 0 = capped max_acc (rad/s^2), default 0
HAL_PIN(ring_dwell);  // *parameter*, measuring window after each move (s), default 1
HAL_PIN(ring_reps);  // *parameter*, number of move + dwell pairs, default 8
HAL_PIN(ring_hp_hz);  // *parameter*, high pass corner of the detector (Hz), default 5
HAL_PIN(ring_min_amp);  // *parameter*, smallest ring amplitude the trigger accepts (rad/s), default 0.02

HAL_PIN(ring_sig);  // *output*, high passed vel_error the detector sees (rad/s)
HAL_PIN(ring_amp);  // *output*, peak of ring_sig in the current dwell (rad/s)
HAL_PIN(ring_n);  // *output*, half periods accepted, 0 = nothing measured
HAL_PIN(ring_ok);  // *output*, 1 = f_ring and j_lpf are usable
HAL_PIN(f_ring);  // *output*, measured ring frequency (Hz), 0 when the two speeds disagree
HAL_PIN(f_ring_a);  // *output*, ring timed at ring_vel (Hz)
HAL_PIN(f_ring_b);  // *output*, ring timed at 1.5 x ring_vel (Hz)
HAL_PIN(zeta_ring);  // *output*, measured damping ratio
HAL_PIN(j_lpf);  // *output*, computed corner for conf0.j_lpf, = f_ring (Hz)

struct ring_t {
  uint8_t armed;      // the 2.2 entry hook has run
  uint8_t spent;      // this dwell's ring has decayed into the noise
  uint8_t arrived;    // the kick has reached its target, the dwell runs
  uint8_t home;       // all dwells done, the axis drives back to pos0
  int8_t sign;        // last polarity the schmitt trigger latched
  uint16_t n_cross;   // trigger events in this dwell
  uint16_t n_dwell;   // half periods accepted in this dwell
  uint16_t n_half[2]; // half periods accepted over all dwells, per cruise speed
  uint16_t n_ln;      // dwells that yielded a log decrement
  uint16_t rep;
  float lp;           // high pass state
  float t;            // time inside the current kick + dwell cycle
  float t_last;       // when the last trigger fired
  float env;          // largest peak seen so far in this dwell
  float pk;           // peak since the last trigger
  float pk_prev;      // peak of the half cycle before it, which sets the band
  float a_first;      // first and last accepted peak, for the log decrement
  float a_last;
  float pos0;         // the axis parks here between kicks
  float sum_half[2];
  float sum_ln;
};

struct ids_ctx_t {
  int warm;        // the warm-up cycle is over
  int n;           // cycles in the current score
  float t;         // time in the current score [s]
  float fb_lp;     // fb_torque low pass
  float noise_sq;  // integral of the noise squared
  float peak;
  int first;      // the next score is the first of this parameter
  int cuts;       // kd cuts of this parameter's start value
  int sat_cycle;  // this cycle saturated
  int sat_n;      // saturated cycles in a row
  struct ring_t r;  // ring down estimator, state 2.x
  float pos0;       // where the gain search started: the stroke is centred on it
  int home;         // the search is done, the axis drives back to pos0
  int kept;         // scores kept for this parameter (1 = only the start)
  int down;         // vel_bw is searching down from its start value
  float cost0;      // the start value's cost, the down search's reference
  // speed mode (1.5)
  int sp_dir;        // +1 / -1, this cycle's direction
  int sp_up;         // 1 = raising vel_bw, 0 = lowering
  int sp_n;          // cycles in this score
  float sp_t;        // time in this cycle [s]
  float sp_sq, sp_tq;  // speed error squared integral, its time
  float sp_dip;      // largest dip in this score
  float sp_best;     // the start value's speed noise, the guard's reference
  float sp_bw;       // last kept vel_bw
  int sp_kept;       // scores kept
  int sp_sat;        // this score saturated
  float sp_dmin;     // raising: the smallest dip so far
  float sp_bwmin;    // raising: the vel_bw that gave it
  int sp_cap;        // raising: this score is at the cap
  int sp_capbad;     // the cap was kept although it misses dip_max
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct ids_ctx_t * ctx = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;
  PIN(pos_bw)                = 10.0;
  PIN(vel_bw)                = 100.0;
  PIN(vel_d)                 = 10.0;
  PINA(params, 0)            = PIN(vel_bw);
  PINA(params, 1)            = 1.0 / PIN(vel_d);
  PINA(params, 2)            = PIN(pos_bw);

  PIN(kp) = 1.0;
  PIN(ks) = 0.0;
  PIN(kv) = 1.0;
  PIN(kg) = 0.05;
  PIN(kd) = 0.7;

  PIN(bw_ratio) = 7.5;  // 400 at cur_bw 3000

  PIN(ff) = 1.0;

  PIN(max_vel) = 100.0;
  PIN(max_acc) = 500.0;  // Y 9 Oct: 1000 peaked at 95 % of max_force, 500 at 53 %, same gains
  PIN(min_pos) = -10.0;
  PIN(max_pos) = 10.0;

  PIN(param) = 0.0;
  PIN(step)  = 0.1;
  PIN(rep)   = 2.0;
  PIN(fb_max) = 0.05;
  PIN(pos_en)  = 1.0;
  PIN(dist)    = 0.2;
  PIN(dip_max) = 0.05;

  PIN(auto_step) = 1.0;

  PIN(ring_pos)     = 0.2;
  PIN(ring_vel)     = 10.0;
  PIN(ring_acc)     = 0.0;  // 0 = fall back to max_acc
  PIN(ring_dwell)   = 1.0;
  PIN(ring_reps)    = 8.0;
  PIN(ring_hp_hz)   = 5.0;
  PIN(ring_min_amp) = 0.02;
}

// The latches and the accumulators live in ctx, and ctx survives a stop, so a
// restart has to come back to a cleared detector rather than half a measurement
// taken against whatever the axis was doing when the loop stopped.
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ids_ctx_t *ctx = (struct ids_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct ids_ctx_t));
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ids_ctx_t *ctx      = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      break;

    case 10:
      PIN(state)  = 1.1;

      if(PIN(pos_en) <= 0.0) {
        printf("<font color='green'># speed mode (pid0.pos_en 0): vel_bw from the speed noise at +-%f rad/s and a %f Nm load step (dip under %f rad/s)</font>\n", PIN(max_vel), PIN(dist) * PIN(max_torque), PIN(dip_max) * PIN(max_vel));
        PIN(state) = 1.5;
      } else if(PIN(auto_step) >= 1) {
        PIN(state) = 1.2;
      } else {
        printf("Tune PPI bandwidth and damping\n");
        printf("the motor will move\n");
        printf("ids0.state = 1.2 <font color='green'>to start</font>\n");
      }
      if(PIN(cur_bw) > 0.0 && PIN(bw_ratio) > 0.0) {
        printf("<font color='green'># vel_bw cap %f (conf0.cur_bw %f / ids0.bw_ratio %f)</font>\n", PIN(cur_bw) / PIN(bw_ratio), PIN(cur_bw), PIN(bw_ratio));
      } else {
        printf("<font color='red'>vel_bw not capped</font>: conf0.cur_bw or ids0.bw_ratio is 0\n");
      }
      if(PIN(max_torque) > 0.0) {
        printf("<font color='green'># noise limit %f Nm rms (ids0.fb_max %f x conf0.max_force %f Nm)</font>\n", PIN(fb_max) * PIN(max_torque), PIN(fb_max), PIN(max_torque));
      } else {
        printf("<font color='red'>noise limit off</font>: ids0.max_torque reads 0, set conf0.max_force\n");
      }
      break;

    case 20:
      PIN(state) = 2.1;
      printf("Measure the coupling resonance for pid0.j_lpf\n");
      if(PIN(ring_vel) > 0.0) {
        printf("the axis will move back and forth at %f rad/s, %f times\n", PIN(ring_vel), PIN(ring_reps));
      } else {
        printf("the motor will twitch %f rad, %f times\n", PIN(ring_pos), PIN(ring_reps));
      }
      if(PIN(auto_step) >= 1) {
        PIN(state) = 2.2;
      } else {
        printf("ids0.state = 2.2 <font color='green'>to start</font>\n");
      }
      break;

    case 23:
      if(PIN(ring_ok) > 0.0) {
        printf("resonance = %f Hz, damping = %f, from %f half periods\n", PIN(f_ring), PIN(zeta_ring), PIN(ring_n));
        if(PIN(ring_vel) > 0.0) {
          printf("<font color='green'># %f Hz at %f rad/s, %f Hz at %f rad/s</font>\n", PIN(f_ring_a), PIN(ring_vel), PIN(f_ring_b), PIN(ring_vel) * RING_VEL_B);
        }
        printf("conf0.j_lpf = %f <font color='green'># append to config</font>\n", PIN(j_lpf));
        printf("<font color='green'># j_lpf is the anti resonance: the closed loop ring sits at it.\n");
        printf("# above it pid.c drops the load out of the torque model.\n");
        printf("# set it BEFORE id_pid. it changes the plant the gains are tuned\n");
        printf("# against, so a j_lpf set afterwards invalidates the tune.\n");
        printf("# it only pays if the velocity loop crosses near this frequency.\n");
        printf("# with conf0.vel_bw well under %f rad/s there is nothing to correct\n", 2.0 * M_PI * PIN(j_lpf));
        printf("# and j_lpf only costs phase: set conf0.j_lpf = 0 instead.</font>\n");
        if(PIN(zeta_ring) > 0.0) {
          printf("<font color='green'># and for zv_ip, if you ever wire it up:\n");
          printf("# zv_ip0.natural_frequency = %f\n", PIN(f_ring));
          printf("# zv_ip0.damping_ratio = %f</font>\n", PIN(zeta_ring));
        }
        if(PIN(j_sys) <= 0.0) {
          printf("<font color='red'>conf0.j_sys is 0</font>: j_lpf splits j_sys off, so it does nothing. run id_sys first\n");
        }
      } else if(PIN(ring_vel) > 0.0 && (PIN(f_ring_a) > 0.0 || PIN(f_ring_b) > 0.0)) {
        // a ring at one speed only, or a different one at each: speed
        // locked ripple got timed, not the coupling
        printf("<font color='red'>the two speeds disagree</font>: %f Hz at %f rad/s, %f Hz at %f rad/s (0 = none)\n", PIN(f_ring_a), PIN(ring_vel), PIN(f_ring_b), PIN(ring_vel) * RING_VEL_B);
        printf("# speed ripple (cogging, the screw) sits on the ring at one of them.\n");
        printf("# run again with another ids0.ring_vel. conf0.j_lpf is left as it is.\n");
      } else if(PIN(ring_n) > 0.0 && PIN(ring_amp) > PIN(ring_min_amp) * 3.0) {
        // it rang, but it died before there was anything to time. That is a
        // well damped coupling, which is the good case: nothing to compensate.
        printf("<font color='green'>ring died too fast to time</font>: %f half periods at %f rad/s\n", PIN(ring_n), PIN(ring_amp));
        printf("# the coupling is well damped. there is no resonance worth\n");
        printf("# modelling. the template default is 100, so set it to 0:\n");
        printf("conf0.j_lpf = 0 <font color='green'># append to config</font>\n");
      } else {
        printf("<font color='red'>no ring found</font>: %f half periods, peak %f rad/s\n", PIN(ring_n), PIN(ring_amp));
        printf("either the coupling is stiff enough that there is nothing here,\n");
        printf("or the mode is faster than this loop rate can time (a period\n");
        printf("measurement needs about 20 samples per cycle),\n");
        printf("or the kick was too small: raise ids0.ring_acc or ids0.ring_vel.\n");
        printf("if the mode is slower than ids0.ring_hp_hz (%f Hz) the high pass\n", PIN(ring_hp_hz));
        printf("ate it -- drop that. put ids0.ring_sig on a scope wave to see\n");
        printf("what the detector is working with.\n");
      }
      printf("done\n");
      PIN(state) = 2.4;
      break;

    case 13:
      printf("conf0.pos_bw = %f <font color='green'># append to config</font>\n", PIN(pos_bw));
      printf("conf0.vel_bw = %f <font color='green'># append to config</font>\n", PIN(vel_bw));
      printf("conf0.vel_d = %f <font color='green'># append to config</font>\n", PIN(vel_d));
      if(PIN(pos_en) <= 0.0) {
        printf("<font color='green'># speed mode: last score speed noise %f rad/s rms, load step dip %f rad/s</font>\n", PIN(sp_noise), PIN(dip));
        if(ctx->sp_capbad) {
          printf("<font color='red'>the kept vel_bw misses ids0.dip_max or saturates</font>: the raise hit the cap or the loop began to oscillate; lower ids0.dist or ids0.max_acc, or check the feedback\n");
        }
      } else {
        printf("<font color='green'># last score: noise %f Nm rms, peak %f Nm, of conf0.max_force %f Nm</font>\n", PIN(noise), PIN(peak), PIN(max_torque));
      }
      if(PIN(skipped) > 0.0) {
        printf("<font color='green'># %i saturated cycles were not scored</font>\n", (int)PIN(skipped));
      }
      if(PIN(max_torque) > 0.0 && PIN(peak) > PIN(max_torque)) {
        printf("<font color='red'>the profile peaks over conf0.max_force</font>: raise it to the drive's real torque or lower ids0.max_acc\n");
      }
      printf("done\n");
      PIN(state) = 1.4;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ids_ctx_t *ctx = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  // Anything other than the measuring state disarms it, so a second run always
  // re-enters through the hook below with cleared accumulators. Doing it here
  // rather than in nrt keeps ctx to a single writer.
  if((int)(PIN(state) * 10.0 + 0.5) != 22) {
    ctx->r.armed = 0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(acc_cmd) = 0.0;
      PIN(vel_cmd) = 0.0;
      PIN(acc)     = 0.0;
      PIN(vel)     = 0.0;

      PIN(param) = 0.0;
      PIN(timer) = 0.0;
      // follow the rotor while idle, so the profile starts without a step
      PIN(pos)     = PIN(pos_fb);
      PIN(pos_cmd) = mod(PIN(pos));
      ctx->pos0    = PIN(pos);
      ctx->home    = 0;
      PIN(target)  = ctx->pos0 + PIN(max_pos);

      PIN(pos_bw)     = PIN(pos_bw0) > 0.0 ? PIN(pos_bw0) : 10.0;
      PIN(vel_bw)     = PIN(vel_bw0) > 0.0 ? PIN(vel_bw0) : 100.0;
      PIN(vel_d)      = PIN(vel_d0) > 0.0 ? PIN(vel_d0) : 10.0;
      PINA(params, 0) = PIN(vel_bw);
      PINA(params, 1) = 1.0 / PIN(vel_d);
      PINA(params, 2) = PIN(pos_bw);

      PINA(max_params, 0) = PIN(cur_bw) > 0.0 && PIN(bw_ratio) > 0.0 ? PIN(cur_bw) / PIN(bw_ratio) : 1e6;
      PINA(max_params, 1) = 1.0;

      PIN(min_cost) = 0.0;  // the cost to beat: the last kept score
      ctx->first    = 1;
      ctx->cuts     = 0;
      ctx->kept     = 0;
      ctx->down     = 0;
      PIN(cost)     = 0.0;
      ctx->warm     = 0;
      ctx->n        = 0;
      ctx->t        = 0.0;
      ctx->fb_lp    = 0.0;
      ctx->noise_sq = 0.0;
      ctx->sat_cycle = 0;
      ctx->sat_n     = 0;
      PIN(skipped)   = 0.0;
      ctx->peak     = 0.0;
      ctx->sp_dir   = 1;
      ctx->sp_up    = 1;
      ctx->sp_n     = 0;
      ctx->sp_t     = 0.0;
      ctx->sp_sq = ctx->sp_tq = ctx->sp_dip = 0.0;
      ctx->sp_best  = 0.0;
      ctx->sp_bw    = PIN(vel_bw);
      ctx->sp_kept  = 0;
      ctx->sp_sat   = 0;
      ctx->sp_cap   = 0;
      ctx->sp_capbad = 0;
      PIN(torque_dist) = 0.0;

      if(PIN(en) > 0.0) {
        PIN(state) = PIN(lpf) > 0.0 ? 2.0 : 1.0;
      }
      break;

    case 22: {
      // the same conf0 caps as the gain search
      float r_lim    = PIN(acc_lim) > 0.0 ? MIN(PIN(max_acc), PIN(acc_lim)) : PIN(max_acc);
      float ring_acc = PIN(ring_acc) > 0.0 ? PIN(ring_acc) : r_lim;
      ring_acc       = MAX(PIN(acc_lim) > 0.0 ? MIN(ring_acc, PIN(acc_lim)) : ring_acc, 1.0);
      float r_vmax   = PIN(vel_lim) > 0.0 ? MIN(PIN(max_vel), PIN(vel_lim)) : PIN(max_vel);
      float ring_pos = MAX(PIN(ring_pos), 0.001);
      float dwell    = MAX(PIN(ring_dwell), 0.1);
      // Kicked from standstill, Coulomb friction sticks the load within a
      // cycle or two and the ring is gone before it can be timed (Y, 9 Oct:
      // 2 half periods). Kicked at the end of an acceleration while cruising,
      // friction is a constant force and the ring runs free. Each leg
      // accelerates to ring_vel, cruises for the dwell, then turns round, so
      // the axis goes back and forth between pos0 and pos0 + one leg.
      // Speed locked ripple (cogging, the screw) is forced, so it does not
      // decay, and where it lands on the ring the detector times it instead
      // (Y at 20 rad/s: 15 Hz, 18-22 Hz at 10 and 30). So the legs alternate
      // in pairs between ring_vel and RING_VEL_B x ring_vel, each speed is
      // timed on its own, and a result needs the two to agree within RING_AGREE: a mode stays
      // put when the speed changes, ripple moves with it.
      float r_cv = PIN(ring_vel) > 0.0 ? MIN(PIN(ring_vel), r_vmax / RING_VEL_B) : 0.0;
      int grp    = r_cv > 0.0 ? (ctx->r.rep >> 1) & 1 : 0;
      if(r_cv > 0.0) {
        float r_cb = RING_VEL_B * r_cv;
        ring_pos   = 4.0 * (r_cb * r_cb / ring_acc + r_cb * (dwell + 0.1));  // never reached: the leg turns at the dwell's end
        r_cv       = grp ? r_cb : r_cv;
        r_vmax     = r_cv;
      }
      float f_max  = MIN(1.0 / period / 20.0, 500.0);
      float f_min  = MAX(2.0 * PIN(ring_hp_hz), 3.0 / dwell);

      if(!ctx->r.armed) {
        // start where the rotor is, so the first kick is not a lurch
        ctx->r.pos0      = PIN(pos_fb);
        ctx->r.armed     = 1;
        ctx->r.arrived   = 0;
        ctx->r.home      = 0;
        ctx->r.sign      = 0;
        ctx->r.n_cross   = 0;
        ctx->r.n_dwell   = 0;
        ctx->r.n_half[0] = 0;
        ctx->r.n_half[1] = 0;
        ctx->r.n_ln      = 0;
        ctx->r.rep       = 0;
        ctx->r.t         = 0.0;
        ctx->r.t_last    = 0.0;
        ctx->r.env       = 0.0;
        ctx->r.pk        = 0.0;
        ctx->r.pk_prev   = 0.0;
        ctx->r.spent     = 0;
        ctx->r.a_first   = 0.0;
        ctx->r.a_last    = 0.0;
        ctx->r.sum_half[0] = 0.0;
        ctx->r.sum_half[1] = 0.0;
        ctx->r.sum_ln    = 0.0;
        PIN(pos)       = ctx->r.pos0;
        PIN(vel)       = 0.0;
        PIN(acc)       = 0.0;
        PIN(target)    = ctx->r.pos0 + ring_pos;
        PIN(f_ring)    = 0.0;
        PIN(f_ring_a)  = 0.0;
        PIN(f_ring_b)  = 0.0;
        PIN(zeta_ring) = 0.0;
        PIN(j_lpf)     = 0.0;
        PIN(ring_ok)   = 0.0;
        PIN(ring_n)    = 0.0;
        PIN(ring_amp)  = 0.0;
      }

      // trajectory, the same generator the gain search uses
      PIN(pos) += PIN(vel) * period + PIN(acc) * period * period / 2.0;
      PIN(vel) += PIN(acc) * period;
      float r_to_go = PIN(target) - PIN(pos);
      float r_ttg   = sqrtf(2.0 * ABS(r_to_go) / ring_acc);
      float r_acc   = ring_acc * SIGN(r_to_go);
      float r_vel   = r_acc * r_ttg;
      r_vel         = LIMIT(r_vel, r_vmax);
      r_acc         = (r_vel - PIN(vel)) / period;

      if(r_ttg < period) {
        r_vel    = 0.0;
        r_acc    = 0.0;
        PIN(pos) = PIN(target);
        PIN(vel) = 0.0;
        if(ctx->r.home) {
          PIN(acc_cmd) = 0.0;
          PIN(vel_cmd) = 0.0;
          PIN(state)   = 2.3;
          break;
        }
      }
      // the dwell is timed from the moment the kick ends, at rest on the
      // target or at cruise speed heading for it: the ring is largest then
      int at_end = r_cv > 0.0 ? (ABS(PIN(vel)) >= 0.999 * r_cv && (PIN(vel) > 0.0) == (r_to_go > 0.0)) : (r_ttg < period);
      if(at_end && !ctx->r.arrived && !ctx->r.home) {
        ctx->r.arrived = 1;
        ctx->r.t       = 0.0;
      }

      PIN(acc)     = LIMIT(r_acc, ring_acc);
      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) = PIN(vel) * PIN(ff);
      PIN(acc_cmd) = PIN(acc) * PIN(ff);

      // The high pass runs the whole time so it is settled by the time a dwell
      // starts; only the trigger is gated. What is left of vel_error once the
      // loop's own bandwidth is filtered out is the mechanical mode.
      float hp = LP_HZ(MAX(PIN(ring_hp_hz), 0.5));
      ctx->r.lp  = PIN(vel_error) * hp + ctx->r.lp * (1.0 - hp);
      float v  = PIN(vel_error) - ctx->r.lp;

      PIN(ring_sig) = v;

      ctx->r.t += period;
      float dt = ctx->r.arrived ? ctx->r.t : -1.0;

      if(dt < 0.0) {
        // still moving: the move's own transient is not the mode
        ctx->r.sign    = 0;
        ctx->r.spent   = 0;
        ctx->r.n_cross = 0;
        ctx->r.n_dwell = 0;
        ctx->r.env     = 0.0;
        ctx->r.pk      = 0.0;
        ctx->r.pk_prev = 0.0;
        ctx->r.a_first = 0.0;
        ctx->r.a_last  = 0.0;
      } else {
        ctx->r.pk  = MAX(ctx->r.pk, ABS(v));
        ctx->r.env = MAX(ctx->r.env, ABS(v));

        // Schmitt trigger whose level comes from the previous half cycle's own
        // peak, so it tracks the decay without anyone having to know the rate
        // in advance. A fixed band cannot: by the time a lightly damped 145 Hz
        // mode has been observed long enough to measure its amplitude it has
        // already fallen an order of magnitude below it. The env and min_amp
        // terms are floors, to stop the band chasing the signal into the noise.
        float band = MAX(MAX(0.25 * ctx->r.pk_prev, 0.1 * ctx->r.env), MAX(PIN(ring_min_amp), 0.001));
        int cross  = 0;

        if(ctx->r.sign >= 0 && v < -band) {
          ctx->r.sign = -1;
          cross     = 1;
        } else if(ctx->r.sign <= 0 && v > band) {
          ctx->r.sign = 1;
          cross     = 1;
        }

        if(cross) {
          // the first interval runs from the kick's own end transient, not
          // from a ring edge: a short half period there seeds the mean and
          // the agreement filter then rejects every real one
          if(ctx->r.n_cross > 1) {
            float half = dt - ctx->r.t_last;
            float mean = ctx->r.n_half[grp] > 0 ? ctx->r.sum_half[grp] / (float)ctx->r.n_half[grp] : 0.0;

            // Once the ring has decayed past the gate, latch the count off for
            // the rest of this dwell. Letting it back on lets the noise floor
            // keep triggering, and noise produces far more edges than the mode
            // does -- that is what poisons the mean.
            if(ctx->r.n_dwell > 0 && ctx->r.pk_prev < 0.15 * ctx->r.env) {
              ctx->r.spent = 1;
            }

            // Three filters, in order of how much they matter: the half period
            // has to be resolvable at this rate and to have survived the high
            // pass; it has to agree with the ones already accepted; and the
            // cycle that produced it has to be a real one. The first four seed
            // the mean, so they are held to the top of the ring where nothing
            // else is big enough to trigger.
            float gate = (ctx->r.n_half[grp] < 4 ? 0.5 : 0.15) * ctx->r.env;

            if(!ctx->r.spent && half > 0.5 / f_max && half < 0.5 / f_min && ctx->r.pk_prev >= gate &&
               (ctx->r.n_half[grp] < 4 || (half > 0.75 * mean && half < 1.35 * mean))) {
              ctx->r.sum_half[grp] += half;
              ctx->r.n_half[grp]++;
              ctx->r.n_dwell++;
              ctx->r.a_last = ctx->r.pk;
              if(ctx->r.n_dwell == 1) {
                ctx->r.a_first = ctx->r.pk;
              }
            }
          }
          ctx->r.t_last  = dt;
          ctx->r.n_cross++;
          ctx->r.pk_prev = ctx->r.pk;
          ctx->r.pk      = 0.0;
        }

        PIN(ring_amp) = ctx->r.env;
      }

      if(dt >= dwell) {
        // log decrement for this dwell: the envelope falls as exp(-zeta*w*t),
        // so ln(first / last) across n half periods is pi * n * zeta
        if(ctx->r.n_dwell >= 3 && ctx->r.a_last > 0.0 && ctx->r.a_first > ctx->r.a_last) {
          ctx->r.sum_ln += logf(ctx->r.a_first / ctx->r.a_last) / (M_PI * (float)(ctx->r.n_dwell - 1));
          ctx->r.n_ln++;
        }

        ctx->r.rep++;
        ctx->r.arrived = 0;
        ctx->r.t       = 0.0;
        ctx->r.t_last  = 0.0;
        ctx->r.sign    = 0;
        ctx->r.spent   = 0;
        ctx->r.pk_prev = 0.0;
        ctx->r.n_cross = 0;
        ctx->r.n_dwell = 0;
        ctx->r.env     = 0.0;
        ctx->r.pk      = 0.0;
        ctx->r.a_first = 0.0;
        ctx->r.a_last  = 0.0;
        // alternate, so the axis rocks about where it started instead of
        // walking away one kick at a time
        PIN(target) = ctx->r.pos0 + ((ctx->r.rep & 1) ? -ring_pos : ring_pos);

        if((float)ctx->r.rep >= MAX(PIN(ring_reps), 1.0)) {
          // back to where it started, then 2.3
          ctx->r.home  = 1;
          PIN(target)  = ctx->r.pos0;
          uint16_t n_all = ctx->r.n_half[0] + ctx->r.n_half[1];
          float sum_all  = ctx->r.sum_half[0] + ctx->r.sum_half[1];
          PIN(ring_n)    = (float)n_all;
          for(int g = 0; g < 2; g++) {
            if(ctx->r.n_half[g] >= 3 && ctx->r.sum_half[g] > 0.0) {
              float f = 0.5 * (float)ctx->r.n_half[g] / ctx->r.sum_half[g];
              if(g) {
                PIN(f_ring_b) = f;
              } else {
                PIN(f_ring_a) = f;
              }
            }
          }

          if(n_all >= 6 && sum_all > 0.0) {
            PIN(f_ring) = 0.5 * (float)n_all / sum_all;
          }
          if(PIN(ring_vel) > 0.0 && ABS(PIN(f_ring_a) - PIN(f_ring_b)) > RING_AGREE * 0.5 * (PIN(f_ring_a) + PIN(f_ring_b))) {
            // the speeds disagree, or one found nothing: speed ripple, not a mode
            PIN(f_ring) = 0.0;
          }
          if(ctx->r.n_ln > 0) {
            PIN(zeta_ring) = ctx->r.sum_ln / (float)ctx->r.n_ln;
          }

          if(PIN(f_ring) < f_min || PIN(f_ring) > f_max) {
            // outside what this rate and this high pass can resolve: nothing
            // was measured, and half an answer here is worse than none
            PIN(f_ring) = 0.0;
          } else {
            // timed in closed loop at the configured gains, the speed loop
            // holds the motor, and the ring sits at the anti resonance (a
            // two mass sim at vel_bw 300-500 puts it at 0.79-0.91 of it), not at
            // the free resonance: f_ring is the corner as it stands
            PIN(j_lpf)   = PIN(f_ring);
            PIN(ring_ok) = 1.0;
          }
        }
      }
      break;
    }

    case 15: {
      // speed mode: ramp, settle, speed noise, load step, release; see top
      float p_acc = PIN(acc_lim) > 0.0 ? MIN(PIN(max_acc), PIN(acc_lim)) : PIN(max_acc);
      float p_vel = PIN(vel_lim) > 0.0 ? MIN(PIN(max_vel), PIN(vel_lim)) : PIN(max_vel);
      p_acc       = MAX(p_acc, 1.0);
      float v_t   = ctx->home ? 0.0 : ctx->sp_dir * p_vel;
      float dv    = v_t - PIN(vel);
      PIN(acc)    = ABS(dv) > p_acc * period ? SIGN(dv) * p_acc : dv / period;
      PIN(vel) += PIN(acc) * period;
      PIN(pos) += PIN(vel) * period;
      PIN(pos_cmd)     = mod(PIN(pos));
      PIN(vel_cmd)     = PIN(vel) * PIN(ff);
      PIN(acc_cmd)     = PIN(acc) * PIN(ff);
      PIN(torque_dist) = 0.0;
      if(ctx->home) {
        if(ABS(PIN(vel)) < 1e-3) {
          PIN(vel) = PIN(vel_cmd) = PIN(acc_cmd) = 0.0;
          PIN(state) = 1.3;
        }
        break;
      }
      float t_ramp = 2.0 * p_vel / p_acc;  // from the other direction
      float t0 = t_ramp + SP_SETTLE, t1 = t0 + SP_NOISE, t2 = t1 + SP_STEP, t3 = t2 + SP_STEP;
      ctx->sp_t += period;
      if(ctx->sp_t > t_ramp && (PIN(sat) > 0.0 || (PIN(max_torque) > 0.0 && ABS(PIN(torque)) >= 0.99 * PIN(max_torque)))) {
        ctx->sp_sat = 1;
      }
      ctx->peak = MAX(ctx->peak, ABS(PIN(torque)));
      if(ctx->sp_t > t0 && ctx->sp_t <= t1) {
        ctx->sp_sq += PIN(vel_error) * PIN(vel_error) * period;
        ctx->sp_tq += period;
      } else if(ctx->sp_t > t1 && ctx->sp_t <= t2) {
        // load against the motion: the rotor slows, vel_error (cmd - fb) goes
        // the direction's way
        PIN(torque_dist) = -ctx->sp_dir * PIN(dist) * PIN(max_torque);
        ctx->sp_dip      = MAX(ctx->sp_dip, ctx->sp_dir * PIN(vel_error));
      }
      if(ctx->sp_t > t3) {
        ctx->sp_t   = 0.0;
        ctx->sp_dir = -ctx->sp_dir;
        if(++ctx->sp_n >= SP_REP) {
          // score: speed noise and dip over rep cycles
          float nz      = ctx->sp_tq > 0.0 ? sqrtf(ctx->sp_sq / ctx->sp_tq) : 0.0;
          PIN(sp_noise) = nz;
          PIN(dip)      = ctx->sp_dip;
          int sat       = ctx->sp_sat;
          int bad       = sat || ctx->sp_dip > PIN(dip_max) * p_vel;
          PIN(peak)     = ctx->peak;
          ctx->peak     = 0.0;
          ctx->sp_n     = 0;
          ctx->sp_sq = ctx->sp_tq = ctx->sp_dip = 0.0;
          ctx->sp_sat = 0;
          float bw    = PINA(params, 0);
          // saturation is too much gain (the loop oscillates) or a profile the
          // drive cannot follow, never a soft loop: it lowers vel_bw or stops
          // the raise. Only a dip over dip_max raises it.
          if(ctx->sp_kept == 0) {  // the start value: the noise reference
            if(sat) {  // not a usable reference: cut and score the start again
              bw /= 1.0 + PIN(step);
              ctx->sp_bw     = MAX(bw, SP_MIN_BW);
              ctx->sp_up     = 0;
              ctx->sp_capbad = 1;  // cleared once a start value scores clean
            } else {
              ctx->sp_capbad = 0;
              ctx->sp_best  = nz;
              ctx->sp_bw    = bw;
              ctx->sp_kept  = 1;
              ctx->sp_up    = bad;  // too soft already: up until the dip passes
              ctx->sp_dmin  = PIN(dip);
              ctx->sp_bwmin = bw;
              bw            = bad ? bw * (1.0 + PIN(step)) : bw / (1.0 + PIN(step));
            }
          } else if(ctx->sp_up) {
            if(sat || PIN(dip) > ctx->sp_dmin * (1.0 + 3.0 * PIN(kg))) {
              // more gain, more dip or saturation: the loop oscillates (on
              // an observer, whose lag sets the margin), it is not soft.
              // Keep the vel_bw with the smallest dip.
              ctx->sp_bw     = ctx->sp_bwmin;
              ctx->sp_capbad = 1;
              bw             = -1.0;
            } else if(bad) {
              if(PIN(dip) < ctx->sp_dmin) {
                ctx->sp_dmin  = PIN(dip);
                ctx->sp_bwmin = bw;
              }
              ctx->sp_bw = bw;
              if(ctx->sp_cap) {  // scored at the cap and still soft
                ctx->sp_capbad = 1;
                bw             = -1.0;
              } else {
                bw *= 1.0 + PIN(step);
              }
            } else {
              ctx->sp_bw = bw;
              bw         = -1.0;  // the first that takes the step
            }
          } else {
            if(!bad && nz <= ctx->sp_best * (1.0 + 3.0 * PIN(kg))) {  // takes the step with less gain
              ctx->sp_bw = bw;
              ctx->sp_kept++;
              bw /= 1.0 + PIN(step);
            } else {
              bw = -1.0;  // done: the last kept
            }
          }
          if(ctx->sp_up && bw > PINA(max_params, 0)) {  // the cap: score it once more there
            bw          = PINA(max_params, 0);
            ctx->sp_cap = 1;
          }
          if(bw < 0.0 || (!ctx->sp_up && bw < SP_MIN_BW)) {
            PINA(params, 0) = ctx->sp_bw;
            ctx->home       = 1;
          } else {
            PINA(params, 0) = bw;
          }
        }
      }
      PIN(vel_bw) = PINA(params, 0);
      PIN(vel_d)  = 1.0 / PINA(params, 1);
      PIN(pos_bw) = PINA(params, 2);
      break;
    }

    case 12: {
      // the profile stays inside the drive's own limits, conf0.max_acc and max_vel
      float p_acc = PIN(acc_lim) > 0.0 ? MIN(PIN(max_acc), PIN(acc_lim)) : PIN(max_acc);
      float p_vel = PIN(vel_lim) > 0.0 ? MIN(PIN(max_vel), PIN(vel_lim)) : PIN(max_vel);
      PIN(pos) += PIN(vel) * period + PIN(acc) * period * period / 2.0;
      PIN(vel) += PIN(acc) * period;
      float to_go      = PIN(target) - PIN(pos);
      float time_to_go = sqrtf(2.0 * ABS(to_go) / p_acc);
      float acc        = p_acc * SIGN(to_go);
      float vel        = acc * time_to_go;
      vel              = LIMIT(vel, p_vel);
      acc              = (vel - PIN(vel)) / period;

      if(time_to_go < period) {
        time_to_go = 0.0;
        to_go      = 0.0;
        vel        = 0.0;
        acc        = 0.0;
        PIN(pos)   = PIN(target);
        PIN(vel)   = 0.0;
        PIN(acc)   = 0.0;
      }

      PIN(acc) = LIMIT(acc, p_acc);

      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) = PIN(vel) * PIN(ff);
      PIN(acc_cmd) = PIN(acc) * PIN(ff);

      if(ctx->home) {
        // back to where the search started, so repeated runs do not walk the
        // axis toward a stroke end
        if(to_go == 0.0) {
          PIN(acc_cmd) = 0.0;
          PIN(vel_cmd) = 0.0;
          PIN(state)   = 1.3;
        }
        break;
      }

      PIN(cost) += ABS(PIN(pos_error)) * PIN(kp) * period;
      PIN(cost) += PIN(pos_error) * PIN(pos_error) * PIN(ks) * period;
      PIN(cost) += PIN(vel_error) * PIN(vel_error) * PIN(kv) * period;

      ctx->fb_lp += (PIN(fb_torque) - ctx->fb_lp) * LIMIT(2.0 * M_PI * NOISE_HZ * period, 1.0);
      ctx->noise_sq += (PIN(fb_torque) - ctx->fb_lp) * (PIN(fb_torque) - ctx->fb_lp) * period;
      ctx->peak = MAX(ctx->peak, ABS(PIN(torque)));
      ctx->t += period;
      if(PIN(sat) > 0.0 || (PIN(max_torque) > 0.0 && ABS(PIN(torque)) >= 0.99 * PIN(max_torque))) {
        ctx->sat_cycle = 1;
      }

      PIN(timer) += period;
      if(PIN(timer) < (ABS(PIN(max_pos) - PIN(min_pos)) / p_vel + 2.0 * p_vel / p_acc)) {
        PIN(target) = ctx->pos0 + PIN(max_pos);
      } else {
        PIN(target) = ctx->pos0 + PIN(min_pos);
      }
      if(PIN(timer) > 2.0 * (ABS(PIN(max_pos) - PIN(min_pos)) / p_vel + 2.0 * p_vel / p_acc)) {
        PIN(timer) = 0.0;
      }

      int score = 0, sat_fail = 0;
      if(PIN(timer) == 0.0) {
        int sat = ctx->sat_cycle;
        ctx->sat_cycle = 0;
        ctx->sat_n     = sat ? ctx->sat_n + 1 : 0;
        if(sat) {
          PIN(skipped)++;
        }
        if(!ctx->warm) {  // warm-up: not scored, runs on while pid saturates
          if(!sat || ctx->sat_n >= WARM_MAX) {
            ctx->warm  = 1;
            ctx->sat_n = 0;
          }
          PIN(cost) = 0.0;
          ctx->t = ctx->noise_sq = ctx->peak = 0.0;
        } else if(sat && ctx->sat_n < SAT_MAX) {  // drop it, same gains again
          ctx->n    = 0;
          PIN(cost) = 0.0;
          ctx->t = ctx->noise_sq = ctx->peak = 0.0;
        } else if(sat) {  // saturates every time: these gains fail
          ctx->sat_n = 0;
          sat_fail   = 1;
          score      = 1;
          ctx->n     = MAX(ctx->n, 1);
        } else if(++ctx->n >= MAX(PIN(rep), 1.0)) {
          score = 1;
        }
      }

      if(score) {
        PIN(cost) /= ctx->n;
        PIN(noise) = ctx->t > 0.0 ? sqrtf(ctx->noise_sq / ctx->t) : 0.0;
        PIN(peak)  = ctx->peak;
        int noisy  = sat_fail || (PIN(max_torque) > 0.0 && PIN(noise) > PIN(fb_max) * PIN(max_torque));
        ctx->n = 0;
        ctx->t = ctx->noise_sq = ctx->peak = 0.0;

        PINA(max_params, 2) = PINA(params, 0) * 2.0;

        int k = (int)PIN(param);
        if(ctx->down) {
          // down: keep cutting while the cost stays within kg of the start's
          float lo = 4.0 * PINA(params, 2);
          if(noisy || PIN(cost) > ctx->cost0 * (1.0 + PIN(kg)) || PINA(params, k) / (1.0 + PIN(step)) < lo) {
            if(noisy || PIN(cost) > ctx->cost0 * (1.0 + PIN(kg))) {
              PINA(params, k) *= 1.0 + PIN(step);  // back to the last kept value
            }
            ctx->down  = 0;
            ctx->first = 1;
            ctx->kept  = 0;
            PIN(param)++;
          } else {
            PIN(min_cost) = PIN(cost);
            PINA(params, k) /= 1.0 + PIN(step);
          }
        } else if(PINA(params, k) > PINA(max_params, k)) {
          PINA(params, k) = PINA(max_params, k);
          ctx->first      = 1;
          ctx->kept       = 0;
          PIN(param)++;
        } else if(ctx->first && sat_fail && ++ctx->cuts < CUT_MAX) {
          PINA(params, k) *= PIN(kd);  // the start value saturates: cut it, try again
        } else if(ctx->first && noisy) {  // noisy at the start value: cut it
          PINA(params, k) *= PIN(kd);
          ctx->cuts = 0;
          ctx->kept = 0;
          PIN(param)++;
        } else if(!ctx->first && (noisy || PIN(cost) > PIN(min_cost) * (1.0 - PIN(kg)))) {
          PINA(params, k) /= 1.0 + PIN(step);  // back to the last kept value
          if(k == 0 && ctx->kept == 1 && PINA(params, 0) / (1.0 + PIN(step)) >= 4.0 * PINA(params, 2)) {
            // vel_bw's first raise failed: search down from the start value
            ctx->down  = 1;
            ctx->cost0 = PIN(min_cost);
            PINA(params, k) /= 1.0 + PIN(step);
          } else {
            ctx->first = 1;
            ctx->cuts  = 0;
            ctx->kept  = 0;
            PIN(param)++;
          }
        } else {
          PIN(min_cost) = PIN(cost);
          ctx->first    = 0;
          ctx->kept++;
          PINA(params, k) *= 1.0 + PIN(step);
        }

        PIN(cost) = 0.0;
      }

      PIN(pos_bw) = PINA(params, 2);
      PIN(vel_bw) = PINA(params, 0);
      PIN(vel_d)  = 1.0 / PINA(params, 1);

      if(PIN(param) > 2.0) {
        PIN(param)  = 0.0;
        ctx->home   = 1;
        PIN(target) = ctx->pos0;
      }
      break;
    }
  }
}


hal_comp_t ids_comp_struct = {
    .name      = "ids",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct ids_ctx_t),
    .pin_count = sizeof(struct ids_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};