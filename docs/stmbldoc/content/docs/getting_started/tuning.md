---
title: "Tuning"
weight: 5
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# Parameter Tuning

STMBL can measure most motor and axis parameters itself. Each measurement is an identification component (`idpmsm`, `idacim`, `iddc`, `idtune`, `idm`, `ids`, `iddac`) loaded by an `id_*` config template. It takes over the power stage, runs its test and prints the results as `conf0.* = ...` (or `hv0.*`, `acim_*`, `vf0.*`) lines in green for you to append to the config.

## Requirements

* [Supply](/docs/supply.md)
* [Servoterm](/docs/getting_started/servoterm.md)
* [Feedback](/docs/getting_started/feedback.md)
* [Motor](/docs/getting_started/motor.md)

## Before You Start

### Which stages for which motor

| Motor | Stages, in this order |
|---|---|
| DC motor | [id_dc](#dc-motor), then [id_mot](#mechanical-parameters-id_mot), [id_sys](#load-inertia-id_sys), [id_pid](#control-loop-id_pid) |
| PMSM with encoder | [id_pmsm](#electrical-parameters-id_pmsm), [id_tune](#advance-and-dead-time-id_tune), [id_mot](#mechanical-parameters-id_mot), [id_sys](#load-inertia-id_sys) (optional), [id_brk](#breakaway-torque-id_brk) (optional), [id_lpf or id_frf](#coupling-resonance-id_lpf) (optional), [id_pid](#control-loop-id_pid) |
| Induction motor (ACIM) | [id_acim](#standstill-test-id_acim) (standstill, optional rotating test), then run it with [V/f](#run-it-with-vf), [V/f with an encoder](#run-it-with-vf-and-an-encoder) or [FOC](#run-it-with-foc); with an encoder also id_mot, id_sys, id_pid ([speed mode](#speed-mode-spindle) for a spindle) |
| Sensorless PMSM / ACIM | identify as above, then [obs_check, sl_pmsm, sl_acim, id_sys_sl](#sensorless) |

On an STMBL v5 HV board also set the [overcurrent trip threshold](#overcurrent-trip-threshold-hv0dac).

### How every stage is run

1. Have a working base config saved (motor template, feedback template, `link misc`, your `conf0.*` limits and all results of the earlier stages), then type `reset`.
2. With the drive disabled, type `link id_xxx` in Servoterm, then `stop` and `start`: a component loaded at runtime only joins the realtime loop at `start` (without it, enabling does nothing). Optionally change the component's test pins now (e.g. `idpmsm0.test_cur = 4`).
3. Type `fault0.en = 1` (same as `enable` or the Enable button). Follow the green text: some stages ask you to type a state, e.g. `iddc0.state = 1.2`.
4. Copy the printed `name = value` lines (most are marked `# append to config`). Lines printed in red, or after a "failed" message, are not measurements.
5. Type `fault0.en = 0`, append the lines to the config (config editor or `appendconf <line>`), save with `flashsaveconf` and type `reset` before the next stage.

{{% hint info %}}
**Why reset between stages:** HAL cannot unload a component, so the `id_*` component and its links stay in place until a reset. Disabling also resets most result pins, so the numbers are only kept in the printed text. Typing `link id_xxx` at the console (followed by `stop`, `start`) keeps it out of the saved config, and is required for `id_tune`. A runtime link is not flattened like the boot config (the boot runs `relink`). This only matters for a chain of links, such as `conf0.com_fb_polecount = conf0.polecount` in `id_pmsm`, so the `id_pmsm` template runs `relink` itself.
{{% /hint %}}

### Safety

{{% hint danger %}}
- Every test current (`idpmsm0.test_cur`, `idacim0.test_cur`, `iddc0.test_cur`, `idtune0.test_cur` and `idtune0.step_cur`) must be below `conf0.max_ac_cur` (default 10 A).
- Take the load off the shaft for all electrical tests and for `id_mot`. Most tests turn the motor: `id_tune` up to 200 rad/s at its defaults, the ACIM rotating test up to plate speed with an encoder (`idacim0.rot_vmax`, default 1.0).
- `id_mot`, `id_sys` and `id_pid` move the axis between `min_pos` and `max_pos`, relative to where the axis stands when the test starts: -20 to +20 rad (about 3.2 turns each way) for `id_mot` / `id_sys`, -10 to +10 rad for `id_pid`. Make sure there is room, or reduce these pins first. There is no step at the start, the first move goes toward `max_pos`, and at the end the axis drives back to where it started. `id_lpf` moves the axis back and forth at `ids0.ring_vel` (10 rad/s) and 1.5 x that, up to about 15 rad from where it stands at the defaults, and also ends where it started. `id_frf` creeps back and forth at `frf0.creep_vel` (3 rad/s) between `frf0.min_pos` and `frf0.max_pos` (-10 / +10 rad) and ends where it started. `id_pid` in speed mode runs the motor to +-`ids0.max_vel` (100 rad/s, capped at `conf0.max_vel`) in both directions.
- To stop any test: `fault0.en = 0` (or `disable` / the Disable button). Every id component aborts and returns to state 0 (a finished `id_dac` result stays until reset).
{{% /hint %}}

## Overcurrent Trip Threshold (hv0.dac)

The v5 HV board has a hardware overcurrent comparator, set by `hv0.dac`.

{{% hint warning %}}
The default `hv0.dac = 2500` stands for about 350 A, so the comparator is effectively off and only the F3 software trip (`io0.oc_lim`) protects the bridge. Set a real level only on a board with the 1 nF caps on the current sense lines: without them switching ringing trips the comparator far below the set current. Then the default is a ceiling, not a setting: set it per axis as below.
{{% /hint %}}

Two ways to set it:

**ocdac (fixed map):** add to the config

```python
link ocdac
ocdac0.cur = 20 # trip current [A peak]
```

`ocdac` converts the current with a linear map (`ocdac0.dac0`, `ocdac0.k`) fitted on one bench board. Check it on your hardware before relying on it; `ocdac0.cur_out` shows the current the clamped dac really stands for. See [ocdac](/docs/hal_components/ocdac.md).

**id_dac (measure it on your board):** a one shot search that prints `hv0.dac` for a trip at `iddac0.cur`.

1. Raise the current limit for the run: `conf0.max_ac_cur` must be more than 1.2 x `iddac0.cur` (1.05 x in mode 0), e.g. `conf0.max_ac_cur = 25` for the template's 20 A. `iddac0.cur` must be over 1 A and at most `iddac0.cur_max` (25 A). The template ramps at `iddac0.ramp = 200` A/s: a slow ramp heats ipm0's junction model at the fixed angle and clamps the current before high trips.
2. Type:

```python
link id_dac
stop
start
iddac0.cur = 20 # wanted trip current [A peak]
iddac0.mode = 1 # template default: loaded or blocked axis, current on the rotor d axis, no torque
fault0.en = 1
```

3. Expect a few dozen comparator trips. Mode 1 tests only the comparator of one phase per run (`iddac0.trip_phase`); turn the axis by hand and rerun to check the others. `iddac0.mode = 0` (free, unloaded axis) tests all three angles; the rotor aligns to them.
4. Append the printed `hv0.dac = ...` line and `reset` (a finished run does not repeat until reset; this also restores your `conf0.max_ac_cur`).

See [iddac](/docs/hal_components/iddac.md).

## DC Motor

Takes about 10 seconds plus your typing. Test current `iddc0.test_cur` (default 2 A), speed `iddc0.test_vel` (default 50 rad/s).

Base config:

```python
link pid
link dc
link <your_feedback_type>
link misc
```

1. `reset`, then type:

```python
link id_dc
stop
start
fault0.en = 1
```

2. The console asks you to **block the rotor**. Then type `iddc0.state = 1.2`. It measures r (2 s) and l (1 s) and prints `conf0.r` and `conf0.l`.
3. The console asks you to **unblock the rotor**. Type `iddc0.state = 2.2`. The motor runs for 5 s under a speed loop and prints `conf0.psi` and, if reversed, `conf0.out_rev = 1`.

| Append | Notes |
|---|---|
| `conf0.r`, `conf0.l` | `l` is a rough value (per tick dV/dI method) |
| `conf0.psi` | torque constant [Nm/A = Vs/rad] |
| `conf0.out_rev = 1` | only if printed |

{{% hint warning %}}
- `iddc` is marked untested (no DC motor was available).
- The speed test ramps its current slowly, so the motor may not reach `test_vel`; `psi` is averaged at whatever speed it reaches.
{{% /hint %}}

Continue with [Mechanical Parameters](#mechanical-parameters-id_mot). Details: [iddc](/docs/hal_components/iddc.md).

## PMSM with Encoder

Base config (feedback set up and working, `conf0.max_ac_cur` set):

```python
link pid
link pmsm
link <your_feedback_type>
link misc
```

### Electrical Parameters (id_pmsm)

Measures r and the dead time curve, Ld/Lq, pole pairs, direction, commutation offset and psi. Defaults: `idpmsm0.test_cur = 6` A, `idpmsm0.test_vel = 50`. The r test alone takes about 50 s; the other tests add a few seconds each.

1. Load off the shaft, drive disabled, `reset`.
2. Optional: `idpmsm0.test_cur = ...` (below `conf0.max_ac_cur`), `idpmsm0.r_known = ...` (measured per phase r = line to line / 2, cold, at the drive end of the motor cable; then only the dead time curve is fitted and `conf0.r` is printed as this value; recommended for runs at 12 A, see [idpmsm](/docs/hal_components/idpmsm.md)). `idpmsm0.auto_step` (default 4.2 = all steps without prompt) can be lowered to stop before a step and wait for `idpmsm0.state = 1.2` / `2.2` / `3.2`.
3. Type:

```python
link id_pmsm
stop
start
fault0.en = 1
```

4. What the motor does:
    - **r and dead time:** d current dwells at four angles, the rotor aligns and stays. Then a short sine injection for Ld and Lq.
    - **pole pairs / direction:** the field turns open loop and drags the rotor for 4 s.
    - **commutation offset:** 2 s hold on the d axis.
    - **psi:** speed loop to `test_vel / 2` and `test_vel` (mechanical rad/s), 2 s each, then the bridge switches off and the rotor coasts while the back emf is measured.
5. Append the printed lines, then `fault0.en = 0`, save, `reset`.

| Append | Notes |
|---|---|
| `conf0.r` | phase resistance |
| `conf0.l`, `conf0.lq` | Ld and Lq; only printed if the injection worked |
| `hv0.drop_k`, `hv0.drop_knee` | dead time compensation curve |
| `conf0.polecount` | pole pairs |
| `conf0.mot_fb_offset` | commutation offset |
| `conf0.com_fb_offset` | only printed with a commutation track (e.g. halls, encf) |
| `conf0.out_rev = 1` | only if printed |
| `conf0.psi` | from the back emf map of the coast |

{{% hint warning %}}
- `id_pmsm` forces `hv0.drop_k = 0` during the test (the r fit needs the uncompensated loss).
- A red warning before the `conf0.polecount` line means `fb_switch0.mot_state_fb` was not 3 after the pole pair test: the offsets are then relative to power-up. Index the encoder first.
- `test_vel` means two things: electrical rad/s in the pole pair test, mechanical rad/s in the psi test. If the pole pair test fails, lower `test_vel` or raise `test_cur`.
- After a failure the component idles with the bridge off until you disable.
- A negative `test_vel` runs the psi test in the other direction.
{{% /hint %}}

Details: [idpmsm](/docs/hal_components/idpmsm.md).

### Advance and Dead Time (id_tune)

Finds `hv0.adv` (commutation advance) and, only with `hv0.drop_knee = 0`, `hv0.drop_k`. Needs all `id_pmsm` results in the config; a correct `conf0.lq` matters.

1. Load off the shaft: the rotor runs at up to 2 x `idtune0.test_vel` (200 rad/s at the default). `reset`.
2. Type at the console (**never put `link id_tune` in the saved config**: the boot `relink` would chain `idtune0.drop_k_cfg` to its own output):

```python
link id_tune
stop
start
fault0.en = 1
```

3. With `drop_knee = 0`: about 3.5 s of d current square wave (no torque). Then the motor runs speed dwells at 50, 100, 150, 200 rad/s (defaults), repeated until `adv` settles.
4. Append, then `fault0.en = 0`, save, `reset`.

| Append | Notes |
|---|---|
| `hv0.adv` | commutation advance [s] |
| `hv0.drop_k` | only printed with `drop_knee = 0`; otherwise your `id_pmsm` value is kept (a comment says so) |

A failure message pointing at `conf0.polecount`, `conf0.mot_fb_offset` and `out_rev` means the motor stalled; one pointing at `conf0.lq` and `conf0.r` means `adv` did not converge. A negative `idtune0.test_vel` runs the speed dwells in the other direction. Details: [idtune](/docs/hal_components/idtune.md).

### Mechanical Parameters (id_mot)

For all motor types once the motor runs with working commutation. Identifies inertia, damping, friction and a constant torque offset (e.g. gravity). Takes about two minutes.

1. Bare motor, no load. The axis moves between `idm0.min_pos` and `idm0.max_pos` (-20 / +20 rad) at up to `idm0.max_vel` (50 rad/s) and `idm0.max_acc` (250 rad/s^2). Leave `idm0.vel_offset` at 0 (it is only for the sensorless `id_sys_sl`).
2. `reset`, then type:

```python
link id_mot
stop
start
fault0.en = 1
```

3. It starts by itself (`idm0.auto_step` 1.4): five growing ramp rounds, then 45 s of adaptation at full speed followed by speed plateaus at 0.1, 0.25, 0.5 and 1.0 x `max_vel`. A least-squares fit over the plateaus gives `f`, `d` and `o`; `idm0.fit_n` and `idm0.fit_rms` show how many plateau bins it used and how well they fit. When the plateaus do not allow a fit (too short a stroke, or no solution), the console prints `f, d, o not fitted` and leaves them as adapted.
4. Append, `fault0.en = 0`, save, `reset`.

| Append | Notes |
|---|---|
| `conf0.j` | motor inertia [kg m^2] |
| `conf0.d` | viscous damping |
| `conf0.f` | coulomb friction |
| `conf0.o` | constant torque offset |

Details: [idm](/docs/hal_components/idm.md).

### Load Inertia (id_sys)

Optional, with the load coupled. Same component and procedure as `id_mot`, but `conf0.j` must already be in the config; `id_sys` links `id_mot` itself.

```python
link id_sys
stop
start
fault0.en = 1
```

Append the printed `conf0.j_sys`, `conf0.o`, `conf0.d`, `conf0.f` (replacing the earlier d, f, o, which were without load). Friction and damping drift with the lubrication state, so run it on a warmed up, lubricated axis.

### Breakaway Torque (id_brk)

A diagnostic, nothing in the drive needs its result: `id_brk` runs it with the drive's own gains and `conf0.o` in the feedforward (after a lubrication change, say, or when an axis feels sticky); `idm0.brk = 1` before `id_sys` makes `id_sys` end with it. `id_sys` measures the friction while the axis slides. What it takes to get the axis moving from rest can be more or less than that, and on a vertical axis it differs by direction. With the trajectory holding the axis where it stands, idm ramps an extra torque (`idm0.brk_rate`, 3 Nm/s) until the axis has moved `idm0.brk_win` (0.002 rad), positive and then negative, and prints the static friction (half the difference) and the static load (the mean, gravity on a vertical axis) next to `conf0.f` and `conf0.o`. The axis moves 0.002 rad each way and comes back. About 10 s on a vertical axis. Holding torque at standstill keeps the current in one bridge leg and heats it fast; if the drive derates or limits the current during a ramp (`fault0.scale` below 1, or the measured current short of the command) the test stops with `idm0.brk_res = -2` instead of reading a torque the motor never made. Let the drive cool and run it again.

```python
link id_brk
stop
start
fault0.en = 1
```

At rest the loop can hold anywhere between the static load minus and plus the static friction, so a stop ends at whatever torque the move left: the static friction is the width of that band, and the static load is where `conf0.o` should sit (the console says so when they differ by more than the static friction). On a vertical axis whose load outweighs the friction both readings have the same sign; that does not change the result. With a disturbance observer, `dob0.max` has to exceed the static load plus the static friction, and `dob0.k` has to be 0 for the run (it would cancel the ramp). The brake has to be released. The velocity integrator is parked during the ramps, since it winds on the position error stiction leaves behind. Nothing from this test goes into the config by itself. On a screw axis the readings can depend on the direction the axis came from and scatter between runs (on the Z bench the down breakaway read 2.9, 2.05, 1.88 and -0.2 Nm); the standstill hold below therefore uses `conf0.o`, not these values.

Details: [idm](/docs/hal_components/idm.md).

### Coupling Resonance (id_lpf)

Optional, with the load coupled through a compliant coupling or belt. Needs `conf0.j` and `conf0.j_sys`. Run it before `id_pid`: `conf0.j_lpf` changes the plant the gains are tuned against. The axis goes back and forth `ids0.ring_reps` (8) times, each leg accelerating to a cruise speed and cruising for `ids0.ring_dwell` (1 s) while `ids` times the ring in `vel_error`. The legs alternate in pairs between `ids0.ring_vel` (10 rad/s) and 1.5 x `ring_vel`, so the axis needs about 1.5 x `ring_vel` x `ring_dwell` (15 rad) of room from where it stands; it ends where it started. Kicked while cruising, friction does not stop the ring the way it does from standstill. The ring is timed at each speed separately and both must agree within 15 %: a coupling resonance stays put when the speed changes, while ripple locked to the speed (cogging, a screw) moves with it. `conf0.j_lpf` is set to the measured ring frequency. `ids0.ring_vel = 0` instead twitches the axis `ids0.ring_pos` (0.2 rad) to either side from standstill.

```python
link id_lpf
stop
start
fault0.en = 1
```

Append the printed `conf0.j_lpf` line. If the ring died too fast to time (a well damped coupling), the console prints `conf0.j_lpf = 0` itself; append that, since the conf template sets 100 Hz. If it reports that the two speeds disagree, speed ripple was timed at one of them: rerun with another `ids0.ring_vel`; `conf0.j_lpf` is left as it is. If it reports no ring found, either raise `ids0.ring_acc` or `ids0.ring_vel` and rerun, or set `conf0.j_lpf = 0` by hand. `j_lpf` only pays when `conf0.vel_bw` comes near 2 pi x `j_lpf`. Scope wave 1 shows `ids0.ring_sig`, the signal the detector sees. Details: [ids](/docs/hal_components/ids.md).

**id_frf (stepped-sine FRF)** replaces the ring down in use: it injects a torque sine of `frf0.amp` (1 Nm) at 40 log-spaced points from 5 to 60 Hz with the loop running, fits a two-mass model to the whole curve and prints `conf0.j_lpf` as the fitted anti-resonance. The axis creeps back and forth at `frf0.creep_vel` (3 rad/s) between `frf0.min_pos` and `frf0.max_pos` (-10 / +10 rad) about where it starts and measures only while cruising; `(max_pos - min_pos) / creep_vel` has to exceed about 3 s for the 5 Hz point. The template sets `conf0.max_pos_error = 0` for the run.

```python
link id_frf
stop
start
fault0.en = 1
```

It prints a table per point (`|G|`, `gj = |G| w`, the open loop `L`), then the fit (anti-resonance, resonance, `j_mot`, `j_load`, delay) and, when `|L|` falls below 1 in the band, the crossover and phase margin. Append the printed `conf0.j_lpf` line (`conf0.j_lpf = 0` when no resonance is below `frf0.f_end`). "poor fit" or an anti-resonance at the low end of the band: look at the table, change `frf0.f_start` / `f_end` and rerun. The sweep starts on enable (the template links `frf0.start = fault0.en_pid`) and ends with the axis back where it started, holding until you disable. Disabling the drive stops a sweep where it is and clears a finished one. Only one `frf` can run per drive. Details: [frf](/docs/hal_components/frf.md).

### Control Loop (id_pid)

Tunes `pos_bw`, `vel_bw`, `vel_d` of the pid loop by moving the axis and searching for the lowest tracking error. Needs `conf0.j` (and `j_sys`, `d`, `f`, `o`, `j_lpf`), `conf0.cur_bw` and `conf0.max_force` in the config. The search starts from the gains already in the config (10 / 100 / 10 where they are 0).

1. The axis moves between `ids0.min_pos` and `ids0.max_pos` (-10 / +10 rad) at up to `ids0.max_vel` (100 rad/s) and `ids0.max_acc` (500 rad/s^2), capped at `conf0.max_vel` and `conf0.max_acc` when those are set. `vel_bw` is capped at `conf0.cur_bw / ids0.bw_ratio` (7.5: 400 at a `cur_bw` of 3000), since above that the velocity loop mostly amplifies noise; `bw_ratio = 0` removes the cap.
2. `reset`, then type:

```python
link id_pid
stop
start
fault0.en = 1
```

3. It starts by itself with one warm-up cycle, then changes one gain at a time, scoring each step over `ids0.rep` (2) back and forth cycles of 1.2 s (defaults). When all three gains are done the axis drives back to where it started. A step whose torque noise is above `ids0.fb_max` x `conf0.max_force` (5 %) is taken back. Watch `ids0.min_cost` and the gains on the scope.
4. Append the printed `conf0.pos_bw`, `conf0.vel_bw`, `conf0.vel_d`, `fault0.en = 0`, save, `reset`. The console also prints the torque peak; if it went over `conf0.max_force`, raise `max_force` to the drive's real torque or lower `ids0.max_acc`.

When the first raise of `vel_bw` gains nothing (noisy feedback, e.g. a magnetic sin/cos encoder), the search turns down instead: `vel_bw` is cut step by step while the cost stays within `ids0.kg` (5 %) of the start value's, down to 4 x `pos_bw`.

#### Speed mode (spindle)

With `pid0.pos_en = 0` (a spindle has no position to track; the spindle example configs set it), `id_pid` tunes only `vel_bw`. Each cycle runs the motor to +-`ids0.max_vel` (alternating), measures the speed noise for 2 s, then applies a load step of `ids0.dist` x `conf0.max_force` (0.2) against the motion through `pid0.torque_ext_cmd` and takes the speed dip. 4 cycles make one score. `vel_bw` is lowered while the dip stays under `ids0.dip_max` x `max_vel` (0.05) and the speed noise within 3 x `kg` of the start value's, or raised until the dip passes if the start value is already too soft. A start value that saturates is cut instead. A raise stops at the `vel_bw` with the smallest dip once more gain makes the dip grow or saturates (the loop begins to oscillate), and stops at the `cur_bw / bw_ratio` cap; if the kept `vel_bw` still misses `dip_max`, the console says so in red: lower `ids0.dist` or `ids0.max_acc`, or check the feedback. `pos_bw` and `vel_d` keep their start values; append only the printed `conf0.vel_bw`. Set `conf0.max_force` to the real torque first (the spindle examples use `acim_foc0.t_max`), since it scales the load step. Same commands as above (`link id_pid`, `stop`, `start`, `fault0.en = 1`). On the example spindle it gave 79.66 on the encoder and 59.85 sensorless.

#### Optional: notch filter and disturbance observer

Both go between `pid0.torque_cmd` and the torque controller (`pmsm_ttc0.torque`). `link filters` (after `link pmsm`, [conf/template/filters.txt](https://github.com/freakontrol/stmbl/blob/main/conf/template/filters.txt)) wires `pid0.torque_cmd` -> `biquad0` -> `dob0` -> `pmsm_ttc0.torque` (and `vel1`/`vel2`/`pe0.torque = dob0.out`) with both passing through (`biquad0.type` 0, `dob0.k` 0, `dob0.j = conf0.j_sys`); set the values below in the config after it. The rt_prio is set in the template: a filter loaded at runtime without it never runs (torque 0, fault 6 at enable), so at runtime stop, `link filters`, start. On ACIM or DC drives relink `acim_ttc0`/`acim_foc0`/`dc_ttc0` `.torque` to `dob0.out`. Without the template, wire them by hand as below.

- **Notch** ([biquad](/docs/hal_components/biquad.md)): `load biquad`, `biquad0.rt_prio = 8.3`, `biquad0.in = pid0.torque_cmd`, `pmsm_ttc0.torque = biquad0.out`. `biquad0.type = 1` (notch) at `biquad0.f0` [Hz] with width f0 / `q` and gain `depth` at f0 (0 = full notch, 0.1 = -20 dB); `type = 2` is a low pass at `f0`, 0 passes through (default). Take f0 from the `id_frf` table (the resonance peak).
- **Disturbance observer** ([dob](/docs/hal_components/dob.md)): estimates the load torque `dob0.d_est` = j dw/dt - u, low passed at `dob0.bw` (20 Hz), and subtracts `dob0.k` x `d_est` from the torque command (clamped to `dob0.max` Nm, the template links it to `conf0.max_force`; 0 = no clamp). Wiring: `load dob`, `dob0.rt_prio = 8.4` (after pid0 at 8 and biquad0, before pmsm_ttc0 at 9), `dob0.en = fault0.en_out`, `dob0.in = pid0.torque_cmd`, `pmsm_ttc0.torque = dob0.out`, `dob0.vel = vel1.vel` (the observer velocity; differencing the raw position left 5x the estimate noise), `vel1.torque = dob0.out`, `vel2.torque = dob0.out`, and `dob0.j` the total inertia (`conf0.j` + `conf0.j_sys`, as a number; with `j` 0, the default, dob stays off and passes the command through). `k` 0 (default) only estimates. Tested in the simulation and on Y with the friction feedforward off (bw 50 halved the load error; bw 200 oscillated in the simulation; 20 is the default from the upstream review: a third more load error than 50 at 40 % of the noise; bench Y ran 30). `conf/fanuc_a3-3000.txt` uses it: `link filters`, `conf0.f`/`d`/`o` = 0, `dob0.j` = the id_frf total, `dob0.max = 3`, `dob0.k = 1` (2-30x load stiffness at 2-25 Hz; LinuxCNC following error about 1 um rms worse than with the friction feedforward). With `k` 1 keep the friction feedforward off (dob estimates friction too), and set `k` 0 before `id_mot` / `id_sys`, which measure the friction and offset dob would cancel.

Details: [ids](/docs/hal_components/ids.md). [conf/fanuc_a3-3000.txt](https://github.com/freakontrol/stmbl/blob/main/conf/fanuc_a3-3000.txt) is an axis tuned this way, with the source of each value marked (`id_mot`, `id_sys`, `id_lpf`, `id_pid`). Then go on to [LinuxCNC](/docs/getting_started/linuxcnc.md).

### Standstill Hold (optional)

With stiction a stop ends at whatever torque the last move left: on a vertical axis gravity plus the friction after an up move, gravity minus it after a down move. The integrator (or the disturbance observer) freezes there, so the current at rest depends on the direction of the last move (Z in the simulation with the friction measured on the bench: 17.9 A after up, 3.3 A after down). `pid0.hold_en` moves the torque to `conf0.o` instead once the command has been at rest for `pid0.hold_t` (0.1 s) and the position error is inside `pid0.hold_arm` (0.0001 rad, or inside `pid0.hold_win` 0.002 rad once it has stopped shrinking for `pid0.hold_stk` 0.3 s). It stays on while the error is inside `hold_win`. Until then the loop pulls the stop overshoot in, so the rest error is about `hold_arm` (Z bench before `hold_arm`: 1.99 mrad at the `hold_win` edge after every stop):

- without a disturbance observer: `pid0.hold_en = 1`. The integrator is frozen and slewed at `pid0.hold_rate` (20 Nm/s). Z: 10.8 A after both; the creep back into `hold_arm` takes about 0.9 s here.
- with the disturbance observer (`link filters`, `dob0.k` 1): `pid0.hold_en = 2`. pid only flags rest and freezes its integrator; `dob0.hold` (linked to `pid0.hold` by the template) slews the observer's compensation instead. `dob0.max` must leave room for it, `abs(o) + f` or more (Z: 8; 4 stops it short). Z: 10.8 A after both, with the best start errors of the variants tried.

If the real friction band does not reach `conf0.o`, the axis creeps until the position error leaves the window and the loop takes over there, so a wrong `o` costs at most `hold_win` of position error and some current. Once the error has left the window the hold stays off until the next move, so the hold cannot toggle at the window edge. `hold_o` is linked to `conf0.o`; with `conf0.o` 0 and the observer carrying gravity, set `dob0.hold_o` to the `o` id_sys printed. Off by default.

## Induction Motor (ACIM)

### Standstill Test (id_acim)

Measures r (dead time separated), leakage inductance `l` (sigma*Ls), rotor time constant `tr` and `lmr` (Lm^2/Lr), and from the motor plate computes a V/f set. The rotor may stay free: a dc field makes no torque on a cage.

Base config: `link pid` (id_acim uses `pid0.en` and `vel1.vel`), your feedback template if there is an encoder, `link misc`, and `conf0.psi = 0` (an induction motor has no magnet flux). Like `id_pmsm`, the `id_acim` template sets `hv0.drop_k = 0` for the run (the r fit needs the uncompensated loss).

1. `reset`, then type:

```python
link id_acim
stop
start
idacim0.test_cur = 8 # default; below conf0.max_ac_cur, near the expected id_n is best
idacim0.n_volt = 380 # plate: line to line rms V
idacim0.n_freq = 50 # plate: Hz
idacim0.n_cur = 5 # plate: rms A
idacim0.n_pp = 2 # plate: pole pairs
fault0.en = 1
```

The V/f set needs all four plate pins and a measured `tr`; otherwise it is skipped. With a four wire measured resistance also set `idacim0.r_known`. Set `idacim0.spin = 1` to go straight on to the [rotating test](#rotating-test-optional).

2. The console asks for `idacim0.state = 1.2`. Type it. The test runs r (4 s), leakage (a few s) and the rotor test (d current steps, up to about 15 s at the defaults).
3. Append the printed lines. Without `spin` the test ends in state 3.0.

| Append | Notes |
|---|---|
| `conf0.r` | |
| `conf0.l` | leakage sigma*Ls: right for `acim_foc` and the observer; with `acim` (acim_ttc) it makes iq fall short at speed |
| `acim_flux0.tr`, `acim_flux0.lmr` | only printed when `tr` was measured |
| `conf0.polecount` | V/f set (from `n_pp`) |
| `vf0.u_n`, `vf0.vel_n`, `vf0.u_boost`, `vf0.boost_vel`, `vf0.slip_n`, `vf0.cur_n` | V/f set, only for the `vf` template |
| `acim_foc0.id_n` | start value for FOC; the rotating test refines it |
| `acim_flux0.tr`, `acim_flux0.i_knee`, `acim_flux0.tr_sat` | only if printed: with `idacim0.knee = 1` the tr knee fit replaces the `tr` above |

{{% hint warning %}}
- The dead time `drop` in the report is added to the printed `vf0.u_boost` (V/f runs without dead time compensation); it is **not** a value for `hv0.drop_k`.
- If `l` failed (printed in red, 1 mH used), measure line to line near 150 Hz with an LCR meter and halve it.
- `tr` is the rotor's at the test temperature: a hot cage reads shorter. Warnings about edge spread or a slow loop suggest more `idacim0.rot_cycles` or a higher `idacim0.rot_bw`.
- If the report says so, rerun with `idacim0.test_cur` near the printed `id_n`.
{{% /hint %}}

### Rotating Test (optional)

Spins the rotor open loop and measures the flux curve and, with an encoder, the inertia. Needs `tr` from the standstill test **in the same enable** (disabling resets `r` and `l`), and `idacim0.n_freq` and `idacim0.n_pp`. Shaft free and unloaded.

1. With `idacim0.spin = 1` it follows the standstill test; otherwise type `idacim0.state = 4` after it finished.
2. The console asks for `idacim0.state = 4.2`. The field ramps to `idacim0.rot_vel` (0.4) of plate speed (at plate speed per 5 s with `rot_acc = 0`). With an encoder (the template links `idacim0.pos_fb = fb_switch0.vel_fb`) two pairs of up and down ramps to 0.8 x plate speed follow, at `id_n` and 0.65 x `id_n`: they give `tr` (air gap torque difference over the slip difference), the field to position delay and the inertia. Then the d current is swept down from 1.15 to 0.25 x `id_n` (0.45 without an encoder); with an encoder each level runs at the speed where its emf is the ramps' top one, up to `idacim0.rot_vmax` (1.0) x plate speed. It ends in state 5.0.

| Append | Notes |
|---|---|
| `acim_foc0.id_n` | rated magnetizing current |
| `acim_flux0.tr` | with an encoder, from the ramps (scaled to `id_n`); replaces the standstill `tr`. A red line says when the ramps gave none: keep the standstill one |
| `acim_flux0.lmr`, `acim_flux0.lmr_sat` | replace the standstill `lmr`; with an encoder and a rejected fit `lmr_sat` is only printed as a comment, keep the config's |
| `acim_flux0.i_n` | `acim_foc` already links it to `acim_foc0.id_n` |
| `acim_flux0.i_knee`, `acim_flux0.tr_sat`, `acim_flux0.i_dip`, `acim_flux0.lmr_dip`, `acim_flux0.tr_dip` | only if printed (with an encoder and an accepted magnetizing curve fit). `tr_dip` is printed equal to `lmr_dip` (tr follows the secant lmr below the dip); without it `tr_dip` stays 0 (tr does not change below `i_dip`) |
| `conf0.j` | only with an encoder |

A red message that the rotor did not follow: lower `idacim0.rot_acc` or unload the shaft. Friction is printed as a comment only.

Optional rotating tr check: `idacim0.rot_tr_vel` > 0 (top speed in rad/s mech, 0 = off by default) adds a run in FOC on the encoder with torque ramps between 0.7 and 1 x `rot_tr_vel`; it prints the rotating `tr` as a multiple of the standstill one and a torque balance inertia, as comments to compare, not lines to append.

### Pole Pairs and Direction

`conf0.out_rev` is only found by the pole pair test, which is started by hand and needs an encoder (it compares the field speed with `vel1.vel`): rotor free, type `idacim0.state = 2`, then `idacim0.state = 2.2`. The field turns at `idacim0.test_vel` for 3 s. Append `conf0.polecount` and, if printed, `conf0.out_rev = 1`.

Details: [idacim](/docs/hal_components/idacim.md).

### Run It with V/f

The [vf](/docs/hal_templates/vf.md) template is a complete config of its own (no pid, no feedback needed):

```python
link vf
<conf0 limits>
conf0.r = ...
conf0.l = ...
conf0.polecount = ...
vf0.u_n = ...
vf0.vel_n = ...
vf0.u_boost = ...
vf0.boost_vel = ...
vf0.slip_n = ...
vf0.cur_n = ...
```

The speed command `vf0.vel_cmd` [rad/s mech] is not linked by the template: set it or link it to your command source. Ramps (`vf0.acc`, `vf0.dec`), stall prevention (`vf0.i_stall`) and damping (`vf0.k_damp`) are set by hand. A command below 0.5 rad/s counts as 0. At a 0 command, once the ramp has reached 0, the field stops and no voltage is applied (no boost current at standstill): the motor coasts and has no holding torque. See [vf](/docs/hal_components/vf.md).

### Run It with V/f and an Encoder

The [vf_enc](/docs/hal_templates/vf_enc.md) template is V/f with encoder slip control (`vf0.enc = 1`). The field runs at the rotor speed from the encoder plus a slip from a speed PI controller, so the speed follows the encoder and the rotor cannot pull out. Link it instead of `vf`, then your feedback template:

```python
link vf_enc
link <your_feedback_type>
<conf0 limits>
conf0.r = ...
conf0.l = ...
conf0.polecount = ...
vf0.u_n = ...
vf0.vel_n = ...
vf0.slip_n = ...
vf0.cur_n = ...
vf0.i_stall = ...
```

- The speed command is `vf0.vel_cmd` [rad/s mech], as with `vf`.
- `vf0.slip_max` clamps the slip; 0 (default) means 2 x `vf0.slip_n`. With `slip_n` and `slip_max` both 0 the encoder path is off and it runs open loop.
- At a 0 command, once the ramp is at 0, the field stops and no voltage is applied, as with `vf`; a new command restarts the speed PI from the open loop slip.
- `vf0.enc_kp` (default 0.25) and `vf0.enc_ki` (default 5) are the speed PI gains. They were tuned on one spindle; a higher `enc_kp` can make the speed hunt.
- From standstill the motor starts on the slip frequency alone, so it needs `vf0.u_boost` (the `vf` template sets 12 V).
- Check the encoder direction before the first run: with `vf0.enc = 0` turn the motor in V/f and compare `vel0.vel` with `vf0.vel`. They must have the same sign; if not, set `conf0.mot_fb_rev`.

See [vf](/docs/hal_components/vf.md).

### Run It with FOC

Rotor flux oriented control on an encoder. `acim_foc` must be linked after `pid`:

```python
link pid
link acim_foc
link <your_feedback_type>
link misc
conf0.r = ...
conf0.l = ... # leakage from idacim
conf0.lq = 0
conf0.psi = 0
conf0.polecount = ...
acim_flux0.tr = ...
acim_flux0.lmr = ...
acim_flux0.lmr_sat = ... # rotating test
acim_foc0.id_n = ...
```

Then continue with [id_mot](#mechanical-parameters-id_mot), [id_sys](#load-inertia-id_sys) and [id_pid](#control-loop-id_pid) as for a PMSM. Optional extras (flux boost `acim_foc0.k_boost`, tr adaptation `acim_flux0.tr_ki`, ...) are listed in the [acim_foc template](/docs/hal_templates/acim_foc.md); they are off by default and not tuned automatically.

- Run the speed command through a ramp with `ramp0.en_delay = 0.3` (as the spindle examples do): `acim_foc` gives no torque for about 280 ms after enable while the flux builds.
- For a spindle tune `vel_bw` with [id_pid in speed mode](#speed-mode-spindle) (`pid0.pos_en = 0`), with `conf0.max_force = acim_foc0.t_max`.
- **Light-load flux reduction** ([fluxopt](/docs/hal_components/fluxopt.md)): the `acim_foc` template loads `fluxopt0` with `min` 1 (off). `fluxopt0.min = 0.5` turns it on: at light load the flux is lowered toward id = iq (least copper loss), at least `min` x `id_n`; it rises at once with the torque and falls at `fluxopt0.fall_hz` (1). On the example spindle the idle copper loss fell from 111 to 28 W, but the torque on a sudden load is lower until the flux is back (9.8 against 13.9 Nm on the bench).
- **Rotor flux observer** ([flux_obs](/docs/hal_components/flux_obs.md)): `link flux_obs` after `acim_foc`, encoder only. It loads in shadow: `flux_obs0.pos_err` shows how far the current model's orientation is off at speed (a wrong `tr`). To trim the slip with it set `flux_obs0.kp_s = 20`, `flux_obs0.ki_s = 2` and `angle0.slip = flux_obs0.slip`; `flux_obs0.k` then follows the rotor temperature. Checked on one spindle without load only; not tested under a cutting load.

The older [acim](/docs/hal_templates/acim.md) template (`acim_ttc`) takes plate values set by hand; `idacim` only prints a hint for `acim_ttc0.vel_n`.

### Field Weakening

Off by default. To enable it with `acim_foc`, add `acim_fw0.duty = hv0.duty`. Optional, by hand:

- `acim_fw0.p_max` [W]: the motor's S1 power, the constant power torque limit. The template multiplies it by `iit0.cur_boost`, the overload headroom.
- `acim_fw0.ls` (voltage feedforward) and `acim_fw0.ki` (regulator gain) are derived at 0: `ls` = `conf0.l` + `acim_flux0.lmr`, `ki` = 2.5 / `tr`. A value in the config wins.

See [acim_fw](/docs/hal_components/acim_fw.md).

### Speed Limits per Mode

f_el = pp x rpm / 60 is the electrical frequency, w_el = 2 pi f_el in rad/s. Speeds in the configs are mechanical rad/s (rpm = rad/s x 60 / 2 pi).

| Mode | Low end | High end | Limited by | What to set |
|---|---|---|---|---|
| V/f (`vf`) | about `vf0.boost_vel` under load, lower on a light one | about `vf0.vel_n` at full torque; above it light loads only | low: r and dead time drop against a small voltage, rated slip. High: the cap `vf0.duty` x `hv0.pwm_volt`; above it flux falls as 1/f, breakdown torque as 1/f^2 | `u_boost`, `i_stall`. **vf has no speed clamp** (`conf0.max_vel` is not linked): limit `vf0.vel_cmd` at its source |
| FOC, encoder (`acim_foc`) | 0, full torque at standstill | base speed | past base speed the voltage runs out and the currents cannot be held. Low speed smoothness: encoder resolution | `conf0.max_vel` at or below base speed |
| FOC + field weakening (`acim_fw`) | 0 | in theory base / `acim_fw0.scale_min` (0.1: 10 x base) | torque falls as `p_max` / speed, faster near the leakage limit; encoder count rate; bearings and the motor's rated top speed | `acim_fw0.p_max`, `conf0.max_vel` |
| Sensorless (`sl_acim`) | `sl_seq0.w_hand` - `hyst` in closed loop; below it only I/f (no speed loop, no torque control), none at standstill | as with an encoder, while f_el stays below about 450 Hz on the F4 observer | emf against `obs0.e_min` (3 V); the F4 rate (5 kHz) and `obs0.max_vel` (3000 rad/s el = 477 Hz). There is no faster observer: an F3 one overran the F3 rt and was removed | `w_hand` where the emf is about 5 x `e_min`; `conf0.max_vel` below about 450 Hz electrical |
| `acim` (`acim_ttc`) | as FOC | as FOC + field weakening (built in, `acim_ttc0.duty = hv0.duty`) | the angle integrates `vel1.vel`, so its filter lag shows as slip error on ramps | as FOC |

The same limits as stator (electrical) frequency f_el, in Hz, for any motor at 15 kHz PWM. rpm = f_el x 60 / pp, ignoring slip.

| Mode | Usable f_el |
|---|---|
| V/f | about 5-10 % of the plate frequency up to the plate frequency at full torque; up to plate f x (`duty` x `pwm_volt` / `u_n`) on light loads |
| FOC, encoder | 0 Hz to base f_el |
| FOC + field weakening | 0 Hz to 10 x base in theory; hard ceiling PWM_FREQ / 15 = 1000 Hz |
| Sensorless | closed loop from (`w_hand` - `hyst`) x pp / 2 pi, I/f below; top about 450 Hz (F4 observer, `obs0.max_vel` 477 Hz, about 10 F4 ticks per period) |
| `acim` (`acim_ttc`) | as FOC + field weakening |

- Voltage: `hv0.pwm_volt` = dc link / sqrt(3) x duty_max (phase peak), with duty_max 0.91 at 15 kHz (0.94 at 10, 0.88 at 20 kHz).
- Base speed, where rated flux reaches that voltage: w_el,base = `acim_fw0.duty_setpoint` x `pwm_volt` / (ls x `id_n`), ls = `conf0.l` + `lmr`. Divide by pp for mechanical. The r x I drop takes a few % more.
- V/f: the voltage cap `vf0.duty` x `pwm_volt` is reached at about `vf0.vel_n` x cap / `vf0.u_n`; above it the flux falls.
- Sample rates: the F3 current loop runs at PWM_FREQ (build option, 15 kHz default) and gets the angle from the F4 every 200 us (5 kHz), extrapolating it with `hv0.vel` in between. Keep f_el below PWM_FREQ / 15 (1 kHz at 15 kHz: 30000 rpm at pp 2) and, for the F4 observer, about 10 F4 ticks per electrical period.
- The observer runs on the F4: the F3 rt has no time left for it at the PWM rate (an F3 observer overran the F3 rt and gave fault 9, link timeout).
- The F3 corrects its own delay: the voltage lands about 1.5 PWM periods after the current sample, and `ls0.pos_v` advances the voltage angle by that (0.11 rad at 1100 rad/s el and 15 kHz). `hv0.adv` covers only the F4 to F3 transport.
- Current loop: the current rises 10-90 % in about 2.2 / `conf0.cur_bw` (0.7 ms at the template's 3000).
- Field weakening: flux scale about base / speed, torque limit `p_max` / speed.
- Sensorless low end: the observer's emf is about w_el x `lmr` x `id_n`. Put `sl_seq0.w_hand` where this is about 5 x `obs0.e_min`: w_el = 5 x `e_min` / (`lmr` x `id_n`).

**Example**, a Fanuc alpha2 spindle (measured on the bench or computed from the formulas above): pp 2, plate 78.4 Hz (2352 rpm), IM06B50GC1, 296 V link, 15 kHz, `id_n` 19 A, ls 14 mH, `lmr` 12.6 mH, `l` 0.92 mH.

- `pwm_volt` = 296 / sqrt(3) x 0.91 = 155 V (computed). Base speed for `acim_fw0.duty_setpoint` 0.8: 0.8 x 155 / (0.014 x 19) = 468 rad/s el = 234 rad/s = 2230 rpm (computed). At the default `duty_setpoint` 0.9 the same formula gives 524 rad/s el = 262 rad/s = 2500 rpm. Measured: duty 0.82 at 246 rad/s (2352 rpm), no load, id 18.8 A.
- V/f: `u_n` 126.6 V at `vel_n` 246.3; the cap 0.9 x 155 = 140 V is reached near 272 rad/s, 2600 rpm (computed). `u_boost` 14.5 V from idacim, r x `id_n` plus the dead time drop (r x `id_n` alone, 3.7 V, did not start the motor), fades out by `boost_vel` 20 rad/s (190 rpm); rated slip 6.1 rad/s. On the bench V/f ran from 20 to 838 rad/s (8000 rpm) and back, with the speed estimate damping of the example config below; without it the rotor hunted at about 9 Hz from about 75 rad/s.
- FOC and field weakening: 838 rad/s (8000 rpm) reached in encoder and sensorless mode (measured), flux scale 0.25 / 0.31, id about 5 A, duty peak 0.90, 314 V on the link braking from 838. `p_max` 2200 W gives 2.6 Nm continuous at 838 rad/s, up to 3.1 Nm with the iit0 overload headroom (`iit0.cur_boost` 1.19 on this spindle). f_el 267 Hz there: 56 PWM periods and 19 F4 ticks per electrical period (computed). Above about 7800 rpm the encoder position can jump by 2 counts (measured); the observer's angle error is 0.05 to 0.10 rad at 600 to 800 rad/s (measured). `conf0.max_vel` is 838.
- Sensorless low end: `w_hand` about 30 rad/s (290 rpm), `hyst` about 10 (both derived by `sl_seq`), so closed loop down to 20 rad/s (190 rpm), I/f below. The emf at 30 rad/s is 60 x 0.0126 x 19 = 14 V, about 5 x `e_min` (computed).

The complete configs for this spindle are in the repository, one per mode: [fanuc_a2_spindle_vf.txt](https://github.com/freakontrol/stmbl/blob/main/conf/fanuc_a2_spindle_vf.txt) (V/f), [fanuc_a2_spindle_vf_enc.txt](https://github.com/freakontrol/stmbl/blob/main/conf/fanuc_a2_spindle_vf_enc.txt) (V/f with encoder), [fanuc_a2_spindle_foc_enc.txt](https://github.com/freakontrol/stmbl/blob/main/conf/fanuc_a2_spindle_foc_enc.txt) (FOC with encoder) and [fanuc_a2_spindle_foc_sl.txt](https://github.com/freakontrol/stmbl/blob/main/conf/fanuc_a2_spindle_foc_sl.txt) (sensorless FOC). Each lists where its values came from and what was checked on the bench: they were built on 13 Oct 2026 from a clean config with only hardware, plate and safety values by hand, the rest from idacim (with the pole pair test), id_sys and id_pid in speed mode.

## Sensorless

The observer needs `conf0.r`, `conf0.l`, `conf0.lq`, `conf0.polecount` (and `hv0.adv`) from the identification above. The start-up and observer settings are **not** auto-tuned; set them by hand.

### PMSM

1. Identify with the encoder (`id_pmsm`, `id_tune`).
2. Check the observer on the encoder: base config plus `link obs_check`, run the motor normally and scope `obs0.pos_err` (wave0). It should stay near 0; an error growing with speed is timing, trimmed by `hv0.adv`.
3. Run sensorless with `link pid`, `link pmsm`, `link sl_pmsm` (the encoder stays for `fault0.fb_ready` and the check `obs0.chk_err`). Set `sl_seq0.i_f` (start current), `sl_seq0.acc` (100 rad/s^2) and `obs0.bw` (100). `sl_seq0.w_hand` 0 derives the handover speed from `conf0.psi` (linked by `sl_pmsm`) where the emf is 5 x `obs0.e_min`, and `hyst` 0 is `w_hand` / 3. The scope shows `sl_seq0.state`, `obs0.chk_err`, `obs0.vel_m`, `hv0.iq_fb`.

### ACIM

1. Identify with `id_acim` (with the rotating test for `id_n`, `lmr_sat`, and `conf0.j` if an encoder is available).
2. Config: `link pid`, `link acim_foc`, `link sl_acim`, with `conf0.lq = 0`. The template links `sl_seq0.i_f = acim_foc0.id_n`; `sl_seq0.align_time` 0 is 3 x `acim_flux0.tr`, `w_hand` 0 is where the emf is 5 x `obs0.e_min` (flux `lmr` x `i_f`), `hyst` 0 is `w_hand` / 3, and the template links `obs0.bw = sl_seq0.obs_bw` (5 x `w_hand` x polecount). Override any of these in the config if the start needs it. `sl_acim` turns the tr MRAS off (`acim_flux0.tr_ki = 0`).
3. Optional, with a spindle encoder: `sl_seq0.vel_enc = vel1.vel`, `sl_seq0.enc_tol = 1`, then `acim_flux0.tr_ks = 1` trims `tr` from the slip error.
4. Load inertia without encoder (`id_sys_sl`): needs `conf0.j` in the config. First `sl_seq0.acc` must be at least `idm0.max_acc` (250), or the ramp cuts the profile. Then:

```python
link id_sys_sl
stop
start
sl_seq0.acc = 250
fault0.en = 1
```

The drive starts sensorless; `idm` starts once `pid` is enabled after the handover and sweeps 50 to 200 rad/s in one direction (`idm0.vel_offset = 125`, `idm0.max_vel = 75`) for about 2 minutes. Append `conf0.j_sys` and `conf0.d`. `conf0.f` and `conf0.o` are both a constant torque here and split arbitrarily: only their sum is meaningful, take `f` from an encoder run.

Details: [sl_seq](/docs/hal_components/sl_seq.md), [obs](/docs/hal_components/obs.md).
