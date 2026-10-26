# Commission a new axis or spindle

The stages, what they print and their safety limits are on the [tuning page](../stmbldoc/content/docs/getting_started/tuning.md). This page is the order and the pitfalls: PMSM (servo axis) first, then [induction motor (ACIM)](#induction-motor-acim). Every stage that moves the motor needs the operator's go-ahead and the [bench safety rules](bench-safety.md).

## 0. Limits before anything turns

- **Units.** STMBL currents are peak phase current (A). Most servo datasheets give RMS: multiply by 1.414. Torque in Nm, positions in motor rad.
- `conf0.max_ac_cur`: the lower of the motor's peak current and what the drive and its fuse can take. `conf0.max_force` (Nm) to match. `conf0.max_dc_cur` to the supply. `conf0.low_dc_volt` to a sane minimum for the supply (the template default of 24 V only fits a bench supply).
- Motor thermal model `iit0` (`max_cur`, `cur_boost`, `max_time`) from the motor's continuous and peak ratings: documented in [`shared/comps/iit.c`](../../shared/comps/iit.c).
- Overcurrent comparator (`hv0.dac`, or `link ocdac`): only on HV boards with the filter caps on the current sense lines. Otherwise leave the default (comparator effectively off). See "Overcurrent Trip Threshold" on the tuning page.
- Feedback working and checked first ([feedback page](../stmbldoc/content/docs/getting_started/feedback.md)).

## 1. Electrical: `id_pmsm`, then `id_tune`

Load off the shaft.

- **Choose `idpmsm0.test_cur` per motor.** It must stay below `conf0.max_ac_cur` and within what the motor takes continuously for about a minute (the r stage dwells about 50 s). The default 6 A is fine for most mid-size motors; small motors need less. On low-resistance motors a low test current lets the dead-time drop dominate and the free r fit reads unreliably. Rather than raise the current past what the motor takes, measure the phase resistance (line to line / 2, cold, at the drive end of the cable) and give it as `idpmsm0.r_known`: then only the dead-time curve is fitted.
- Don't take `psi` from a datasheet; use the measured value (the coast at the end of `id_pmsm`).
- `id_tune` (advance and dead time) runs from a console `link id_tune`, never from the saved config.

## 2. Mechanical: `id_mot`, `id_sys`

- `id_mot` on the bare motor gives `conf0.j`. If the motor can't come off, start with `id_sys` and the datasheet rotor inertia as `conf0.j`.
- `id_sys` with the load coupled gives `j_sys`, `f`, `d`, `o` (gravity on a vertical axis). Run it warm and lubricated. It moves the axis up to +-20 rad from where it stands: set `idm0.min_pos` / `idm0.max_pos` to the free travel first.
- Optional: `id_brk` (breakaway torque), `id_lpf` or `id_frf` (coupling resonance, `conf0.j_lpf`).

## 3. Loop: `id_pid`

- Needs `conf0.j` (and `j_sys`, `f`, `d`, `o`, `j_lpf`), `conf0.cur_bw` and `conf0.max_force`. It moves +-10 rad by default: set `ids0.min_pos` / `ids0.max_pos` to the travel.
- On a position axis set `vel1.vel_p = 1` and `vel_int0.vel_ip = 1` (velocity without the estimator lag); see `conf/fanuc_a3-3000.txt`.
- Optional afterwards: notch / disturbance observer (`link filters`), standstill hold (`pid0.hold_en`), short brake for a vertical axis (`fault0.sbrake_en`). The tuning page covers each.

## 4. LinuxCNC (smart serial)

1. `link sserial`, the LinuxCNC side as on the [LinuxCNC page](../stmbldoc/content/docs/getting_started/linuxcnc.md).
2. **Last:** `sserial0.pos_advance` with `tools/sserial_advance.py` on the running machine. Repeat it after any loop, filter or firmware change, because the advance absorbs the loop's tracking lag.

## Induction motor (ACIM)

The same limits first (section 0). Then:

1. **Plate values in, not tuning by hand.** `id_acim` needs the four plate pins: `idacim0.n_volt` (line to line rms V), `n_freq` (Hz), `n_cur` (**rms** A), `n_pp` (pole pairs). Set `conf0.psi = 0` and `conf0.lq = 0`. Keep `idacim0.test_cur` below `conf0.max_ac_cur` and near the expected magnetizing current; rerun near the printed `id_n` if the report asks. Prefer identified values over hand-set ones; the extras (flux boost, tr adaptation, field weakening, fluxopt, flux_obs) are off by default and are switched on one at a time with a reason.
2. **Delta or star.** The firmware works with per-phase equivalent-Y values. For a hand reading of a delta motor, the equivalent-Y resistance is still line to line / 2; give it as `idacim0.r_known` if the fit is doubtful.
3. **Rotating test** (`idacim0.spin = 1`, or `idacim0.state = 4` in the same enable): shaft free and unloaded, runs up to plate speed with an encoder. It gives `id_n`, `lmr_sat` and, with an encoder, `tr` and `conf0.j`. Pole pairs and direction (`conf0.out_rev`) need the separate pole pair test (`idacim0.state = 2`).
4. **Pick the mode**, each a template of its own (stage list and limits in the tuning page's "Speed Limits per Mode"):
   - `vf`: no feedback. **It has no speed clamp**: limit `vf0.vel_cmd` at the source.
   - `vf_enc`: V/f with encoder slip control. Check the encoder direction first (`vf0.enc = 0`, compare `vel0.vel` with `vf0.vel`).
   - `acim_foc` (after `link pid`): FOC on an encoder, then `id_mot` / `id_sys` / `id_pid` in speed mode (`pid0.pos_en = 0`) as for a PMSM. It gives no torque for about 0.3 s after enable while the flux builds: run the command through a ramp with `ramp0.en_delay`.
   - `sl_acim`: sensorless FOC. Handover speeds are derived from the identified flux; there is no torque at standstill (I/f below the handover).
   - `uf` / `uf2` are deprecated: use `vf` / `vf_enc`.
5. **Temperature matters.** The rotor time constant `tr` shortens as the cage warms, so a cold identification is off on a warm motor. Compare runs at the same temperature, or use `flux_obs` to follow it.
6. **Motor over-temperature contact.** Many spindle motors have a normally closed thermostat, not an NTC. On the motor temperature input that reads as a fixed value and never trips: wire it as a digital fault input instead, and rely on `iit0` for the thermal model.
7. A sin/cos speed sensor (common on spindles) can be a high-impedance source: check the encoder input's termination and the signal amplitude (`adc0` / `enc_fb0` pins) before blaming the control.

## Recording the result

Mark the source of every value in the config with a `#` comment (`# [id] id_sys`, `# datasheet`, `# hand`), keep a dated copy per board, and save only with the drive disabled.
