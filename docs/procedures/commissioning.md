# Commission a new PMSM axis

The stages, what they print and their safety limits are on the [tuning page](../stmbldoc/content/docs/getting_started/tuning.md). This page is the order and the pitfalls. Every stage that moves the motor needs the operator's go-ahead and the [bench safety rules](bench-safety.md).

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

## Recording the result

Mark the source of every value in the config with a `#` comment (`# [id] id_sys`, `# datasheet`, `# hand`), keep a dated copy per board, and save only with the drive disabled.
