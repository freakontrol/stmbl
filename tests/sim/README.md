# Host simulation harness

Runs the real F4 and F3 firmware (hal.c, shared comps, conf templates, board configs) on a PC against
a PMSM + two-mass load model. Scripted scenarios, CSV traces, pass/fail exit code.

## Use

```
make -C tests/sim            # build build/stmbl_sim
make -C tests/sim test       # run all scenarios, CSV/log in build/out/
tests/sim/build/stmbl_sim [-v] [-o out.csv] [-d decim] [-e "line"] scenario.txt
tests/sim/build/stmbl_sim -p # list plant parameters
python3 tests/sim/plot.py build/out/y_move.csv
```

## Scenario language

One command per line, `#` comments, optional `at T` prefix (seconds):

- `config FILE`: board config fed through hal_parse (path relative to scenario dir, walking upward)
- `f4 LINE` / `f3 LINE`: hal command on that side (`f4 en0.en = 1`, `f4 hv_pause 5`)
- `plant k=v ...`, `link k=v ...` (latency, jitter, ...)
- `lcnc enable|scale|latency|jitter|period|move DIST VEL ACC|jog VEL ACC|dwell T`: 1 kHz sserial source
- `trace SIG ...`, `sample f3`, `f3_phase`, `strict` (parse errors fail), `title`, `run T`
- `expect STAT SIG OP VAL [from T0] [to T1]`, STAT = max min absmax mean rms std p2p first final
- Signals: `comp0.pin` (F4), `f3:comp0.pin`, `plant.x`, `lcnc.x`

Motor: `plant motor=0` PMSM (dq, r ld lq psi pp), `motor=1` induction motor (inverse-gamma
in the stator frame: r, ld = leakage sigma*Ls, lmr, tr, with acim_flux's saturation shape i_n,
i_knee, lmr_sat, tr_sat, i_dip, lmr_dip, tr_dip as a static psi(i_m) curve). An ACIM config
(acim_flux0 or vf0 present) selects motor=1 and fills these from acim_flux0/acim_foc0. Extra plant
signals: psi_r, i_mr, lm, tr_act, th_flux, slip. Dead-time loss shape: `dt_shape=0` tanh(i/dt_i0),
`1` hv.c's 1-1/(1+|i|/dt_i0)^2. Encoder: encf0 (fanuc) or enc_fb0 (sin/cos, res x ires counts).

Plant defaults come from the config (conf0.r, l, lq, psi, polecount, j, j_sys, f, d). Two-mass via
`k`/`c` or `ring_hz`/`ring_zeta`; Karnopp friction; `enc_delay` defaults to hv0.adv minus the
packet-to-F3 time.

## Scenarios

- `y_*`: Y axis (fanuc_a3-3000): hold, current step, move, ring, load step, link loss.
- `s_*`: spindle: FOC encoder with load, FOC field weakening to 838 rad/s, sensorless FOC
  (I/f start, reverse), V/f open loop, V/f encoder.
- `id_*`: identification regression against plants with known values: idpmsm, id_mot (idm),
  idacim (standstill + rotating). Each file's comments give the result against the truth.

## Limits

No rt CPU time, no ISR races, no recon/PLL/fault pin, averaged PWM (dead time as tanh voltage loss),
real hardware drivers replaced by pin-compatible stand-ins (comps_f4/hv.c, comps_f3/ls.c, io.c,
generated shells). The coupling ring is a plant input, not validated against bench captures.
