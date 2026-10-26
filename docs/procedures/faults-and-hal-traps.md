# Fault codes and HAL traps

## Fault codes

The full table with causes and fixes is on the [errors page](../stmbldoc/content/docs/errors.md) (hand-written: update it with any change to `shared/comps/fault.c`). Where to read the code: Servoterm prints `ERROR: Fault N: <text>`, `fault0.last_fault` holds it until the next enable, the red LED blinks it, LinuxCNC sees it on the `fault_code` pin.

| N | Short meaning | First thing to check |
|---|---|---|
| 1 | Command error (smart serial) | Normal while LinuxCNC (re)loads; otherwise the sserial link |
| 2, 3, 4 | Motor, commutation, joint feedback | Encoder cable and config |
| 5 | Position error | Loop tuning, mechanics, `conf0.max_pos_error` |
| 6 | Saturation | Torque at its limit too long: load, tuning, `conf0.max_force`, `conf0.max_sat`; right after a runtime `link`: a torque-path comp without `rt_prio` |
| 7 | Motor temperature (I²t model) | RMS current vs `iit0` settings |
| 9 | HV link timeout | **Is HV on?** Then the F4-F3 cable, F3 firmware |
| 10 | HV over-temperature | Fan, heatsink |
| 11 | HV voltage out of range | Supply, regeneration while braking |
| 12 | IPM fault pin | Power module |
| 13 | Current offset at F3 start | Motor carrying current at power-up |
| 14, 15 | F3 overcurrent (filtered, peak) | Current loop, commutation, `conf0.max_ac_cur` |
| 16 | Hardware comparator | Short circuit, or `hv0.dac` too low |
| 17 | IPM junction temperature (model) | Long standstill current, cooling |
| 18 | F3 supply undervoltage | HV collapsed while enabled |

LinuxCNC reloads typically show 1, 9 or 11 and are harmless. Read `fault0.scale` too: it is the minimum of all derates (motor, driver and IPM temperature, DC voltage, AC and DC current, see `fault.c`), so a drive that "won't give full torque" without a fault is usually derating.

## HAL traps

- **Runtime links are not flattened.** `a = b` links `a` to the pin instance `b`, not to `b`'s source. At boot the config is followed by `relink`, which collapses chains; a link typed at runtime is not. So after `x.in = y.in` (where `y.in` is linked to `z.out`), `x.in` reads `y.in`'s own stored value, usually 0. Link to output pins, or type `relink`.
- **The scope has the same trap**: `term0.waveN` on a linked input pin shows a stale value. Scope the source.
- **Runtime-loaded components only run after `stop` / `start`**, and only with `rt_prio` > 0 set before `start`. `start` sorts the rt list by `rt_prio`, lowest first; the value decides where in the chain the component runs.
- **Components can't be unloaded.** After an id stage or any `link <template>` typed for a test, `reset` before the next one.
- **`link <template>` typed at the console stays out of the saved config**; `flashsaveconf` saves the config text, not the running HAL.
- At most `HAL_MAX_COMPS` (32) component instances.
- A terminal line is at most 127 bytes; a longer one is dropped whole with "line too long".
- Config lines run top to bottom; a later setting of the same pin wins. Template defaults are overridden by lines after the `link`.
- Inspect: `list` (instances), `show_hal`, `linked` (links), `hal` (rt time per component), a component name such as `pid0` (all its pins), `hv hal` for the F3.
