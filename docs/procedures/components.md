# Add or change a HAL component

Read two or three existing components of the same kind first (`shared/comps/not.c` is the smallest; `pid.c`, `dob.c` or `ramp.c` show a real one) and follow their layout.

## Where it goes

| Runs on | File | Register in |
|---|---|---|
| F4 only | `src/comps/<name>.c` | `LV_SRC_COMPS` in `comps.mak` |
| F4, shared code | `shared/comps/<name>.c` | `LV_SHARED_COMPS` in `comps.mak` |
| F3 | `stm32f303/src/comps/<name>.c` or `shared/comps/` | `HV_SRC_COMPS` / `HV_SHARED_COMPS` in `stm32f303/comps.mak` |

The build generates `inc/comps/<name>_comp.h` (or `inc/shared_comps/`) and the HAL table from the `HAL_COMP` / `HAL_PIN` lines, so those lines must keep their exact form. Every new component on the F3 costs flash and HAL memory (1024 bytes shared by all F3 components): put it on the F4 unless it must run at the PWM rate.

## Shape

- `HAL_COMP(name);`, then one `HAL_PIN(x);  // *input*|*output*|*parameter*, what it is (unit), default N` per pin. The comment becomes the wiki pin table: give unit and default, no `|`.
- Functions: `nrt_init` (defaults: set every parameter pin here), `rt_start`, `rt_func(period, ...)`, `rt_stop`, `nrt` (console output, slow work), `hw_init` for peripherals. Unused ones are 0 in the `hal_comp_t` struct.
- Per-instance state goes in a `ctx` struct (`.ctx_size`), not in `static` variables, unless the component can only be loaded once.
- One `/** ... */` doc block: Brief, how it is wired (which template loads it), what each stage does, warnings. See [AGENTS.md](../../AGENTS.md) for the doc rules.

## Rules for rt code

- No `printf`, no blocking, no loops of unbounded length, no heap. Set a flag or pin in rt and print from `nrt`.
- Guard every division and `sqrtf` / `logf` argument; a NaN in rt spreads through every linked pin. Clamp outputs that drive current or voltage.
- Use the period argument for time, never assume a fixed rate.
- Units: SI, rad (mechanical unless the pin says electrical), Nm, peak phase A, V. Say which in the pin comment.
- Measure the cost: `hal` prints time per component (`hv hal` on the F3). The F3 rt runs from 8 KB CCM RAM that is nearly full; check the linker output.

## Defaults and compatibility

- **A new feature is off by default** (gain 0, enable 0) and changes nothing for an existing config. Defaults are neutral or derived from other pins, never one motor's values.
- Don't rename or remove a pin that configs in `conf/` use; grep `conf/` before changing a pin name. If a rename is unavoidable, update every config and template in the same change.
- F4-F3 packets (`shared/common.h`) are append-only.

## Template and config

- A template in `conf/template/<name>.txt` starts with a `#` header saying what it does and when to `link` it, then `load <name>`, `<name>0.rt_prio = <n>`, and the links.
- `rt_prio` sets the order inside the rt tick (lower runs first; see `conf/template/pid.txt` for the existing chain: feedback around 4-6, pid 8, torque path 8.x-9, hv0 and fault0 10). A component in the torque path that is loaded at runtime without `rt_prio` never runs: torque 0, fault 6 on enable.
- If a config in `conf/` should use the feature, add it there with a comment, and mark machine-specific values.

## Before calling it done

1. `make -j16 DOCS=0` builds, sizes checked, no new warnings ([build.md](build.md)).
2. `make format`.
3. A host sim scenario covers it if it touches control, feedback or faults ([sim-scenarios.md](sim-scenarios.md)); `make -C tests/sim test` passes.
4. Doc block, pin comments, template header and wiki page updated in the same commit.
5. The [review checklist](review-checklist.md) is clean.
