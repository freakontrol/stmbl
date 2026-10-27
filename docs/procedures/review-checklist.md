# Review checklist for firmware changes

Go through it before calling a change done and when reviewing someone else's. Each item has bitten this code before.

## Build and tests

- [ ] `make -j16 DOCS=0` builds without new warnings; F3 flash and CCM use checked ([build.md](build.md)).
- [ ] Builds with GCC 12.2 too if it goes upstream (the CI build does this).
- [ ] `make -C tests/sim test` passes; a scenario covers the change ([sim-scenarios.md](sim-scenarios.md)).
- [ ] `make format` run; the diff has no unrelated reformatting.

## Correctness

- [ ] Units consistent: peak phase A (not RMS), Nm, rad mechanical vs electrical (`polecount`), seconds, rad/s. Datasheet values converted.
- [ ] Every division, `sqrtf`, `logf`, `atan2f` argument is guarded; no NaN or Inf can reach an output pin.
- [ ] Angles wrapped where they are compared or subtracted (`mod`, `minus` in `angle.h`), signs right in both directions of motion and with `conf0.out_rev` / `mot_fb_rev`.
- [ ] Integrators have anti-windup and reset on disable; filters reset on enable.
- [ ] State machines handle disable and fault from every state.
- [ ] Outputs that command current, voltage or a brake are clamped and safe at 0 / disabled.

## rt code

- [ ] No `printf`, blocking or unbounded loops in the `rt` / `frt` functions.
- [ ] Time comes from `period`, not an assumed rate.
- [ ] Cost measured with `hal` (`hv hal` on the F3) when the rt path grew.

## Compatibility

- [ ] New features default off; defaults are neutral, not one motor's values.
- [ ] No pin renamed or removed that a config in `conf/` uses (grep it).
- [ ] F4-F3 packet layout only appended to.
- [ ] Templates set `rt_prio` for every component they load.

## Docs (same commit)

- [ ] Component `/** */` block, pin comments (unit, default, no `|`), template `#` header and wiki page updated.
- [ ] Error table (`docs/stmbldoc/content/docs/errors.md`) updated if a fault code or its cause changed.

## Hardware claims

- [ ] Anything said about behaviour on a real drive is marked measured (with the capture) or inferred.
- [ ] Nothing in the change needs a board flashed or a motor run to be safe; if bench testing is needed, the test plan states axis, range and stop conditions.
