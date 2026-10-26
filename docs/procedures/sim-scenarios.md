# Host simulation scenarios

`tests/sim` runs the real F4 and F3 firmware (HAL, shared components, templates, board configs) on a PC against a motor and load model. The scenario language, plant parameters and limits are in [tests/sim/README.md](../../tests/sim/README.md); read it first.

```sh
make -C tests/sim                 # build tests/sim/build/stmbl_sim
make -C tests/sim test            # run every scenario; CSV and logs in tests/sim/build/out/
tests/sim/build/stmbl_sim -v tests/sim/scenarios/y_move.txt
python3 tests/sim/plot.py tests/sim/build/out/y_move.csv -o y_move.png
```

## When to add one

- Every change to control, feedback, fault or identification code gets a scenario that fails without the change and passes with it, or an existing scenario whose `expect` lines cover it.
- A bench finding (a ring, a stop error, a wrong identified value) becomes a scenario once it is understood, so it can't come back unnoticed.
- Before changing a default, run the whole suite: a default moves every config that relies on it.

## Writing one

Copy the closest scenario in `tests/sim/scenarios/` and keep its naming (`y_*` horizontal axis, `z_*` vertical axis, `s_*` spindle, `id_*` identification). A scenario:

1. starts with comments that say what it tests and why the limits are what they are,
2. `title`, `config <file>`, then only the `f4` / `f3` / `plant` lines the test needs,
3. drives the axis (`lcnc enable`, `lcnc move ...`, or an id component) with `at T` times,
4. `trace`s the signals someone debugging a failure would want,
5. ends with `expect` lines and `run T`.

Rules for `expect` lines:

- Always `expect max fault0.last_fault < 0.5` over the active part.
- Set limits from the physics or the requirement, with margin, not from the number the current code happens to produce. Say in a comment where each limit comes from.
- For identification, compare the result with the plant's true value (`plant k=v` sets it) and allow the method's known error.
- Use `from` / `to` to skip start-up transients rather than loosening the limit.

## Reading results

- A failing `expect` prints the statistic and the limit; look at the CSV with `plot.py` before changing anything.
- The plant is not the machine: friction, couplings and the encoder are models. When the sim and the bench disagree, report it; don't tune the plant to make a scenario pass unless the bench data supports the new plant values (and say which data).
- The sim has no rt timing: it can't show an rt overrun. Check `hal` on a board for that.
