# Notes for AI coding agents

Rules for any AI agent (Claude Code, Codex, Cursor, Copilot, Gemini CLI, ...) working in this repository. People are welcome to read it too. Step-by-step procedures are in [docs/procedures/](docs/procedures/).

## What this is

STMBL is servo drive firmware for two MCUs on one drive:

- **F4** (LV board, STM32F405): HAL runtime, control loops, feedback, USB terminal, smart serial to LinuxCNC. App at 0x08010000, STMBL bootloader at 0x08000000, saved config in between.
- **F3** (HV board, STM32F303): PWM, current loop, overcurrent trips. Its image is embedded in the F4 app and sent with `hv_update`.

| Path | Contents |
|---|---|
| `shared/` | HAL core (`hal.c`, `hal.h`) and components used on both MCUs (`shared/comps/`) |
| `src/`, `src/comps/` | F4 app and F4-only components (`hv`, `sserial`, `encf`, `conf`, ...) |
| `stm32f303/` | F3 app and its Makefile |
| `bootloader/`, `f3_boot/` | F4 and F3 STMBL bootloaders |
| `conf/` | Board configs; `conf/template/` holds the `link <name>` templates |
| `tests/` | Python doc-tool tests; `tests/sim/` runs the real firmware on a PC against a motor model |
| `docs/stmbldoc/` | Hugo wiki; component pages are generated from the C sources |
| `tools/` | Build generators, `bootloader.py`, `sserial_advance.py` |

## Working on code

- Build every firmware change before calling it done: `make -j16 DOCS=0` builds both MCUs. Details and the GCC version check: [build.md](docs/procedures/build.md).
- Run `make format` (clang-format) before a pull request.
- Run the host simulation for anything that touches control, feedback or fault paths: `make -C tests/sim test`. Doc tool tests: `python3 -m unittest discover -s tests`.
- The F3 image fills most of its flash and its CCM RAM. Check the size output after F3 changes and on a new compiler.
- Keep rt code fast: the F4 and F3 rt loops have fixed budgets. The `hal` command (and `hv hal`) prints the time per component.
- The F4-F3 packet layouts are append-only: add new words at the end so an older image on the other side still parses the rest.
- `HAL_MAX_COMPS` is 32.

## Docs go in the same change

The docs live in the code. A change to a component, template, config or user-visible behaviour updates, in the same commit:

- the component's single `/** ... */` block (one per `.c` file; `docs/tools/create_comp_doc.py` concatenates every `/**`),
- the trailing `//` comment on each new or changed `HAL_PIN` line (no `|` in the text; it ends up in a table),
- the `#` header of the `conf/` template or config,
- the user page in `docs/stmbldoc/content/docs/` when setup, tuning or errors change.

Comment-only commits can be checked with `arm-none-eabi-gcc -fpreprocessed -dD -E -P` before and after: the output must not change. Don't repeat pin comments inside the doc block. No em-dashes.

## Hardware and safety

This firmware drives motors on real machines. An agent must never start motion, release a brake, enable a drive, flash a board or save a config on its own initiative: those steps need the operator's explicit go-ahead, each time. Read [bench-safety.md](docs/procedures/bench-safety.md) before suggesting any command that is typed into a live drive.

## Procedures

- [build.md](docs/procedures/build.md): toolchain, make targets, GCC 12.2 upstream check
- [flashing.md](docs/procedures/flashing.md): F4 app, bootloader, F3 `hv_update`, config backup and restore
- [bench-safety.md](docs/procedures/bench-safety.md): brakes, resets, HV-off traps, drive-local moves
- [commissioning.md](docs/procedures/commissioning.md): new axis, from limits to `pos_advance`
- [capture-and-analysis.md](docs/procedures/capture-and-analysis.md): scope waves, triggered captures, recordings, fitting
- [faults-and-hal-traps.md](docs/procedures/faults-and-hal-traps.md): fault codes and HAL pitfalls

`.claude/skills/` holds thin Claude Code skills that point at these pages; the pages are the source, so edit them, not the skills.

User-facing documentation (setup, tuning, Servoterm, errors) is the wiki in `docs/stmbldoc/content/docs/`. Link to it rather than copying it.
