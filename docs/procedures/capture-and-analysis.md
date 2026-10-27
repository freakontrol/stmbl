# Capture and analyse drive data

How to get signals out of a drive and turn them into a conclusion. Details of the scope pins and the wire format: the `term` component (`shared/comps/term.c`) and the [Servoterm protocol page](../stmbldoc/content/docs/getting_started/servoterm_protocol.md).

## Pick the signals

- Up to 8 channels: `term0.wave0` .. `term0.wave7` = a pin, `term0.gainN` and `term0.offsetN` to scale it for the live view.
- **Point a wave at an output pin** (the source), for example `term0.wave0 = pid0.pos_error`, not at an input pin that is itself linked: a runtime link copies the pin instance, so the wave reads that input's own stored value (usually 0) instead of its source, until `relink`. See [faults-and-hal-traps.md](faults-and-hal-traps.md).
- Wave settings typed at the console are not saved unless you save the config. Keep scope lines out of the saved config unless they are meant to stay.
- The F3 sends no wave data (its `send_step` is 0). F3 values reach the F4 as `hv0.*` pins; scope those.
- Slow values (temperatures, fault state, fit results) are read by typing the pin name, for example `fault0.scale`, or the component name for all its pins (`iit0`).

## Live recording (Servoterm)

- Servoterm sets `term0.fmt = 1` (float samples, sample period in a header) on connect; older hosts use 8 bit samples.
- `term0.send_step` sets the live rate (one sample every N rt ticks). If `term0.lost` counts up, the host can't keep up: raise `send_step` or drop channels.
- **Data > Record** writes a CSV. Its `#` header carries the date, `send_step`, the scope period and each channel's pin, gain and offset. Values are in engineering units (gain and offset taken out). Empty cells are samples the drive lost; don't interpolate across them silently.

## Triggered capture (F4 only)

For fast events (a current step, a stop, a fault): 1024 samples of all 8 waves at up to the full rt rate, uploaded after the event.

```
term0.fmt = 1
term0.cap_step = 1      # rt ticks per sample
term0.cap_pre = 0.25    # share of the buffer before the trigger
term0.cap_ch = 0        # trigger on wave0
term0.cap_level = 0.5   # in the wave's own units
term0.cap_edge = 1      # 1 rising, -1 falling, 0 at once
term0.cap_arm = 1       # or the "Arm capture" button
```

`term0.cap_state` goes 1 (pre-trigger), 2 (waiting), 3 (post-trigger), 4 (upload), then back to 0. In a Servoterm recording the capture sits between `# capture period=..., pre=..., count=...` and `# end capture`. `cap_arm = 0` aborts.

## Context to record with every capture

A trace without its conditions can't be compared later. Save next to it:

- `about` (firmware version and hash) and `hv about`,
- the config (`showconf`) or its file name and CRC (`confcrc`),
- what was done: command, direction, distance, speed, load, brake state,
- HV on or off and the DC link voltage, motor and drive temperatures,
- the board (USB serial) and the date.

Name files `<board>_<what>_<date>.csv`.

## Analysis

- Work from the raw CSV with a script (Python with numpy / matplotlib is enough); keep the script with the result so it can be rerun.
- Convert to SI units first and check them: currents are peak phase A, torque Nm, positions motor rad, `pos_error` rad.
- For a fit (friction, inertia, `pos_advance`, a time constant) state the model, the sample count, the fit range and the residual. One run is an anecdote: repeat a measurement before drawing conclusions, and give the spread.
- Compare against a baseline taken the same way (same `send_step`, gains, conditions), not against numbers from memory.
- Say which parts of a conclusion were measured and which are inferred.
- To test an explanation without the machine, reproduce it in the host simulation (`tests/sim`): scenarios run the real firmware against a motor model, and `tests/sim/plot.py` plots their traces. Report when the sim and the bench disagree instead of tuning one to match.

## LinuxCNC side

- `halscope` / `halsampler` on the LinuxCNC host see the smart serial pins (following error, commanded vs fed back position) at the servo rate.
- `tools/sserial_advance.py` fits `sserial0.pos_advance` from LinuxCNC's following error during a move.
