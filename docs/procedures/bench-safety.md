# Bench safety rules

For any agent that suggests or types commands into a live drive (Servoterm, a serial script, a remote session). Also read the [Safety section of the tuning page](../stmbldoc/content/docs/getting_started/tuning.md#safety).

## The operator decides

- Never enable a drive, start motion, release a brake, flash or save a config without the operator's explicit go-ahead for that step. A go-ahead for one step does not cover the next.
- Before any move, state the axis, the direction, the range and the speed, and wait for the go-ahead. The same goes for LinuxCNC programs and MDI moves: give the program path and what it moves.
- Keep a way to stop at hand: `fault0.en = 0` (or `disable`, or Esc in Servoterm) aborts every id component and the planner.

## Brakes and gravity-loaded axes

- **Never release a holding brake unless the position loop is running and holding the axis.** `fault0.brake_release > 0` forces the brake open in every state except a fault, including with the drive disabled. On a vertical axis that drops the load.
- An id run that sets `fault0.brake_release = 1` leaves the brake released after the run. Disable, check the brake engaged (`fault0.mot_brake` = 0), then `reset` to clear it.
- **Reset with HV on only when the drive is disabled and the brake is engaged** (`fault0.mot_brake` = 0). A reset while enabled drops the axis for the boot time.
- For electrical id tests (`id_pmsm`, `id_tune`) the load should be off the shaft; on a brake-held axis that usually means doing them with the motor off the machine or the axis supported mechanically.
- Short brake (`fault0.sbrake_en`) and a standstill hold heat the modelled IPM junction. A "WARNING: over temperature (ipm junction) current clamping active" message during a long hold or brake is expected, not a fault.

## HV off: the frozen-reading trap

The F3 (HV board) is powered from the HV side. With HV off:

- the F4 keeps showing the **last** `hv0.dc_volt`, `hv0.*` temperatures and currents; they do not drop to 0,
- `hv_update`, `hv_verify` and every `hv ...` command get no answer, and the link reports fault 9.

So **check that HV is on before** blaming the link, suggesting `hv_update` or reading any `hv0` value. A live link shows `hv0.dc_volt` changing and `hv0.pwm_freq` > 0. When HV comes on after the F4 has booted, the `hv ...` lines of the config (F3-side settings) were sent into nothing: resend them or `reset`.

## Drive-local moves

- Move the axis from the drive (without LinuxCNC) only through a ramped planner, for example `link jog_cmd` (`stp0.target`, `stp0.max_vel`, `stp0.max_acc`). Never step a position command directly.
- Set `stp0.min_pos` / `stp0.max_pos` to the free travel before the first move. The planner works in motor rad, relative to where the encoder counts from.
- If the config contains `sserial0` lines, LinuxCNC owns enable and command. Take them out (and keep a copy) before drive-local tests, flashing or `stop` / `start` runs, and put them back afterwards.

## Configs

- Check the USB serial number or `about` before every `flashsaveconf`. A config saved to the wrong board is a silent, dangerous mistake.
- Save only with the drive disabled.
- Keep a dated copy of every config you change, per board.

## Faults

- Read the fault code first (`ERROR: Fault N` in Servoterm, `fault0.last_fault`, the LED blinks): see [faults-and-hal-traps.md](faults-and-hal-traps.md).
- Clear a fault with `fault0.en = 0` after fixing the cause. Don't raise a trip level to get past a fault without understanding why it tripped.
