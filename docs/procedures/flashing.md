# Flash a drive

User instructions: [introduction.md, Flashing firmware](../stmbldoc/content/docs/introduction.md). This page is the short checklist. Flashing needs the operator's go-ahead every time.

## Before

1. **Only one drive on USB.** `tools/bootloader.py` (used by `make btburn`) sends `bootloader` to every STMBL it finds, and dfu-util writes the first DFU device. Check the board with `about` and its USB serial number (`lsusb -v` or `/dev/serial/by-id/`) and compare it with the board you mean.
2. **Back up the config.** Copy the output of `showconf` to a file named after the board. `make btburn` keeps the config, but a backup costs nothing.
3. **Close Servoterm** and anything else that holds the serial port (loggers, LinuxCNC tools).
4. **Drive disabled**, motor at rest, a vertical axis on its brake.
5. **Take LinuxCNC out of the loop.** If the config uses smart serial (`link sserial`), stop LinuxCNC or remove the `sserial0` lines first, so nothing enables or commands the drive while it comes back up.

## F4 app

```sh
make -j16 DOCS=0
make btburn            # F4 app at 0x08010000, keeps bootloader and config
```

Then connect Servoterm and check `about`: the version and git hash must be the new build.

## F4 bootloader

Only when the bootloader itself changed, or once on boards with a bootloader older than v0.10.0 (those crash in the ROM on the bad-CRC and PA13 paths, so a failed app flash leaves the board invisible on USB):

```sh
make boot_btburn       # F4 bootloader at 0x08000000
```

**Don't use `make all_btburn` on a configured board.** It writes `f4.bin`, which carries `conf/festo.txt` as the config and replaces the saved one. It is for blank boards only.

## F3 (HV board)

The F3 image is inside the F4 app. After any F4 flash that changes F3 code (or the PWM frequency):

1. Make sure HV (or the low-voltage HV-side supply) is **on**: the F3 is powered from the HV side. With it off, the F4 shows the last `hv0.dc_volt` frozen and nothing answers. A live link shows `hv0.dc_volt` moving and `hv0.pwm_freq` > 0.
2. Drive disabled, then `hv_update`. It must end with `SLAVE_IN_APP`. A failure right after an F4 flash (link still in fault 9) usually works on a retry some seconds later.
3. `hv_verify` (drive disabled) compares the F3 flash with the embedded image.
4. `hv about` shows the F3 version.

Flashing the F3 over its own USB port (`make f3_all_btburn`) is for a blank HV board, and only with the HV board on an isolated low-voltage supply.

## After

1. `about` and `hv about` show the expected builds.
2. `showconf` matches the backup; `confcrc` if you keep CRCs.
3. If you removed `sserial0` lines, restore them (`appendconf <line>` or the config editor), then `flashsaveconf` with the drive disabled and `reset`.
4. Check the fault LED and `fault0.state` before the operator enables.
