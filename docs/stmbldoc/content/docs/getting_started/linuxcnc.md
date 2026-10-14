---
title: "Linuxcnc"
weight: 6
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# LinuxCNC

Here is an example of a LinuxCNC configuration for one axis, using Mesa smart serial.

## STMBL side

The drive config needs `link sserial` after the motor and feedback templates, for example:

```python
link pid
link pmsm
link enc_fb0
link sserial
link misc
```

The `sserial` template makes the drive a smart serial remote with the card name `stbl`. It:

* links `fault0.en = sserial0.enable`, so LinuxCNC enables and disables the drive, and sends the drive fault and its fault number (`sserial0.fault_code = fault0.last_fault`) back to LinuxCNC
* feeds the position and velocity command from LinuxCNC through `linrev0` and `vel_int0` into the controller, and sends the position from `linrev0` back to LinuxCNC
* derives an acceleration feedforward from the host frames: `vel_int0.acc_out` (the change of `vel_cmd` per frame, held until the next frame, 0 while the watchdog holds, clamped to `conf0.max_acc`) goes through `rev0.out_dd` to `pid0.acc_ext_cmd`. It is on by default; `vel_int0.acc_k` scales it (1 = as commanded, 0 = off)
* places the host frames on a smoothed frame clock, `vel_int0.sync_k = 0.05`: a frame that arrives an rt tick early or late (host jitter) no longer puts a sawtooth of `vel_cmd` x the timing error on the position command (10-20 um at 200 mm/s on Y). The mean offset is 0, so no latency is added; `sync_k = 1` snaps to each frame as before (see [vel_int](/docs/hal_components/vel_int.md))
* runs `sserial0` with rt_prio 2.3 and frt_prio 2.0, and sets `sserial0.pos_advance = 0.0003` (tune it per axis, see the last section)
* raises fault 1 (CMD_ERROR) if no valid packet arrives for 5 ms (`sserial0.timeout`, 100 frt cycles)

For a position axis (not a spindle) also set these two, both 0 by default:

```python
vel1.vel_p = 1 # velocity feedback without the observer's lag in acceleration
vel_int0.vel_ip = 1 # vel_cmd extrapolated through the frame
```

Without them the drive ran about 38 um ahead of LinuxCNC through every acceleration on the example Y axis ([fanuc_a3-3000.txt](https://github.com/freakontrol/stmbl/blob/main/conf/fanuc_a3-3000.txt) sets both). `vel_p` passes more encoder noise into the velocity.

`sserial_uf` (for the `uf` template, velocity feedback only) sets `sserial0.pos_extrap = 0`, so `pos_fb` is not extrapolated from `vel_fb`.

LinuxCNC reads the position at the servo rate, 1 kHz with `SERVO_PERIOD = 1000000` below, well inside the 5 ms timeout.

### Wiring

Connect the Mesa smart serial port to the STMBL command connector. The drive receives on pins 1/2 and transmits on pins 3/6, see [Pinouts](/docs/getting_started/pinouts.md). In the INI file below, `sserial_port_0=00000000` enables the smart serial channels of the first sserial port of the Mesa card. In the HAL pin names, `stbl.0.0` is port 0, channel 0.

### Remote pins

The `stbl` remote has these pins in LinuxCNC (prefix `hm2_<board>.0.stbl.<port>.<channel>.`):

| Pin | Direction | Meaning |
|-----|-----------|---------|
| `pos_cmd` | to drive | position command (machine units) |
| `vel_cmd` | to drive | velocity command (machine units per second) |
| `enable` | to drive | drive enable (`fault0.en`) |
| `out` | to drive | 4 bits: `io0.out0`, `io0.out1`, `io0.out2` and `fault0.brake_release` |
| `pos_fb` | from drive | position feedback (machine units) |
| `vel_fb` | from drive | velocity feedback |
| `current` | from drive | q-axis current (A, ±30 A range) |
| `in` | from drive | 4 bits, the first two are `io0.ind0` and `io0.ind1` |
| `fault` | from drive | drive fault |
| `fault_code` | from drive | fault number while `fault` is set (the blink count in [Errors](/docs/errors.md)), else 0 |
| `index_enable` | both | index homing |
| `scale` | to drive | machine units per motor revolution |

`index_enable` homes to the encoder index: LinuxCNC sets it, `idx_home0` on the drive clears it when the index is found and resets the revolution count. To use it, uncomment the `xindex` line in the HAL file and the homing settings, including `HOME_USE_INDEX = YES`, in `[JOINT_0]`.

{{% hint warning %}}
Servoterm's Enable, Disable and Esc set `fault0.en` to a constant, which removes the `fault0.en = sserial0.enable` link. After that, LinuxCNC no longer enables the drive. With LinuxCNC, do not use Servoterm's Enable or Disable. If you did, type `reset` in Servoterm.

Saving the drive config (`flashsaveconf`, or Save in the Config window) erases a flash sector and stalls the drive for longer than the 5 ms smart serial timeout, so the drive faults. Save the drive config only with the machine off in LinuxCNC.
{{% /hint %}}

## Hal file

```c++
loadrt [KINS]KINEMATICS
loadrt [EMCMOT]EMCMOT servo_period_nsec=[EMCMOT]SERVO_PERIOD num_joints=[KINS]JOINTS
loadrt hostmot2
loadrt [HOSTMOT2](DRIVER) config=[HOSTMOT2](CONFIG)
loadrt estop_latch

setp    hm2_[HOSTMOT2](BOARD).0.watchdog.timeout_ns 50000000

addf hm2_[HOSTMOT2](BOARD).0.read         servo-thread
addf motion-command-handler               servo-thread
addf motion-controller                    servo-thread
addf hm2_[HOSTMOT2](BOARD).0.write        servo-thread
addf estop-latch.0                        servo-thread

net xposcmd joint.0.motor-pos-cmd  => hm2_[HOSTMOT2](BOARD).0.stbl.0.0.pos_cmd
net xvelcmd joint.0.vel-cmd        => hm2_[HOSTMOT2](BOARD).0.stbl.0.0.vel_cmd
net xposfb  joint.0.motor-pos-fb  <=  hm2_[HOSTMOT2](BOARD).0.stbl.0.0.pos_fb
net xenable joint.0.amp-enable-out => hm2_[HOSTMOT2](BOARD).0.stbl.0.0.enable
net xfault  joint.0.amp-fault-in  <=  hm2_[HOSTMOT2](BOARD).0.stbl.0.0.fault
#net xindex  joint.0.index-enable  <=> hm2_[HOSTMOT2](BOARD).0.stbl.0.0.index_enable
setp hm2_[HOSTMOT2](BOARD).0.stbl.0.0.scale 6

# A basic estop loop that only includes the hostmot watchdog.
net user-enable iocontrol.0.user-request-enable => estop-latch.0.reset
net enable-latch estop-latch.0.ok-out => iocontrol.0.emc-enable-in
net watchdog hm2_[HOSTMOT2](BOARD).0.watchdog.has_bit => estop-latch.0.fault-in

# create signals for tool loading loopback
net tool-prep-loop iocontrol.0.tool-prepare => iocontrol.0.tool-prepared
net tool-change-loop iocontrol.0.tool-change => iocontrol.0.tool-changed

```

`scale` is the number of machine units per motor revolution, for example 5 for a 5 mm pitch ball screw driven directly by the motor (6 in this example). It is sent to the drive in every process data packet, as `sserial0.scale`, and used by `linrev0`. It travels as a 24 bit unsigned number in steps of 0.0001, so it can be 0.0001 to about 1677.7; a 0 leaves the drive's scale as it is. Do not set `linrev0.scale` on the drive. While the scale is below 0.01 in magnitude (for example 0 before LinuxCNC has set it), the drive ignores the command and the motor does not move.

## Last commissioning step: feedback timing (sserial0.pos_advance)

LinuxCNC compares its position command with the position the drive reports
in the same servo period. The drive extrapolates its reply by
`vel_fb * (sserial0.pos_advance + its own snapshot age)`; `pos_advance` covers
the delays the drive cannot see: when hm2 reads and writes in the servo
thread, the encoder's transport time, and the drive's loop following its
observer rather than the raw encoder. A wrong value shows as a following
error proportional to velocity (the same sign as the velocity, flat during
constant feed), not as noise.

The sserial template sets 0.0003 s. Tune it per axis once the motor and loop
are identified (id_pmsm, id_tune, id_sys, id_pid, `vel1.vel_p`,
`vel_int0.vel_ip`), and again after any firmware or loop change:

1. With the machine on and homed, run a back and forth move on one joint at a
   steady feed, e.g. 20 repeats of a 150 mm move at 50-100 % of the joint's
   speed, through `tools/sserial_advance.py` on the LinuxCNC computer:

   ```
   tools/sserial_advance.py --joint 1 --advance 0.0003 --program /abs/path/move.ngc
   ```

   (or without `--program`, log `--time 60` s while you jog).
2. It fits `ferror = a * vel + b * acc + c` and prints
   `sserial0.pos_advance = <advance + a>`. Set it on the drive, rerun: `a`
   should come out within a few us of 0 and the cruise means near 0.
   Runs scatter by about +-15 us (on Y, 0.00029 read -1 us once and -17 us
   the next run), so changes under about 20 us are not worth making.
3. Add the line to the axis config and save.

Example (Y, 7i94, Fanuc aA64 encoder, 1 kHz servo): 0.0002 gave
`a = +89 us` (+18 um at 200 mm/s), 0.00029 gave `a = -1 us` and the
following error rms fell from 18 to 11 um.

## INI file

This INI file has one joint (X). For more axes, add a `[JOINT_n]` and an `[AXIS_<letter>]` section, raise `JOINTS`, add the axis letter to `COORDINATES`, and net the `stbl` pins of the next drive (`stbl.1.0`, `stbl.2.0`, ...) to `joint.n` in the HAL file.

```ini
[EMC]
VERSION = 1.1
MACHINE = stmbl
DEBUG = 0

[DISPLAY]
DISPLAY =              axis
CYCLE_TIME =            0.0500
POSITION_OFFSET =       RELATIVE
POSITION_FEEDBACK =     ACTUAL
MAX_FEED_OVERRIDE =     1.5
PROGRAM_PREFIX = ../../nc_files/
INTRO_GRAPHIC =         linuxcnc.gif
INTRO_TIME =            5
MAX_LINEAR_VELOCITY = 100
MAX_ANGULAR_VELOCITY = 50

[RS274NGC]
PARAMETER_FILE =        stmbl.var

[EMCMOT]
EMCMOT =                motmod
COMM_TIMEOUT =          1.0
SERVO_PERIOD =          1000000

[TASK]
TASK =                  milltask
CYCLE_TIME =            0.010

[HAL]
HALFILE = stmbl.hal

[HALUI]
#No Content

[TRAJ]
COORDINATES =           X
LINEAR_UNITS =          mm
ANGULAR_UNITS =         degree

[EMCIO]
EMCIO =                 io
CYCLE_TIME =            0.100
TOOL_TABLE =            tool.tbl

[KINS]
KINEMATICS = trivkins
JOINTS = 1

[AXIS_X]
MIN_LIMIT = -1000.0
MAX_LIMIT = 1000.0
MAX_VELOCITY = 100
MAX_ACCELERATION = 20

[JOINT_0]
TYPE =              LINEAR
MAX_VELOCITY =       50
MAX_ACCELERATION =   100
BACKLASH =           0.000
MIN_LIMIT =             -1000.0
MAX_LIMIT =             1000.0
FERROR =     1
MIN_FERROR = 1
#HOME =                  0.000
#HOME_OFFSET =           0.10
#HOME_SEARCH_VEL =       0.10
#HOME_LATCH_VEL =        -0.01
#HOME_USE_INDEX =        YES
#HOME_IGNORE_LIMITS =    YES
HOME_SEQUENCE =         1

[HOSTMOT2]
DRIVER=hm2_eth board_ip="192.168.1.121"
BOARD=7i92
CONFIG="num_encoders=0 num_pwmgens=0 num_stepgens=0 sserial_port_0=00000000"
```