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

Here an example of linuxcnc configuration using smart serial.

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
COORDINATES =           X Y Z
LINEAR_UNITS =          mm
ANGULAR_UNITS =         degree

[EMCIO]
EMCIO =                 io
CYCLE_TIME =            0.100
TOOL_TABLE =            tool.tbl

[KINS]
KINEMATICS = trivkins
JOINTS = 3

[AXIS_X]
MIN_LIMIT = -1000.0
MAX_LIMIT = 1000.0
MAX_VELOCITY = 100
MAX_ACCELERATION = 20

[AXIS_Y]
MIN_LIMIT = -1000.0
MAX_LIMIT = 1000.0
MAX_VELOCITY = 100
MAX_ACCELERATION = 20

[AXIS_Z]
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

[JOINT_1]
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

[JOINT_2]
TYPE =              LINEAR
MAX_VELOCITY =      50
MAX_ACCELERATION =  100
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
HOME_SEQUENCE =         0

[HOSTMOT2]
DRIVER=hm2_eth board_ip="192.168.1.121"
BOARD=7i92
CONFIG="num_encoders=0 num_pwmgens=0 num_stepgens=0 sserial_port_0=00000000"
```