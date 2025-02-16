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