---
title: "Motor"
weight: 3
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# Motor

## Motor parameters

Unfortunately, servo motor datasheets often lack a good description of the parameters. There is a difference between line-to-line and phase values. Here are listed the most important motor parameters and how to determine them.

### Resistance (conf0.r)

1. Measure line-to-line resistance R
2. Convert to phase resistance conf0.r = R/2

### Inductance (conf0.l)

1. Measure line-to-line inductance L
2. Convert to phase inductance conf0.l = L/2

If you can't measure this, set it to 0.001/(phase resistance).

### Moment of Inertia (conf0.j)

1. Read moment of inertia from datasheet

### Pole Pair Count (conf0.polecount)

1. Attach a power supply to the motor (limit the current)
2. Turn the motor one rotation and count the number N of positions it snaps in conf0.polecount = N

### Torque Constant (conf0.psi)

The default value works for most motors. Normally, you don't have to measure this! There is no common ground for the torque constant. Some manufacturers state the current in RMS, others in peak-to-peak. We also stumbled over values that were neither. How to determine psi:

#### Scope

1. Drive motor at constant speed (lathe, power drill, ...)
2. Connect an oscilloscope between two phases and measure frequency F (Hz) and peak, line-to-line voltage U0 (V)
3. Convert to line-to-neutral voltage U1 = U0/sqrt(3)
4. Convert to amplitude U2 = U1/2
5. Convert to torque constant conf0.psi = U2/F/2.0/PI

#### STMBL

It's also possible to measure psi directly with STMBL. Maybe even automatically in a future version.

1. Disconnect the HV power, leave the servo connected.
2. Connect with Servoterm, drag&drop this file into Servoterm: [https://github.com/freakontrol/stmbl/blob/master/conf/template/psi.txt](https://github.com/freakontrol/stmbl/blob/master/conf/template/psi.txt)
3. Type `stop` and `start`
4. Turn the shaft of the motor. You can use your hand for a rough estimation, but it's probably better to drive with a cordless drill. The closer to the nominal RPM rating, the more accurate the result will be. There is no need to turn the shaft continuously, a quick short turn of one revolution is sufficient.
5. Type `psi0.max_psi` and you will get the peak psi value that was measured. You can set `conf0.psi` to the measured peak psi value now. If you did not drive the motor reasonably fast, the proper psi value is probably a couple percent higher.

#### KV

psi = 60.0 / #POLE_PAIRS / sqrt(3) / 2.0 / PI / KV

#### Nm/A

psi = (Nm/A) / 3.0 \* 2.0 / #POLE_PAIRS

#### PID

The default values work for most motors. Normally, you don't have to tune this. The STMBL PID works differently than common PID loops. Understand the [code](https://github.com/freakontrol/stmbl/blob/master/shared/comps/pid.c) and its interaction with the [motor model](https://github.com/freakontrol/stmbl/blob/master/shared/comps/pmsm_limits.c) first!

* conf0.pos_p position pid proportional gain (1/s)
* conf0.vel_p velocity pid proportional gain (1/s)
* conf0.vel_i velocity pid integral gain
* conf0.cur_p current pid proportional gain (V/A)
* conf0.cur_i current pid integral gain
* conf0.cur_ff current pid resistance feedforward gain
* conf0.cur_ind current pid BEMF feedforward gain

#### Command

* conf0.cmd_rev command reverse
* conf0.cmd_res command resolution (1/rev)

#### Limits

Exceeding a limit results in an action.

* conf0.max_dc_volt voltage limit (V): disable drive
* conf0.high_dc_volt brake resistor limit (V) : activate brake resistor (Note: there is no built-in brake resistor in STMBL V3 or V4)
* conf0.low_dc_volt voltage limit (V): disable drive
* conf0.max_hv_temp temperature limit (°C): disable drive
* conf0.high_hv_temp temperature limit (°C): reduce max current
* conf0.fan_hv_temp temperature limit (°C): activate fan
* conf0.max_pos_error max position error (rad): disable drive
* conf0.max_sat max saturation time (s): disable drive

## Config

### DC motor

```python
link dc
```

### PMSM motor

```python
link pmsm
```

### ASYNC motor

```python
link acim
```