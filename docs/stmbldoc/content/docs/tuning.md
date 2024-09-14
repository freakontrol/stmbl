---
title: "Tuning"
weight: 4
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# Parameter Tuning

## Requirements

* [Supply](configuration/supply.md)
* [Servoterm](servoterm.md)
* [Feedback](configuration/feedback.md)
* [Motor](configuration/motor.md)

## Electrical Parameters

### DC Motor

#### Manual Tuning

```python
conf0.r # motor phase resistance
conf0.l # motor phase inductance
conf0.psi # torque constant [Nm/A] or [V/rad/s]
conf0.mot_fb_rev
```

#### Auto Tuning

This takes about 20 seconds. Disconnect the motor from the load. The motor will turn with a max velocity of `iddc0.test_vel (rad/s)` and apply a max current of `iddc0.test_cur (A)`. Follow the green instructions in Servoterm:

- Set your config to:

```python
link pid
link dc
link <your_feedback_type>
link misc
link id_dc
```

- Reset STMBL by typing:

```python
reset
```

- Start the tuning by typing:

```python
fault0.en = 1
```

- Lock the rotor and type:

```python
iddc0.state = 1.2
```

- Unlock the rotor and type:

```python
iddc0.state = 2.2
```

- Append the result (`conf0.r`, `conf0.l`, `conf0.psi`, opt. `conf0.out_rev`) to your config.
- Go to [Motor Mechanical Parameters](#motor-mechanical-parameters).

### PMSM Motor

#### Manual Tuning

```python
conf0.r # motor phase resistance
conf0.l # motor phase inductance
conf0.polecount # number of pole pairs
conf0.mot_fb_offset
conf0.out_rev
conf0.psi # torque constant [Nm/A] or [V/rad/s]
```

#### Auto Tuning

This takes about 20 seconds. Disconnect the motor from the load. The motor will turn with a max velocity of `idpmsm0.test_vel (rad/s)` and apply a max current of `idpmsm0.test_cur (A)`. Follow the green instructions in Servoterm:

- Set your config to:

```python
link pid
link pmsm
link <your_feedback_type>
link misc
link id_pmsm
```

- Reset STMBL by typing:

```python
reset
```

- Start the tuning by typing:

```python
fault0.en = 1
```

- Append the result (`conf0.r`, `conf0.l`, `conf0.psi`, `conf0.polecount`, `conf0.mot_fb_offset`, opt. `conf0.out_rev`) to your config.
- Go to [Motor Mechanical Parameters](#motor-mechanical-parameters).

## Motor Mechanical Parameters

### Manual Tuning

```python
conf0.j
```

### Auto Tuning

This takes about 2 minutes. Disconnect the motor from the load. The motor will move between `idm0.min_pos (rad)` and `idm0.max_pos (rad)` with a max velocity of `idm0.max_vel (rad/s)` and a max acceleration of `idm0.max_acc (rad/s^2)`. Follow the green instructions in Servoterm:

- Set your config to:

```python
link pid
link <your_motor_type>
link <your_feedback_type>
link misc
link id_mot
<your_electrical_parameters>
```

- Reset STMBL by typing:

```python
reset
```

- Start the tuning by typing:

```python
fault0.en = 1
```

- Append the result (`conf0.j`, `conf0.d`, `conf0.f`, `conf0.o`) to your config.
- Go to [System Mechanical Parameters](#system-mechanical-parameters).

## System Mechanical Parameters

You can skip this step if you want to tune the motor without load.

### Manual Tuning

```python
conf0.j_sys
conf0.j_lpf
```

### Auto Tuning

This takes about 2 minutes. Connect the motor to the load. The motor will move between `idm0.min_pos (rad)` and `idm0.max_pos (rad)` with a max velocity of `idm0.max_vel (rad/s)` and a max acceleration of `idm0.max_acc (rad/s^2)`. Follow the green instructions in Servoterm:

- Set your config to:

```python
link pid
link <your_motor_type>
link <your_feedback_type>
link misc
link id_sys
<your_electrical_parameters>
<your_mechanical_parameters>
```

- Reset STMBL by typing:

```python
reset
```

- Start the tuning by typing:

```python
fault0.en = 1
```

- Append the result (`conf0.j_sys`, `conf0.d`, `conf0.f`, `conf0.o`) to your config.
- Go to [Control Loop Tuning](#control-loop-tuning).

## Control Loop Tuning

### Manual Tuning

```python
conf0.pos_bw
conf0.vel_bw
conf0.vel_d
```

### Auto Tuning

This takes about 2 minutes. Connect the motor to the load. The motor will move between `ids0.min_pos (rad)` and `ids0.max_pos (rad)` with a max velocity of `ids0.max_vel (rad/s)` and a max acceleration of `ids0.max_acc (rad/s^2)`. Follow the green instructions in Servoterm:

- Set your config to:

```python
link pid
link <your_motor_type>
link <your_feedback_type>
link misc
link id_pid
<your_electrical_parameters>
<your_mechanical_parameters>
```

- Reset STMBL by typing:

```python
reset
```

- Start the tuning by typing:

```python
fault0.en = 1
```

- Append the result (`conf0.pos_bw`, `conf0.vel_bw`, `conf0.vel_d`) to your config.
- Go to [Cmd](configuration/cmd.md).