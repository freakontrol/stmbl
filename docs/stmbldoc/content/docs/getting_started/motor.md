---
title: "Motor"
weight: 4
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# Motor

{{% hint info %}}
It is possible to use [autotuning](/docs/getting_started/tuning.md) to estimate the motor parameters.  
You can still use this guide if autotuning fails.  
{{% /hint %}} 

## Motor parameters

Unfortunately, servo motor datasheets often lack a good description of the parameters. There is a difference between line-to-line and phase values. Here are listed the most important motor parameters and how to determine them.  

### Resistance (conf0.r)

1. Measure line-to-line resistance R
2. Convert to phase resistance conf0.r = R/2

### Inductance (conf0.l)

1. Measure line-to-line inductance L
2. Convert to phase inductance conf0.l = L/2

If you can't measure this, a rough guess is conf0.l = 0.001 × conf0.r (a 1 ms electrical time constant). Better, measure it with `link id_pmsm` (see [Tuning](/docs/getting_started/tuning.md#electrical-parameters-id_pmsm)).

For a salient (IPM) motor, set `conf0.l` to the d-axis inductance Ld and `conf0.lq` to the q-axis inductance Lq. `conf0.lq` = 0 means the same as `conf0.l`. `link id_pmsm` measures both.

### Moment of Inertia (conf0.j)

1. Read moment of inertia from datasheet

### Pole Pair Count (conf0.polecount)

1. Attach a power supply to the motor (limit the current)
2. Turn the motor one rotation and count the number N of positions it snaps in conf0.polecount = N

### Torque Constant (conf0.psi)

Set psi for every motor. It sets the torque to current gain (iq = torque / (1.5 · polecount · psi)) and the BEMF feedforward of the current loop, so the default of 0.055 is only right by chance. The [auto tuning](/docs/getting_started/tuning.md#electrical-parameters-id_pmsm) measures it. There is no common ground for the torque constant. Some manufacturers state the current in RMS, others in peak-to-peak. We also stumbled over values that were neither. How to determine psi:

#### Scope

1. Drive motor at constant speed (lathe, power drill, ...)
2. Connect an oscilloscope between two phases and measure the electrical frequency F (Hz) and the peak-to-peak line-to-line voltage U0 (V)
3. Convert to line-to-neutral voltage U1 = U0/sqrt(3)
4. Convert peak-to-peak to amplitude U2 = U1/2
5. Convert to torque constant conf0.psi = U2/F/2.0/PI

#### STMBL

It's also possible to measure psi directly with STMBL. Maybe even automatically in a future version.

1. Disconnect the HV power, leave the servo connected.
2. Connect with Servoterm and paste these lines into the command line one by one, or drag and drop them as a file into Servoterm. They are the same as [conf/experimental/psi.txt](https://github.com/freakontrol/stmbl/blob/main/conf/experimental/psi.txt) and end with `stop` and `start`, which restart the realtime system:
   ```python
   load psi
   psi0.rt_prio = 10
   psi0.dc_volt = hv0.dc_volt
   psi0.u = hv0.u_fb
   psi0.v = hv0.v_fb
   psi0.w = hv0.w_fb
   psi0.vel = vel1.vel
   psi0.polecount = conf0.polecount
   fault0.en = 0
   fault0.brake_release = 1
   vel1.en = 1
   term0.wave0 = psi0.psi
   term0.wave1 = psi0.max_psi
   term0.gain0 = 500
   term0.gain1 = 500
   stop
   start
   ```
3. Turn the shaft of the motor. You can use your hand for a rough estimation, but it's probably better to drive with a cordless drill. The closer to the nominal RPM rating, the more accurate the result will be. There is no need to turn the shaft continuously, a quick short turn of one revolution is sufficient.
4. Type `psi0.max_psi` and you will get the peak psi value that was measured. You can set `conf0.psi` to the measured peak psi value now. If you did not drive the motor reasonably fast, the proper psi value is probably a couple percent higher.

#### KV

psi = 60.0 / #POLE_PAIRS / sqrt(3) / 2.0 / PI / KV

This formula is for a hobby style KV in rpm per volt, with the volt measured line to line (peak or DC). It does not fit a back EMF constant in V rms per 1000 rpm, as industrial datasheets give it. Use the datasheet section below for those.

#### Nm/A

psi = (Nm/A) / 3.0 \* 2.0 / #POLE_PAIRS

This formula needs the torque constant in Nm per peak ampere. Divide a value given in Nm per A rms by √2 first.

#### From a datasheet (Fanuc example)

STMBL uses peak phase current and peak phase flux: torque = 1.5 · polecount · psi · i_peak. Industrial datasheets usually give the torque constant per A rms and the back EMF constant in rms volts, so convert both to peak. Fanuc gives Kt in Nm/A rms and Ke in V rms per 1000 min⁻¹ phase to neutral (Kv is the same value in V·s/rad).

Example: Fanuc α3/3000 (A06B-0123), datasheet B-65142E/02: Kt = 0.65 Nm/A rms, Ke = 23 V rms / 1000 min⁻¹, Kv = 0.22 V·s/rad, 4 pole pairs.

* From Kt: psi = Kt / (1.5 · polecount · √2) = 0.65 / (1.5 · 4 · 1.414) = 0.0766
* From Kv: psi = Kv · √2 / polecount = 0.22 · 1.414 / 4 = 0.0778
* From Ke: 1000 min⁻¹ = 104.72 rad/s, so Kv = 23 / 104.72 = 0.2196 V·s/rad and psi = 0.2196 · √2 / 4 = 0.0777

Ke and Kv are the same number in different units, so a datasheet usually gives two independent routes (Kt and Ke), and they should agree within a few percent.

Check whether the back EMF constant is phase or line to line with Kt ≈ 3 · Kv (Kv in phase rms V·s/rad). Here 3 · 0.2196 = 0.659 against Kt 0.65, so Ke is phase rms. Read as line to line, Kv would be 0.2196 / √3 = 0.127 and 3 · 0.127 = 0.380, far from 0.65. For a line to line rms value use psi = Kv · √2 / √3 / polecount.

Datasheet values have about ±10 % tolerance, and a used motor can have a weaker rotor, so measure psi with the [auto tuning](/docs/getting_started/tuning.md#electrical-parameters-id_pmsm) or the scope method when you can. With psi known, the torque limit for a current limit is conf0.max_force = 1.5 · polecount · psi · conf0.max_ac_cur.

### PID

The STMBL PID works differently than common PID loops: the gains are set as bandwidths. Understand the [pid component](/docs/hal_components/pid.md) and its interaction with the [motor model](/docs/hal_components/pmsm_limits.md) first. The [control loop tuning](/docs/getting_started/tuning.md#control-loop-id_pid) (`link id_pid`) helps to find the values. The defaults come from the conf template.

* conf0.pos_bw position loop bandwidth (rad/s/rad, default 25)
* conf0.vel_bw velocity loop bandwidth (default 250)
* conf0.vel_d velocity loop damping (default 5)
* conf0.vel_g velocity loop proportional limit scaling (default 0.1)
* conf0.torque_g torque loop proportional limit scaling (default 1)
* conf0.cur_bw current loop bandwidth (rad/s, default 3000)
* conf0.cur_ff current loop resistance feedforward gain
* conf0.cur_ind current loop BEMF feedforward gain

### Command

* conf0.cmd_rev command reverse
* conf0.cmd_res command resolution (1/rev)

### Limits

Exceeding a limit results in an action. The defaults come from the conf template.

* conf0.max_ac_cur max phase current (A peak, default 10)
* conf0.max_dc_cur max DC link current (A, default 15)
* conf0.max_force max torque (Nm, default 5)
* conf0.max_vel max velocity (rad/s, default 320)
* conf0.max_acc max acceleration (rad/s², default 80000)
* conf0.max_dc_volt voltage limit (V, default 380): disable drive
* conf0.high_dc_volt voltage limit (V, default 350): current derating starts here and reaches 0 at max_dc_volt. STMBL has no brake chopper; size the DC link capacitance or add an external chopper to absorb regenerated energy.
* conf0.low_dc_volt undervoltage limit (V, default 24): fault 11 while enabled
* conf0.max_hv_temp temperature limit (°C): disable drive
* conf0.high_hv_temp temperature limit (°C): reduce max current
* conf0.fan_hv_temp temperature limit (°C): activate fan
* conf0.max_pos_error max position error (rad): disable drive
* conf0.max_sat max saturation time (s): disable drive
* conf0.high_mot_temp / conf0.max_mot_temp motor temperature from the I²t model (°C, default 80 / 100): reduce max current / disable drive
* conf0.high_ipm_temp / conf0.max_ipm_temp power module junction temperature model (°C, default 125 / 140): reduce max current / disable drive

Setting conf0.max_ac_cur or conf0.max_dc_cur to 0 makes the derating factor `fault0.scale` 0, so the motor has no torque.

## Config

A motor template alone is not a working config. A typical config links, in this order:

```python
link pid
link pmsm
link enc_fb0
link sserial
link misc
```

1. `link pid`: the controller, limits and the `conf` component
2. the motor template: `pmsm`, `dc` or `acim_foc`
3. the feedback template, e.g. `enc_fb0` or `res_fb0`
4. the command source: `sserial` for LinuxCNC, or `jog_cmd` for jogging in Servoterm
5. `link misc` for the Servoterm scope

followed by the `conf0.*` lines for the motor. See the configs in the [conf folder](https://github.com/freakontrol/stmbl/tree/main/conf) for complete examples.

### DC motor

```python
link dc
```

### PMSM motor

```python
link pmsm
```

### ASYNC motor

Field oriented control on an encoder (link it after `link pid`):

```python
link acim_foc
```

Open loop V/f, no feedback needed (a complete config of its own, without `link pid`):

```python
link vf
```

V/f with encoder slip control, so the speed follows the encoder (link your feedback template after it):

```python
link vf_enc
```

The older slip controller `link acim` (acim_ttc) is still available. See [Tuning](/docs/getting_started/tuning.md#induction-motor-acim) for the parameters each one needs.