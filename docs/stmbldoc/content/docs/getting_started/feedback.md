---
title: "Feedback"
weight: 3
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# Feedback

## Incremental Encoder  

 An [incremental encoder](https://en.wikipedia.org/wiki/Incremental_encoder) operates on the principle of quadrature encoding, meaning it uses two channels of data to provide accurate measurement information. When the encoder is rotated or displaced, both output signals A and B change in a specific sequence that indicates direction and magnitude of movement. However, since it doesn't maintain an absolute position reference, it only provides relative changes from its initial state.  

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   | A+         |
| 2   | A-         |
| 3   | B+         |
| 4   | Z- (optional) |
| 5   | Z+ (optional) |
| 6   | B-         |
| 7   | VCC        |
| 8   | GND        |

### Config

```python
link enc_fb0
conf0.mot_fb_res = 4096 # counts per revolution = 4 x lines
```

An incremental encoder has no absolute angle, so the drive also needs commutation: either hall sensors on FB1 (`link uvw_fb1`) or a line saving (wire saving) encoder that sends U/V/W on A/B/Z at power-up (`link encws_fb0`, see below).

Without either, the drive autophases on every enable: `fb_switch` ramps a d current up to `conf0.phase_cur` (1 A in the conf template) and finds the angle within `conf0.phase_time` (0.5 s). The motor must be free to move a little while it does this.

With an index (Z) track, `link enc_fb0` sets `enc_fb0.en_index = 1`: at the first index pulse the encoder becomes absolute (state 3), and commutation moves to it with the `conf0.mot_fb_offset` measured by [id_pmsm](/docs/getting_started/tuning.md).

## Sin/Cos Encoder  

 A [sin-cos encoder](https://electronics.stackexchange.com/a/187115) is a type of rotary encoder that outputs two signals, sine and cosine waves, which are phase-shifted by 90 degrees. These signals can be used to determine the position and direction of rotation with high precision. The encoder generates these signals based on the angular position of its shaft relative to a datum point. By comparing the values of the sine and cosine outputs at any given time, it's possible to calculate the exact angle or position of the shaft.  

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   | Sin+       |
| 2   | Sin-       |
| 3   | Cos+       |
| 4   | Z- (optional) |
| 5   | Z+ (optional) |
| 6   | Cos-       |
| 7   | VCC        |
| 8   | GND        |

### Config

```python
link enc_fb0
conf0.mot_fb_res = 4096
fb_switch0.mot_pos = enc_fb0.ipos
```

## Resolver  

 A [resolver](https://en.wikipedia.org/wiki/Resolver_(electrical)) is an electrical device that converts the angular position of a rotating shaft into an output signal by using magnetic fields and coils. It generates two phase-shifted sine signals that can be used to determine the position and direction of rotation with high precision.  

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   | Sin+       |
| 2   | Sin-       |
| 3   | Cos+       |
| 4   | Ref- (excitation, required) |
| 5   | Ref+ (excitation, required) |
| 6   | Cos-       |
| 7   | AIN        |
| 8   | GND        |

### Config

```python
link res_fb0
conf0.mot_fb_polecount = 1 # resolver pole pairs, goes to res0.poles
```

The drive generates the excitation on Ref+/Ref-. Tune `res0.phase` for the largest `res0.amp`.

With more than one pole pair the position is not absolute after power-up: the resolver counts its electrical cycles from 0 at power-up, and a missed quadrant change shifts the position by one cycle.

## Tamagawa Smartabs
```
 _____
|2 4 6\
|1_3_5/
```

### Cable pinout

| Pin         | Signal   |
|-------------|----------|
| Red         | Vcc      |
| Black       | Gnd      |
| Brown       | Bat+     |
| Brown/Black | Bat-     |
| Blue        | D+       |
| Blue/Black  | D-       |
| Gray        | CASE GND |
| Shield      | NC       |

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   |            |
| 2   |            |
| 3   |            |
| 4   | Blue/Black |
| 5   | Blue       |
| 6   |            |
| 7   | Red        |
| 8   | Black      |


### Config

```python
link smartabs_fb0
```

## Mitsubishi

Mitsubishi servos such as HA-FF usually have OBA17-051 or OBA17-052 encoders.

STMBL supports these Mitsubishi serial encoders with the [encm](/docs/hal_components/encm.md) component (single-turn position only, the multiturn data is not decoded).

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   |            |
| 2   |            |
| 3   |            |
| 4   | 2          |
| 5   | 1          |
| 6   |            |
| 7   | VCC        |
| 8   | GND        |


### Config

```python
link encm_fb0
```

## Kawasaki - Sanyo Denki absolute encoder

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   |            |
| 2   |            |
| 3   |            |
| 4   | Blue       |
| 5   | Brown      |
| 6   |            |
| 7   | Red        |
| 8   | Black      |

### Config

```python
link encs_fb0
```

## Omron absolute encoder

{{% hint warning %}}
Wiring only. The current firmware has no component for the Omron serial protocol.
{{% /hint %}}

### Cable pinout
```
 _________
| 4 3 2 1 |
 \_7_6_5_/
 ```

| Pin | Signal     |
|-----|------------|
| 1   | Shield     |
| 2   | Bat-       |
| 3   | GND        |
| 4   | D-         |
| 5   | BAT+       |
| 6   | 5v         |
| 7   | D+         |

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   |            |
| 2   |            |
| 3   |            |
| 4   | 4          |
| 5   | 7          |
| 6   |            |
| 7   | 6          |
| 8   | 3          |

## Panasonic absolute encoder

{{% hint warning %}}
Wiring only. The current firmware has no component for the Panasonic serial protocol.
{{% /hint %}}

### Cable pinout
```
 _____
|1 2 3|
|4_5_6|
```

| Pin | Signal     |
|-----|------------|
| 1   | NC         |
| 2   | D+         |
| 3   | D-         |
| 4   | 5v         |
| 5   | GND        |
| 6   | Shield     |

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   |            |
| 2   |            |
| 3   |            |
| 4   | 3          |
| 5   | 2          |
| 6   |            |
| 7   | 4          |
| 8   | 5          |

## Fanuc serial pulse coder

Fanuc alpha series serial absolute encoders (pulse coder Aa64, A860-360, tested; Aa1000, A860-370, uses the same frame, but only the Aa64 layout is verified and the Aa1000's extra low resolution bits are not used) are read by the [encf](/docs/hal_components/encf.md) component at 1.024 Mbit/s. The drive sends a request on the REQ pair and the encoder answers on the SD pair. The encoder delivers a 22 bit single-turn position, a 16 bit turn counter (kept by the encoder battery) and a commutation track.

### STMBL FB0 Pinout

Only FB0 works (the pins are fixed in the firmware).

| Pin | Signal |
|-----|--------|
| 1   | SD (data from the encoder) |
| 2   | *SD |
| 3   |        |
| 4   | *REQ (request to the encoder) |
| 5   | REQ    |
| 6   |        |
| 7   | +5 V   |
| 8   | 0 V    |

The encoder battery (+6 V and 0 V) is wired on the encoder side as in the Fanuc amplifier manual; it does not go through STMBL. Take the encoder connector pin numbers from the Fanuc manual for your motor.

### Config

```python
link fanuc_fb0
```

The template sets `conf0.mot_fb_res = 1048576`, commutation from the encoder and `encf0.hold = fault0.en_out` (see below). Run [id_pmsm](/docs/getting_started/tuning.md) and append the `conf0.mot_fb_offset` and `conf0.com_fb_offset` lines it prints.

### Check

- `encf0.crc_ok` counts up and `encf0.crc_er` stays (almost) still. If `crc_er` counts and `crc_ok` does not, check the SD and REQ pairs (swapped or reversed pairs).
- `encf0.state` is 3 when the encoder is indexed. `encf0.batt` 1 means the encoder battery failed.
- `encf0.freq` (default 1024000) is the assumed bit rate. A bench sweep decoded clean frames from about 986000 to 1060000; leave it at the default.

### Battery loss and the index

After a battery loss the encoder starts un-indexed (`encf0.index` 1, `encf0.state` 1). Until the motor crosses the encoder's index, the drive commutates from the commutation track plus `conf0.com_fb_offset`, so it can run. At the first index crossing the encoder re-references its position and turn count: the position jumps by up to one motor turn, and the two directions of travel give positions one turn apart. If that happens while the drive is enabled, `encf` holds the jump as an offset on `encf0.pos` and `encf0.turns` (shown on `encf0.hold_off`, in turns), so the position loop does not pull the motor back by up to a turn. The offset is dropped when the drive is disabled, and the new reference then shows, with `encf0.state` 1 for one frame so `linrev` takes the new turns. Commutation (`abs_pos`, `com_pos`) is never offset. Re-home the machine after a battery loss.

### Multiturn position for LinuxCNC

To send the absolute multiturn position over Smart Serial, link the encoder turns into `linrev` after `link sserial`:

```python
linrev0.abs_en = 1
linrev0.abs_rev = encf0.turns
linrev0.abs_pos = encf0.pos
linrev0.abs_state = encf0.state
```

`link sserial` sets `linrev0.abs_neg = conf0.mot_fb_rev`, so a reversed motor feedback is handled.

[conf/fanuc_a3-3000.txt](https://github.com/freakontrol/stmbl/blob/main/conf/fanuc_a3-3000.txt) is a complete example: a Fanuc alpha 3/3000 axis with this encoder, the multiturn position and LinuxCNC over smart serial.

`encf0.pos_offset` (1/65536 turn) shifts the position and the point where `turns` steps; leave it at 0 and take the offset in LinuxCNC.

## Other feedback templates

| Template | Use |
|----------|-----|
| `link encws_fb0` | Line saving (wire saving) incremental encoder on FB0 that sends U/V/W on the A/B/Z lines at power-up, then switches to A/B |
| `link uvw_fb1` | Hall sensors U/V/W on FB1 for commutation, next to an incremental encoder on FB0 |
| `link uvw_fb0` | Hall sensors U/V/W on FB0 |
| `link fanuc_io` | Fanuc 4-bit commutation tracks C1/C2/C4/C8 on the CMD port, next to an incremental motor encoder (pinout not verified on real hardware) |
| `link encdmm_fb0` | DMM serial encoder ([dmm](/docs/hal_components/dmm.md)) |

Yaskawa Sigma serial encoders ([yaskawa](/docs/hal_components/yaskawa.md)) have no template; see `conf/sgmph-01a1.txt` or `conf/sda_b.txt` for the lines to load it.
