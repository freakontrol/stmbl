---
title: "Feedback"
weight: 2
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# Feedback

## Incremental Encoder

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
conf0.mot_fb_res = 4096
```

## Sin/Cos Encoder

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   | Sin+       |
| 2   | Sin-       |
| 3   | Cos+       |
| 4   | Ref- (optional) |
| 5   | Ref+ (optional) |
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

### STMBL FBx Pinout

| Pin | Signal     |
|-----|------------|
| 1   | Sin+       |
| 2   | Sin-       |
| 3   | Cos+       |
| 4   | Ref- (optional) |
| 5   | Ref+ (optional) |
| 6   | Cos-       |
| 7   | AIN        |
| 8   | GND        |

### Config

```python
link res_fb0
```

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

Mitsubishi servos such HA-FF usually have OBA17-051 or OBA17-052 encoders.

STMBL has support the "mit 02-4" and "mit 02-2" protocols for these encoders.  

* [some protocol description](https://github.com/freakontrol/stmbl/blob/main/src/comps/encm.c#L114 )  

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

### Config

## Panasonic absolute encoder

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