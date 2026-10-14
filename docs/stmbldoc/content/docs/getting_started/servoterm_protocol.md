---
title: "Servoterm protocol"
weight: 7
---

# Servoterm protocol v1 (float scope, capture, header, ack)

Drive -> host on the USB CDC port. Text and binary share the stream. A binary frame starts with a marker byte in
0xFB..0xFF, and every payload byte of a 0xFD/0xFC/0xFB frame is in 1..250, so markers are unique there and text
(ASCII/UTF-8) never contains them. (Base 250, not 253: digits 250..252 would have been bytes 0xFB..0xFD.)

## Encodings

- **d250:** an unsigned integer as base-250 digits, most significant first; each digit is stored as digit + 1
  (1..250).
- **f32:** an IEEE754 float's 32 bits as a 5-digit d250 (250^5 > 2^32).
- **u16:** a 2-digit d250 (0..62499).
- **u8:** a 1-digit d250 (0..249).

## Frames

| marker | name | payload | length |
|---|---|---|---|
| 0xFF | legacy 8-bit sample (term0.fmt 0) | 8 bytes, v + 128 clamped 1..253 (was 1..254; 254 = 0xFE collided with reset) | 9 |
| 0xFE | reset | none: clear the scope | 1 |
| 0xFD | float sample (term0.fmt 1) | u8 seq, 8 x f32 | 42 |
| 0xFC | header | 6 x f32: proto (1), kind (0 live, 1 capture), period [s], channels (8), count, pre | 31 |
| 0xFB | capture sample | u16 index (0..count-1), 8 x f32 | 43 |

- **Float values** are `(wave + offset) * gain`, the same number the 8-bit path rounds to a byte before adding 128.
  The host divides by 128 to get today's normalised -1..1 (it can go beyond ±1, never clipped).
- **seq** counts every live sample the drive took, modulo 250, including ones it had to drop. A jump of n > 1 means
  n - 1 samples were lost.
- **Live header (kind 0)** goes out after a 0xFE on:
  - connect (`con` 0 -> 1);
  - a change of fmt, send_step or the rt period;
  - the end of a capture upload.

  Its period is rt period x send_step; count and pre are 0.
- **Capture:** 0xFE, then a header (kind 1: period = rt period x cap_step, count samples, pre of them before the
  trigger), then count 0xFB frames in index order, then a live header. Live samples pause during the upload.

## Host -> drive

- Lines up to 127 bytes. A longer line is dropped whole and answered `line too long`.
- `#<n> <command>`: the command runs and `#<n> done` follows its output. `<n>` is decimal; a line starting with `#`
  and a non-digit is a comment, as before.

## Pins (term0)

- **fmt:** 0 = legacy 8-bit (default), 1 = float.
- **cap_arm:** write 1 to arm. It returns to 0 when the upload is done. Writing 0 aborts a capture, and so does setting fmt back to 0. A finished capture waits for a host to connect and is then sent whole.
- **cap_ch:** trigger channel 0..7.
- **cap_level:** trigger level, in the channel's own units (the raw wave).
- **cap_edge:** 1 rising, -1 falling, 0 = trigger at once.
- **cap_pre:** fraction of the buffer before the trigger, 0..0.95.
- **cap_step:** rt periods per capture sample (1 = 5 kHz on the F4).
- **cap_state:** 0 idle, 1 armed (filling pre-trigger), 2 waiting for the trigger, 3 filling post-trigger, 4 uploading.
- **cap_len:** samples in the buffer (1024 on the F4, 0 on the F3, which has no capture).
- **lost:** live samples dropped because the ring was full.
