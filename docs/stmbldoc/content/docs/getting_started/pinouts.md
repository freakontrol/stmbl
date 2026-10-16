---
title: "Pinouts"
weight: 1
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

## Pinouts

Interfaces supported by the current firmware (with the config template or component that reads them):

* Mesa SmartSerial = full-duplex UART (`link sserial`)
* Encoder = QUAD (`link enc_fb0`, component `enc_fb`), optionally with analog SIN/COS tracks (1Vpp)
* Resolver = SIN/COS (`link res_fb0`, component `res`)
* Hall / commutation signals UVW (`link uvw_fb0`, component `uvw`)
* Mitsubishi (mit 02) = half-duplex UART (`link encm_fb0`, component `encm`)
* Sanyo Denki = half-duplex UART (`link encs_fb0`, component `encs`)
* Tamagawa SmartABS = half-duplex UART (`link smartabs_fb0`, component `smartabs`)
* Fanuc serial = half-duplex UART (`link fanuc_fb0`, component `encf`)
* Yaskawa Sigma serial (component `yaskawa`, used by the `conf/sgmph-*.txt` configs)
* DMM = RS485 (`link encdmm_fb0`, component `dmm`)
* EnDat = half-duplex SPI (component `endat`, experimental, no template loads it)

The hardware can also carry SSI, BiSS (half-duplex SPI) and Hyperface (SIN/COS + half-duplex UART), but there is no firmware for them.

### Command connector wiring:

{{% hint info %}}
`link enc_cmd` decodes quadrature by default. For Step/Dir set `enc_cmd0.mode = 1` (step on pins 1/2, dir on 3/6; `mode = 2` swaps them). Up/down is not supported. A lost direction change gives fault 1 (CMD error). The Enable input on pins 7/8 is `io0.C78`, which no config template links. Link it by hand if you need it.
{{% /hint %}}

| Pin | Color     | Smart Serial | Step/Dir (`enc_cmd0.mode = 1`) | Quadrature  |
|-----|-----------|--------------|------------|-------------|
| 1   | Orange Stripe | RX+        | Step+      | A+          |
| 2   | Orange    | RX-        | Step-      | A-          |
| 3   | Green Stripe | TX+        | Dir+       | B+          |
| 4   | Blue      |              | Err-       |             |
| 5   | Blue Stripe |              | Err+       |             |
| 6   | Green     | TX-        | Dir-       | B-          |
| 7   | Brown Stripe |            | Enbl+      |             |
| 8   | Brown     |            | Enbl-      |             |

### Feedback connector wiring - encoders etc

In the Mitsubishi, Yaskawa and Omron columns the numbers are the pin numbers on the encoder's own connector. In the Sanyo-Denki column the entries are the wire colours of the encoder cable. In the 1Vpp column, pins 4/5 carry the optional index Z-/Z+.

| Pin | Color    | Resolver | Encoder | 1Vpp   | UVW    | Mitsubishi | Sanyo-Denki | Yaskawa | Omron  |
|-----|----------|---------|---------|--------|--------|------------|--------------|---------|--------|
| 1   | Orange Stripe | Sin+     | A+      | Sin+    | U+       |            |              |         |        |
| 2   | Orange   | Sin-     | A-      | Sin-    | U-       |            |              |         |        |
| 3   | Green Stripe | Cos+     | B+      | Cos+    | V+       |            |              |         |        |
| 4   | Blue     | Ref-     | Z-      | Z- (optional) | W-       | 2          | Blue        | 6        | 4      |
| 5   | Blue Stripe | Ref+     | Z+      | Z+ (optional) | W+       | 1          | Brown       | 5        | 7      |
| 6   | Green    | Cos-     | B-      | Cos-    | V-       |            |              |         |        |
| 7   | Brown Stripe | AIN      | VCC     | VCC     | VCC       | VCC        | Red         | 1        | 6      |
| 8   | Brown    | GND      | GND     | GND     | GND       | GND        | Black       | 2        | 3      |

### Connector wiring - serial protocols

| Pin | Color    | RS485   | RS422   | UART    | USART   | UART HD  | USART HD | SPI     | SPI HD   |
|-----|----------|---------|---------|---------|---------|----------|----------|---------|----------|
| 1   | Orange Stripe |         | A       | RX+      | RX+      |          |          | MISO+    | CS+      |
| 2   | Orange   |         | B       | RX-      | RX-      |          |          | MISO-    | CS-      |
| 3   | Green Stripe |         |         |         | CLK+     |          | CLK+     | CLK+     | CLK+     |
| 4   | Blue     | B       | Z       | TX-      | TX-      | TX/RX-   | TX/RX-   | MOSI-    | MOSI-    |
| 5   | Blue Stripe | A       | Y       | TX+      | TX+      | TX/RX+   | TX/RX+   | MOSI+    | MOSI+    |
| 6   | Green    |         |         |         | CLK-     |          | CLK-     | CLK-     | CLK-     |
| 7   | Brown Stripe | VCC      | VCC      | VCC      | VCC      | VCC      | VCC      | VCC      | VCC      |
| 8   | Brown    | GND      | GND      | GND      | GND      | GND      | GND      | GND      | GND      |