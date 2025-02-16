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

Interfaces:

* Mesa SmartSerial = full-duplex UART
* Encoder = QUAD
* Resolver = SIN/COS
* SSI, BiSS, EnDAT = half-duplex SPI
* Misu 02 = half-duplex UART
* Sanyo Denki = half-duplex UART
* Hyperface = SIN/COS + half-duplex UART

### Command connector wiring:

| Pin | Color     | Smart Serial | Step/Dir   | Quadrature  |
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

| Pin | Color    | Resolver | Encoder | 1Vpp   | UVW    | Mitsubishi | Sanyo-Denki | Yaskawa | Omron  |
|-----|----------|---------|---------|--------|--------|------------|--------------|---------|--------|
| 1   | Orange Stripe | Sin+     | A+      | Sin+    | U+       |            |              |         |        |
| 2   | Orange   | Sin-     | A-      | Sin-    | U-       |            |              |         |        |
| 3   | Green Stripe | Cos+     | B+      | Cos+    | V+       |            |              |         |        |
| 4   | Blue     | Ref-     | Z-      |        | W-       | 2          | Blue        | 6        | 4      |
| 5   | Blue Stripe | Ref+     | Z+      |        | W+       | 1          | Brown       | 5        | 7      |
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