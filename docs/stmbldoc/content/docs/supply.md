---
title: "Supply"
weight: 6
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

 # Power Supply Requirements for STMBL Board

The STMBL board is designed to operate with a specific range of voltage and current for its various components. Understanding these requirements is crucial for proper operation and safety. This page provides an overview of the power supply requirements for the STMBL board, including information about the big electrolytic capacitors needed in the power supply close to the HV board.

## Power Supply Specifications

The following table summarizes the power supply specifications for the STMBL board:

### PCB Version 5

| Connector | Abs. MIN | MIN | TYP | MAX | Abs. MAX | Unit | Note |
|-----------|----------|-----|-----|-----|----------|------|------|
| 24V       | 0        | 15  | 24  | 25  | 26       | V    |      |
| 24V       |          |     | 0.2 | 3   | 6        | A    | 1    |
| HV        | 0        | >24 | 320 | 380 | 400      | V    | 4    |
| HV        | -28      | -17 |     | 17  | 28       | A    | 2    |
| CMD       | -7       | 0   |     | 5   | 12       | V    |      |
| CMD       | -250     | -50 |     | 50  | 250      | mA   |      |
| FB VPP    | 0        | 0   |     | 12  | 24       | V    |      |
| FB VPP    | 0        | 0   | 500 | 1000| 1500     | mA   |      |
| FB diff.  | -7       | 0   |     | 5   | 12       | V    |      |
| FB diff.  |          | -1.2|     | +1.2|          | Vpp  | 3    |
| FB        | -250     | -50 |     | 50  | 250      | mA   |      |
| output    | 0        | 0   |     | 24  | 26       | V    |      |
| output    | 0        | 0   |     | 1   | 2        | A    |      |
| input     | -36      | -24 |     | 24  | 36       | V    |      |
| IO        | 0        | 0   |     | 5   | 5        | V    |      |
| UVW       | 0        | 0   |     | 380 | 400      | V    |      |
| UVW       | -60      | -35 |     | 35  | 60       | A    | 5    |
| UVW avg.  |          | -27 |     | 27  |          | A    | 2    |

*Note 1: Depends on brake, fan and encoder consumption*  
*Note 2: Depends on PCB and driver cooling*  
*Note 3: For analog input*  
*Note 4: MIN: while the drive is enabled, the DC link must stay above `conf0.low_dc_volt` (24V by default), or the drive gives fault 11. A 24V bench supply sits right at this limit, so for a low-voltage test set `conf0.low_dc_volt` lower, for example 12. Flashing firmware is not affected. MAX: with the default config, the current is reduced from `conf0.high_dc_volt` (350V) and the drive faults (11) above `conf0.max_dc_volt` (380V). Without the conf template, the fault component defaults are 370V and 390V. Abs. MAX: the F3 on the HV board trips at 400V.*  
*Note 5: 35A peak is the range of the current shunts and a fixed F3 trip (fault 15). The usual trip is lower: 1.3 × `conf0.max_ac_cur`. 60A is the peak rating of the IKCM30F60GD module.*

## Undervoltage and Regeneration

Undervoltage: if the HV supply drops below `conf0.low_dc_volt` (24V by default) while the drive is enabled, the drive gives fault 11 (see [Errors](/docs/errors.md)).

Regeneration: when the motor brakes, its energy flows back into the DC link and raises the voltage. The firmware does not drive a brake chopper (the `fault0.dc_brake` output is computed but no config links it). With a supply that cannot take energy back, such as a rectifier on the mains, hard braking can pump the DC link up to the 380V fault (fault 11). This applies in particular to the regenerative stop (`fault0.rstop_en`, off by default). Add enough bulk capacitance, use an external brake chopper, or decelerate more slowly.

## Big Electrolytic Capacitors

The STMBL board requires the use of big electrolytic capacitors in its power supply, particularly close to the HV board. On PCB version 5 they are not on the board: the board only has a film snubber capacitor, and the bulk capacitors go in the external rectifier module, as close as possible to the drives. These capacitors are essential for filtering and stabilizing the high-voltage power supply, ensuring smooth operation and reducing electrical noise. The specific capacitance values may vary depending on the exact application and motor load. Size them for the motor power and for the braking energy that the DC link has to absorb (see [Undervoltage and Regeneration](/docs/supply.md#undervoltage-and-regeneration)). If in doubt, consult a qualified engineer.

## Safety Considerations

When working with high-voltage components like the STMBL board, it is crucial to prioritize safety. Always follow proper installation and usage guidelines, and ensure that all connections are secure and properly insulated. Make all connections before applying HV power, and switch off and disconnect the HV power before changing any wiring. In case of any doubts or concerns about the power supply requirements, consult a qualified engineer or ask on the [STMBL GitHub page](https://github.com/freakontrol/stmbl/issues).
