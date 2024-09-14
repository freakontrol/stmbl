---
title: "Supply"
weight: 1
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

 # Power Supply Requirements for STMBL Board

The STMBL board is designed to operate with a specific range of voltage and current for its various components. Understanding these requirements is crucial for proper operation and safety. This page provides an overview of the power supply requirements for the STMBL board, including information about the big electrolitic capacitors needed in the power supply close to the HV board.

## Power Supply Specifications

The following table summarizes the power supply specifications for the STMBL board:

### PCB Version 5

| Connector | Abs. MIN | MIN | TYP | MAX | Abs. MAX | Unit | Note |
|-----------|----------|-----|-----|-----|----------|------|------|
| 24V       | 0        | 15  | 24  | 25  | 26       | V    |      |
| 24V       |          |     | 0.2 | 3   | 6        | A    | 1    |
| HV        | 0        | 24  | 320 | 380 | 400      | V    |      |
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
| UVW       | -30      | -58 |     | 58  | 60       | A    |      |
| UVW avg.  |          | -27 |     | 27  |          | A    | 2    |

*Note 1: Depends on brake, fan and encoder consumption*  
*Note 2: Depends on PCB and driver cooling*  
*Note 3: For analog input*

## Big Electrolitic Capacitors

The STMBL board requires the use of big electrolitic capacitors in its power supply, particularly close to the HV board. These capacitors are essential for filtering and stabilizing the high-voltage power supply, ensuring smooth operation and reducing electrical noise. The specific capacitance values may vary depending on the exact application and motor load. It is recommended to consult the STMBL documentation or a qualified engineer for the most accurate capacitor selection.

## Safety Considerations

When working with high-voltage components like the STMBL board, it is crucial to prioritize safety. Always follow proper installation and usage guidelines, and ensure that all connections are secure and properly insulated. Never attempt to operate the board without the necessary power supply or motor load connected. In case of any doubts or concerns about the power supply requirements, consult a qualified engineer or refer to the official STMBL documentation for further guidance.