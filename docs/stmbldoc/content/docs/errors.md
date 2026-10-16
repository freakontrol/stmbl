---
title: "Errors"
weight: 7
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# Blink Codes

## LV Side

### Normal Operation
   * No Error (Green LED on top illuminated. Good!)
   * Drive disabled (Yellow LED on top. Drive ready to go but disabled.)
   * Autophasing (Green and Yellow)
   * All LEDs on top off for a moment: the drive is in a brake delay while enabling or disabling (`fault0.brake_en_delay`, `fault0.brake_dis_delay`)
### Errors
   * Soft fault, resettable (Red LED blinks according to the list below). Set the enable to 0 to clear it, for example with `disable` in Servoterm or by toggling the enable in LinuxCNC. The fault is held for at least `fault0.fault_hold` (0.02 s by default) before an enable of 0 clears it, so a fault taken while the drive was already disabling (for example in a regenerative stop) still reaches LinuxCNC.

The current firmware does not use the hard fault state, so all LEDs blinking is not expected.

Count the number of blinks of the red LED. Each number indicates a different class of faults. The LED blinks N times (300ms on, 300ms off), then pauses for another 600ms before the code repeats. Servoterm also prints the fault as `ERROR: Fault N: <text>`. With LinuxCNC over smart serial, the `fault_code` pin of the `stbl` remote reads N while the fault is set (see [LinuxCNC](/docs/getting_started/linuxcnc.md)).


 | LED Blink Times | Error                                 | Possible Solution                                          |
   |-----------------|-------------------------------------|------------------------------------------------------------|
   | 1               | Command Error                       | Common during setup. Only really relevant to smart-serial   |
   | 2               | Motor Feedback Error                | Verify motor feedback connections.         |
   | 3               | Commutation Feedback Error          | Check commutation feedback connections and configuration.    |
   | 4               | Joint Feedback Error                | Ensure joint feedback is properly connected and configured.|
   | 5               | Position Error                      | Review positioning mechanisms and settings.                |
   | 6               | Saturation Error                    | Adjust parameters to avoid saturation.                     |
   | 7               | Motor temperature high              | The modelled motor temperature (I²t model, `fault0.mot_temp = iit0.temp`) is above `conf0.max_mot_temp`. Lower the RMS current, or check `conf0.max_ac_cur`. |
   | 8               | HV serial CRC Error                 | Not raised by the current firmware. CRC errors on the F4-F3 link are only counted in `hv0.crc_error`. |
   | 9               | HV comms Timeout Error              | No answer from the F3, or a link dropout while enabled (counted in `hv0.link_drops`). The HV side is not powered or programmed correctly, the F4-F3 ribbon cable is bad, or the F3 is overloaded at the PWM rate (rt overrun). |
   | 10              | HV over-temperature                 | Above `conf0.max_hv_temp` (90°C by default), or above the F3 limit of 110°C. Check the fan, air intake/output obstruction or dust in the heatsink.  |
   | 11              | HV Voltage out of range             | Above `conf0.max_dc_volt` (380V by default), below `conf0.low_dc_volt` (24V by default) while enabled, or above the F3 limit of 400V. Check if the power supply is stable. Overvoltage while braking is regeneration: no brake chopper is driven, so decelerate more slowly, add bulk capacitance or an external chopper (see [Supply](/docs/supply.md)). |
   | 12              | HV Fault                            | The fault pin of the power module (IPM) is active. |
   | 13              | HV current offset fault             | A current offset above 5A was measured at F3 start-up. It stays set until reset. The motor must not carry current while the drive starts. Otherwise a damaged F3 ADC or current sensing? |
   | 14              | HV overcurrent filtered             | A filtered peak: the phase current was above 0.95 × 35A for about 0.33ms. Check the current-loop and commutation settings. |
   | 15              | HV Overcurrent Peak                 | The phase current was above 1.3 × `conf0.max_ac_cur` (at least 5A, at most 35A) for a single PWM cycle, or above 35A. Check the current-loop and commutation settings and `conf0.max_ac_cur`. |
   | 16              | HV Overcurrent HW                   | The hardware comparators tripped (against `hv0.dac`). Short circuit or damaged module, or the comparator level is too low. |
   | 17              | IPM junction overtemperature        | The modelled junction temperature of the power module is above `conf0.max_ipm_temp` (140°C by default). Lower the current or improve cooling. |

## HV Side

### Normal operation
   * Green LED at power input
### Errors
The red LED on the HV board blinks a code in the same way as the LV side (N blinks, then a pause). It shows the higher of the link fault and the HV fault:

   * 1 blink - No communication with F4 (no packets from the F4)
   * 2 blinks - The F3 HAL is not running
   * 3 blinks - CRC errors on the link from the F4
   * 10 to 16 blinks - The HV fault codes from the table above (17 is computed on the F4 only)
   * Red LED flashes fast (50ms on, 50ms off) - F3 bootloader, but no or broken firmware
   * Red LED on steadily - The F3 firmware stopped at a setup error
   * Red LED does not blink _AND_ F4 not plugged in - No firmware or bootloader
