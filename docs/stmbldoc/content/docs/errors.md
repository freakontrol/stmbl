---
title: "Errors"
weight: 6
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
### Errors
   * Soft fault, resettable (Red LED blinks according to the list below)
   * Hard fault, bad! (All LEDs on top blink.)

Count the number of blinks of the red LED. Each number indicates a different class of faults.


 | LED Blink Times | Error                                 | Possible Solution                                          |
   |-----------------|-------------------------------------|------------------------------------------------------------|
   | 1               | Command Error                       | Common during setup. Only really relevant to smart-serial   |
   | 2               | Motor Feedback Error                | Verify motor feedback connections.         |
   | 3               | Commutation Feedback Error          | Check commutation feedback connections and configuration.    |
   | 4               | Joint Feedback Error                | Ensure joint feedback is properly connected and configured.|
   | 5               | Position Error                      | Review positioning mechanisms and settings.                |
   | 6               | Saturation Error                    | Adjust parameters to avoid saturation.                     |
   | 7               | Motor temperature high              | Check for overheating issues or motor ptc misconfiguration.   |
   | 8               | HV Side serial CRC Error           | Ensure data integrity during communication with the side.|
   | 9               | HV Side comms Timeout Error        | HV side is not powered or programmed correctly.             |
   | 10              | HV Side over-temperature           | Check the fan, air intake/output obstruction or dust in the heatsink.  |
   | 11              | HV Voltage out of range             | Check if power supply is stable and if the dynamic braking circuit is working. |
   | 12              | HV Side Fault                      | Investigate specific faults reported by the HV side.      |
   | 13              | HV current offset fault             | Damaged F3 micro ADC?            |
   | 14              | HV Overcurrent RMS                  | Reduce average current consumption to stay within limits.  |
   | 15              | HV Overcurrent Peak                 | Limit peak current spikes to prevent overcurrent issues.   |
   | 16              | HV Overcurrent HW                   | The motor power size is too high (or the bridge is broken) |

## HV Side

### Normal operation
   * Green LED at power input
### Errors
   * Red LED blinks fast - Bootloader but no or broken firmware
   * Red LED blinks slow - No communication with F4
   * Red LED does not blink _AND_ F4 not plugged in - No firmware or bootloader