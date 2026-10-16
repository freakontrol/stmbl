---
title: "Introduction"
weight: 1
# bookFlatSection: true
bookToc: true
# bookHidden: false
# bookCollapseSection: true
# bookComments: true
# bookSearchExclude: false
---

# STMBL Introduction Guide

## Description

STMBL is a motor drive controlled by the STM32 microprocessor. It can power motors of up to 2.2kW. With the default config (conf/template/conf.txt) the drive reduces the motor current above 350V DC bus voltage (`conf0.high_dc_volt`) and faults at 380V (`conf0.max_dc_volt`). The F3 on the HV board also has its own fixed trip at 400V. See [Supply](/docs/supply.md) for details. The name "STMBL" is based on the combination of the STM32 microprocessor and BrushLess motor. However, the drive is also capable of powering AC induction or DC motors. The hardware is also capable of driving 3-phase stepper motors.

The drive is configurable for a wide range of command and feedback types through a hardware abstraction layer (HAL) analogous to that used by [LinuxCNC](https://linuxcnc.org/docs/stable/html/hal/intro.html).

Currently supported command interfaces are:

* Mesa Smart-Serial  
* Quadrature A/B or Step/Dir ([enc_cmd](/docs/hal_templates/enc_cmd.md), `link enc_cmd`; step/dir with `enc_cmd0.mode = 1` or `2`, see the [enc_cmd](/docs/hal_components/enc_cmd.md) component). Up/down is not supported.  
* Servoterm jog over USB, for testing (`link jog_cmd`)  

Encoder power is 5V by default but 12V can be selected by jumper pads on the PCB. The firmware cannot select the voltage. It can only switch the 5V encoder supply off (`io0.fbsd`).

In addition to motor control and dual feedback, each STMBL drive has two analog inputs (±24V, `io0.in0` and `io0.in1`, also usable as digital inputs with a threshold of 12V by default) and three 24V / 1A digital outputs (`io0.out0` is on/off, `io0.out1` and `io0.out2` are PWM outputs). The values follow the table on the [Supply](/docs/supply.md) page.

To use the STMBL, you will need to:

* connect your command, feedback, and motor to STMBL using **suitable cables**.
* [Flash the firmware](/docs/introduction/#flashing-firmware)
* configure the drive for your motor using [Servoterm](/docs/getting_started/servoterm.md)
  * configure your [Feedback interface](/docs/getting_started/feedback.md)
  * find the correct [Motor parameters](/docs/getting_started/motor.md) for your motor
  * configure your command interface: [LinuxCNC](/docs/getting_started/linuxcnc.md) (smart serial) or [enc_cmd](/docs/hal_templates/enc_cmd.md) (quadrature or step/dir)

## Anatomy of the STMBL

The STMBL consists of two separate PCBs that are made as one then assembled and split to be connected in the manner shown below. The vertical (top) board is the low-voltage (LV) board, and this handles the command, feedback, and configuration tasks. The STM32F405 microprocessor is in charge of these tasks.

The lower board is the high-voltage (HV) board, and this is where the power driver is situated. The only connection between the two boards is a serial connection through a 2.5kV isolation IC. To make this possible, there is a second STM32 chip on the lower board. This is an STM32F303 and is referred to as "F3" in the remainder of this document. The processor on the upper board is referred to as "F4".

![STMBL Anatomy](/stmbl/images/iso1-dark.png)

Logic power to the LV board should be 24V. A green LED will light adjacent to the socket when power is supplied.

{{% hint warning %}}
The LV board is safe up to about 26V but take care that 0V is common with the PC GND before connecting a USB cable.  
{{% /hint %}}

Motor power is typically about 320V (rectified 230V mains). While the drive is enabled, the HV supply must stay above `conf0.low_dc_volt` (24V by default) and below `conf0.max_dc_volt` (380V by default), or the drive faults. The logic parts of the HV board work at 24V for firmware flashing etc. Again, a green LED adjacent to the connector confirms that the board is powered-up.  

{{% hint warning %}}
The HV and LV boards are isolated in normal use but it is easy to accidentally connect them. One way to do this is via USB cables which can easily tie GND lines together through the setup PC.  
{{% /hint %}}  

{{% hint danger %}}
 It is imperative that the HV board should be powered from an isolated, low voltage supply when flashing firmware.
{{% /hint %}}  

![STMBL Connectors](/stmbl/images/iso2-dark.png)

The command and feedback connectors use standard 8P8C (RJ45) connectors and standard CAT5 or CAT6 cables can be conveniently used. To connect to cables with larger conductors than supported by CAT5, it is possible to use, for example, [Industrial CAT6a](https://octopart.com/j00026a2001-telegärtner-24873031) connectors which can accept core wires up to 1.6mm and overall cable diameters up to 9.0mm.

Feedback 0 will typically be the encoder or resolver mounted on the motor, and feedback 1 can be used to connect either Hall sensors for initial commutation or (potentially) scales mounted directly to the axes. See the [Pinouts](/docs/getting_started/pinouts) section of this document for pin assignments and typical wiring color codes.

The 6-way socket below the 24V logic power connector contains the three digital outputs. These are current-sinking (switch-to-GND) and each is adjacent to a 24V supply pin. DIO0 (nearest the top, `io0.out0`) is the one that is typically used to operate the holding brake on motors so-equipped. Motor configs link it with `io0.out0 = fault0.mot_brake`. With `link sserial`, the template links `io0.out0 = sserial0.out0` instead, so LinuxCNC switches it.

On the top of the drive are two analog inputs, with 0V and 24V on either side so that active sensors can be connected. These are typically used as variable-threshold digital inputs and are used, for example, for axis limit switches. However, it is relatively simple to configure them for other uses in the HAL.

Three LEDs on top of the unit indicate drive status. Red displays error codes (using [Blink Codes](/docs/errors/)). Amber indicates that all is well but the drive is not enabled, and green shows that the drive is active and operating normally. If no LEDs on the top of the board are illuminated, and the green power LEDs near the power connectors _are_ illuminated, then it is probably necessary to [flash the firmware](/docs/introduction/#flashing-firmware). If there are LEDs lit on top of the drive, then it is probably safe to assume that firmware is loaded.

## HAL (Hardware Abstraction Layer)

STMBL uses a data flow graph to configure the drive for different types of motor, feedback, and operation mode. This is conceptually similar to the HAL in [LinuxCNC](https://linuxcnc.org/docs/stable/html/hal/intro.html), but the format and commands are different. Also, all pins are floating point so no data conversion is needed.  

Here is the graphical representation of the default configuration for the LV board and for the HV board

{{< zoomable-image src="/stmbl/graph/f4_festo_graph.dot.svg" alt="Low Voltage default hal config" >}}

{{< zoomable-image src="/stmbl/graph/hvf3_graph.dot.svg" alt="High Voltage default hal config" >}}  

An Application called [Servoterm](/docs/getting_started/servoterm/) is used to interact with the HAL interface and configure the drive. You will need to install and launch this before it is possible to configure the STMBL.

A STMBL HAL configuration uses `pin = value` and `pin = other_pin` lines, `load <component>`, `link <template>` and `#` comments. The [Servoterm Commands](/docs/getting_started/servoterm/#servoterm-commands) can also be typed in the terminal.

Assuming that there is already a motor connected to the drive and that the drive is powered up, the Servoterm display should already be indicating the motor position feedback. Rotating the motor shaft by hand might produce something like:

![Servoterm Display](/stmbl/images/servoterm.png)

Though it equally well might not if the configuration is set up for a resolver and the motor has an encoder. It should be possible to make the motor turn at this point without any further configuration. The commands that follow will set the hv0 module up to simply rotate the motor open-loop in direct-mode (like a stepper motor) with an excitation current of 0.5A. This is a current only in a PMSM or DC config, where `hv0.cmd_mode = 1` (current mode). In a V/f config (uf, vf, rl), `hv0.cmd_mode = 0` and the value is a voltage, 0.5V. This should be safe for most motors that the STMBL is a good match for, but you should choose your own value. For the motor parameters, see the [Motor](/docs/getting_started/motor.md) page.

```
hv0.pos = sim0.vel
hv0.d_cmd = 0.5
hv0.en = 1
```

{{% hint danger %}}
Setting `hv0.en = 1` removes its link to `fault0.en_out`, so this test bypasses the fault handling. Esc, Disable and `fault0.en = 0` do not stop the bridge. Stop it with `hv0.en = 0`, or type `reset` to restart the drive with its saved config.
{{% /hint %}}

The rotation speed can be altered by changing the sim0 frequency. `sim0.freq` is the electrical frequency in Hz, so the shaft turns at `sim0.freq` divided by the pole pair count:

```
sim0.freq = 5
```

STMBL HAL contains a number of components that have built-in linking behavior.

### Resolvers

Resolver phase - Setting the phase is very important to get resolver output. 'res0.phase = X' sets the phase. The number is between 0 and 1. Set for the highest output.

Resolver phase is the `res0.phase` pin (default 0.85, the configs use about 0.89 to 0.9).

Resolver speed - In motor drives, resolver speed is typically 1. One rotation of the resolver equals one rotation of the motor. The resolver template sets `res0.poles = conf0.mot_fb_polecount`, so set the resolver speed with `conf0.mot_fb_polecount`. You will want to add these settings to the config so that the drive powers up with the settings as default (see [Saving a config](/docs/getting_started/servoterm/#saving-a-config)).

### Motor pole count / setting motor feedback offset

Motor pole count in data sheets is often a total pole count rather than STMBL required 'pole pair' count. This is easy to see when running the com_test: type 'link com_test'. This should start the motor turning slowly in open loop, while displaying on the graph. The template plots these waves:

* wave0: `sim0.vel`, the open-loop electrical angle that drives the motor
* wave1: `fb_switch0.com_fb_no_offset`, the raw position of a separate commutation feedback (for example Hall sensors), if there is one
* wave2: `fb_switch0.mot_abs_fb_no_offset`, the absolute motor position (mechanical), without offset
* wave3: `fb_switch0.com_fb`, the electrical angle the drive computes from the feedback, using `conf0.polecount` and the offset

Ideally, wave0 and wave3 will match closely. If wave0 and wave3 are not the same speed (same number of peaks), then the motor poles setting is wrong. 'conf0.polecount' can be set without stopping the test. Set it till the number of peaks is the same. Now the peaks may be offset on the graph. 'conf0.mot_fb_offset' will set this. Adjust this number till wave0 and wave3 match. You'll want to add these settings to the config so the drive starts with these defaults (see [Saving a config](/docs/getting_started/servoterm/#saving-a-config)).

{{% hint danger %}}
com_test sets `hv0.d_cmd = 1` and `hv0.en = 1`. Like the open-loop test above, this bypasses the fault handling: Esc and `fault0.en = 0` do not stop the bridge. Stop it with `hv0.en = 0`, or type `reset`.
{{% /hint %}}

### Jogging test

Jogging needs the jog component, so add `link jog_cmd` to the config first. If you tick the **Jog** option in Servoterm and then enable the drive with: 'fault0.en = 1', using the left and right cursor keys on the keyboard can be used to jog the motor. The escape key will disable the drive quickly if something is not quite right. If there is a fault, then you will need to toggle the enable pin to 0 and back to 1 again. If the drive oscillates, try adjusting the 'conf0.j' setting for inertia.

### Drive the motor with a sine wave

1. Connect it `rev0.in = sim0.msin`
2. Set amplitude `sim0.amp = 1` (in rad)
3. Set frequency `sim0.freq = 0.5` (in Hz)
4. Enable `fault0.en = 1`

### Or constant velocity

1. Connect it `rev0.in = sim0.vel`
2. Set frequency `sim0.freq = 0.5` (in Hz)
3. Enable `fault0.en = 1`

Pressing Esc at any time will disable the drive. To re-enable after a fault, type `disable` followed by `enable` (or `fault0.en = 0` followed by `fault0.en = 1`). A fault is cleared when the enable goes to 0. The `reset` command restarts the drive and reloads the saved config, which also undoes any pin changes that were not saved.

### Components/templates/config list and examples  

You can find the list of components, template and configs in the Hal components, templates and configs section.  

## Flashing Firmware

### Requirements to build firmware

The GCC cross-compiler for Arm, `arm-none-eabi-gcc` version 5 or newer: either the `gcc-arm-none-eabi` package of your Linux distribution, or the [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads). You will also need the STMBL source code, available from [https://github.com/freakontrol/stmbl](https://github.com/freakontrol/stmbl). You can either clone this as a [git](https://git-scm.com/) archive or just download a current snapshot as a zip file. In order for the STMBL Makefiles to be able to find the gcc binaries, you may need to create the file toolchain-user.mak to point to the correct folder and version number.

The build also needs:

* `make` and `python3`.
* The python3 `graphviz` module, because `make` also builds this documentation (the HAL graphs). Use `make DOCS=0` to skip the documentation.
* `dfu-suffix` (part of the `dfu-util` package) for the `.dfu` files.
* The python3 `pyserial` module for `make btburn` and `make all_btburn`, which send the `bootloader` command to the drive over USB. Disconnect Servoterm first, or the serial port is busy.

The F3 PWM frequency is set at build time: `make PWM_FREQ=10000`, `make PWM_FREQ=15000` (default) or `make PWM_FREQ=20000`.

### Requirements to flash firmware

The STM32 chips have a built-in ROM bootloader, this means that it should be impossible to "brick" the boards. Each of the two CPUs in the STMBL drive needs both a dedicated bootloader to start the STMBL firmware and the firmware itself.

#### Linux/Unix

To flash the boards with USB, you will need the `dfu-util` package [https://dfu-util.sourceforge.net](https://dfu-util.sourceforge.net). To flash the boards with a stlink programmer over SWD, you will need the stlink package [https://github.com/stlink-org/stlink](https://github.com/stlink-org/stlink)

#### Windows

You will need the STM Virtual Comport driver to connect with Servoterm [http://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-stm32102.html](http://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-stm32102.html) And the DfuSe USB device firmware upgrade STMicroelectronics extension to flash the firmware over USB [https://www.st.com/en/development-tools/stsw-stm32080.html](https://www.st.com/en/development-tools/stsw-stm32080.html)

### Checking for Existing Firmware - F4 board

Before flashing firmware, it is worth trying to figure out if your board is completely blank or has been pre-flashed with a bootloader or firmware. If the board will connect with Servoterm, then it already has a firmware and STMBL bootloader. "about" will show the firmware information of the F4 board. "hv about" will give the same information about the F3 board. Go to the [Updating Firmware](/docs/introduction/#updating-firmware) section to flash new firmware. If the board lights any LEDs other than the green power-good ones near to the power input connectors, then there is likely to already be a firmware installed. Go to the [Updating Firmware](/docs/introduction/#updating-firmware) section if you need to update the firmware.

If the board is powered with 24V to the LV board and connected with USB to a PC, and the firmware is running, it reports as a virtual serial port with USB ID 0483:5740 and the product name "STM32 Virtual ComPort". The exact name shown in the Apple System Profiler, in `lsusb` on Linux or in the Windows Device Manager depends on the operating system.

If the board shows "STM32 BOOTLOADER" (Mac), "0483:df11 STMicroelectronics STM Device in DFU Mode" (Linux lsusb), or "STM32 BOOTLOADER" (Windows Device Manager) when powered up (without using the boot pads), then this is the STM32 ROM bootloader. The STMBL bootloader starts it when the app is missing or corrupt (bad CRC), after the `bootloader` command, or when PA13 (the SWDIO pin) is held low at power-up. So the board already has an STMBL bootloader, but the app needs to be flashed (though no harm is done by re-flashing the bootloader).

If the LV board does not show up at all on the USB bus, then attempt to put it in ROM boot mode by shorting the boot pads together while connecting the 24V. You should see "STM32 BOOTLOADER" (Mac), "STMicroelectronics STM Device in DFU Mode" (Linux lsusb), or "STM32 BOOTLOADER" (Windows Device Manager). In this case, you will need to flash both the STMBL bootloader and the STMBL firmware. Go to the [Flashing the LV board with no bootloader](/docs/introduction/#flashing-the-lv-board-with-no-bootloader) section.

### Checking for existing firmware - F3 board

With 24V to the F3 board and with the F4 board _unpowered_, look at the red LED under the fan, near the USB connector. If the HV board has both an STMBL bootloader and an STMBL Firmware installed, then it will illuminate only the green power LED and will flash the red LED slowly (one blink, then a pause) to indicate no comms with the F4 board (which is why this check should be done with the F4 board unpowered). Go to the [Updating Firmware](/docs/introduction/#updating-firmware) section in this scenario. If the red LED stays on steadily, the F3 firmware stopped at a setup error. If the F3 board does not flash the red LED when the F4 is unpowered, then there is no bootloader and no firmware flashed. Go to the [Flashing the HV board with no bootloader](/docs/introduction/#flashing-the-hv-board-with-no-bootloader) section. If the F3 board has only a bootloader flashed and no or broken firmware, then the red LED will flash rapidly. Use the instructions in [Updating Firmware](/docs/introduction/#updating-firmware) in this case.

It can be convenient to flash the boards to test them before separating the halves and before installing the power module (IKCM30F60GD on v5) if you have a self-built or part-assembled board. On v5, the bulk capacitors are external, close to the drive (see [Supply](/docs/supply.md)). Precompiled Binary versions of the firmware can be downloaded from [https://github.com/freakontrol/stmbl/releases](https://github.com/freakontrol/stmbl/releases) When compiling from the source code, firmware flashing is handled by specifying a makefile target for each of the firmware sections.

### Updating Firmware

The firmware on both the F3 and F4 board can be updated through the F4 USB port and without access to the boot pads. Connect 24V to both the F3 and F4 boards. In the source software folder, type `git pull` to get the latest software version. Type `make clean` to ensure that all files are freshened. Then use one of these targets:

* `make btburn` builds the firmware and programs the F4 app (at 0x08010000) over USB. It keeps the STMBL bootloader and the saved config. The F3 firmware is embedded in the F4 app and is transferred later with `hv_update`.
* `make all_btburn` is for blank boards. It writes f4.bin to 0x08000000, which contains the STMBL bootloader, conf/festo.txt as the config, and the app.

{{% hint warning %}}
`make all_btburn` replaces your config with conf/festo.txt. Save a copy of your config first. `make btburn` keeps the config.
{{% /hint %}}  

`make binall` is only needed for DfuSe on Windows or a manual dfu-util run. It creates f4.dfu (bootloader + festo.txt config + app, at 0x08000000, so it replaces the config), f3.dfu (F3 bootloader + F3 app, at 0x08000000) and stmbl.dfu (the F4 app only, at 0x08010000).

There should then be a quantity of text output culminating with a progress bar like:

```
Downloading to address = 0x08010000, size = ...
Download [=========================] 100%
```

You can now re-connect with Servoterm and check the firmware version with `about`. Updating the F3 firmware is done via Servoterm using the `hv_update` command. Because the F3 image is embedded in the F4 app, run `hv_update` after every F4 update that changes the F3 code (or the PWM frequency). This should give output similar to:

```
hv_update: SEND_TO_BOOTLOADER
hv_update: ERASE_FLASH
hv_update: SEND_APP
hv_update: status: 4%
...
hv_update: status: 95%
hv_update: CRC_CHECK
hv_update: SEND_TO_APP
hv_update: SLAVE_IN_APP
```

If the update fails, it prints `hv_update: FLASH_FAILED`, after "the f3 bootloader says the app CRC is wrong" or "no answer to CRC_CHECK" when that was the cause. The `hv_verify` command reads the F3 app area back from the F3 bootloader and compares it with the image embedded in the F4 firmware. If this fails multiple times, go to the [Flashing the HV board with no bootloader](/docs/introduction/#flashing-the-hv-board-with-no-bootloader) section.

The boards can also be flashed with an stlink programmer over SWD: `make flash` (F4 app), `make boot_flash` (F4 bootloader), `make all_flash` (f4.bin: bootloader, festo.txt config and app), `make f3_boot_flash` (F3 bootloader) and `make f3_all_flash` (f3.bin: F3 bootloader and app).

### Flashing the LV board with no bootloader

To flash the initial bootloader and firmware, it is necessary to put the STM32 CPU into ROM bootloader mode. You do this by shorting together the two pads marked "boot" on the LV board while connecting the 24V supply. This is a bit of a fiddle but should only need to be done once when the board is first built. For the exact location of these pads, see the illustration in the [Anatomy of the STMBL](/docs/introduction/#anatomy-of-the-stmbl) section. Typically, a small screwdriver can be used for this purpose. At this point, the board should appear as "STM32 BOOTLOADER" in the USB tree of the attached PC. Follow the [Updating Firmware](/docs/introduction/#updating-firmware) instructions and use the `make all_btburn` command.

### Flashing the HV board with no bootloader

{{% hint danger %}}
The USB port of the HV board is referenced to the DC link. While USB is connected, power the HV input only from an isolated low-voltage (24V) bench supply, never from the ~300V link or the mains power supply.
{{% /hint %}}

Connect the USB cable to the HV board and short the boot pads on the HV board while connecting 24V to the HV input, to put it into bootloader mode. Again, it should appear in the USB device tree. Follow the [Updating Firmware](/docs/introduction/#updating-firmware) instructions but use the `make f3_all_btburn` command. It writes f3.bin (F3 bootloader and app) with dfu-util through the F3's own USB port.

## linuxcnc

STMBL supports Mesa Smartserial to communicate with LinuxCNC. [http://linuxcnc.org/docs/html/man/man9/sserial.9.html](http://linuxcnc.org/docs/html/man/man9/sserial.9.html)

[https://www.youtube.com/watch?v=5CKMrOy0ZXk](https://www.youtube.com/watch?v=5CKMrOy0ZXk)

### STMBL config

At the end of your STMBL config, load sserial:

```
link sserial
```

The sserial template links `linrev0.scale = sserial0.scale`, so the scale is set from LinuxCNC, not in the STMBL config. Setting `linrev0.scale` to a constant would break that link. In the LinuxCNC .hal file, set for example:

```
setp hm2_5i25.0.stbl.0.0.scale 6
```

See [LinuxCNC](/docs/getting_started/linuxcnc.md) for a complete example.

#### Example: HAL config for Position mode with Position feedback

This is an example for one axis, Mesa 5i25 and one STMBL board, added to a .hal file in a machine config:

```
# This is an example for linuxcnc hal in position mode for the x-axis:
net xposcmd joint.0.motor-pos-cmd => hm2_5i25.0.stbl.0.0.pos_cmd
net xvelcmd joint.0.vel-cmd => hm2_5i25.0.stbl.0.0.vel_cmd
net xposfb joint.0.motor-pos-fb <= hm2_5i25.0.stbl.0.0.pos_fb
net xenable joint.0.amp-enable-out => hm2_5i25.0.stbl.0.0.enable
net xfault joint.0.amp-fault-in <= hm2_5i25.0.stbl.0.0.fault
net xindex joint.0.index-enable <=> hm2_5i25.0.stbl.0.0.index_enable
```

#### Example: Full Machine Config with STMBL boards, Mesa 7i80, Mesa 7i77, Analog Outputs, Encoder Inputs

See [https://github.com/aShure/cnc-configs/tree/master/justinbieber](https://github.com/aShure/cnc-configs/tree/master/justinbieber)