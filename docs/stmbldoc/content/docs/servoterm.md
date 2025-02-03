---
title: "Servoterm"
weight: 3
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---

# Servoterm

Servoterm (servo terminal) provides an interface which allows editing of the drive HAL configuration. It also provides a rolling graphical representation of any chosen parameter in the HAL, which can be a great aid to tuning and motor setup.

Servoterm is now available as a C++ application. You can download it from [this GitHub repository](https://github.com/STMBL/QtServoterm).

## Installation

To install Servoterm, follow these steps:

1. Clone the repository:
   ```sh
   git clone https://github.com/STMBL/QtServoterm.git
   ```

2. Navigate to the project directory:
   ```sh
   cd QtServoterm
   ```

3. Build the application using qmake and make:
   ```sh
   qmake
   make
   ```

4. Run the application:
   ```sh
   ./Servoterm
   ```

## Connecting to STMBL

To connect to the STMBL, you will need a mini-USB B cable. 
{{% hint danger %}}
Be sure that the 24V PSU is floating or shares a ground reference with the PC.  
(Maybe even check the voltage between the connector and socket before inserting the plug.)
{{% /hint %}}  
You can then click the "connect" button and you should get something like the image below.  

## Servoterm Interface

Taking the buttons from left to right:

* Connect / Disconnect - Should be fairly self-evident
* Clear - simply clears the screen
* Reset - Reenables the drive and clears faults. Does not reset the STMBL. To do that type the [Servoterm Commands](#servoterm-commands), `reset`
* Capture - Allows the output of the graphs to be saved and exported as CSV. First click starts the recording, second click stops and saves the file.
* Jog - when ticked the left-right arrow keys on the keyboard can be used to jog the motor.
* Trigger - stops the graph plot until the trigger condition is met.
* Trigger Status Indicator - not a button
* Edit Config - Brings up a sub-window in which the basic system config can be edited.

Other than the buttons described above, the remainder of Servoterm (and the STMBL HAL) is controlled by a command-line interface at the bottom. Servoterm uses the up and down arrow keys to scroll through previous commands, but there is no tab-completion. This is mainly described in the [HAL](#hal-hardware-abstraction-layer) section of this document. The graphing display is controlled by the "term0" interface. Typing `term0` at the prompt will show output similar to:

![Servoterm Terminal Interface](../../images/servoterm.png)

The first two entries are internal information about the HAL component and can be ignored for now. The next 8 lines say what internal signal each of the wave plots is connected to. In this case, wave0 (the black one) is connected to a sim signal, in this case, the sine wave. (as you might have guessed, typing "sim0" will show you the parameters of the simulated signals.) To connect wave1 (red) to the sawtooth output (which simulates both encoder feedback and a position command for steady rotation), then simply type `term0.wave1 = sim0.vel`. Each wave has an associated offset and gain parameter that can be used to adjust vertical scale and position. The `term0.send_step` parameter functions like the time-base of an oscilloscope.

## Servoterm Commands

The Servoterm command list can be obtained at the command line by using the `help` command. Commands to be used by the user:

* bootloader: enter bootloader
* reset: reset STMBL
* about: show system infos
* help: print this
* link: load config template
* hal: print HAL stats
* hv_update: update the F3 firmware
* show_config: show config templates
* show: show comps in flash
* list: show comp instances
* hv FOOBAR XYZ: send "FOOBAR XYZ" to the HV board

Commands for internal use:

* confcrc: Shows the CRC checksum of the loaded config.
* flashloadconf: load config from flash
* flashsaveconf: save config to flash
* loadconf: parse config
* showconf: show config - pressing the `Edit config` button is better.
* appendconf: append string to config - also redundant with the config editor
* deleteconf: delete config
* load: load comp from flash
* start: start rt system
* stop: stop rt system
* fault: trigger fault

## Servoterm Connection Problems

If you encounter connection issues, ensure that your serial setup is correct. Here are some steps to troubleshoot:

1. **Check Serial Port**: Ensure that the correct serial port is selected in Servoterm.
2. **Ground Reference**: Make sure that the 24V PSU is floating or shares a ground reference with the PC.
3. **USB Cable**: Use a good quality mini-USB B cable to connect the STMBL to your computer.


Linux/Mac? Add your username to the dialout group to be able to access the com ports. If you have used the Arduino IDE before, this is probably already set:

```
sudo usermod -a -G dialout username

```

Linux udev rules:

```
SUBSYSTEM=="usb", ACTION!="add", GOTO="objdev_rules_end"
#stmbl
ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1", GROUP="users", MODE="0666"
#ST USB bootloader
ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", GROUP="users", MODE="0666"
LABEL="objdev_rules_end"

```

If you get AT commands showing up in servoterm some application is sending commands to the virtual serial port despite the `ENV{ID_MM_DEVICE_IGNORE}="1"` above. For example on xfce with debian 10, ModemManager may need to be stopped with `systemctl disable ModemManager.service` or removed with `sudo apt-get purge modemmanager`