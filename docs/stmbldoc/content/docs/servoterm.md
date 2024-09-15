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

Servoterm (servo terminal) provides an interface which allows editing of the drive HAL configuration. It also provides a rolling graphical representation of any chosen parameter in the HAL which can be a great aid to tuning and motor setup.

Servoterm is supplied as a Google Chrome extension. This might seem somewhat odd, but does provide for good cross-platform availability. Servoterm can be downloaded from [this link](https://github.com/STMBL/Servoterm-app). Use the green button to download as a ZIP file and then extract on your PC (Linux / Mac / PC). Open Google Chrome and click the three-dots icon -> more-tools -> extensions.

![Google Chrome Extensions](../../images/Extensions.png)

Click "developer mode" and then "Load Unpacked Extension". Then navigate to the downloaded files and select the "Servoterm" folder. You should then be presented with the following, including an option to launch the application.

![Servoterm Extension](../../images/Extensions2.png)

If you do not get the option to launch Servoterm, you can visit [chrome://apps](chrome://apps) and click on the Servoterm logo. To connect to the STMBL, you will need a mini-USB B cable. WARNING: Be sure that the 24V PSU is floating or shares a ground reference with the PC.(Maybe even check the voltage between the connector and socket before inserting the plug) You can then click the "connect" button and you should get something like the image below. [What if I can not connect](#servoterm-connection-problems)

![Servoterm Interface](../../images/servoterm1.png)

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

![Servoterm Terminal Interface](../../images/servoterm2.png)

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

For more information, see [Development](https://github.com/rene-dev/stmbl/wiki/Development).