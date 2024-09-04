[![Join the chat at https://gitter.im/rene-dev/stmbl](https://badges.gitter.im/rene-dev/stmbl.svg)](https://gitter.im/rene-dev/stmbl?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

DISCLAIMER
===

THE AUTHORS OF THIS SOFTWARE ACCEPT ABSOLUTELY NO LIABILITY FOR
ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
harming persons must have provisions for completely removing power
from all motors, etc, before persons enter any danger area.  All
machinery must be designed to comply with local and national safety
codes, and the authors of this software can not, and do not, take
any responsibility for such compliance.

This software is released under the GPLv3.

STMBL
=====

This Repository is a fork of the original project [STMBL](https://github.com/rene-dev/stmbl), an open-source servo drive designed for Retrofitting CNC machines and Robots. It supports Industrial AC and DC servos.

Original stmbl documentation: https://github.com/rene-dev/stmbl/blob/master/docs/src/Getting%20Started.adoc

A detailed documentation of this new revision will be available soon! 

We are proposing a new revision of the project making significant improvements and updates:

- H-bridge
- Power connectors
- Bulk Capacitor
- Heat dissipation components and case
- Other obsolete components

<img src="./img/stmbl_case.jpg" alt="image" width="600"/> 
<img src="./img/stmbl_top.jpg" alt="image" width="300"/> <img src="./img/stmbl_bottom.jpg" alt="image" width="300"/> 

## H-Bridge
First we needed to replace the original and obsolete H-bridge IRAM256 with a more powerful and easily available one.
We chose the [Infineon IKCM30F60GD](https://www.mouser.it/datasheet/2/196/Infineon_IKCM30F60GD_DataSheet_v02_05_EN-3361791.pdf) capable of peak output currents up to 60Amps instead of 30 of the original bridge.

## Upgrading Power connectors
During our tests with earlier prototypes we noticed that the original power connectors overheated and even got damaged when the board drove big loads.

<img src="./img/old_damag_connector.jpg" alt="image" width="300"/> 

So we decided to replace power connectors, both HV DC input and 3 Phase output with golden plated XT60PW and MR60PW connectors from Amass®, designed for high current up to 60Amps.

## Bulk Capacitor

We have relocated big bulk capacitors outside in a rectifier module as close as possible to the drives. 
Instead internally we kept a more compact MKP Polypropylene snubber capacitor for decoupling the bridge HV power supply.

<img src="./img/pfc.excalidraw.png" alt="image" width="400"/> 

## Heat dissipation components and case

We also optimized heat dissipation components and layout, reducing overall volume by half compared to the original design.

<img src="./img/airflow1.png" alt="image" width="600"/> 
<img src="./img/temp.png" alt="image" width="600"/> 


## Other obsolete components

there were several other components that have been replaced because they have become obsolete:

- ACT4088US-T (DC/DC converter) replaced with [RT8259GE](https://www.mouser.it/datasheet/2/1458/DS8259_03-3104661.pdf) from Richtek.
- With DC/DC converters, the power inductors have also been updated to:
    - [ME3220-472MLC 4.7uH ](https://www.mouser.it/ProductDetail/994-ME3220-472MLC)
    - [ME3220-682MLC 6.8uH](https://www.mouser.it/ProductDetail/994-ME3220-682MLC)
    - [ME3220-103KLC 10uH](https://www.mouser.it/ProductDetail/994-ME3220-103KLC)
- ZLDO1117G33TA (LDO 3.3V 1A) replaced with its new version [LDL1117S33R](https://www.mouser.it/ProductDetail/511-LDL1117S33R) from STMicroelettronics
- both microUSB connectors have been updated to the more modern USBc

* * * 
[!["Buy Me A Coffee"](https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png)](https://www.buymeacoffee.com/freakontrol)

If you want to get a prototype of this board contact us in [![Join the chat at https://gitter.im/rene-dev/stmbl](https://badges.gitter.im/rene-dev/stmbl.svg)](https://gitter.im/rene-dev/stmbl?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
