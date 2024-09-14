---
title: "FAQ"
weight: 7
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: true
# bookComments: false
# bookSearchExclude: false
---

### How to contact the devs?
- https://gitter.im/rene-dev/stmbl
- https://webirc.hackint.org/#stmbl or #stmbl on irc.hackint.eu
### Does it work with DC Servos?
Yes, but DC servos usually operate at a much lower voltage, and the IGBT module we use is not very efficient at low voltages.
### Does it work with stepper motors?
No.
### Why do you use 2 CPUs?
The driver is designed to run of rectified mains, without a transformer. Therefore the high voltage side has to be fully isolated. Here we need to supply about 8 digital I/Os, and measure many analog values. The cheapest way of doing this is to use a second CPU, and only isolate a UART. The communication is running at 3Mbit, but 9Mbit is possible.
### Does it work with +-10V?
No.
### GCC complains about missing files.
Use the Makefile. It generates code at compile time.