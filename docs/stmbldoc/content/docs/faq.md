---
title: "FAQ"
weight: 8
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: true
# bookComments: false
# bookSearchExclude: false
---

### How to contact the devs?
- https://matrix.to/#/@stmbl:freakontrol.com
- https://gitter.im/rene-dev/stmbl
- https://webirc.hackint.org/#stmbl or #stmbl on irc.hackint.eu
- GitHub issues for this fork: https://github.com/freakontrol/stmbl/issues

The Gitter room is the one of the original upstream project (rene-dev/stmbl).
### Does it work with DC Servos?
Yes, but DC servos usually operate at a much lower voltage, and the IGBT module we use is not very efficient at low voltages.
### Does it work with stepper motors?
Not with a ready template. The hardware can drive 3-phase stepper motors, and `hv0.phase_mode` also has 90° modes (0 and 1) besides the usual 120° 3-phase mode. The only stepper configs, conf/experimental/bene-stepper.txt and conf/experimental/stp.txt, are experimental and outdated.
### Why do you use 2 CPUs?
The driver is designed to run off rectified mains, without a transformer. Therefore the high voltage side has to be fully isolated. Here we need to supply about 8 digital I/Os, and measure many analog values. The cheapest way of doing this is to use a second CPU, and only isolate a UART. The communication is running at 3Mbit/s.
### Does it work with +-10V?
There is no ready template for a ±10V command. The two ±24V analog inputs (`io0.in0`, `io0.in1`) can be wired as a speed command by hand: conf/bene_uf_2kw.txt does this with `avg0.in = io0.in1` and `ramp0.vel_ext_cmd = avg0.out`.
### GCC complains about missing files.
Use the Makefile. It generates code at compile time.