---
title: "HAL Templates"
bookCollapseSection: true
weight: 4
---

# HAL Templates

A template is a reusable piece of config from conf/template/, for example `pid` (position control), `pmsm` (motor type), `res_fb0` (resolver feedback) or `sserial` (LinuxCNC). A config includes a template with `link <name>`. Templates load the components they need and link their pins, mostly to the `conf0` parameters, so a motor config only has to set the `conf0` values and link the templates for its motor, feedback and command interface. `show_config` in Servoterm lists the templates built into the firmware. The pages in this section show each template.
