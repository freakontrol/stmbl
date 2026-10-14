---
title: "HAL Components"
bookCollapseSection: true
weight: 3
---

# HAL Components

A component is a block of firmware code with input, output and parameter pins, for example `hv` (the link to the HV board), `pid` (the controller) or `res` (resolver feedback). A config adds an instance with `load <component>`, which creates `<component>0` (a second `load` creates `<component>1`), and connects its pins with lines like `pid0.pos_ext_cmd = reslimit0.pos_out`. The pages in this section describe each component and its pins. They are generated from the doc comments in the source code.
