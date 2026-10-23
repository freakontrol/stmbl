# Host build of the drive firmware for the simulation harness.
#
#   make              build build/stmbl_sim
#   make test         build and run every scenario in scenarios/
#   make clean
#
# Each side (F4, F3) is compiled from the firmware sources with its own HAL,
# component table and command list, linked into one object, and everything
# in it but its sim_side_t is made local, so both fit in one program.
# Hardware components get a sim stand-in (comps_f4/, comps_f3/: same pins,
# checked) or a generated pin-only shell (tools/gen_shell.py).

ROOT     := ../..
B        := build
CC       ?= gcc
PYTHON   ?= python3
PWM_FREQ ?= 15000

include $(ROOT)/comps.mak
include $(ROOT)/stm32f303/comps.mak
TEMPLATES := $(wildcard $(ROOT)/conf/template/*.txt)

# ---- F4 ----
F4_SIM    := comps_f4/hv.c
F4_REAL   := $(addprefix $(ROOT)/,$(LV_SHARED_COMPS) src/comps/conf.c)
F4_HW     := $(filter-out src/comps/conf.c src/comps/hv.c,$(LV_SRC_COMPS))
F4_SHELLS := $(addprefix $(B)/f4/shell/,$(notdir $(F4_HW)))
F4_COMPS  := $(F4_REAL) $(F4_SHELLS) $(F4_SIM)
F4_SRC    := $(addprefix $(ROOT)/shared/,hal.c commands.c config.c angle.c ringbuf.c) host/side.c
F4_GEN    := $(B)/f4/hal_tbl.c $(B)/f4/conf_templates.c

# ---- F3 ----
F3_SIM    := comps_f3/io.c comps_f3/ls.c
F3_REAL   := $(addprefix $(ROOT)/,$(filter-out stm32f303/src/comps/io.c stm32f303/src/comps/ls.c,$(HV_SRC_COMPS)) $(HV_SHARED_COMPS))
F3_COMPS  := $(F3_REAL) $(F3_SIM)
F3_SRC    := $(addprefix $(ROOT)/shared/,hal.c commands.c angle.c ringbuf.c) host/side.c
F3_GEN    := $(B)/f3/hal_tbl.c

RUNNER    := $(wildcard runner/*.c) $(B)/f3_init.c

CFLAGS_COMMON := -O2 -g -std=gnu11 -fsingle-precision-constant -fno-math-errno -fno-common \
	-U_FORTIFY_SOURCE -include host/compat.h -DSIDE_HOOKS \
	-Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function \
	-Wno-format -Wno-maybe-uninitialized -Wno-unused-label
F4_CFLAGS := $(CFLAGS_COMMON) -DV4 -DSIDE=f4_side -DSIDE_TAG='""' -DSIDE_CONSOLE=0 \
	-I$(B)/f4/inc -Ihost -I$(ROOT)/shared -I$(ROOT)/inc
F3_CFLAGS := $(CFLAGS_COMMON) -DPWM_FREQ=$(PWM_FREQ) -DHAL_MAX_CTX=1024 -DTERM_BUF_SIZE=8 \
	-DSIDE=f3_side -DSIDE_TAG='"[f3] "' -DSIDE_CONSOLE=1 \
	-I$(B)/f3/inc -Ihost/shim_f3 -Ihost -I$(ROOT)/shared -I$(ROOT)/stm32f303/inc
RUN_CFLAGS := -O2 -g -std=gnu11 -Wall -Wno-unused-function -DPWM_FREQ=$(PWM_FREQ) \
	-include host/compat.h -Ihost -Ihost/shim_f3 -Irunner -I$(ROOT)/shared

obj = $(patsubst %.c,$(B)/$(1)/obj/%.o,$(subst $(ROOT)/,root/,$(2)))

F4_OBJS := $(call obj,f4,$(F4_COMPS) $(F4_SRC) $(F4_GEN))
F3_OBJS := $(call obj,f3,$(F3_COMPS) $(F3_SRC) $(F3_GEN))
RUN_OBJS := $(call obj,run,$(RUNNER))

all: $(B)/stmbl_sim

$(B)/stmbl_sim: $(B)/f4.o $(B)/f3.o $(RUN_OBJS)
	$(CC) -o $@ $^ -lm

# one object per side, only its sim_side_t global
$(B)/f4.o: $(F4_OBJS)
	$(CC) -r -nostdlib -o $(B)/f4_all.o $^
	objcopy -G f4_side $(B)/f4_all.o $@
$(B)/f3.o: $(F3_OBJS)
	$(CC) -r -nostdlib -o $(B)/f3_all.o $^
	objcopy -G f3_side $(B)/f3_all.o $@

# ---- generated sources ----
$(B)/f4/shell/%.c: $(ROOT)/src/comps/%.c tools/gen_shell.py
	@mkdir -p $(@D)
	$(PYTHON) tools/gen_shell.py $@ $<
$(B)/f4/shell/%.c: $(ROOT)/src/comps/hw/%.c tools/gen_shell.py
	@mkdir -p $(@D)
	$(PYTHON) tools/gen_shell.py $@ $<

$(B)/f4/.headers: $(F4_COMPS) $(ROOT)/tools/create_comp_h.py
	@mkdir -p $(B)/f4/inc
	@for c in $(F4_COMPS); do $(PYTHON) -W ignore $(ROOT)/tools/create_comp_h.py $(B)/f4/inc/$$(basename $$c .c)_comp.h $$c || exit 1; done
	$(PYTHON) -W ignore $(ROOT)/tools/create_cmd.py $(B)/f4/inc/commandslist.h $(F4_SRC) $(F4_COMPS)
	@touch $@
$(B)/f3/.headers: $(F3_COMPS) $(ROOT)/tools/create_comp_h.py
	@mkdir -p $(B)/f3/inc
	@for c in $(F3_COMPS); do $(PYTHON) -W ignore $(ROOT)/tools/create_comp_h.py $(B)/f3/inc/$$(basename $$c .c)_comp.h $$c || exit 1; done
	$(PYTHON) -W ignore $(ROOT)/tools/create_cmd.py $(B)/f3/inc/commandslist.h $(F3_SRC) $(F3_COMPS)
	@touch $@

$(B)/f4/hal_tbl.c: $(F4_COMPS)
	@mkdir -p $(@D)
	$(PYTHON) -W ignore $(ROOT)/tools/create_hal_tbl.py $@ $(F4_COMPS) > /dev/null
$(B)/f3/hal_tbl.c: $(F3_COMPS)
	@mkdir -p $(@D)
	$(PYTHON) -W ignore $(ROOT)/tools/create_hal_tbl.py $@ $(F3_COMPS) > /dev/null
$(B)/f4/conf_templates.c: $(TEMPLATES)
	@mkdir -p $(@D)
	$(PYTHON) -W ignore $(ROOT)/tools/create_config.py $@ $^
$(B)/f3_init.c: $(ROOT)/stm32f303/src/main.c tools/extract_init.py
	@mkdir -p $(@D)
	$(PYTHON) tools/extract_init.py $@ $<

# the stand-ins must carry exactly the firmware's pins
$(B)/.pins: comps_f4/hv.c comps_f3/io.c comps_f3/ls.c tools/check_pins.py
	@mkdir -p $(B)
	$(PYTHON) tools/check_pins.py $(ROOT)/src/comps/hv.c comps_f4/hv.c
	$(PYTHON) tools/check_pins.py $(ROOT)/stm32f303/src/comps/io.c comps_f3/io.c
	$(PYTHON) tools/check_pins.py $(ROOT)/stm32f303/src/comps/ls.c comps_f3/ls.c
	@touch $@

# ---- compile ----
$(B)/f4/obj/root/%.o: $(ROOT)/%.c $(B)/f4/.headers $(B)/.pins $(wildcard host/*.h)
	@mkdir -p $(@D)
	$(CC) $(F4_CFLAGS) -c $< -o $@
$(B)/f4/obj/%.o: %.c $(B)/f4/.headers $(B)/.pins $(wildcard host/*.h)
	@mkdir -p $(@D)
	$(CC) $(F4_CFLAGS) -c $< -o $@
$(B)/f3/obj/root/%.o: $(ROOT)/%.c $(B)/f3/.headers $(B)/.pins $(wildcard host/*.h host/shim_f3/*.h)
	@mkdir -p $(@D)
	$(CC) $(F3_CFLAGS) -c $< -o $@
$(B)/f3/obj/%.o: %.c $(B)/f3/.headers $(B)/.pins $(wildcard host/*.h host/shim_f3/*.h)
	@mkdir -p $(@D)
	$(CC) $(F3_CFLAGS) -c $< -o $@
$(B)/run/obj/%.o: %.c $(wildcard runner/*.h host/*.h)
	@mkdir -p $(@D)
	$(CC) $(RUN_CFLAGS) -c $< -o $@

SCENARIOS := $(wildcard scenarios/*.txt)

test: $(B)/stmbl_sim
	@mkdir -p $(B)/out
	@fail=0; for s in $(SCENARIOS); do \
	  n=$$(basename $$s .txt); \
	  ( cd $(ROOT) && tests/sim/$(B)/stmbl_sim -o tests/sim/$(B)/out/$$n.csv tests/sim/$$s ) > $(B)/out/$$n.log 2>&1; r=$$?; \
	  tail -n 1 $(B)/out/$$n.log | sed "s|^|$$n: |"; \
	  [ $$r -eq 0 ] || { fail=1; grep -E "FAIL|not found" $(B)/out/$$n.log | sed "s|^|    |"; }; \
	done; exit $$fail

clean:
	rm -rf $(B)

.PHONY: all test clean
.DELETE_ON_ERROR:
