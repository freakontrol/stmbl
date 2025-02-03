# Include the firmware Makefile to access the COMPS variable
include comps.mak
include stm32f303/comps.mak

# Default target
all: build graph

build: graph

# New target to generate HAL table graph
docs/graph/hvf3_graph.dot: docs/tools/create_hal_graph.py $(F3COMPS)
	@echo Generating HAL table graph
	@$(MKDIR) -p docs/graph
	@$(PYTHON) docs/tools/create_hal_graph.py $@ stm32f303/src/main.c $(F3COMPS)

graph: docs/graph/hvf3_graph.dot

# Target: clean project
clean:
	@echo Cleaning project:
	rm -rf docs/graph

# Include the base rules
include base.mak

# Listing of phony targets
.PHONY: all build clean graph
