# Default target
all: build graph

build: graph

include stm32f303/comps.mak

docs/graph/hvf3_graph.dot: docs/tools/create_hal_graph.py $(F3COMPS) $(CONFIG_TEMPLATES)
	@echo Generating HAL table graph
	@$(MKDIR) -p docs/graph
	@$(PYTHON) docs/tools/create_hal_graph.py $@ stm32f303/src/main.c $(F3COMPS) __TEMPLATE_MARKER__ $(CONFIG_TEMPLATES)

include comps.mak

docs/graph/f4_festo_graph.dot: docs/tools/create_hal_graph.py $(F4COMPS) $(CONFIG_TEMPLATES)
	@echo Generating HAL table graph
	@$(MKDIR) -p docs/graph
	@$(PYTHON) docs/tools/create_hal_graph.py $@ conf/festo.txt $(F4COMPS) __TEMPLATE_MARKER__ $(CONFIG_TEMPLATES)

graph:  docs/graph/hvf3_graph.dot docs/graph/f4_festo_graph.dot

# Target: clean project
clean:
	@echo Cleaning project:
	rm -rf docs/graph

# Include the base rules
include base.mak

# Listing of phony targets
.PHONY: all build clean graph