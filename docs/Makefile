# Default target
all: build graph

build: graph docs

include stm32f303/comps.mak

docs/stmbldoc/static/graph/hvf3_graph.dot: docs/tools/create_hal_graph.py $(F3COMPS) $(CONFIG_TEMPLATES)
	@echo Generating graph for default HV
	@$(MKDIR) -p docs/stmbldoc/static/graph
	@$(PYTHON) docs/tools/create_hal_graph.py $@ stm32f303/src/main.c $(F3COMPS) __TEMPLATE_MARKER__ $(CONFIG_TEMPLATES)

include comps.mak

docs/stmbldoc/static/graph/f4_festo_graph.dot: docs/tools/create_hal_graph.py $(F4COMPS) $(CONFIG_TEMPLATES)
	@echo Generating graph for default LV
	@$(MKDIR) -p docs/stmbldoc/static/graph
	@$(PYTHON) docs/tools/create_hal_graph.py $@ conf/festo.txt $(F4COMPS) __TEMPLATE_MARKER__ $(CONFIG_TEMPLATES)

graph:  docs/stmbldoc/static/graph/hvf3_graph.dot docs/stmbldoc/static/graph/f4_festo_graph.dot

# Target: clean project
clean:
	@echo Cleaning project:
	rm -rf docs/stmbldoc/static/graph

# Include the base rules
include base.mak

# Listing of phony targets
.PHONY: all build clean graph

# Define a variable to hold the list of configuration files in conf/
CONF_FILES := $(wildcard conf/*.txt)

# Generate the list of graph files for conf/*
GRAPH_FILES_CONF := $(foreach conf, $(CONF_FILES), docs/stmbldoc/static/graph/f4_$(notdir $(conf:.txt=.dot)))

# Pattern rule for generating graph files from conf/
docs/stmbldoc/static/graph/f4_%.dot: conf/%.txt $(F4COMPS) $(CONFIG_TEMPLATES)
	@echo Generating graph for $<
	@$(MKDIR) -p docs/stmbldoc/static/graph
	@$(PYTHON) docs/tools/create_hal_graph.py $@ $< $(F4COMPS) __TEMPLATE_MARKER__ $(CONFIG_TEMPLATES)

# Add the dynamic targets to the graph target for conf/*
graph: $(GRAPH_FILES_CONF)

# Generate the list of graph files for conf/template/*
GRAPH_FILES_TEMPLATE := $(foreach conf, $(CONFIG_TEMPLATES), docs/stmbldoc/static/graph/f4_template_$(notdir $(conf:.txt=.dot)))

# Pattern rule for generating graph files from conf/template/
docs/stmbldoc/static/graph/f4_template_%.dot: conf/template/%.txt $(F4COMPS) $(CONFIG_TEMPLATES)
	@echo Generating graph for $<
	@$(MKDIR) -p docs/stmbldoc/static/graph
	@$(PYTHON) docs/tools/create_hal_graph.py $@ $< $(F4COMPS) __TEMPLATE_MARKER__ $(CONFIG_TEMPLATES)

# Add the dynamic targets to the graph target for conf/template/*
graph: $(GRAPH_FILES_TEMPLATE)

DOC_FILES := $(foreach conf, $(F4COMPS), docs/stmbldoc/content/docs/hal_components/$(notdir $(conf:.c=.md)))

docs/stmbldoc/content/docs/hal_components/%.md: $(F4COMPS) $(CONFIG_TEMPLATES)
	@echo Generating docs for component $<
	@$(PYTHON) docs/tools/create_hal_doc.py $@ conf/festo.txt $(F4COMPS) __TEMPLATE_MARKER__ $(CONFIG_TEMPLATES)

docs: $(F4COMPS)

