# Default target
all: build

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


clean:
	@echo Cleaning documentation files:
	find docs/stmbldoc/content/docs/hal_components/ -type f -name '*.md' ! -name '_index.md' -exec rm {} \;
	find docs/stmbldoc/content/docs/hal_templates/ -type f -name '*.md' ! -name '_index.md' -exec rm {} \;
	find docs/stmbldoc/content/docs/hal_configs/ -type f -name '*.md' ! -name '_index.md' -exec rm {} \;
	rm -rf docs/stmbldoc/static/graph

# Include the base rules
include base.mak

# Listing of phony targets
.PHONY: all build clean graph docs clean_docs

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

ALL_COMPS = $(F3COMPS) $(F4COMPS)
# Define the list of component files (example)
COMP_DOCS := $(foreach comp,$(ALL_COMPS),docs/stmbldoc/content/docs/hal_components/$(notdir $(comp:.c=.md)))

# Explicitly define targets for each component file
docs/stmbldoc/content/docs/hal_components/%.md: $(ALL_COMPS) $(CONFIG_TEMPLATES)
	@echo Generating docs for component $@
	@$(PYTHON) docs/tools/create_comp_doc.py $@ $(ALL_COMPS) __TEMPLATE_MARKER__ $(CONFIG_TEMPLATES)

# Phony target to generate all documentation files
.PHONY: docs

docs: $(COMP_DOCS)

TEMPLATE_DOCS := $(foreach comp,$(CONFIG_TEMPLATES),docs/stmbldoc/content/docs/hal_templates/$(notdir $(comp:.txt=.md)))

# Explicitly define targets for each template file
docs/stmbldoc/content/docs/hal_templates/%.md: $(CONFIG_TEMPLATES) docs/stmbldoc/static/graph/f4_template_%.dot
	@echo Generating docs for component $@
	@$(PYTHON) docs/tools/create_template_doc.py $@ $(CONFIG_TEMPLATES)

# Phony target to generate all documentation files
.PHONY: docs

docs: $(TEMPLATE_DOCS)

CONF_DOCS := $(foreach conf, $(CONF_FILES), docs/stmbldoc/content/docs/hal_configs/$(notdir $(conf:.txt=.md)))

# Explicitly define targets for each template file
docs/stmbldoc/content/docs/hal_configs/%.md: conf/%.txt docs/stmbldoc/static/graph/f4_%.dot
	@echo Generating docs for component $@
	@$(PYTHON) docs/tools/create_config_doc.py $@ $(CONF_FILES)
# Phony target to generate all documentation files
.PHONY: docs

docs: $(CONF_DOCS)

