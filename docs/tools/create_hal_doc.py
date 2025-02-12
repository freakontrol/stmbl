#!/usr/bin/env python
import re
import sys
import os

from hal_graph import *
def main(output_file, main_file, comp_files, config_templates):

    # Collect components and configuration commands
    comps = collect_components(comp_files)
    config_commands = collect_template_conf(config_templates)

    # with open(main_file, 'r') as m_file:
    #     main_content = m_file.read()

    # if main_file.endswith('.c'):
    #     main_commands = parse_commands(main_content)
    # elif main_file.endswith('.txt'):
    #     main_commands = parse_config_commands(main_content)


    # Collect component documentation and write to Markdown file
    comp_docs = collect_comp_docs(main_file)
    
    with open(output_file, 'w') as md_file:
        for compname, description in comp_docs.items():
            md_file.write(description + '\n\n')

if __name__ == "__main__":
    if len(sys.argv) < 5 or '__TEMPLATE_MARKER__' not in sys.argv:
        #print("Usage: create_hal_graph.py <output_file> <main_file> <comp_files...> __TEMPLATE_MARKER__ <config_templates...>")
        sys.exit(1)

    output_file = sys.argv[1]
    main_file = sys.argv[2]
    
    print("doc " + output_file + " source " + main_file)

    # Find the index of the marker
    marker_index = sys.argv.index('__TEMPLATE_MARKER__')
    comp_files = sys.argv[3:marker_index]  # Extract components
    config_templates = sys.argv[marker_index + 1:]  # Extract configuration templates

    main(output_file, main_file, comp_files, config_templates)
