#!/usr/bin/env python
import re
import sys
import os

from hal_graph import *

def extract_pins(infile):
    pins = []
    with open(infile, 'r') as f:
        content = f.readlines()

        # Regular expression to match HAL_PIN and HAL_PINA definitions with optional comments
        pin_pattern = re.compile(r'^\s*HAL_PIN\((\w+)\)\s*;?\s*(//\s*(.*)|)$')
        pin_a_pattern = re.compile(r'^\s*HAL_PINA\((\w+),\s*(\d+)\)\s*;?\s*(//\s*(.*)|)$')

        for line in content:
            match = pin_pattern.match(line)
            if match:
                pin_name, _, description = match.groups()
                pins.append((pin_name, description.strip() if description else ""))
            else:
                match_a = pin_a_pattern.match(line)
                if match_a:
                    base_pin_name, count, _, description = match_a.groups()
                    for i in range(int(count)):
                        pins.append((f"{base_pin_name}{i}", description.strip() if description else ""))

    return pins

def collect_comp_docs(infile):
    description = ""
    with open(infile, 'r') as f:
        content = f.read()
        # Extract Doxygen-style comments
        comment_pattern = re.compile(r'/\*\*(.*?)\*/', re.DOTALL)
        comments = comment_pattern.findall(content)

        if comments:
            # Join consecutive comments to form the description and remove leading asterisks
            description_lines = []
            for comment in comments:
                lines = comment.split('\n')
                cleaned_lines = [re.sub(r'^\s*\*+', '', line).strip() for line in lines]
                description_lines.extend(cleaned_lines)
            description = '\n'.join(description_lines).strip()

    compname = os.path.basename(infile).split('.')[0].upper()
    formatted_description = f"# {compname}\n{description}"
    return formatted_description

def main(output_file, comp_files, config_templates):
    # Collect components and configuration commands
    comps = collect_components(comp_files)
    config_commands = collect_template_conf(config_templates)

    # Extract the base name of the output file (excluding the path and extension)
    output_base_name = os.path.splitext(os.path.basename(output_file))[0]

    # Find which component file has the same name as the output file, ignoring extensions.
    # F3 and F4 can both have a component with the same name (hv): the F4 one gets <name>.md,
    # the F3 one (under stm32f303/) gets <name>_f3.md.
    comp_name = output_base_name
    want_f3 = False
    if comp_name.endswith('_f3'):
        comp_name = comp_name[:-3]
        want_f3 = True
    candidates = [c for c in comp_files if os.path.splitext(os.path.basename(c))[0] == comp_name]
    if want_f3:
        candidates = [c for c in candidates if c.startswith('stm32f303/')]
    else:
        candidates.sort(key=lambda c: c.startswith('stm32f303/'))
    matching_comp_file = candidates[0] if candidates else None

    # Collect component documentation and write to Markdown file
    description = collect_comp_docs(matching_comp_file)
    description = description.replace(f"# {comp_name.upper()}\n", f"# {output_base_name.upper()}\n", 1)

    pins = []
    if matching_comp_file:
        pins = extract_pins(matching_comp_file)

    with open(output_file, 'w') as md_file:
        md_file.write(description + '\n\n')
        if pins:
            md_file.write("## Pins  \n| Name | Description |\n|----|----|\n")
            for pin_name, pin_description in pins:
                md_file.write(f"| **{pin_name}** | {pin_description} |\n")
        md_file.write(f"\n## References  \n Source: [.c File](https://github.com/freakontrol/stmbl/blob/main/{matching_comp_file})\n")

if __name__ == "__main__":
    if len(sys.argv) < 4 or '__TEMPLATE_MARKER__' not in sys.argv:
        #print("Usage: create_hal_graph.py <output_file> <comp_files...> __TEMPLATE_MARKER__ <config_templates...>")
        sys.exit(1)

    output_file = sys.argv[1]

    # Find the index of the marker
    marker_index = sys.argv.index('__TEMPLATE_MARKER__')
    comp_files = sys.argv[2:marker_index]  # Extract components
    config_templates = sys.argv[marker_index + 1:]  # Extract configuration templates

    main(output_file, comp_files, config_templates)