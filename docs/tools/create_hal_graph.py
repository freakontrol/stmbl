#!/usr/bin/env python
import re
import sys
import os

from hal_graph import Graph, generate_dot_file

graph = Graph()
component_counter = {}

def remove_commented_lines(content):
    # Remove lines that are comments or contain comments
    lines = content.split('\n')
    non_comment_lines = [line for line in lines if not line.strip().startswith('//') or not line.strip().startswith('#')]
    return '\n'.join(non_comment_lines)

def parse_commands(content):
    # Remove commented lines
    cleaned_content = remove_commented_lines(content)
    # Extract commands from hal_parse() function calls
    command_pattern = re.compile(r'hal_parse\("([^"]+)"\)')
    commands = command_pattern.findall(cleaned_content)
    return commands

def parse_config_commands(content):
    # Remove commented lines
    cleaned_content = remove_commented_lines(content)
    commands = cleaned_content.split('\n')
    return commands

def build_graph_from_commands_and_pins(commands, pins_dict, config_commands):
    def get_next_component_name(base_name):
        if base_name not in component_counter:
            component_counter[base_name] = -1  # Start from 0
        component_counter[base_name] += 1
        return f"{base_name}{component_counter[base_name]}"
    
    for command in commands:
        parts = re.split(r'(\s+|=)', command)
        parts = [part for part in parts if part.strip()]
        if len(parts) < 2:
            continue
        if parts[0] == "load":
            # Create a new component
            base_name = parts[1]
            component_name = get_next_component_name(base_name)
            graph.add_component(component_name)

            if base_name in pins_dict:
                for pin in pins_dict[base_name]:
                    graph.get_component(component_name).add_pin(pin)

        elif parts[0] == "link":
            template = parts[1]
            build_graph_from_commands_and_pins(config_commands[template], pins_dict, config_commands)

        else:
            # Handle pin assignments and connections
            if '=' in parts[1]:
                src_comp_name = parts[0].split('.')[0]
                src_pin_name = parts[0].split('.')[1]
                dst_pin_info = parts[2]

                if re.match(r'^-?\d+(\.\d+)?$', dst_pin_info):
                    # Pin value assignment (integer or float)
                    pin_value = float(dst_pin_info) if '.' in dst_pin_info else int(dst_pin_info)
                    graph.get_component(src_comp_name).get_pin(src_pin_name).value = pin_value

                elif '.' in dst_pin_info:
                    # Pin connection
                    try:
                        dst_comp_name, dst_pin_name = dst_pin_info.split('.', 1)  # Split only at the first '.'
                        graph.connect_pins(src_comp_name, src_pin_name, dst_comp_name, dst_pin_name)
                    except ValueError as e:
                        print(f"Error parsing pin connection: {e}")

    return graph

def main(output_file, main_file, comp_files, config_templates):
    comps = {}

    for infile in comp_files:
        with open(infile) as f:
            pins = []
            compname = ''
            for line in f:
                comp = re.search('COMP\((\w*)\);', line)
                if comp:
                    compname = comp.groups()[0]
                    comps[compname] = []
                pin = re.search('HAL_PIN\((\w*)\)', line)
                if pin:
                    pins.append(pin.groups()[0])
                pin = re.search('HAL_PINA\((\w*),\s*(\d*)\)', line)
                if pin:
                    for i in range(int(pin.groups()[1])):
                        pins.append(f"{pin.groups()[0]}{i}")
            pins.append("rt_prio")
            pins.append("frt_prio")
            comps[compname].extend(pins)

    # Parse config templates
    config_commands = {}
    for template_file in config_templates:
        with open(template_file, 'r') as f:
            content = f.read()
            commands = parse_config_commands(content)
            config_name = os.path.basename(template_file).split('.')[0]
            config_commands[config_name] = commands

    with open(main_file, 'r') as m_file:
        main_content = m_file.read()
        
    if main_file.endswith('.c'):
        main_commands = parse_commands(main_content)
    elif main_file.endswith('.txt'):
        main_commands = parse_config_commands(main_content)
        main_commands = ["load term"] + main_commands

    graph = build_graph_from_commands_and_pins(main_commands, comps, config_commands)
    dot = generate_dot_file(graph)
    dot.render(output_file, format='svg')
    dot.render(output_file, format='png')

if __name__ == "__main__":
    if len(sys.argv) < 5 or '__TEMPLATE_MARKER__' not in sys.argv:
        print("Usage: create_hal_graph.py <output_file> <main_file> <comp_files...> __TEMPLATE_MARKER__ <config_templates...>")
        sys.exit(1)

    output_file = sys.argv[1]
    main_file = sys.argv[2]

    # Find the index of the marker
    marker_index = sys.argv.index('__TEMPLATE_MARKER__')
    comp_files = sys.argv[3:marker_index]  # Extract components
    config_templates = sys.argv[marker_index + 1:]  # Extract configuration templates

    main(output_file, main_file, comp_files, config_templates)
