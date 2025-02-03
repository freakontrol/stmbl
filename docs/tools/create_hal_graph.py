#!/usr/bin/env python
import re
import sys

from hal_graph import Graph, generate_dot_file

def remove_commented_lines(content):
    # Remove lines that are comments or contain comments
    lines = content.split('\n')
    non_comment_lines = [line for line in lines if not line.strip().startswith('//')]
    return '\n'.join(non_comment_lines)

def parse_commands(content):
    # Remove commented lines
    cleaned_content = remove_commented_lines(content)
    # Extract commands from hal_parse() function calls
    command_pattern = re.compile(r'hal_parse\("([^"]+)"\)')
    commands = command_pattern.findall(cleaned_content)
    return commands

def load_pins_from_pickle(pickle_file_path):
    with open(pickle_file_path, 'rb') as file:
        pins_dict = pickle.load(file)
    return pins_dict

def build_graph_from_commands_and_pins(commands, pins_dict):
    graph = Graph()
    component_counter = {}

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
                    
            graph.get_component(component_name).add_pin('rt_prio')

        elif parts[0] == "link":
            pass

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

def generate_dot_file_from_commands_and_pins(main_content, pins_dict):
    commands = parse_commands(main_content)
    graph = build_graph_from_commands_and_pins(commands, pins_dict)
    dot = generate_dot_file(graph)
    return dot

def main(output_file, main_file, *input_files):
    comps = {}
    
    for infile in input_files:
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
            comps[compname].extend(pins)
            
    with open(main_file, 'r') as main_file:
        main_content = main_file.read()

    dot = generate_dot_file_from_commands_and_pins(main_content, comps)
    dot.render(output_file, format='svg')

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: create_hal_tbl_graph.py <main_file> <input_files...>")
        sys.exit(1)
    main(sys.argv[1], *sys.argv[2:])
