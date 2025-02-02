#!/usr/bin/env python
import re
import sys
import pickle
from typing import Dict, List
from graphviz import Digraph
import hashlib

class Pin:
    def __init__(self, name, value, component):
        self.name = name
        self.value = value
        self.component = component  # Reference to the component
        self.connections = []

    def connect(self, pin):
        if pin not in self.connections and pin is not self:
            self.connections.append(pin)

    def __eq__(self, other):
        return self.name == other.name

    def __repr__(self):
        return f"Pin({self.name})"

class Component:
    def __init__(self, name):
        self.name = name
        self.pins = {}

    def add_pin(self, pin_name, value=None):
        if pin_name not in self.pins:
            self.pins[pin_name] = Pin(pin_name, value, self)  # Set the component reference

    def get_pin(self, pin_name):
        return self.pins.get(pin_name)

    def __eq__(self, other):
        return self.name == other.name

    def __repr__(self):
        return f"Component({self.name})"

class Graph:
    def __init__(self):
        self.components = {}

    def add_component(self, component_name):
        if component_name not in self.components:
            self.components[component_name] = Component(component_name)

    def get_component(self, component_name):
        return self.components.get(component_name)

    def connect_pins(self, src_comp_name, src_pin_name, dst_comp_name, dst_pin_name):
        src_pin = self.get_component(src_comp_name).get_pin(src_pin_name)
        dst_pin = self.get_component(dst_comp_name).get_pin(dst_pin_name)
        if src_pin and dst_pin:
            src_pin.connect(dst_pin)

    def __repr__(self):
        return f"Graph({list(self.components.keys())})"
    
def get_color_for_component(component_name):
    # Create a hash object
    hash_object = hashlib.md5(component_name.encode())
    # Get the hex digest of the hash
    hash_hex = hash_object.hexdigest()
    # Convert the hex digest to an integer
    hash_int = int(hash_hex, 16)
    # Generate RGB values from the hash integer
    r = (hash_int >> 16) & 0xFF
    g = (hash_int >> 8) & 0xFF
    b = hash_int & 0xFF
    # Return the color as a hex string
    return f'#{r:02X}{g:02X}{b:02X}'



def generate_dot_file(graph):
    dot = Digraph()
    dot.attr(fontname="Arial")
    dot.node_attr.update(fontname="Arial", fontsize="16", shape="ellipse")
    dot.edge_attr.update(fontname="Arial", len="1.0")
    dot.graph_attr.update(rankdir="LR", overlap="false", nodesep="0.3", ranksep="7.0")

    for comp in graph.components.values():
        color = get_color_for_component(comp.name)
        label = f'<<table border="0" cellborder="1" cellspacing="0">\n\t\t\t<tr><td colspan="1" bgcolor="{color}"><b>{comp.name}</b></td></tr>\n'
        
        for pin in comp.pins.values():
            label += f'\t\t\t<tr><td port="{pin.name}">{pin.name}{" = " + str(pin.value) if pin.value is not None else ""}</td></tr>' + '\n'
        label += "\t\t\t</table>>"
        dot.node(str(comp.name), label=label, shape="none")

    for comp_name, comp in graph.components.items():
        for pin_name, pin in comp.pins.items():
            for connected_pin in pin.connections:
                dst_pin_ref = f'{comp_name}:{pin_name}'
                src_pin_ref = f'{connected_pin.component.name}:{connected_pin.name}'
                dot.edge(src_pin_ref, dst_pin_ref)

    return dot

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
