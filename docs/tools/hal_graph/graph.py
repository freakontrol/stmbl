from .component import Component
import re
from graphviz import Digraph
import hashlib

class Graph:
    def __init__(self):
        self.components = {}
        self.component_counter = {}

    def add_component(self, component_name):
        if component_name not in self.components:
            self.components[component_name] = Component(component_name)

    def get_component(self, component_name):
        return self.components.get(component_name)

    def connect_pins(self, src_comp_name, src_pin_name, dst_comp_name, dst_pin_name):
        try:
            src_component = self.get_component(src_comp_name)
            src_pin = src_component.get_pin(src_pin_name)
        except AttributeError as e:
            # Handle the case where the component or pin does not exist
            print(f"Warning: {src_comp_name}.{src_pin_name} does not exist.")
            return
        except Exception as e:
            # Handle any other exceptions that might occur
            print(f"An error occurred while retrieving src_pin: {e}")
            return

        try:
            dst_component = self.get_component(dst_comp_name)
            dst_pin = dst_component.get_pin(dst_pin_name)
        except AttributeError as e:
            # Handle the case where the component or pin does not exist
            print(f"Warning: {dst_comp_name}.{dst_pin_name} does not exist.")
            return
        except Exception as e:
            # Handle any other exceptions that might occur
            print(f"An error occurred while retrieving dst_pin: {e}")
            return
        if src_pin and dst_pin:
            src_pin.connect(dst_pin)
            
    def get_next_component_name(self, base_name):
        if base_name not in self.component_counter:
            self.component_counter[base_name] = -1  # Start from 0
        self.component_counter[base_name] += 1
        return f"{base_name}{self.component_counter[base_name]}"

    def build_graph_from_commands_and_pins(self, commands, pins_dict, config_commands):

        for command in commands:
            parts = re.split(r'(\s+|=)', command)
            parts = [part for part in parts if part.strip()]
            if len(parts) < 2:
                continue
            if parts[0] == "load":
                # Create a new component
                base_name = parts[1]
                component_name = self.get_next_component_name(base_name)

                if base_name in pins_dict:
                    self.add_component(component_name)
                    for pin in pins_dict[base_name]:
                        self.get_component(component_name).add_pin(pin)
                else:
                     print(f"Warning: {component_name} does not exist.")

            elif parts[0] == "link":
                template = parts[1]
                self.build_graph_from_commands_and_pins(config_commands[template], pins_dict, config_commands)

            else:
                # Handle pin assignments and connections
                if '=' in parts[1]:
                    src_comp_name = parts[0].split('.')[0]
                    src_pin_name = parts[0].split('.')[1]
                    dst_pin_info = parts[2]

                    if re.match(r'^-?\d+(\.\d+)?$', dst_pin_info):
                        # Pin value assignment (integer or float)
                        pin_value = float(dst_pin_info) if '.' in dst_pin_info else int(dst_pin_info)
                        try:
                            self.get_component(src_comp_name).get_pin(src_pin_name).value = pin_value
                        except AttributeError as e:
                            # Handle the case where the component or pin does not exist
                            print(f"Warning: {src_comp_name}.{src_pin_name} does not exist.")
                        except Exception as e:
                            # Handle any other exceptions that might occur
                            print(f"An error occurred: {e}")

                    elif '.' in dst_pin_info:
                        # Pin connection
                        try:
                            dst_comp_name, dst_pin_name = dst_pin_info.split('.', 1)  # Split only at the first '.'
                            self.connect_pins(src_comp_name, src_pin_name, dst_comp_name, dst_pin_name)
                        except ValueError as e:
                            print(f"Error parsing pin connection: {e}")
        return self

    def generate_dot_file(self):
        dot = Digraph()
        dot.attr(fontname="Roboto")
        dot.node_attr.update(fontsize="16", shape="ellipse", fontname="Roboto", color="white", fontcolor="white")
        dot.edge_attr.update(len="0.3", fontname="Roboto", color="white", fontcolor="white")
        dot.graph_attr.update(rankdir="LR", overlap="false", splines="true", nodesep="0.15", ranksep="5.0",bgcolor="transparent")

        for comp in self.components.values():
            color = get_color_for_component(comp.name)
            label = f'<<table border="0" cellborder="1" cellspacing="0">\n\t\t\t<tr><td colspan="1" bgcolor="{color}"><b>{comp.name}</b></td></tr>\n'

            for pin in comp.pins.values():
                label += f'\t\t\t<tr><td port="{pin.name}">{pin.name}{" = " + str(pin.value) if pin.value is not None else ""}</td></tr>' + '\n'
            label += "\t\t\t</table>>"
            dot.node(str(comp.name), label=label, shape="none")

        for comp_name, comp in self.components.items():
            for pin_name, pin in comp.pins.items():
                for connected_pin in pin.connections:
                    dst_pin_ref = f'{comp_name}:{pin_name}'
                    src_pin_ref = f'{connected_pin.component.name}:{connected_pin.name}'
                    edge_color = get_color_for_edge(comp_name, pin_name, connected_pin.component.name, connected_pin.name)
                    dot.edge(src_pin_ref, dst_pin_ref, color=edge_color)

        return dot
    
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
    # Ensure the color is not brighter than RGB (150, 150, 150)
    r = min(r, 100)
    g = min(g, 100)
    b = min(b, 100)
    # Return the color as a hex string
    return f'#{r:02X}{g:02X}{b:02X}'

def get_color_for_edge(src_comp_name, src_pin_name, dst_comp_name, dst_pin_name):
    edge_key = f"{src_comp_name}_{src_pin_name}_{dst_comp_name}_{dst_pin_name}"
    # Create a hash object
    hash_object = hashlib.md5(edge_key.encode())
    # Get the hex digest of the hash
    hash_hex = hash_object.hexdigest()
    # Convert the hex digest to an integer
    hash_int = int(hash_hex, 16)
    # Generate RGB values from the hash integer
    r = (hash_int >> 16) & 0xFF
    g = (hash_int >> 8) & 0xFF
    b = hash_int & 0xFF
    # Ensure the color is not brighter than RGB (255, 255, 255)
    r = min(r, 255)
    g = min(g, 255)
    b = min(b, 255)
    r = max(r, 150)
    g = max(g, 150)
    b = max(b, 150)
    # Return the color as a hex string
    return f'#{r:02X}{g:02X}{b:02X}'