from .component import Component

from graphviz import Digraph
import hashlib
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