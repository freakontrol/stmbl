import unittest
from graphviz import Digraph
from docs.tools.hal_graph.graph import Graph, Component

class TestGraph(unittest.TestCase):

    def setUp(self):
        self.graph = Graph()

    def test_add_component(self):
        component_name = "test_comp"
        self.graph.add_component(component_name)
        self.assertIn(component_name, self.graph.components)

    def test_get_component(self):
        component_name = "test_comp"
        self.graph.add_component(component_name)
        component = self.graph.get_component(component_name)
        self.assertIsInstance(component, Component)
        self.assertEqual(component.name, component_name)

    def test_connect_pins(self):
        src_comp_name = "src_comp"
        dst_comp_name = "dst_comp"
        src_pin_name = "pin1"
        dst_pin_name = "pin2"

        self.graph.add_component(src_comp_name)
        self.graph.add_component(dst_comp_name)

        src_component = self.graph.get_component(src_comp_name)
        dst_component = self.graph.get_component(dst_comp_name)

        src_component.add_pin(src_pin_name)
        dst_component.add_pin(dst_pin_name)

        self.graph.connect_pins(src_comp_name, src_pin_name, dst_comp_name, dst_pin_name)

        src_pin = src_component.get_pin(src_pin_name)
        dst_pin = dst_component.get_pin(dst_pin_name)

        self.assertEqual(dst_pin.connected_pin, src_pin)

    def test_get_next_component_name(self):
        base_name = "base"
        component_name1 = self.graph.get_next_component_name(base_name)
        component_name2 = self.graph.get_next_component_name(base_name)
        self.assertEqual(component_name1, f"{base_name}0")
        self.assertEqual(component_name2, f"{base_name}1")

    def test_build_graph_from_commands_and_pins(self):
        commands = [
            "load base",
            "load another_comp",
            "base0.pin1 = 10",
            "base0.pin2 = another_comp0.pin3",
            "another_comp0.pin3 = 5"
        ]
        pins_dict = {
            "base": ["pin1", "pin2"],
            "another_comp": ["pin3"]
        }
        config_commands = {}

        self.graph.build_graph_from_commands_and_pins(commands, pins_dict, config_commands)

        self.graph.update()

        self.assertIn("base0", self.graph.components)
        base_component = self.graph.get_component("base0")
        another_component = self.graph.get_component("another_comp0")
        
        self.assertEqual(base_component.pins["pin1"].value, 10)
        self.assertEqual(base_component.pins["pin2"].connected_pin, another_component.pins["pin3"])
        self.assertEqual(base_component.pins["pin2"].value, 5)

    def test_generate_dot_file(self):
        commands = [
            "load base",
            "load another_comp",
            "base0.pin1 = another_comp0.pin2"
        ]
        pins_dict = {
            "base": ["pin1"],
            "another_comp": ["pin2"]
        }
        config_commands = {}

        self.graph.build_graph_from_commands_and_pins(commands, pins_dict, config_commands)

        dot = self.graph.generate_dot_file()
        self.assertIsInstance(dot, Digraph)

if __name__ == '__main__':
    unittest.main()