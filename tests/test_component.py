import unittest
from docs.tools.hal_graph.component import Component
from docs.tools.hal_graph.pin import Pin

class TestComponent(unittest.TestCase):

    def test_init(self):
        component = Component("test_comp")
        self.assertEqual(component.name, "test_comp")
        self.assertIsInstance(component.pins, dict)

    def test_add_pin(self):
        component = Component("test_comp")
        component.add_pin("pin1", 10)
        self.assertIn("pin1", component.pins)
        self.assertIsInstance(component.pins["pin1"], Pin)
        self.assertEqual(component.pins["pin1"].value, 10)

    def test_get_pin(self):
        component = Component("test_comp")
        component.add_pin("pin1", 10)
        pin = component.get_pin("pin1")
        self.assertIsInstance(pin, Pin)
        self.assertEqual(pin.value, 10)

    def test_add_pin_duplicate(self):
        component = Component("test_comp")
        component.add_pin("pin1", 10)
        component.add_pin("pin1", 20)  # Adding the same pin again
        self.assertIn("pin1", component.pins)
        self.assertEqual(component.pins["pin1"].value, 10)

    def test_eq(self):
        comp1 = Component("test_comp")
        comp2 = Component("test_comp")
        comp3 = Component("different_comp")
        self.assertEqual(comp1, comp2)
        self.assertNotEqual(comp1, comp3)

    def test_repr(self):
        component = Component("test_comp")
        self.assertEqual(repr(component), "Component(test_comp)")

if __name__ == '__main__':
    unittest.main()