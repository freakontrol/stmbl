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

    def test_update_pins(self):
        component = Component("test_comp")
        another_comp = Component("another_comp")
        component.add_pin("pin1", 10)
        component.add_pin("pinx", 30)
        another_comp.add_pin("pinx", 50)
        another_comp.add_pin("pin2", 20)
        
        pin1 = component.get_pin("pin1")
        pin2 = another_comp.get_pin("pin2")

        pin1.connect(pin2)

        pin1.update()

        self.assertEqual(pin1.value, 20)
        self.assertTrue(pin1.updated)
        self.assertTrue(pin2.updated)

    def test_reset_pins(self):
        component = Component("test_comp")
        component.add_pin("pin1", 10)

        pin1 = component.get_pin("pin1")
        pin1.updated = True

        component.reset()

        self.assertFalse(pin1.updated)

if __name__ == '__main__':
    unittest.main()