import unittest
from docs.tools.hal_graph.pin import Pin

class TestPin(unittest.TestCase):

    def test_init(self):
        pin = Pin("pin1", 10, None)
        self.assertEqual(pin.name, "pin1")
        self.assertEqual(pin.value, 10)
        self.assertIsNone(pin.component)
        self.assertIsNone(pin.connected_pin)
        self.assertFalse(pin.updated)

    def test_connect(self):
        pin1 = Pin("pin1", 10, None)
        pin2 = Pin("pin2", 20, None)
        pin3 = Pin("pin2", 30, None)

        pin1.connect(pin2)
        self.assertEqual(pin2, pin1.connected_pin)

        pin1.connect(pin3)
        self.assertEqual(pin3, pin1.connected_pin)

        # Connecting to itself should not add it to connections
        pin1.connect(pin1)
        self.assertNotEqual(pin1, pin1.connected_pin)

    def test_eq(self):
        pin1 = Pin("pin1", 10, None)
        pin2 = Pin("pin1", 20, None)
        pin3 = Pin("pin2", 30, None)
        self.assertEqual(pin1, pin2)
        self.assertNotEqual(pin1, pin3)

    def test_repr(self):
        pin = Pin("pin1", 10, None)
        self.assertEqual(repr(pin), "Pin(pin1)")

    def test_update(self):
        pin1 = Pin("pin1", 10, None)
        pin2 = Pin("pin2", 20, None)
        pin3 = Pin("pin3", 30, None)

        pin1.connect(pin2)
        pin2.connect(pin3)
        
        self.assertFalse(pin1.updated)
        self.assertFalse(pin2.updated)
        self.assertFalse(pin3.updated)
        
        pin1.update()

        self.assertEqual(pin1.value, 30)
        self.assertTrue(pin1.updated)
        self.assertTrue(pin2.updated)
        self.assertTrue(pin3.updated)

    def test_reset(self):
        pin = Pin("pin1", 10, None)
        pin.updated = True
        pin.reset()
        self.assertFalse(pin.updated)

if __name__ == '__main__':
    unittest.main()