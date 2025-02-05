import unittest
from docs.tools.hal_graph.pin import Pin

class TestPin(unittest.TestCase):

    def test_init(self):
        pin = Pin("pin1", 10, None)
        self.assertEqual(pin.name, "pin1")
        self.assertEqual(pin.value, 10)
        self.assertIsNone(pin.component)
        self.assertIsInstance(pin.connections, list)

    def test_connect(self):
        pin1 = Pin("pin1", 10, None)
        pin2 = Pin("pin2", 20, None)
        pin3 = Pin("pin3", 30, None)

        pin1.connect(pin2)
        self.assertIn(pin2, pin1.connections)

        # Connecting the same pin again should not add it twice
        pin1.connect(pin2)
        self.assertEqual(len(pin1.connections), 1)

        # Connecting to itself should not add it to connections
        pin1.connect(pin1)
        self.assertNotIn(pin1, pin1.connections)

    def test_eq(self):
        pin1 = Pin("pin1", 10, None)
        pin2 = Pin("pin1", 20, None)
        pin3 = Pin("pin2", 30, None)
        self.assertTrue(pin1 == pin2)
        self.assertFalse(pin1 == pin3)

    def test_repr(self):
        pin = Pin("pin1", 10, None)
        self.assertEqual(repr(pin), "Pin(pin1)")

if __name__ == '__main__':
    unittest.main()