from .pin import Pin
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