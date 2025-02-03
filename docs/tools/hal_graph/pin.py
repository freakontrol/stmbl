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