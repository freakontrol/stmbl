class Pin:
    def __init__(self, name, value, component):
        self.name = name
        self.value = value
        self.component = component  # Reference to the component
        self.connected_pin = None # self <= connected_pin

    def connect(self, src_pin):
        if(src_pin is not self):
            self.connected_pin = src_pin
            self.update()
        
    def update(self):
        self.value = self.connected_pin.value

    def __eq__(self, other):
        return self.name == other.name

    def __repr__(self):
        return f"Pin({self.name})"