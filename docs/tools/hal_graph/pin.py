class Pin:
    def __init__(self, name, value, component):
        self.name = name
        self.value = value
        self.component = component  # Reference to the component
        self.connected_pin = None # self <= connected_pin
        self.updated = False

    def connect(self, src_pin):
        if(src_pin is not self):
            self.connected_pin = src_pin
        
    def update(self):
        if self.connected_pin is not None and self.updated is False:
            self.connected_pin.update()
            if self.connected_pin.value is not None:
                self.value = self.connected_pin.value
        self.updated = True
        
    def reset(self):
        self.updated = False

    def __eq__(self, other):
        return self.name == other.name

    def __repr__(self):
        return f"Pin({self.name})"