from machine import Pin


class InfraRed:
    def __init__(self, pin):
        """Initialise IR Sensor object"""
        self.ir = Pin(pin, Pin.IN)

    def is_hazard(self):
        return not self.ir.value()
