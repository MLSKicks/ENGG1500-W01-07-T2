from machine import ADC


class InfraRed:
    def __init__(self, analog_pin):
        """Initialise IR Sensor object"""
        self.adc = ADC(analog_pin)

        # sensitivity constants
        self.SENSITIVITY = 10000  # any analog voltage above this is considered 'road'

    def set_sensitivity(self, sensitivity):
        self.SENSITIVITY = sensitivity

    def get_sensitivity(self):
        return self.SENSITIVITY

    def is_on_road(self):
        reading = self.reading()
        if reading > self.SENSITIVITY:
            return True
        else:
            return False

    def reading(self):
        return self.adc.read_u16()  # value between 0 - 65535
