from machine import Pin, I2C
import oled_screen
import rgb_sensor


def print_device_info(devices):
    """Print out device debugging info from an I2C.scan()"""
    if len(devices) == 0:
        print("peripherals.py Error: i2c device(s) not found. Is it plugged in to the correct pins?")
    else:
        print('i2c devices found:', len(devices))
        for device in devices:
            print("Decimal address: ", device, " | Hexadecimal address: ", hex(device))


class Peripherals:
    def __init__(self):
        """Sets up oled display and rgb sensor on I2C channel 0, sda=Pin(12), scl=Pin(13)."""
        self.i2c_bus = I2C(0, sda=Pin(12), scl=Pin(13))
        devices = self.i2c_bus.scan()
        print_device_info(devices)

        # initialise each peripheral
        self.screen = oled_screen.Screen(self.i2c_bus)
        self.rbg_sensor = rgb_sensor.Sensor(self.i2c_bus)

