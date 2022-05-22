from machine import Pin, I2C
from time import sleep_ms
import os
from components.ir_sensor import InfraRed
from components.encoder import EncoderClicker
from components.motor import Motor
from components.oled_screen import Screen
from motor_control import MotorController


# - - - - - - - - - - - - - - - - - - - - DEBUG PRINTING - - - - - - - - - - - - - - - - - - - - #
def print_device_info(devices):
    """Print out device debugging info from an I2C.scan()"""
    if len(devices) == 0:
        print("vehicle_components.py Error: i2c device(s) not found. Is it plugged into the correct pins?")
    else:
        print('i2c devices found:', len(devices))
        for device in devices:
            print("Decimal address: ", device, " | Hexadecimal address: ", hex(device))


class Vehicle:
    # - - - - - - - - - - - - - - - - - - - - INITIALISATION - - - - - - - - - - - - - - - - - - - - #
    def __init__(self, motor=False, enc=False, screen=False, ir_f=False, ir_b=False):
        """This is basically a big interface class. It initialises all devices and components that are
        requested, calibrating them if not already calibrated."""
        # Hold on to the flags containing what we wanted to initialise
        self.init_motor = motor
        self.init_encoder = enc
        self.init_screen = screen
        self.init_ir_f = ir_f
        self.init_ir_b = ir_b

        # Initialise motor
        if self.init_motor:
            self.right_motor = Motor("left", 8, 9, 6)  # NOTE THESE ARE THE WRONG WAY AROUND DUE TO HARDWARE SETUP
            self.left_motor = Motor("right", 10, 11, 7)

        # Initialise i2c devices
        if self.init_screen:
            self.i2c_bus = I2C(0, sda=Pin(12), scl=Pin(13))
            if len(self.i2c_bus.scan()) == 0:
                self.init_screen = False
            else:
                self.screen = Screen(self.i2c_bus)

        # Initialise sensors
        if self.init_ir_f:
            self.ir_f = InfraRed(26)
        if self.init_ir_b:
            self.ir_b = InfraRed(27)
        if self.init_encoder:
            self.encoder = EncoderClicker(19, 18)  # ENC_L corresponds to MOTOR_RIGHT so have to swap pin order!
            self.controller = MotorController(self.encoder)

    # - - - - - - - - - - - - - - - - - - - - GENERAL FUNCTIONS - - - - - - - - - - - - - - - - - - - - #
    def has_screen(self):
        return self.init_screen

    def set_motor(self, lduty, rduty):
        """Set motor duties. This function safely clamps the duties to values between -100 to 100
            :type: lduty: int
            :type: rduty: int"""
        # Sanitise input (0 <= left duty && right duty <= 100)
        lduty = min(lduty, 100)
        rduty = min(rduty, 100)
        lduty = max(lduty, -100)
        rduty = max(rduty, -100)

        # Set left motor
        if lduty >= 0:
            self.left_motor.set_forwards()
            self.left_motor.duty(lduty)
        else:  # Negative duty -> rotate backwards
            self.left_motor.set_backwards()
            self.left_motor.duty(lduty * -1)
        # Set right motor
        if rduty >= 0:
            self.right_motor.set_forwards()
            self.right_motor.duty(rduty)
        else:  # Negative duty -> rotate backwards
            self.right_motor.set_backwards()
            self.right_motor.duty(rduty * -1)
