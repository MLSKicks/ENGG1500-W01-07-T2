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
            self.left_motor = Motor("left", 8, 9, 6)
            self.right_motor = Motor("right", 10, 11, 7)

        # Initialise i2c devices
        if self.init_screen:
            self.i2c_bus = I2C(0, sda=Pin(12), scl=Pin(13))
            self.screen = Screen(self.i2c_bus)

        # Initialise sensors
        if self.init_ir_f:
            self.ir_f = InfraRed(Pin(27))
            self.get_calibration_ir('ir_f.txt', 'F', self.ir_f)
        if self.init_ir_b:
            self.ir_b = InfraRed(Pin(26))
            self.get_calibration_ir('ir_b.txt', 'B', self.ir_b)
        if self.init_encoder:
            self.encoder = EncoderClicker(19, 18)  # ENC_L corresponds to MOTOR_RIGHT so have to swap pin order!
            self.controller = MotorController(self.encoder)

    # - - - - - - - - - - - - - - - - - - - - GENERAL FUNCTIONS - - - - - - - - - - - - - - - - - - - - #
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

    # - - - - - - - - - - - - - - - - - - - - CALIBRATION ROUTINES - - - - - - - - - - - - - - - - - - - - #
    def get_calibration_ir(self, ir_str, ir_letter, ir):
        """Routine for reading the current calibration for the ir sensors"""
        try:
            f = open(ir_str, 'r')
            ir.set_sensitivity(int(f.read()))
            f.close()
        except OSError:
            f = open(ir_str, 'w')
            self.calibrate_ir(ir, ir_letter)
            f.write(str(ir.get_sensitivity()))
            f.close()
        except ValueError:
            os.remove(ir_str)
            f = open(ir_str, 'w')
            self.calibrate_ir(ir, ir_letter)
            f.write(str(ir.get_sensitivity()))
            f.close()

    def calibrate_ir(self, ir, ir_letter):
        """Calibrates the IR Sensor to distinguish between road and off-road surfaces"""
        self.screen.print("~Calibrate IR~\nHold IR" + ir_letter + " above\nthe ROAD surface")
        for i in range(5, 0, -1):  # countdown
            self.screen.fill_rect(0, 7, 16, 1, 0)  # clear old msg
            self.screen.print_unformatted("Measuring in " + str(i), 1, 7)
            sleep_ms(1000)

        self.screen.fill_rect(0, 7, 16, 1, 0)  # clear old msg
        road_readings = []
        animation_stage = 0
        for i in range(0, 12, 1):
            # get reading
            road_readings.append(ir.reading())
            # every 3 loops update animation
            if i % 3 == 0:
                self.screen.fill_rect(5, 6, 6, 1, 0)  # clear old animation
                self.screen.print_unformatted(". " * animation_stage, 5, 6)
                animation_stage = (animation_stage + 1) % 4  # loop animation
            # delay
            sleep_ms(150)

        self.screen.print("~Calibrate IR~\nHold IR" + ir_letter + " above\nthe OFF-ROAD surface")
        for i in range(5, 0, -1):  # countdown
            self.screen.fill_rect(0, 7, 16, 1, 0)  # clear old msg
            self.screen.print_unformatted("Measuring in " + str(i), 1, 7)
            sleep_ms(1000)

        self.screen.fill_rect(0, 7, 16, 1, 0)  # clear old msg
        animation_stage = 0
        off_road_readings = []
        for i in range(0, 12, 1):
            # get reading
            off_road_readings.append(ir.reading())
            # every 3 loops update animation
            if i % 3 == 0:
                self.screen.fill_rect(5, 6, 6, 1, 0)  # clear old animation
                self.screen.print_unformatted(". " * animation_stage, 5, 6)
                animation_stage = (animation_stage + 1) % 4  # loop animation
            # delay
            sleep_ms(150)

        # calculate sensitivity threshold as in between road and off-road readings
        road = min(road_readings)
        print(road)
        off_road = max(off_road_readings)
        print(off_road)
        if road > off_road:  # our readings are good
            sensitivity = int((road + off_road) / 2)
            print(sensitivity)
            ir.set_sensitivity(sensitivity)
        else:  # let's try again since we didn't get a clear distinction
            self.screen.print("~Calibrate IR~\nError: unclear distinction between road and off-road. Retry!")
            sleep_ms(2000)
            self.calibrate_ir(ir, ir_letter)
