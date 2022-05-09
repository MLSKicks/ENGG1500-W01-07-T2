from machine import Pin, I2C
from time import ticks_ms, ticks_diff, sleep_ms
from array import array
import os
import oled_screen
from rgb_sensor import RGB
from us_sensor import UltraSonic
from ir_sensor import InfraRed
from encoder import EncoderClicker
import motor
import pid_control
from pid_control import clicks_to_mm


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
    def __init__(self, init_screen=False, init_rgb=False, init_us_l=False, init_us_r=False,
                 init_ir_l=False, init_ir_r=False, init_encoder=False, init_motor=False):
        """Sets up oled display and rgb sensor on I2C channel 0, sda=Pin(12), scl=Pin(13)."""
        self.init_screen = init_screen
        self.init_rgb = init_rgb
        self.init_us_l = init_us_l
        self.init_us_r = init_us_r
        self.init_ir_l = init_ir_l
        self.init_ir_r = init_ir_r
        self.init_encoder = init_encoder
        self.init_motor = init_motor

        # initialise motor
        if self.init_motor:
            self.left_motor = motor.Motor("left", 8, 9, 6)
            self.right_motor = motor.Motor("right", 10, 11, 7)

        # initialise i2c devices
        if self.init_screen or self.init_rgb:
            self.i2c_bus = I2C(0, sda=Pin(12), scl=Pin(13))
            print_device_info(self.i2c_bus.scan())  # print debugging info
        if self.init_screen:
            self.screen = oled_screen.Screen(self.i2c_bus)
        if self.init_rgb:
            self.rgb = RGB(self.i2c_bus)
            self.get_calibration_rgb_road()

        # initialise sensors
        if self.init_us_l:
            self.us_l = UltraSonic(trig=3, echo=2)
        if self.init_us_r:
            self.us_r = UltraSonic(trig=5, echo=4)
        if self.init_ir_l:
            self.ir_l = InfraRed(Pin(27))
            self.get_calibration_ir('ir_l.txt', 'L', self.ir_l)
        if self.init_ir_r:
            self.ir_r = InfraRed(Pin(26))
            self.get_calibration_ir('ir_r.txt', 'R', self.ir_r)
        if self.init_encoder:
            self.encoder = EncoderClicker(19, 18)  # ENC_L corresponds to MOTOR_RIGHT so have to swap pin order!
            self.pid = pid_control.PIDController(self.encoder)

        # initialise constants
        self.SENSOR_SLEEP_MS = 10

    # - - - - - - - - - - - - - - - - - - - - GENERAL FUNCTIONS - - - - - - - - - - - - - - - - - - - - #
    def set_motor(self, lduty, rduty):
        """function to quickly set both motor values
            :type: lduty: int
            :type: rduty: int"""
        # sanitise input (0 <= left duty & right duty <= 100)
        lduty = min(lduty, 100)
        rduty = min(rduty, 100)
        lduty = max(lduty, -100)
        rduty = max(rduty, -100)

        # a negative duty means rotate backwards
        if lduty > 0:
            self.left_motor.set_forwards()
            self.left_motor.duty(lduty)
        else:
            self.left_motor.set_backwards()
            self.left_motor.duty(lduty * -1)
        if rduty > 0:
            self.right_motor.set_forwards()
            self.right_motor.duty(rduty)
        else:
            self.right_motor.set_backwards()
            self.right_motor.duty(rduty * -1)

    def update_sleep(self, milliseconds):
        """Sleeps for a time (ms) while maintaining sensor readings"""
        t0 = ticks_ms()
        while ticks_diff(ticks_ms(), t0) < milliseconds:
            if self.init_us_l:
                self.us_l.proximity()
            if self.init_us_r:
                self.us_r.proximity()
            sleep_ms(self.SENSOR_SLEEP_MS)

    # - - - - - - - - - - - - - - - - - - - - CALIBRATION ROUTINES - - - - - - - - - - - - - - - - - - - - #
    def motor_calibration(self):
        f = open('motor.txt', 'w')
        f.write('speed,offset\n')
        for speed in range(40, 90, 2):
            # get up to speed
            self.screen.print("~MotorCalibrate~\n\nSpeed {}".format(speed))
            self.set_motor(0, 0)
            sleep_ms(3000)
            self.set_motor(speed, speed)
            sleep_ms(500)

            # our reference motor is the left motor -> increase/decrease right motor until it matches
            tolerance = 3
            offset = 0
            while True:
                # travel up to 500mm so we get an error value
                self.screen.print("~MotorCalibrate~\n\nSpeed {}\nOffset {}\nclk_l {}\nclk_r {}".format(
                    speed, offset, self.encoder.get_left(), self.encoder.get_right()))

                self.encoder.clear_count()
                self.set_motor(speed-offset, speed+offset)
                while clicks_to_mm(self.encoder.get_left()) < 500:
                    self.screen.print("~MotorCalibrate~\n\nSpeed {}\nOffset {}\nclk_l {}\nclk_r {}".format(
                        speed, offset, self.encoder.get_left(), self.encoder.get_right()))

                # check what the error we got was
                error = self.encoder.get_left() - self.encoder.get_right()
                if abs(error) < tolerance:   # if error is small enough, we found our target offset!
                    break
                elif error > 0:              # for a positive error -> increase right motor
                    offset += 1
                else:                        # for a negative error -> decrease right motor
                    offset -= 1
            f.write('{},{}\n'.format(speed, offset))
        f.close()

    def get_calibration_rgb_road(self):
        try:
            f = open('rgb_road.txt', 'r')
            self.rgb.set_road_sensitivity(int(f.read()))
            f.close()
        except OSError:
            f = open('rgb_road.txt', 'w')
            self.calibrate_rgb_road()
            f.write(str(self.rgb.get_road_sensitivity()))
            f.close()
        except ValueError:
            os.remove('rgb_road.txt',)
            f = open('rgb_road.txt', 'w')
            self.calibrate_rgb_road()
            f.write(str(self.rgb.get_road_sensitivity()))
            f.close()

    def calibrate_rgb_road(self):
        """Calibrates the RGB Sensor to distinguish between road and off-road surfaces"""
        self.screen.print("~CalibrateRGB~\nPosition RGB on\nthe ROAD surface")
        for i in range(5, 0, -1):  # countdown
            self.screen.fill_rect(0, 7, 16, 1, 0)  # clear old msg
            self.screen.print_unformatted("Measuring in " + str(i), 1, 7)
            sleep_ms(1000)

        self.screen.fill_rect(0, 7, 16, 1, 0)  # clear old msg
        road_readings = []
        animation_stage = 0
        for i in range(0, 12, 1):
            # get reading
            road_readings.append(self.rgb.ambient())
            # every 3 loops update animation
            if i % 3 == 0:
                self.screen.fill_rect(5, 6, 6, 1, 0)  # clear old animation
                self.screen.print_unformatted(". " * animation_stage, 5, 6)
                animation_stage = (animation_stage + 1) % 4  # loop animation
            # delay
            sleep_ms(150)

        self.screen.print("~CalibrateRGB~\nPosition RGB on\nthe OFF-ROAD surface")
        for i in range(5, 0, -1):  # countdown
            self.screen.fill_rect(0, 7, 16, 1, 0)  # clear old msg
            self.screen.print_unformatted("Measuring in " + str(i), 1, 7)
            sleep_ms(1000)

        self.screen.fill_rect(0, 7, 16, 1, 0)  # clear old msg
        animation_stage = 0
        off_road_readings = []
        for i in range(0, 12, 1):
            # get reading
            off_road_readings.append(self.rgb.ambient())
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
        if road < off_road:  # our readings are good
            sensitivity = int((road + off_road) / 2)
            print(sensitivity)
            self.rgb.set_road_sensitivity(sensitivity)
        else:  # let's try again since we didn't get a clear distinction
            self.screen.print("~CalibrateRGB~\nError: unclear distinction between road and off-road. Retry!")
            sleep_ms(2000)
            self.calibrate_rgb_road()

    def get_calibration_ir(self, ir_str, ir_letter, ir):
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

    def calibrate_rgb(self, reference_us):
        """Calibrates RGB Sensor based on Ultrasonic Sensor data"""
        clearance = 25
        self.screen.clear()
        self.screen.print("~Calibrate RGB~\nPlace a solid \nwhite surface \n~150mm away" +
                          "\nNOTE: place US {}mm under RGB".format(clearance))
        while True:
            diff = (reference_us.proximity()-clearance) - 150
            if abs(diff) < 3:
                break
            elif diff > 0:
                self.screen.fill_rect(2, 6, 11, 1, 0)  # clear old msg
                self.screen.print_unformatted("Move Closer", 2, 7)
            else:  # diff < 0
                self.screen.fill_rect(2, 6, 11, 1, 0)  # clear old msg
                self.screen.print_unformatted("Move Away", 3, 7)
            sleep_ms(self.SENSOR_SLEEP_MS)

        self.screen.print("~Calibrate RGB~\n Now slowly move \ncloser as we\n take readings\n")
        us_readings = []
        rgb_readings = []
        i = 0
        animation_stage = 0
        while True:
            # get readings until we get close enough
            us_readings.append(int(reference_us.proximity()) - clearance)
            rgb_readings.append(int(self.rgb.proximity()))
            print(us_readings[i])
            if us_readings[i] < 10:
                break

            # every 3 loops update animation
            if i % 3 == 0:
                self.screen.fill_rect(5, 6, 6, 1, 0)  # clear old animation
                self.screen.print_unformatted(". "*animation_stage, 5, 6)
                animation_stage = (animation_stage + 1) % 4  # loop animation

            # sleep to reset readings
            i += 1
            self.update_sleep(150)

        self.screen.print("Success!")

        for (us, rgb) in zip(us_readings, rgb_readings):
            print("us: {} -> rgb: {}".format(us, rgb))

