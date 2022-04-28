from machine import Pin, I2C
from time import ticks_ms, ticks_diff, sleep_ms
import os
import oled_screen
import rgb_sensor
import us_sensor
import ir_sensor
import encoder
import motor
import pid_control


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
    def __init__(self, init_screen=True, init_rgb=True, init_us=True, init_ir=True, init_encoder=True, init_motor=True):
        """Sets up oled display and rgb sensor on I2C channel 0, sda=Pin(12), scl=Pin(13)."""
        # initialise motor
        if init_motor:
            self.left_motor = motor.Motor("left", 8, 9, 6)
            self.right_motor = motor.Motor("right", 10, 11, 7)

        # initialise i2c devices
        if init_screen or init_rgb:
            self.i2c_bus = I2C(0, sda=Pin(12), scl=Pin(13))
            print_device_info(self.i2c_bus.scan())  # print debugging info
        if init_screen:
            self.screen = oled_screen.Screen(self.i2c_bus)
        if init_rgb:
            self.rgb_sensor = rgb_sensor.Sensor(self.i2c_bus)

        # initialise other sensors
        if init_us:
            self.us_sensor = us_sensor.Sensor(trig=3, echo=2)
        if init_ir:
            self.ir_sensor = ir_sensor.Sensor(Pin(26))
            try:
                f = open('ir1.txt', 'r')
                self.ir_sensor.set_sensitivity(int(f.read()))
                f.close()
            except OSError:
                f = open('ir1.txt', 'w')
                self.calibrate_ir(self.ir_sensor, 1)
                f.write(str(self.ir_sensor.get_sensitivity()))
                f.close()
            except ValueError:
                os.remove('ir1.txt')
                f = open('ir1.txt', 'w')
                self.calibrate_ir(self.ir_sensor, 1)
                f.write(str(self.ir_sensor.get_sensitivity()))
                f.close()

        if init_encoder:
            self.encoder = encoder.Encoder(19, 18)  # ENC_L corresponds to MOTOR_RIGHT so have to swap pin order!
            self.pid = pid_control.PIDController(self.encoder)

        # initialise constants
        self.SENSOR_SLEEP_MS = 10

    # - - - - - - - - - - - - - - - - - - - - GENERAL FUNCTIONS - - - - - - - - - - - - - - - - - - - - #
    def set_motor(self, lduty, rduty):
        """function to quickly set both motor values
            :type: lduty: int
            :type: rduty: int"""
        # sanitise input (0 <= left duty & right duty <= 100)
        if lduty > 100:
            lduty = 100
        elif lduty < -100:
            lduty = -100
        if rduty > 100:
            rduty = 100
        elif rduty < -100:
            rduty = -100
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
            self.us_sensor.proximity()
            sleep_ms(self.SENSOR_SLEEP_MS)

    # - - - - - - - - - - - - - - - - - - - - CALIBRATION ROUTINES - - - - - - - - - - - - - - - - - - - - #
    def calibrate_ir(self, ir, ir_num):
        """Calibrates the IR Sensor to distinguish between road and off-road surfaces"""
        self.screen.print("~Calibrate IR~\nHold IR" + str(ir_num) + " above\nthe ROAD surface")
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

        self.screen.print("~Calibrate IR~\nHold IR" + str(ir_num) + " above\nthe OFF-ROAD surface")
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
            self.calibrate_ir(ir, ir_num)

    def calibrate_rgb(self):
        """Calibrates RGB Sensor based on Ultrasonic Sensor data"""
        clearance = 25
        self.screen.clear()
        self.screen.print("~Calibrate RGB~\nPlace a solid \nwhite surface \n~150mm away" +
                          "\nNOTE: place US {}mm under RGB".format(clearance))
        while True:
            diff = (self.us_sensor.proximity()-clearance) - 150
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
            us_readings.append(int(self.us_sensor.proximity()) - clearance)
            rgb_readings.append(int(self.rgb_sensor.proximity()))
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
