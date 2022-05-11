from time import ticks_ms, ticks_diff, sleep_ms
from math import atan, exp


def mm_to_clicks(mm):
    # calculate target clicks from target mm assuming wheel diameter is 65mm
    wheel_circumference = 3.1416 * 65
    clicks_per_revolution = 40
    return int((clicks_per_revolution * mm) / wheel_circumference)


def clicks_to_mm(clicks):
    wheel_circumference = 3.1416 * 65
    clicks_per_revolution = 40
    return int((clicks * wheel_circumference) / clicks_per_revolution)


def clamp(duty, max_duty, min_duty):
    if duty > max_duty:
        return max_duty
    elif duty < min_duty:
        return min_duty
    else:
        return duty


def output(target, error):
    amplitude = 200/3.1416 * atan(target/150)
    width = -1/target**1.5
    offset = target/2
    return amplitude*exp(width*(error-offset)**2)


class MotorController:
    def __init__(self, encoder, target_mm_left=0, target_mm_right=0):
        """initialise all PID controller constants, variables, and encoder object"""
        # initialise target and encoder object
        self.target_clicks_left = mm_to_clicks(target_mm_left)
        self.target_clicks_right = mm_to_clicks(target_mm_right)
        self.encoder = encoder

        # encoder polarity is true for a forwards/zero target, and false for a backwards target
        encoder_polarity_left = self.target_clicks_left >= 0
        encoder_polarity_right = self.target_clicks_right >= 0
        self.encoder.clear_count()
        self.encoder.set_left_dir(encoder_polarity_left)
        self.encoder.set_right_dir(encoder_polarity_right)
        self.enc_left_is_fwd, self.enc_right_is_fwd = encoder_polarity_left, encoder_polarity_right
        self.toggle_left_enc = False
        self.toggle_right_enc = False

        # initialise variables for PID control
        self.click_left, self.click_right = 0, 0
        self.error_left, self.error_right = 0, 0
        self.prev_error_left, self.prev_error_right = 0, 0
        self.duty_left, self.duty_right = 0, 0

        self.t0 = ticks_ms()
        self.dt = 0

        # clamp and bias constants
        self.max_duty = 75
        self.min_duty = -75
        self.bias = 3

    def set_target(self, target_mm_left, target_mm_right):
        """Reset PID control with a new target"""
        self.__init__(self.encoder, target_mm_left, target_mm_right)

    def add_target(self, target_mm_left, target_mm_right):
        self.target_clicks_left += mm_to_clicks(target_mm_left)
        self.target_clicks_right += mm_to_clicks(target_mm_right)

    def target_met(self):
        target_met = True

        if not (-2 < self.error_left < 2):
            target_met = False

        if not (-2 < self.error_right < 2):
            target_met = False

        return target_met

    def run(self):
        """Calculates the pwm values using a closed feedback loop"""
        self.update_duty()
        self.update_encoder()
        return self.duty_correction()

    def duty_correction(self):
        """Fix bias to correct the motor imbalances. Note: a positive bias means we need to
        increase power to the right motor, and decrease power to the left motor"""
        return int(self.duty_left - self.bias), int(self.duty_right + self.bias)

    def update_elapsed_time(self):
        """Calculates the elapsed time, dt, since the last call. Also resets self.t0"""
        t1 = ticks_ms()
        self.dt = ticks_diff(t1, self.t0)
        self.t0 = t1

    def update_duty(self):
        """Calculates the error terms and PID values"""
        self.update_elapsed_time()

        # save old errors for integral section
        self.prev_error_left = self.error_left
        self.prev_error_right = self.error_right

        # calculate current error
        self.click_left, self.click_right = self.encoder.get_left(), self.encoder.get_right()
        self.error_left = self.target_clicks_left - self.encoder.get_left()
        self.error_right = self.target_clicks_right - self.encoder.get_right()

        # calculate duties
        self.duty_left = output(self.target_clicks_left, self.error_left)
        self.duty_right = output(self.target_clicks_right, self.error_right)

        # clamp duties
        self.duty_left = clamp(self.duty_left, self.max_duty, self.min_duty)
        self.duty_right = clamp(self.duty_right, self.max_duty, self.min_duty)

    def update_encoder(self):
        """if pwm polarity changes, we must change the encoder count direction,
        however, we need to stop the vehicle first since the encoder will count backwards
        while it is still travelling forwards due to inertia. We will therefore overwrite the
        duties to zero while we are awaiting a change in the encoder direction"""
        # check for polarity change in left duty
        if not self.toggle_left_enc:
            if self.duty_left > 0 and not self.enc_left_is_fwd:
                print("switching encoder polarity l->fwd")
                self.toggle_left_enc = True
                self.enc_left_is_fwd = True
            elif self.duty_left < 0 and self.enc_left_is_fwd:
                print("switching encoder polarity l->bkwd")
                self.toggle_left_enc = True
                self.enc_left_is_fwd = False

        # check for polarity change in right duty
        if not self.toggle_right_enc:
            if self.duty_right > 0 and not self.enc_right_is_fwd:
                print("switching encoder polarity r->fwd")
                self.toggle_right_enc = True
                self.enc_right_is_fwd = True
            elif self.duty_right < 0 and self.enc_right_is_fwd:
                print("switching encoder polarity r->bkwd")
                self.toggle_right_enc = True
                self.enc_right_is_fwd = False

        if self.toggle_left_enc:
            # overwrite any duties
            self.duty_left = 0
            # wait until vehicle is stationary
            if self.click_left + self.prev_error_left - self.target_clicks_left == 0:
                self.encoder.toggle_left_dir()
                self.toggle_left_enc = False

        if self.toggle_right_enc:
            # overwrite any duties
            self.duty_right = 0
            # wait until vehicle is stationary
            if self.click_right + self.prev_error_right - self.target_clicks_right == 0:
                self.encoder.toggle_right_dir()
                self.toggle_right_enc = False

    def print_csv_data(self):
        """prints out csv data with headings 'dt, lprp, lint, ldrv, ldty, lclk, rprp, rint, rdrv, rdty, rclk'"""
        print("{},{},{},{},{}".format(self.dt, self.duty_left, self.click_left, self.duty_right, self.click_right))

