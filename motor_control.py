from time import ticks_ms, ticks_diff
from math import atan, exp


def mm_to_clicks(mm):
    # calculate target clicks from target mm assuming wheel diameter is 65mm
    wheel_circumference = 3.1416 * 65
    clicks_per_revolution = 40
    return int((clicks_per_revolution * mm) / wheel_circumference)


def clicks_to_mm(clicks):
    wheel_circumference = 3.1416 * 65
    clicks_per_revolution = 40
    return (clicks * wheel_circumference) / clicks_per_revolution


def clamp(duty, max_duty, min_duty):
    if duty > max_duty:
        return max_duty
    elif duty < min_duty:
        return min_duty
    else:
        return duty


class MotorController:
    def __init__(self, encoder, target_mm_left=0, target_mm_right=0, amplitude=32.5, offset=1.2, base_duty=30, bias=5):
        """initialise all PID controller constants, variables, and encoder object"""
        # initialise target and encoder object
        self.target_mm_left = target_mm_left
        self.target_mm_right = target_mm_right
        self.encoder = encoder

        # encoder polarity is true for a forwards/zero target, and false for a backwards target
        encoder_polarity_left = self.target_mm_left >= 0
        encoder_polarity_right = self.target_mm_right >= 0
        self.encoder.clear_count()
        self.encoder.set_left_dir(encoder_polarity_left)
        self.encoder.set_right_dir(encoder_polarity_right)
        self.enc_left_is_fwd, self.enc_right_is_fwd = encoder_polarity_left, encoder_polarity_right
        self.toggle_left_enc = False
        self.toggle_right_enc = False

        self.t0 = ticks_ms()
        self.dt = 0

        # initialise variables for PID control
        self.mm_left, self.mm_right = 0, 0
        self.error_left, self.error_right = target_mm_left, target_mm_right
        self.prev_error_left, self.prev_error_right = 0, 0
        self.duty_left, self.duty_right = 0, 0

        # constants for output calculations
        self.amplitude = amplitude
        self.base_duty = base_duty  # This is the baseline duty value
        self.offset_amount = offset

        # clamp and bias constants
        self.max_duty = 75
        self.min_duty = -75
        self.bias = bias

    def reset(self, target_mm_left, target_mm_right, amplitude, offset, base_duty, bias):
        self.__init__(self.encoder, target_mm_left, target_mm_right, amplitude, offset, base_duty, bias)

    def set_target(self, target_mm_left, target_mm_right):
        """Reset controller with a new target"""
        self.__init__(self.encoder, target_mm_left, target_mm_right)

    def add_target(self, target_mm_left, target_mm_right):
        self.target_mm_left += target_mm_left
        self.target_mm_right += target_mm_right

    def target_met(self):
        target_met = True

        if not (-20 <= self.error_left <= 20):
            target_met = False

        if not (-20 <= self.error_right <= 20):
            target_met = False

        return target_met

    def output(self, target, error):
        """Create the output function w.r.t error. This function is a bell curve,
        with amplitude dependent on the size of the target."""
        print("Target = {}, error = {}, ".format(target, error), end="")
        if target == 0:
            return 0

        if -20 >= error >= 20:
            return 0

        if error > 0:  # find out which direction to go
            polarity = 1
        else:
            polarity = -1

        if target < 0:  # Ensure target is positive, its just how the maths works
            target = -target

        # Do the actual calculation
        amplitude = self.amplitude * atan(target/150)
        width = -1 / target ** 1.5
        offset = target / self.offset_amount
        print("amplitude = {}, width = {}, offset = {}, offset_amount = {}".format(amplitude, width, offset, self.offset_amount))
        return polarity * (amplitude * exp(width * (polarity*error - offset) ** 2) + self.base_duty)

    def run(self):
        """Calculates the pwm values using a closed feedback loop"""
        self.update_duty()
        self.update_encoder()
        self.print_csv_data()
        return self.duty_correction()

    def duty_correction(self):
        """Fix bias to correct the motor imbalances. Note: a positive bias means we are
        increasing power to the left motor, and decreasing power to the right motor"""
        return int(self.duty_left + self.bias), int(self.duty_right - self.bias)

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
        self.mm_left, self.mm_right = clicks_to_mm(self.encoder.get_left()), clicks_to_mm(self.encoder.get_right())
        self.error_left = self.target_mm_left - self.mm_left
        self.error_right = self.target_mm_right - self.mm_right

        # calculate duties
        self.duty_left = self.output(self.target_mm_left, self.error_left)
        self.duty_right = self.output(self.target_mm_right, self.error_right)

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
            if self.mm_left + self.prev_error_left - self.target_mm_left == 0:
                self.encoder.toggle_left_dir()
                self.toggle_left_enc = False

        if self.toggle_right_enc:
            # overwrite any duties
            self.duty_right = 0
            # wait until vehicle is stationary
            if self.mm_right + self.prev_error_right - self.target_mm_right == 0:
                self.encoder.toggle_right_dir()
                self.toggle_right_enc = False

    def print_csv_data(self):
        """prints out csv data with headings dt, left_duty, left_clicks, right_duty, right_clicks"""
        print("{},{},{},{},{}".format(self.dt, self.duty_left, self.mm_left, self.duty_right, self.mm_right))
