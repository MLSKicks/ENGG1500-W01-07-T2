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
        """Initialise all controller constants, variables, and encoder object"""
        # Initialise encoder
        self.encoder = encoder      # Set the encoder object
        self.encoder.clear_count()  # Make sure the encoder is zero'd

        # Set encoder polarity: True for travelling forwards, False for travelling backwards
        self.encoder.set_left_dir(target_mm_left >= 0)
        self.encoder.set_right_dir(target_mm_right >= 0)
        self.toggle_left_enc, self.toggle_right_enc = False, False  # These flags indicate a request to toggle polarity

        # Initialise working variables for our control functions
        self.target_mm_left = target_mm_left
        self.target_mm_right = target_mm_right
        self.target_tolerance = 11  # Once we are within _ mm, we are done!
        self.mm_left, self.mm_right = 0, 0  # Current distance travelled
        self.error_left, self.error_right = target_mm_left, target_mm_right  # How far we are from out target
        self.prev_error_left, self.prev_error_right = 0, 0
        self.duty_left, self.duty_right = 0, 0
        self.t0 = ticks_ms()
        self.dt = 0
        self.max_duty = 75  # This is the max duty we are allowed to produce
        self.min_duty = -75  # This is the min duty we are allowed to produce
        self.bias = bias  # Correcting motor imbalances
        self.stuck_count_left, self.stuck_count_right = 0, 0

        # Constants for output calculations
        self.amplitude = amplitude
        self.base_duty = base_duty  # This is the baseline duty value
        self.offset_amount = offset

    def reset(self, target_mm_left, target_mm_right, amplitude, offset, base_duty, bias):
        """Reset the whole class with new values"""
        self.__init__(self.encoder, target_mm_left, target_mm_right, amplitude, offset, base_duty, bias)

    def set_target(self, target_mm_left, target_mm_right):
        """Reset controller with a new target"""
        self.__init__(self.encoder, target_mm_left, target_mm_right)

    def add_target(self, target_mm_left, target_mm_right):
        """Artificially add to our targets... TODO: This will not work well without a 'boost'"""
        self.target_mm_left += target_mm_left
        self.target_mm_right += target_mm_right

    def run(self):
        """Calculates the pwm values using a closed feedback loop"""
        self.update_duty()
        self.update_encoder()
        self.print_csv_data()
        return self.duty_correction()

    def target_met(self):
        """Checks if the target has been met"""
        target_met = True
        if not (-self.target_tolerance <= self.error_left <= self.target_tolerance):
            target_met = False
        if not (-self.target_tolerance <= self.error_right <= self.target_tolerance):
            target_met = False
        return target_met

    def output(self, target, error, prev_error, stuck_count):
        """Create the output function w.r.t error. This function is a bell curve,
        with amplitude dependent on the size of the target."""
        # If the target is already met, we do nothing
        if -self.target_tolerance >= error >= self.target_tolerance:
            return 0

        # Find out which direction we are going
        if error > 0:
            polarity = 1
        else:
            polarity = -1

        # Do the actual calculation, this is a 'proportional term'
        target = abs(target)  # This is required for how the maths is working atm
        amplitude = self.amplitude * atan(target/150)
        width = -1 / target ** 1.5
        offset = target / self.offset_amount

        duty = polarity * (amplitude * exp(width * (polarity*error - offset) ** 2) + self.base_duty)

        # In case we get stuck at one spot, add an 'integral term'
        if abs(prev_error - error) <= 5:
            print("Trying to get unstuck {}: ".format(stuck_count), end="")
            duty += stuck_count / 10
            stuck_count += 1
        else:
            stuck_count = 0

        print("Target={}, err={}, amplitude={}, width={}, offset={}".format(target, error, amplitude, width, offset))

        return stuck_count, duty

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

        # Save old errors for integral section
        self.prev_error_left = self.error_left
        self.prev_error_right = self.error_right

        # Calculate current error
        self.mm_left, self.mm_right = clicks_to_mm(self.encoder.get_left()), clicks_to_mm(self.encoder.get_right())
        self.error_left = self.target_mm_left - self.mm_left
        self.error_right = self.target_mm_right - self.mm_right

        # Calculate duties
        self.stuck_count_left, self.duty_left = \
            self.output(self.target_mm_left, self.error_left, self.prev_error_left, self.stuck_count_left)
        self.stuck_count_right, self.duty_right = \
            self.output(self.target_mm_right, self.error_right, self.prev_error_right, self.stuck_count_right)

        # Clamp duties
        self.duty_left = clamp(self.duty_left, self.max_duty, self.min_duty)
        self.duty_right = clamp(self.duty_right, self.max_duty, self.min_duty)

    def update_encoder(self):
        """if pwm polarity changes, we must change the encoder count direction,
        however, we need to stop the vehicle first since the encoder will count backwards
        while it is still travelling forwards due to inertia. We will therefore overwrite the
        duties to zero while we are awaiting a change in the encoder direction"""
        # Check for polarity change in left duty
        if not self.toggle_left_enc:
            if self.duty_left > 0 and not self.encoder.left_dir_is_fwd():
                print("Switching encoder polarity l->fwd")
                self.toggle_left_enc = True  # Queue a polarity change to forwards
            elif self.duty_left < 0 and self.encoder.left_dir_is_fwd():
                print("Switching encoder polarity l->bkwd")
                self.toggle_left_enc = True  # Queue a polarity change to backwards

        # Check for polarity change in right duty
        if not self.toggle_right_enc:
            if self.duty_right > 0 and not self.encoder.right_dir_is_fwd():
                print("Switching encoder polarity r->fwd")
                self.toggle_right_enc = True  # Queue a polarity change to forwards
            elif self.duty_right < 0 and self.encoder.right_dir_is_fwd():
                print("Switching encoder polarity r->bkwd")
                self.toggle_right_enc = True  # Queue a polarity change to backwards

        # If a polarity change is requested, do it once the vehicle is stationary
        if self.toggle_left_enc:
            # overwrite any duties
            self.duty_left = 0
            # wait until vehicle is stationary
            if self.mm_left + self.prev_error_left - self.target_mm_left == 0:
                self.encoder.toggle_left_dir()
                self.toggle_left_enc = False

        # If a polarity change is requested, do it once the vehicle is stationary
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
