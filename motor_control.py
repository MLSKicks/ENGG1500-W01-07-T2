from time import ticks_ms, ticks_diff
from math import atan, exp


def mm_to_clicks(mm):
    """Calculate how many clicks corresponds to a certain target mm
        Assumption: wheel diameter is 65mm"""
    wheel_circumference = 3.1416 * 65
    clicks_per_revolution = 40
    return int((clicks_per_revolution * mm) / wheel_circumference)


def clicks_to_mm(clicks):
    """Calculate how many mm corresponds to a certain amount of clicks
        Assumption: wheel diameter is 65mm"""
    wheel_circumference = 3.1416 * 65
    clicks_per_revolution = 40
    return (clicks * wheel_circumference) / clicks_per_revolution


def clamp(duty, max_duty, min_duty):
    """Clamp a given pwm duty to the range [min_duty, max_duty]"""
    if duty > max_duty:
        return max_duty
    elif duty < min_duty:
        return min_duty
    else:
        return duty


class MotorController:
    """MotorController class feeds us pwm duty values to reach a desired target
        Key Assumptions:
        1. Wheels are not slipping
        2. Wheel diameter is 65 mm -> 40 clicks = pi*65 mm
        3. The output function"""
    # a = 25, of = 1.2, base = 28, bias = 5
    def __init__(self, encoder, target_mm_left=0, target_mm_right=0, amplitude=45, offset=1.2, base_duty=30, bias=0):
        """Initialise all controller constants, variables, and encoder object."""
        # Initialise encoder
        self.encoder = encoder      # Save the encoder object
        self.encoder.clear_count()  # Make sure the encoder count is zeroed

        # Set encoder polarity: True for travelling forwards, False for travelling backwards
        self.encoder.set_left_dir(target_mm_left >= 0)
        self.encoder.set_right_dir(target_mm_right >= 0)

        # These flags indicate a request to toggle the encoder direction
        self.toggle_left_enc = False
        self.toggle_right_enc = False

        # Initialise our targets
        self.target_mm_left = target_mm_left
        self.target_mm_right = target_mm_right
        self.target_tolerance = 11  # Once we are within this tolerance, we are done

        # Working variables
        self.mm_left, self.mm_right = 0, 0  # Current distance travelled
        self.error_left, self.error_right = target_mm_left, target_mm_right  # How far we are from out target
        self.prev_error_left, self.prev_error_right = 0, 0
        self.duty_left, self.duty_right = 0, 0
        self.t0 = ticks_ms()
        self.dt = 0
        self.stuck_count_left, self.stuck_count_right = 0, 0  # Count how long we have been stuck in place for

        # Various 'constants'
        self.max_duty = 65  # This is the max duty we are allowed to produce
        self.min_duty = -65  # This is the min duty we are allowed to produce
        self.bias = bias  # Corrects the imbalance in motor outputs
        self.amplitude = amplitude   # Amplitude of the output function
        self.base_duty = base_duty   # Base duty value of the output function
        self.offset_amount = offset  # Offset for the output function
        self.STRAIGHT_LINE_TRAVEL = True

    def set_max_duty(self, max_duty):
        self.max_duty = abs(max_duty)
        self.min_duty = -abs(max_duty)

    def get_averaged_remainder_target(self):
        averaged_remainder = (abs(self.error_left) + abs(self.error_left))/2

        lpolarity, rpolarity = 1, 1
        if self.error_left < 0:
            lpolarity = -1
        if self.error_right < 0:
            rpolarity = -1

        return lpolarity*averaged_remainder, rpolarity*averaged_remainder

    def run(self):
        """Calculates the pwm values using a closed feedback loop"""
        self.update_duty()
        self.update_encoder()
        # self.print_csv_data()
        return self.duty_correction()

    def reset(self, target_mm_left, target_mm_right, amplitude, offset, base_duty, bias):
        """Reset the whole class with new constants"""
        self.__init__(self.encoder, target_mm_left, target_mm_right, amplitude, offset, base_duty, bias)

    def set_target(self, target_mm_left, target_mm_right):
        """Reset controller with a new target"""
        self.__init__(self.encoder, target_mm_left, target_mm_right)

    def add_target(self, target_mm_left, target_mm_right):
        """Reset with a new target, but holding onto the old error"""
        self.__init__(self.encoder, target_mm_left+self.error_left, target_mm_right+self.error_right)
        # self.target_mm_left += target_mm_left
        # self.target_mm_right += target_mm_right

    def target_met(self):
        """Checks if the target has been met within a certain tolerance
            Assumption: some code might assume that if this function returns true,
            that the vehicle is stationary. However, there is likely some small inertia left over!"""
        target_met = True
        if not (-self.target_tolerance <= self.error_left <= self.target_tolerance):
            target_met = False
        if not (-self.target_tolerance <= self.error_right <= self.target_tolerance):
            target_met = False
        return target_met

    def get_duties(self):
        """Create the output function w.r.t our current errors. This function creates a bell curve,
        with amplitude dependent on the size of the target, and an offset so that we go faster when we are
        further away from our target. version 2: calculates for left and right"""
        # - - - - - - - - - - - - - - - - - - Set Up - - - - - - - - - - - - - - - - - - #
        # Find out which direction the wheels need to turn
        l_polarity, r_polarity = 1, 1
        if self.error_left < 0:
            l_polarity = -1
        if self.error_right < 0:
            r_polarity = -1

        # Get the absolute value of our targets, now that we know what directions we are going
        l_target, r_target = abs(self.target_mm_left), abs(self.target_mm_right)

        # - - - - - - - - - - - - - - - - - Calculate the Duty Value - - - - - - - - - - - - - - - - - #
        # Calculate the amplitudes of the bell curve. A bigger target == a bigger amplitude, however we limit the
        # amplitude to around 50 by using the inverse tan function
        l_amplitude = self.amplitude * atan(l_target/300)
        r_amplitude = self.amplitude * atan(r_target/300)

        # Calculate the 'width' of the bell curve. A bigger target means a flatter bell curve that stays
        # at a higher speed for longer. We want this flattening to decrease with target, so we use an inverse function
        if l_target == 0:
            l_width = 0
        else:
            l_width = 1 / l_target**1.6

        if r_target == 0:
            r_width = 0
        else:
            r_width = 1 / r_target**1.6

        # Calculate the 'offset' of our bell curve. This allows us to concentrate higher speeds at the start
        # of our trip, such that we have sufficient time to slow down and stop at the target (countering overshooting)
        r_offset = r_target / self.offset_amount
        l_offset = l_target / self.offset_amount

        # Finally, calculate the actual duty!
        self.duty_left = l_polarity * (l_amplitude * exp(-l_width * ((l_polarity * self.error_left - l_offset) ** 2))
                                       + self.base_duty)
        self.duty_right = r_polarity * (r_amplitude * exp(-r_width * ((r_polarity * self.error_right - r_offset) ** 2))
                                        + self.base_duty)

    def get_unstuck(self):
        """In case we get stuck, slowly add more power"""
        l_polarity, r_polarity = 1, 1

        if abs(self.prev_error_left - self.error_left) <= 1:
            print("Trying to get left unstuck {}: ".format(self.stuck_count_left))
            self.stuck_count_left += 1
        else:
            self.stuck_count_left = max(self.stuck_count_left-0.5, 0)

        if abs(self.prev_error_right - self.error_right) <= 1:
            print("Trying to get right unstuck {}: ".format(self.stuck_count_right))
            self.stuck_count_right += 1
        else:
            self.stuck_count_right = max(self.stuck_count_right-0.5, 0)

        if self.error_left < 0:
            l_polarity = -1
        if self.error_right < 0:
            r_polarity = -1

        self.duty_left += l_polarity * self.stuck_count_left * 2
        self.duty_right += r_polarity * self.stuck_count_right * 2

    def duty_correction(self):
        """Fix bias to correct the motor imbalances. A positive bias means we are
        increasing power to the left motor, and decreasing power to the right motor.
        New addition: if the percentage completion of one side is greater than the other,
        try correct that as well!"""
        lduty = self.duty_left
        rduty = self.duty_right

        lpolarity = 1
        rpolarity = 1
        if lduty < 0:
            lpolarity = -1
        if rduty < 0:
            rpolarity = -1

        sideways_error = 0
        if self.STRAIGHT_LINE_TRAVEL:
            sideways_error = 5*(abs(self.error_left) - abs(self.error_right))
            sideways_error = clamp(sideways_error, 25, -25)  # clamp to |25|
            # if our error is smaller on the right compared to the left, we want to increase
            # the left motor
        print("sideways_error=", sideways_error)
        return int(lduty + lpolarity*self.bias + lpolarity*sideways_error), \
            int(rduty - rpolarity*self.bias - rpolarity*sideways_error)

    def update_elapsed_time(self):
        """Calculates the elapsed time, dt, since the last call. Also resets self.t0"""
        t1 = ticks_ms()
        self.dt = ticks_diff(t1, self.t0)
        self.t0 = t1

    def update_duty(self):
        """Calculates the error terms and duty values"""
        self.update_elapsed_time()

        # Save old errors so that we know if we are stuck in place
        self.prev_error_left = self.error_left
        self.prev_error_right = self.error_right

        # Calculate current error
        self.mm_left, self.mm_right = clicks_to_mm(self.encoder.get_left()), clicks_to_mm(self.encoder.get_right())
        self.error_left = self.target_mm_left - self.mm_left
        self.error_right = self.target_mm_right - self.mm_right

        # Calculate duties
        # If the target is met within the tolerance, we are done!
        if self.target_mm_left == 0 and self.target_mm_right == 0:
            self.duty_left, self.duty_right = 0, 0
        elif self.target_met():
            self.duty_left, self.duty_right = 0, 0
        else:
            self.get_duties()
            self.get_unstuck()

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

    # def output(self, target, error, prev_error, stuck_count):
    #     """Create the output function w.r.t our current error. This function is a bell curve,
    #     with amplitude dependent on the size of the target, and offset so that we go faster
    #     when we are further away."""
    #     # If the target is already met, we do nothing
    #     if -self.target_tolerance >= error >= self.target_tolerance:
    #         return 0
    #
    #     # - - - - - - - - - - Find the duty value - - - - - - - - - - #
    #     # Find out which direction we need to travel
    #     if error > 0:
    #         polarity = 1
    #     else:
    #         polarity = -1
    #
    #     # Ensure target is positive
    #     target = abs(target)
    #
    #     # Calculate the amplitude of our bell curve. We want a bigger amplitude when the target is bigger,
    #     #    but we want this to limit towards 50. Thus, we use the inverse tan function
    #     amplitude = self.amplitude * atan(target/150)
    #
    #     # Calculate the 'width' of our bell curve. A bigger smaller means a flatter bell curve that stays
    #     #   at a higher speed for longer. We want this to decrease with target, so we use an inverse function
    #     width = 1 / target ** 1.5
    #
    #     # Calculate the 'offset' of our bell curve. This allows us to concentrate higher speeds at the start
    #     #   of our trip, so that we have sufficient time to slow down and stop at our target
    #     offset = target / self.offset_amount
    #
    #     # Calculate the actual duty!
    #     duty = polarity * (amplitude * exp(-width * (polarity*error - offset) ** 2) + self.base_duty)
    #
    #     # - - - - - - - - - - In case we get stuck, slowly add more power - - - - - - - - - - #
    #     if abs(prev_error - error) <= 5:
    #         print("Trying to get unstuck {}: ".format(stuck_count), end="")
    #         duty += stuck_count / 10
    #         stuck_count += 1
    #     else:
    #         stuck_count = 0
    #
    #     # - - - - - - - - - - Print helpful stats - - - - - - - - - - #
    #     print("Target={}, err={}, amplitude={}, width={}, offset={}".format(target, error, amplitude, width, offset))
    #
    #     return stuck_count, duty
