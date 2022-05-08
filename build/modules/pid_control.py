from time import ticks_ms, ticks_diff, sleep_ms


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


class PIDController:
    def __init__(self, encoder, target_mm_left=0, target_mm_right=0, kp=2, ki=0.00015, kd=1):
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
        self.error_left, self.error_right = 0, 0
        self.prev_error_left, self.prev_error_right = 0, 0
        self.proportional_left, self.proportional_right = 0, 0
        self.integral_left, self.integral_right = 0, 0
        self.derivative_left, self.derivative_right = 0, 0
        self.duty_left, self.duty_right = 0, 0

        self.t0 = ticks_ms()
        self.dt = 0
        self.click_left, self.click_right = 0, 0

        # proportionality constants: P = proportional, I = integral, D = derivative
        self.KP = kp  # motor_duty is proportional to (click error * KP) plus...
        self.KI = ki  # motor_duty is proportional to (sum of click error * KI) plus...
        self.KD = kd  # motor_duty is proportional to (projected click error * KD)

        # clamp and bias constants
        self.min_duty_trim = 20  # anything between -20pwm to +20pwm doesn't move; so add this to all duties
        self.max_duty = 55  # effective max is min_duty_trim + max_duty = 75
        self.min_duty = -55  # effective min is -min_duty_trim + min_duty = -75
        self.max_integral = 20
        self.min_integral = -20
        self.max_bias = 12
        self.min_bias = -12

        # proportional on measurement (clicks input) option
        self.p_on_m = False
        # derivative on output (motor duty) option
        self.d_on_o = False

    def reset_target(self, target_mm_left, target_mm_right):
        """Reset PID control with a new target"""
        self.__init__(self.encoder, target_mm_left, target_mm_right)

    def reset(self, target_mm_left, target_mm_right, kp, ki, kd):
        """Reset PID control with a new target and constants"""
        self.__init__(self.encoder, target_mm_left, target_mm_right, kp, ki, kd)

    def add_target(self, target_mm_left, target_mm_right):
        self.target_clicks_left += mm_to_clicks(target_mm_left)
        self.target_clicks_right += mm_to_clicks(target_mm_right)

    def target_met(self):
        target_met = True
        if self.target_clicks_left >= 0:
            if self.click_left < self.target_clicks_left:
                target_met = False
        elif self.click_left > self.target_clicks_left:
            target_met = False

        if self.target_clicks_right >= 0:
            if self.click_right < self.target_clicks_right:
                target_met = False
        elif self.click_right > self.target_clicks_right:
            target_met = False

        return target_met

    def run(self):
        """Calculates the pwm values using PID control (a closed feedback loop)"""
        self.update_pid()
        self.update_encoder()
        self.print_csv_data()
        return self.duty_correction()

    def duty_correction(self):
        """First correction: Add a trim to the duties because there is a portion of duty between
        25 and -25 that effectively does nothing, creating noise in our PID system.
        Second correction: Reactively fix the bias in the motors.
        Note: a positive bias means we need to increase power to the right motor,
        and decrease power to the left motor"""
        # trim correction
        if self.duty_left > 0:
            self.duty_left += self.min_duty_trim
        else:
            self.duty_left -= self.min_duty_trim

        if self.duty_right > 0:
            self.duty_right += self.min_duty_trim
        else:
            self.duty_right -= self.min_duty_trim

        # bias correction IF attempting to travel straight
        bias = 0
        if abs(self.target_clicks_left - self.target_clicks_right) <= 5:
            scale_factor = 1.5
            bias = clamp((self.error_right - self.error_left) * scale_factor, self.max_bias, self.min_bias)

        return int(self.duty_left - bias), int(self.duty_right + bias)

    def proportional(self):
        """Calculates the proportional part of PID control"""
        if self.p_on_m:  # proportional on measurement -> aims to eliminate overshoot
            self.proportional_left = - self.KP * self.click_left
            self.proportional_right = - self.KP * self.click_right
        else:  # proportional on error (normal method)
            self.proportional_left = self.KP * self.error_left
            self.proportional_right = self.KP * self.error_right

    def integral(self):
        """Calculates the integral (error sum) part of PID control"""
        self.integral_left += self.KI * self.prev_error_left * self.dt
        self.integral_right += self.KI * self.prev_error_right * self.dt
        self.integral_left = clamp(self.integral_left, self.max_integral, self.min_integral)  # limit integral windup
        self.integral_right = clamp(self.integral_right, self.max_integral, self.min_integral)  # limit integral windup

    def derivative(self):
        """Calculates the derivative (projected error) part of PID control"""
        if self.dt != 0:  # save us from math errors
            if self.d_on_o:  # predictive calculation of derivative based on output (motor duty)
                # old non-predictive way results in spikey motion due to it interfering with itself
                # # # # self.derivative_left = self.KD * (self.duty_left - self.prev_duty_left) / self.dt
                # # # # self.derivative_right = self.KD * (self.duty_right - self.prev_duty_right) / self.dt
                self.derivative_left = self.KD * (self.proportional_left + self.derivative_left -
                                                  self.duty_left) / self.dt
                self.derivative_right = self.KD * (self.proportional_right + self.derivative_right -
                                                   self.duty_right) / self.dt
            else:  # calculate derivative on errors (normal method)
                self.derivative_left = self.KD * (self.error_left - self.prev_error_left) / self.dt
                self.derivative_right = self.KD * (self.error_right - self.prev_error_right) / self.dt

        else:
            self.derivative_left, self.derivative_right = 0, 0

    def update_elapsed_time(self):
        """Calculates the elapsed time, dt, since the last call. Also resets self.t0"""
        t1 = ticks_ms()
        self.dt = ticks_diff(t1, self.t0)
        self.t0 = t1

    def update_pid(self):
        """Calculates the error terms and PID values"""
        self.update_elapsed_time()

        # save old errors for integral section
        self.prev_error_left = self.error_left
        self.prev_error_right = self.error_right

        # calculate current error
        self.click_left, self.click_right = self.encoder.get_left(), self.encoder.get_right()
        self.error_left = self.target_clicks_left - self.click_left
        self.error_right = self.target_clicks_right - self.click_right

        # calculate new PID values
        self.proportional()
        self.integral()
        self.derivative()

        # calculate duties
        self.duty_left = self.proportional_left + self.integral_left - self.derivative_left
        self.duty_right = self.proportional_right + self.integral_right - self.derivative_right
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
        print("{},{},{},{},{},{},{},{},{},{},{}".format(self.dt,
                                                        self.proportional_left,
                                                        self.integral_left,
                                                        self.derivative_left,
                                                        self.duty_left,
                                                        self.click_left,
                                                        self.proportional_right,
                                                        self.integral_right,
                                                        self.derivative_right,
                                                        self.duty_right,
                                                        self.click_right))

