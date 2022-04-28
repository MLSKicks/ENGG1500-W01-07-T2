# PID CONTROL
# Modify KP constant to improve performance.
# Move onto KD.
# Move onto KI.
#
# If Motor Adjusts aggressive reduce constant.
#
# If Motor Speed changes too fast increase the constant.
#
# Make changes in small increments.
#
# PS THIS SUCKS ASS AND DONT JUDGE I WILL REFACTOR AND ADJUST LATER FOR A MILESTONE -Luv Mark
# I luv it - Nick

from time import ticks_ms, ticks_diff, sleep_ms

class PIDController:
    def __init__(self, encoder):
        """initialise all PID controller constants, variables, and encoder object"""
        # initialise other member variables
        self.target_clicks = 0
        self.encoder = encoder
        self.encoder.clear_count()
        if self.target_clicks > 0:
            self.encoder.set_left_fwd()
            self.encoder.set_right_fwd()
            self.enc_left_is_fwd, self.enc_right_is_fwd = True, True
        else:
            self.encoder.set_left_bkwd()
            self.encoder.set_right_bkwd()
            self.enc_left_is_fwd, self.enc_right_is_fwd = False, False

        self.error_left, self.error_right = 0, 0
        self.error_sum_left, self.error_sum_right = 0, 0
        self.error_proj_left, self.error_proj_right = 0, 0
        self.t0 = ticks_ms()

        # proportionality constants for PID control
        self.KP = 2  # motor_duty is proportional to (click error * KP) plus...
        self.KI = 0.0001  # motor_duty is proportional to (sum of click error * KI) plus...
        self.KD = 0  # motor_duty is proportional to (projected click error * KD)

    def set_target(self, target_mm):
        # calculate target clicks from target mm assuming wheel diameter is 65mm
        wheel_circumference = 3.1416 * 65
        clicks_per_revolution = 20
        self.target_clicks = int((clicks_per_revolution * target_mm) / wheel_circumference)

    def error(self):
        """calculates the error terms"""
        # calculate elapsed time since last call, useful for calculating the derivative/integral term
        t1 = ticks_ms()
        elapsed_time = ticks_diff(t1, self.t0)
        print(elapsed_time)
        self.t0 = t1

        # save old errors for integral and derivative sections
        prev_error_left = self.error_left
        prev_error_right = self.error_right

        # calculate current error
        self.error_left = self.target_clicks - self.encoder.get_left()
        self.error_right = self.target_clicks - self.encoder.get_right()

        # calculate error integral (sum)
        self.error_sum_left += elapsed_time * prev_error_left
        self.error_sum_right += elapsed_time * prev_error_right

        # calculate error derivative (projected error)
        if elapsed_time == 0:  # save us from math errors
            self.error_proj_left, self.error_proj_right = 0, 0
        else:
            self.error_proj_left = (self.error_left - prev_error_left) / elapsed_time
            self.error_proj_right = (self.error_right - prev_error_right) / elapsed_time

    def update_encoder(self, duty_left, duty_right):
        """if pwm polarity changes, we must change the encoder count direction"""
        toggle_left = False
        toggle_right = False

        # check for polarity change in left duty
        if duty_left > 0 and not self.enc_left_is_fwd:
            print("switching encoder polarity l->fwd")
            toggle_left = True
            self.enc_left_is_fwd = True
        elif duty_left < 0 and self.enc_left_is_fwd:
            print("switching encoder polarity l->bkwd")
            toggle_left = True
            self.enc_left_is_fwd = False

        # check for polarity change in right duty
        if duty_right > 0 and not self.enc_right_is_fwd:
            print("switching encoder polarity r->fwd")
            toggle_right = True
            self.enc_right_is_fwd = True
        elif duty_right < 0 and self.enc_right_is_fwd:
            print("switching encoder polarity r->bkwd")
            toggle_right = True
            self.enc_right_is_fwd = False

        # sleep to (hopefully) stop motors if duties changed
        if toggle_left or toggle_right:
            sleep_ms(150)

        # now actually toggle the direction
        if toggle_left:
            self.encoder.toggle_left_dir()
        if toggle_right:
            self.encoder.toggle_right_dir()

    def run(self):
        """calculates the pwm values using PID control (a closed feedback loop)"""
        # calculate error term
        self.error()

        # calculate control function
        duty_left = int(self.KP * self.error_left
                        + self.KI * self.error_sum_left
                        + self.KD * self.error_proj_left)
        duty_right = int(self.KP * self.error_right
                         + self.KI * self.error_sum_right
                         + self.KD * self.error_proj_right)

        # return motor duties!
        return duty_left, duty_right

        # update encoder direction based on duty polarity
        # self.update_encoder(duty_left, duty_right) RUN IN MAIN

    def reset(self, kp, ki, kd):
        """helper for quickly testing"""
        self.encoder.clear_count()
        self.t0 = ticks_ms()
        self.error_left, self.error_right = 0, 0
        self.error_sum_left, self.error_sum_right = 0, 0
        self.error_proj_left, self.error_proj_right = 0, 0
        self.KP, self.KI, self.KD = kp, ki, kd

    def reset_if_target_met(self):
        """reset target and encoder count once target is met"""
        if self.error_left == 0 and self.error_right == 0:
            self.encoder.clear_count()
            self.target_clicks = 0
            return True
        return False

