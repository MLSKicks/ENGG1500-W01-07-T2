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

class PIDController:
    def __init__(self, encoder, ticks=20, sample_time=1, target_modifier=0.5):
        """initialise all PID controller constants, variables, and encoder object"""
        self.left_speed, self.right_speed = (0, 0)
        self.left_error, self.right_error = (0, 0)
        self.prev_left_error, self.prev_right_error = (0, 0)
        self.left_error_sum, self.right_error_sum = (0, 0)
        self.encoder = encoder
        self.SAMPLE_TIME = sample_time
        self.TARGET = ticks * target_modifier
        self.KP = 1/ticks
        self.KD = 0.5 * self.KP
        self.KI = 0.5 * self.KD

    def run(self):
        self.left_error = self.TARGET - self.encoder.get_left()
        self.right_error = self.TARGET - self.encoder.get_right()
        left_speed, right_speed = self.proportional()
        self.encoder.clear_count()
        return int(left_speed), int(right_speed)

    def proportional(self):
        self.left_speed += (self.left_error * self.KP)
        self.right_speed += (self.right_error * self.KP)
        return self.left_speed, self.right_speed

    def derivative(self):
        self.left_speed += (self.prev_left_error * self.KD)
        self.right_speed += (self.prev_right_error * self.KD)
        return self.left_speed, self.right_speed

    def integral(self):
        self.left_speed += (self.left_error_sum * self.KI)
        self.right_speed += (self.right_error_sum * self.KI)
        return self.left_speed, self.right_speed

    def save_error(self):
        self.prev_left_error, self.prev_right_error = (self.left_error, self.right_error)
        self.left_error_sum += self.left_error
        self.right_error_sum += self.right_error
