# PID CONTROL
# Modify KP constant to improve performance.
# Move onto KD.
# Move onto KI.
#
# If Motor Adjusts aggresive reduce constant.
#
# If Motor Speed changes too fast increase the constant.
#
# Make changes in small increments.
#
# PS THIS SUCKS ASS AND DONT JUDGE I WILL REFACTOR AND ADJUST LATER FOR A MILESTONE -Luv Mark

# from time import sleep
# from motor import Motor
# from encoder import Encoder
# # Create left and right ‘Motor’ objects
# motor_left = Motor("left", 8, 9, 6)
# motor_right = Motor("right", 10, 11, 7)
# # Create encoder object
# ENC_L = 18
# ENC_R = 19
# enc = Encoder(ENC_L, ENC_R)
# motor_left.set_forwards()
# motor_right.set_forwards()

# SAMPLE_TIME = 1
# TICK = 20   # Encoder Ticks between each cycle
# TARGET = (TICK * 0.75)  # Target ENC Ticks Tick * 0.75
# KP = (1 / TICK)  # Proportional Control: 1 / Tick
# KD = (0.5 * KP)  # Derivative Control: Half of KP
# KI = (0.5 * KD)  # Integral Control: Half of KD
# left_speed = 0
# right_speed = 0
# left_error = 0
# right_error = 0
# left_prev_error = 0  # Derivative
# right_prev_error = 0  # Derivative
# left_sum_error = 0  # Integral
# right_sum_error = 0  # Integral

# while True:
#     left_error = TARGET - enc.get_left()
#     right_error = TARGET - enc.get_right()

#     left_speed += (left_error * KP) + (left_prev_error * KD) + (left_sum_error * KI)
#     right_speed += (right_error * KP) + (right_prev_error * KD) + (right_sum_error * KI)

#     left_speed += max(min(1, left_speed), 0)
#     right_speed += max(min(1, right_speed), 0)

#     motor_left.duty(int(left_speed))
#     motor_right.duty(int(right_speed))

#     print("enc_l {} enc_r {}".format(enc.get_left(), enc.get_right()))
#     print("l_pwm {} r_pwm {}".format(left_speed, right_speed))

#     enc.clear_count()
#     sleep(SAMPLE_TIME)

#     left_prev_error = left_error
#     right_prev_error = right_error

#     left_sum_error += left_error
#     right_sum_error += right_error

## PID Class
class PIDController():
    left_speed, right_speed = (0,0)
    left_error, right_error = (0,0)
    prev_left_error, prev_right_error = (0,0)
    left_error_sum, right_error_sum = (0,0)

    def __init__(self, SAMPLE_TIME, TARGET, KP, KD, KI):
        self.SAMPLE_TIME = SAMPLE_TIME
        self.TARGET = TARGET
        self.KP = KP
        self.KD = KD
        self.KI = KI

    @classmethod
    def proportional(cls):
        cls.left_speed += (cls.left_error * self.KP)
        cls.right_speed += (cls.right_error * self.KP)
        return cls.left_speed, cls.right_speed

    @classmethod
    def derivative(cls):
        cls.left_speed += (cls.prev_left_error * self.KD)
        cls.right_speed += (cls.prev_right_error * self.KD)
        return cls.left_speed, cls.right_speed

    @classmethod
    def integral(cls):
        cls.left_speed += (cls.left_error_sum * self.KI)
        cls.right_speed += (cls.right_error_sum * self.KI)
        return cls.left_speed, cls.right_speed

    @classmethod
    def save_error(cls):
        cls.prev_left_error, cls.prev_right_error = (cls.left_error, cls.right_error)
        cls.left_error_sum, cls.right_error_sum += (cls.left_error, cls.right_error)

