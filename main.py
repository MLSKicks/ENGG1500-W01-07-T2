from vehicle_components import Vehicle
from time import sleep_ms, ticks_ms, ticks_diff

# TODO: TRACK DISTORTED BY 1.1x ????

# - - - - - - - - - - - - - - - - - - - - - - - RANDOM STUFF - - - - - - - - - - - - - - - - - - - - - - - - - -#
ascii_cat = ("State: PRINT_ART\n\n    _,,/|\n"
             "    \\o o'\n"
             "    =_~_=\n"
             "    /   \\ (\\\n"
             "   (////_)//\n"
             "   ~~~")

# max pwm values
ROTATE_PWM = 55
STRAIGHT_PWM = 55
SLOW_STRAIGHT_PWM = 45
SLOW_ROTATE_PWM = 40

# rotations
QUARTER_TURN = 110
HALF_TURN = 220

# avoidance methods
NO_AVOIDANCE = 0
BYPASS_AVOIDANCE = 1
ADJUST_AVOIDANCE = 2

# - - - - - - - - - - - - - - - - - - - - - - - STATE ENUMERATION - - - - - - - - - - - - - - - - - - - - - - - #
NULL = -1
SPLASH_SCREEN = 0
STOP = 1
PRINT_ROAD_INFO = 2

DEPLOY_SENSOR = 3

HAZARD_FORWARDS = 200
HAZARD_BACKWARDS = 201
HAZARD_FORWARDS_BYPASS = 202
HAZARD_BACKWARDS_BYPASS = 203

FORWARDS = 100
BACKWARDS = 101
ROTATE_LEFT = 102
ROTATE_RIGHT = 103
PARKING = 104
UNPARKING = 105
CUSTOM = 106


# ! ! ! ! ! ! ! ! ! EDIT HERE ! ! ! ! ! ! ! ! ! ! #

v_sf = 1  # vertical scale factor
h_sf = 1  # horizontal scale factor

# 2 roundabouts track
extra_track_states = [
    # reverse out
    (UNPARKING, -560*h_sf, -560*h_sf, STRAIGHT_PWM, NO_AVOIDANCE),  # backwards
    (ROTATE_RIGHT, QUARTER_TURN, -QUARTER_TURN, ROTATE_PWM, NO_AVOIDANCE),   # rotate right

    # get to first roundabout
    (FORWARDS, 600*v_sf, 600*v_sf, STRAIGHT_PWM, BYPASS_AVOIDANCE),   # forwards
    DEPLOY_SENSOR,

    # get to second roundabout
    (FORWARDS, 1650*v_sf, 1650*v_sf, STRAIGHT_PWM, BYPASS_AVOIDANCE),  # forwards
    DEPLOY_SENSOR,

    # return home
    (ROTATE_RIGHT, HALF_TURN, -HALF_TURN, ROTATE_PWM, NO_AVOIDANCE),   # rotate right
    (FORWARDS, 2300*v_sf, 2300*v_sf, STRAIGHT_PWM, BYPASS_AVOIDANCE),  # forwards

    # reverse in
    (ROTATE_RIGHT, QUARTER_TURN, -QUARTER_TURN, ROTATE_PWM, NO_AVOIDANCE),   # rotate right
    (PARKING, 560*h_sf, 560*h_sf, STRAIGHT_PWM, NO_AVOIDANCE),  # forwards
    STOP
]

# 4 roundabouts track
track_states = [
    # reverse out
    (UNPARKING, -560*h_sf, -560*h_sf, STRAIGHT_PWM, NO_AVOIDANCE),  # backwards
    (ROTATE_RIGHT, QUARTER_TURN, -QUARTER_TURN, ROTATE_PWM, NO_AVOIDANCE),   # rotate 90 degrees right

    # get to first roundabout
    (FORWARDS, 600*v_sf, 600*v_sf, STRAIGHT_PWM, BYPASS_AVOIDANCE),   # forwards
    DEPLOY_SENSOR,

    # get to second roundabout
    (FORWARDS, 1650*v_sf, 1650*v_sf, STRAIGHT_PWM, BYPASS_AVOIDANCE),  # forwards
    DEPLOY_SENSOR,

    # get to third roundabout
    (ROTATE_RIGHT, QUARTER_TURN, -QUARTER_TURN, ROTATE_PWM, NO_AVOIDANCE),  # rotate 90 degrees right
    (FORWARDS, 1200*h_sf, 1200*h_sf, STRAIGHT_PWM, ADJUST_AVOIDANCE),  # forwards
    DEPLOY_SENSOR,

    # get to fourth roundabout
    (BACKWARDS, -1750*h_sf, -1750*h_sf, STRAIGHT_PWM, BYPASS_AVOIDANCE),  # backwards
    (ROTATE_RIGHT, QUARTER_TURN, -QUARTER_TURN, ROTATE_PWM, NO_AVOIDANCE),  # rotate 90 degrees right
    (FORWARDS, 480*v_sf, 480*v_sf, SLOW_STRAIGHT_PWM, BYPASS_AVOIDANCE),  # forwards
    DEPLOY_SENSOR,

    # return home
    (ROTATE_LEFT, -QUARTER_TURN, QUARTER_TURN, SLOW_ROTATE_PWM, NO_AVOIDANCE),  # rotate 90 degrees left
    (FORWARDS, 550*h_sf, 550*h_sf, SLOW_STRAIGHT_PWM, BYPASS_AVOIDANCE),  # forwards
    (ROTATE_RIGHT, QUARTER_TURN, -QUARTER_TURN, SLOW_ROTATE_PWM, NO_AVOIDANCE),  # rotate 90 degrees right
    (FORWARDS, 2000*v_sf, 2000*v_sf, STRAIGHT_PWM, BYPASS_AVOIDANCE),  # forwards

    # reverse in
    (ROTATE_RIGHT, QUARTER_TURN, -QUARTER_TURN, ROTATE_PWM, NO_AVOIDANCE),   # rotate right
    (PARKING, 560*h_sf, 560*h_sf, STRAIGHT_PWM, NO_AVOIDANCE),  # forwards
    STOP
]


# - - - - - - - - - - - - - - - - - - - - - STATE MACHINE - - - - - - - - - - - - - - - - - - - - - - - #
class StateMachine:
    def __init__(self, initial_state):
        self.hazard_callback_state = NULL  # State to re-enter after avoiding a hazard
        self.prev_state = NULL  # Previous state
        self.state = initial_state  # Current state
        self.state_phase = 0
        self.avoidance_method = NO_AVOIDANCE
        self.is_transition = True  # Transition flag telling us if we just switched states
        self.update_is_transition = False  # Flag tells us to update is_transition to false
        self.t0 = ticks_ms()  # For calculating time spent in a state

        # Special state variables for the path state
        self.track_counter = 0
        self.custom_target_left, self.custom_target_right = 0, 0
        self.max_speed = 65
        self.HAZARD_TIMEOUT = 4000  # ms
        self.DEPLOY_SENSOR_TIMEOUT = 600  # ms
        self.SPLASH_SCREEN_TIMEOUT = 600  # ms
        self.STATE_CHANGE_DELAY = 1000  # ms
        self.HAZARD_RUNBACK_MM = 20  # mm
        self.HAZARD_BYPASS_MM = 250  # mm to travel around an obstacle
        self.HAZARD_CLEARANCE_MM = 300  # mm to travel forwards

        # Other initialisation
        self.vehicle = Vehicle(motor=True, enc=True, screen=True, ir_f=True, ir_b=False)
        self.controller = self.vehicle.controller  # motor control object can set duties to achieve desired targets
        if self.vehicle.has_screen():
            self.screen = self.vehicle.screen  # Get OLED screen object -> can print useful information

    def update_state(self, new_state, is_transition=True):
        """Update our current state"""
        self.prev_state = self.state
        self.state = new_state
        self.state_phase = 0
        self.is_transition = is_transition

    def update_state_variables(self):
        """Updates our transition flag (is_transition). This is dirty set it and forget it code...
        It automatically sets the is_transition flag so that it is only True for the first loop
        of a new state"""
        if self.prev_state == self.state:
            if self.is_transition:
                if self.update_is_transition:
                    self.update_is_transition = False
                    self.is_transition = False
                else:
                    self.update_is_transition = True
        else:
            self.update_is_transition = True
        self.prev_state = self.state

        if self.is_transition:
            self.t0 = ticks_ms()

    def print_state(self):
        """Print out what state we are in"""
        # Run this ONCE when we have just entered a new state -> avoids flickering
        if self.is_transition and self.vehicle.has_screen():
            if self.state == NULL:
                self.screen.print("State: NULL\n\nNo initial state\nwas specified!")
            elif self.state == SPLASH_SCREEN:
                self.screen.print_art(ascii_cat)
            elif self.state == DEPLOY_SENSOR:
                self.screen.print("State: Deploying\nCordel Sensors")
            elif self.state == PRINT_ROAD_INFO:
                self.screen.print("State: Road Info")
            elif self.state == STOP:
                self.screen.print("State: Stopped\n\nMy job is done!")
            elif self.state == HAZARD_FORWARDS:
                self.screen.print("State: Hazard\n\nAn object is in\nfront of me!\nWaiting:")
            elif self.state == HAZARD_BACKWARDS:
                self.screen.print("State: Hazard\n\nAn object is be-\nhind me!\nWaiting:")
            elif self.state == HAZARD_FORWARDS_BYPASS:
                self.screen.print("State: Bypassing\n\nTrying to dodge\nan object in fr-\nont of me!")
            elif self.state == HAZARD_BACKWARDS_BYPASS:
                self.screen.print("State: Bypassing\n\nTrying to dodge\nan object be-\nhind me!")
            elif self.state == FORWARDS:
                self.screen.print("State: Travell-\ning Forwards")
            elif self.state == BACKWARDS:
                self.screen.print("State: Travell-\ning Backwards")
            elif self.state == ROTATE_LEFT:
                self.screen.print("State: Rotate\nLeft")
            elif self.state == ROTATE_RIGHT:
                self.screen.print("State: Rotate\nRight")
            elif self.state == CUSTOM:
                self.screen.print("State: Custom")
            elif self.state == PARKING:
                self.screen.print("State: Parking")
            elif self.state == UNPARKING:
                self.screen.print("State: Unparking")
            else:
                self.screen.print("State: Not Found")

    def elapsed_ms(self):
        """Calculates the elapsed ms in the current state, relies on update_state_variables()"""
        return ticks_diff(ticks_ms(), self.t0)

    def track_next_state(self):
        """Gets the next state in the queue"""
        next_state = track_states[self.track_counter]

        if type(next_state) is tuple:
            self.custom_target_left = next_state[1]
            self.custom_target_right = next_state[2]
            self.max_speed = next_state[3]
            self.avoidance_method = next_state[4]
            next_state = next_state[0]

        self.track_counter += 1
        sleep_ms(self.STATE_CHANGE_DELAY)
        return next_state

    def emergency_brake(self):
        if self.state == HAZARD_FORWARDS:
            self.vehicle.set_motor(-75, -75)
            sleep_ms(100)
        if self.state == HAZARD_BACKWARDS:
            self.vehicle.set_motor(75, 75)
            sleep_ms(100)
        self.vehicle.set_motor(0, 0)

    def veer_left(self):
        self.vehicle.set_motor(-45, 45)
        sleep_ms(50)
        self.vehicle.set_motor(0, 0)

    # - - - - - - - - - - - - - - - - - - - - - - MAIN STATE MACHINE - - - - - - - - - - - - - - - - - - - - - - - #
    def main(self):
        """This is our main state machine: a big loop that performs actions based on the current state!

        Initialisation: initialise our Vehicle object which initialises objects for each sensor, controller, etc.

        Sensor Data Collection: collect data from all our sensors

        Global Transitions: these are IMPORTANT TRANSITIONS which can overwrite anything. They are important as they
        are reacting to things like obstacles on the road, in which case we want to stop ASAP!

        State Machine Body: goes through all the nitty-gritty details """
        # - - - - - - - - - - - - - - - - - - - - - - - INITIALISATION - - - - - - - - - - - - - - - - - - - - - - - #
        self.t0 = ticks_ms()

        while True:
            # - - - - - - - - - - - - - - - - - - - - SENSOR DATA COLLECTION - - - - - - - - - - - - - - - - - - #
            hazard_forwards = self.vehicle.ir_f.is_hazard()
            hazard_backwards = False

            # - - - - - - - - - - - - - - - - - - - - GLOBAL TRANSITIONS - - - - - - - - - - - - - - - - - - - - #
            if hazard_forwards:  # Something is in front so lets stop ...
                # only if we are currently travelling forwards
                if self.state == FORWARDS:
                    self.hazard_callback_state = self.state
                    self.update_state(HAZARD_FORWARDS)
                    self.custom_target_left, self.custom_target_right = self.controller.get_averaged_remainder_target()
                elif self.state == PARKING:
                    self.update_state(STOP)

            if hazard_backwards:  # Something is behind us so lets stop ...
                # only if we are currently travelling backwards
                if self.state == BACKWARDS or self.state == UNPARKING:
                    self.hazard_callback_state = self.state
                    self.update_state(HAZARD_BACKWARDS)
                    self.custom_target_left, self.custom_target_right = self.controller.get_averaged_remainder_target()

            # - - - - - - - - - - - - - - - - - - - - STATE MACHINE HEADER - - - - - - - - - - - - - - - - - - - #
            self.update_state_variables()  # Updates prev_state and is_transition flag
            self.print_state()             # Prints current state information if we just transitioned

            # - - - - - - - - - - - - - - - - - - - - STATE MACHINE BODY - - - - - - - - - - - - - - - - - - - - #
            # - SPLASH_SCREEN -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Prints a cat to the screen for a second
            if self.state == SPLASH_SCREEN:
                if self.is_transition:
                    self.controller.set_target(0, 0)

                if self.elapsed_ms() > self.SPLASH_SCREEN_TIMEOUT:
                    self.update_state(self.track_next_state())

            # - DEPLOY_SENSOR -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == DEPLOY_SENSOR:
                if self.is_transition:
                    self.controller.set_target(0, 0)
                # wait until sensors are deployed
                if self.elapsed_ms() > self.DEPLOY_SENSOR_TIMEOUT:
                    self.update_state(self.track_next_state())
                elif self.vehicle.has_screen():
                    if self.elapsed_ms() > self.DEPLOY_SENSOR_TIMEOUT*(3/4):
                        self.screen.print_variable(". . .", 5, 4)
                    elif self.elapsed_ms() > self.DEPLOY_SENSOR_TIMEOUT*(2/4):
                        self.screen.print_variable(". .", 5, 4)
                    elif self.elapsed_ms() > self.DEPLOY_SENSOR_TIMEOUT*(1/4):
                        self.screen.print_variable(".", 5, 4)

            # - PRINT_ROAD_INFO -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Displays what our IR sensors are saying
            elif self.state == PRINT_ROAD_INFO:
                if self.is_transition:
                    self.controller.set_target(0, 0)

                if self.vehicle.has_screen():
                    self.screen.print_variable("IR_F Hazard{!s:>5}".format(hazard_forwards), 0, 2)

                if self.elapsed_ms() > 1000:
                    pass

            # - STOP -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Stop once we have finished our task
            elif self.state == STOP:
                if self.is_transition:
                    self.controller.set_target(0, 0)

            # - HAZARDS -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Stop if we encounter a hazard in front of us on the road
            elif self.state == HAZARD_FORWARDS:
                if self.is_transition:
                    if self.avoidance_method == ADJUST_AVOIDANCE:
                        self.veer_left()
                    else:
                        self.emergency_brake()
                self.controller.set_target(0, 0)

                if not hazard_forwards:  # if object has moved
                    self.update_state(self.hazard_callback_state)

                if self.elapsed_ms() > self.HAZARD_TIMEOUT:  # if object does not move, we quit
                    self.update_state(HAZARD_FORWARDS_BYPASS)
                elif self.vehicle.has_screen():
                    if self.elapsed_ms() > self.HAZARD_TIMEOUT*(3/4):
                        self.screen.print_variable(". . .", 5, 5)
                    elif self.elapsed_ms() > self.HAZARD_TIMEOUT*(2/4):
                        self.screen.print_variable(". .", 5, 5)
                    elif self.elapsed_ms() > self.HAZARD_TIMEOUT*(1/4):
                        self.screen.print_variable(".", 5, 5)

            # Stop if we encounter a hazard behind us on the road
            elif self.state == HAZARD_BACKWARDS:
                if self.is_transition:
                    self.emergency_brake()
                    self.controller.set_target(0, 0)

                if not hazard_backwards:  # if object has moved
                    self.update_state(self.hazard_callback_state)

                if self.elapsed_ms() > self.HAZARD_TIMEOUT:  # if object does not move, we quit
                    self.update_state(HAZARD_BACKWARDS_BYPASS)
                elif self.vehicle.has_screen():
                    if self.elapsed_ms() > self.HAZARD_TIMEOUT*(3/4):
                        self.screen.print_variable(". . .", 5, 5)
                    elif self.elapsed_ms() > self.HAZARD_TIMEOUT*(2/4):
                        self.screen.print_variable(". .", 5, 5)
                    elif self.elapsed_ms() > self.HAZARD_TIMEOUT*(1/4):
                        self.screen.print_variable(".", 5, 5)

            # - HAZARDS REACTIONS -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == HAZARD_FORWARDS_BYPASS:
                if self.is_transition:
                    self.max_speed = SLOW_STRAIGHT_PWM
                    self.controller.set_target(-50, -50)  # clearance for rotate

                if self.controller.target_met():
                    sleep_ms(self.STATE_CHANGE_DELAY)  # (make sure we are stopped after each target)
                    if self.state_phase == 0:  # rotate left
                        self.max_speed = SLOW_ROTATE_PWM
                        self.controller.set_target(-QUARTER_TURN, QUARTER_TURN)
                        self.state_phase += 1

                    elif self.state_phase == 1:  # travel forwards a little
                        self.max_speed = SLOW_STRAIGHT_PWM
                        self.controller.set_target(self.HAZARD_BYPASS_MM, self.HAZARD_BYPASS_MM)
                        self.state_phase += 1

                    elif self.state_phase == 2:  # rotate right, and ...
                        self.max_speed = SLOW_ROTATE_PWM
                        self.controller.set_target(QUARTER_TURN, -QUARTER_TURN)
                        self.state_phase += 1

                    elif self.state_phase == 3:  # ... hopefully we are clear!
                        if hazard_forwards:
                            self.update_state(HAZARD_FORWARDS_BYPASS)  # try, try, try again
                        else:
                            self.max_speed = SLOW_STRAIGHT_PWM
                            self.controller.set_target(self.HAZARD_CLEARANCE_MM, self.HAZARD_CLEARANCE_MM)
                            self.custom_target_left -= self.HAZARD_BYPASS_MM
                            self.custom_target_right -= self.HAZARD_BYPASS_MM
                            self.state_phase += 1

                    elif self.state_phase == 4:  # rotate right
                        self.max_speed = SLOW_ROTATE_PWM
                        self.controller.set_target(QUARTER_TURN, -QUARTER_TURN)
                        self.state_phase += 1

                    elif self.state_phase == 5:  # undo the bypass travel we did
                        self.max_speed = SLOW_STRAIGHT_PWM
                        self.controller.set_target(self.HAZARD_BYPASS_MM, self.HAZARD_BYPASS_MM)  # TODO: CORRECT AMOUNT
                        self.state_phase += 1

                    elif self.state_phase == 6:  # rotate left to get back on track
                        self.max_speed = SLOW_ROTATE_PWM
                        self.controller.set_target(-QUARTER_TURN, QUARTER_TURN)
                        self.state_phase += 1

                    else:
                        self.max_speed = SLOW_STRAIGHT_PWM
                        self.update_state(self.hazard_callback_state)

            elif self.state == HAZARD_BACKWARDS_BYPASS:
                if self.is_transition:  # rotate left
                    self.max_speed = ROTATE_PWM
                    self.controller.set_target(-QUARTER_TURN, QUARTER_TURN)

                if self.controller.target_met():
                    sleep_ms(self.STATE_CHANGE_DELAY)  # (make sure we are stopped after each target)
                    if self.state_phase == 0:  # travel backwards a little
                        self.max_speed = STRAIGHT_PWM
                        self.controller.set_target(-self.HAZARD_BYPASS_MM, -self.HAZARD_BYPASS_MM)
                        self.state_phase += 1

                    elif self.state_phase == 1:  # rotate right, and ...
                        self.max_speed = ROTATE_PWM
                        self.controller.set_target(QUARTER_TURN, -QUARTER_TURN)
                        self.state_phase += 1

                    elif self.state_phase == 2:  # ... hopefully we are clear!
                        if hazard_backwards:
                            self.update_state(HAZARD_BACKWARDS_BYPASS)  # try, try, try again
                        else:
                            self.update_state(self.hazard_callback_state)

            # - FORWARDS -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == FORWARDS:
                # set initial target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - BACKWARDS -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == BACKWARDS:
                # set initial target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - ROTATE RIGHT -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == ROTATE_RIGHT:
                # set initial target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - ROTATE LEFT -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == ROTATE_LEFT:
                # set initial target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - PARKING -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == PARKING:
                # set initial target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - UNPARKING -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == UNPARKING:
                # set initial target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - CUSTOM -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == CUSTOM:
                # set initial target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())
            # - - - - - - - - - - - - - - - - - - - - CONTROL MOTORS - - - - - - - - - - - - - - - - - - - - #
            self.controller.set_max_duty(self.max_speed)
            self.vehicle.set_motor(*self.controller.run())


# - - - - - - - - - - - - - - - - - - - - MOTOR CONTROLLER METHODS - - - - - - - - - - - - - - - - - - - - - - #
def test_controller(target_mm_l, target_mm_r, amplitude, offset, base_duty, bias=3, loops=30, sleep=50):
    """Test different proportionality constants for controller"""
    vehicle = Vehicle(screen=True, enc=True, motor=True)
    vehicle.controller.reset(target_mm_l, target_mm_r, amplitude, offset, base_duty, bias)
    while not vehicle.controller.target_met():
        vehicle.set_motor(*vehicle.controller.run())
        sleep_ms(sleep)
    vehicle.set_motor(0, 0)


def test2_controller(target_mm_l, target_mm_r, max_duty, bias=3, loops=30, sleep=50):
    """Test different proportionality constants for controller"""
    vehicle = Vehicle(screen=True, enc=True, motor=True)
    vehicle.controller.set_target(target_mm_l, target_mm_r)
    vehicle.controller.set_max_duty(max_duty)
    while not vehicle.controller.target_met():
        vehicle.set_motor(*vehicle.controller.run())
        sleep_ms(sleep)
    vehicle.set_motor(0, 0)

# - - - - - - - - - - - - - - - - - - - - - - - - ENTRY POINT - - - - - - - - - - - - - - - - - - - - - - - - - - #
if __name__ == "__main__":
    statemachine = StateMachine(SPLASH_SCREEN)
    statemachine.main()
