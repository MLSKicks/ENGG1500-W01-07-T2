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

# - - - - - - - - - - - - - - - - - - - - - - - STATE ENUMERATION - - - - - - - - - - - - - - - - - - - - - - - #
NULL = -1
SPLASH_SCREEN = 0
PRINT_ROAD_INFO = 1
STOP = 3
HAZARD_FORWARDS = 4
HAZARD_BACKWARDS = 5
FORWARDS = 100
BACKWARDS = 101
ROTATE_LEFT = 102
ROTATE_RIGHT = 103
CUSTOM = 104
PARKING = 105
UNPARKING = 106
DEPLOY_SENSOR = 107

# ! ! ! ! ! ! ! ! ! EDIT HERE ! ! ! ! ! ! ! ! ! ! #

v_sf = 1  # vertical scale factor
h_sf = 1  # horizontal scale factor

track_states = [
    # reverse out
    (UNPARKING, -550*h_sf, -550*h_sf, 45),  # backwards
    (ROTATE_RIGHT, 100, -100, 50),   # rotate right

    # get to first roundabout
    (FORWARDS, 750*v_sf, 750*v_sf, 45),   # forwards
    DEPLOY_SENSOR,

    # get to second roundabout
    (FORWARDS, 1650*v_sf, 1650*v_sf, 45),  # forwards
    DEPLOY_SENSOR,

    # return home
    (ROTATE_RIGHT, 200, -200, 50),   # rotate right
    (FORWARDS, 2430*v_sf, 2430*v_sf, 45),  # forwards

    # reverse in
    (ROTATE_RIGHT, 100, -100, 50),   # rotate right
    (PARKING, 550*h_sf, 550*h_sf, 45),  # forwards
    STOP
]


# - - - - - - - - - - - - - - - - - - - - - STATE MACHINE - - - - - - - - - - - - - - - - - - - - - - - #
class StateMachine:
    def __init__(self, initial_state):
        self.prev_state = NULL  # Previous state
        self.state = initial_state  # Current state
        self.is_transition = True  # Transition flag telling us if we just switched states
        self.update_is_transition = False  # Flag tells us to update is_transition to false
        self.t0 = ticks_ms()  # For calculating time spent in a state

        # Special state variables for the path state
        self.track_counter = 0
        self.custom_target_left, self.custom_target_right = 0, 0
        self.max_speed = 65
        self.HAZARD_TIMEOUT = 5000  # ms
        self.DEPLOY_SENSOR_TIMEOUT = 1000  # ms
        self.SPLASH_SCREEN_TIMEOUT = 1000  # ms

        # Other initialisation
        self.vehicle = Vehicle(motor=True, enc=True, screen=True, ir_f=True, ir_b=False)
        self.controller = self.vehicle.controller  # motor control object can set duties to achieve desired targets
        self.screen = self.vehicle.screen  # Get OLED screen object -> can print useful information

    def update_state(self, new_state, is_transition=True):
        """Update our current state"""
        self.prev_state = self.state
        self.state = new_state
        self.is_transition = is_transition

    def update_state_variables(self):
        """Updates our transition flag (is_transition).
        The transition flag tells us if we have just transitioned to a new state. This is helpful
        in running a piece of code in a state one and once only."""
        if self.prev_state == self.state:
            if self.is_transition:
                if self.update_is_transition:
                    self.t0 = ticks_ms()
                    self.update_is_transition = False
                    self.is_transition = False
                else:
                    self.update_is_transition = True
        else:
            self.update_is_transition = True
        self.prev_state = self.state

    def print_state(self):
        """Print out what state we are in"""
        if self.is_transition:  # Run this ONCE when we have just entered a new state -> avoids flickering
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
                self.screen.print("State: Hazard\n\nSomething is in\nfront of me!")
            elif self.state == HAZARD_BACKWARDS:
                self.screen.print("State: Hazard\n\nSomething is be-\nhind me!")
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
                self.screen.print("State: parking")
            elif self.state == UNPARKING:
                self.screen.print("State: unparking")
            else:
                self.screen.print("State: Not Found")

    def elapsed_ms(self):
        """Calculates the elapsed ms in the current state, relies on update_state_variables()"""
        return ticks_diff(ticks_ms(), self.t0)

    def track_next_state(self):
        next_state = track_states[self.track_counter]

        if type(next_state) is tuple:
            self.custom_target_left = next_state[1]
            self.custom_target_right = next_state[2]
            self.max_speed = next_state[3]
            next_state = next_state[0]

        self.track_counter += 1
        sleep_ms(1000)
        return next_state

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
                if self.state == FORWARDS or self.state == PARKING:
                    self.update_state(HAZARD_FORWARDS)
                    self.custom_target_left, self.custom_target_right = self.controller.get_remainder_target()

            if hazard_backwards:  # Something is behind us so lets stop ...
                # only if we are currently travelling backwards
                if self.state == BACKWARDS or self.state == UNPARKING:
                    self.update_state(HAZARD_BACKWARDS)
                    self.custom_target_left, self.custom_target_right = self.controller.get_remainder_target()

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
            if self.state == DEPLOY_SENSOR:
                if self.is_transition:
                    self.controller.set_target(0, 0)

                if self.elapsed_ms() > self.DEPLOY_SENSOR_TIMEOUT:  # wait until sensors are deployed
                    self.update_state(self.track_next_state())
                elif self.elapsed_ms() > self.DEPLOY_SENSOR_TIMEOUT*3/4:
                    self.screen.print_variable(". . .", 5, 4)
                elif self.elapsed_ms() > self.DEPLOY_SENSOR_TIMEOUT*2/4:
                    self.screen.print_variable(". .", 5, 4)
                elif self.elapsed_ms() > self.DEPLOY_SENSOR_TIMEOUT*1/4:
                    self.screen.print_variable(".", 5, 4)

            # - PRINT_ROAD_INFO -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Displays what our IR sensors are saying
            elif self.state == PRINT_ROAD_INFO:
                if self.is_transition:
                    self.controller.set_target(0, 0)

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
            elif self.state == HAZARD_FORWARDS:  # TODO: Timeout go around hazard?
                if self.is_transition:
                    self.controller.set_target(0, 0)

                if not hazard_forwards:  # if object has moved
                    self.update_state(self.prev_state)

                if self.elapsed_ms() > self.HAZARD_TIMEOUT:  # if object does not move, we quit
                    self.update_state(STOP)
                elif self.elapsed_ms() > self.HAZARD_TIMEOUT*3/4:
                    self.screen.print_variable(". . .", 5, 4)
                elif self.elapsed_ms() > self.HAZARD_TIMEOUT*2/4:
                    self.screen.print_variable(". .", 5, 4)
                elif self.elapsed_ms() > self.HAZARD_TIMEOUT*1/4:
                    self.screen.print_variable(".", 5, 4)

            # Stop if we encounter a hazard behind us on the road
            elif self.state == HAZARD_BACKWARDS:  # TODO: Timeout go around hazard?
                if self.is_transition:
                    self.controller.set_target(0, 0)

                if not hazard_backwards:  # if object has moved
                    self.update_state(self.prev_state)

                if self.elapsed_ms() > self.HAZARD_TIMEOUT:  # if object does not move, we quit
                    self.update_state(STOP)
                elif self.elapsed_ms() > self.HAZARD_TIMEOUT*3/4:
                    self.screen.print_variable(". . .", 5, 4)
                elif self.elapsed_ms() > self.HAZARD_TIMEOUT*2/4:
                    self.screen.print_variable(". .", 5, 4)
                elif self.elapsed_ms() > self.HAZARD_TIMEOUT*1/4:
                    self.screen.print_variable(".", 5, 4)

            # - FORWARDS -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == FORWARDS:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - BACKWARDS -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == BACKWARDS:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - ROTATE RIGHT -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == ROTATE_RIGHT:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - ROTATE LEFT -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == ROTATE_LEFT:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - PARKING -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == PARKING:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - UNPARKING -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == UNPARKING:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.track_next_state())

            # - CUSTOM -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == CUSTOM:
                # set target for travel
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


if __name__ == "__main__":
    # sleep_ms(1000)
    # complete_basic_track()
    statemachine = StateMachine(SPLASH_SCREEN)
    statemachine.main()
