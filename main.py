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
HAZARD = 4
FORWARDS = 100
BACKWARDS = 101
ROTATE_LEFT = 102
ROTATE_RIGHT = 103
CUSTOM = 104

default_track_states = [
    # reverse out
    (CUSTOM, -450, -450, 55),  # backwards
    (CUSTOM, 110, -110, 65),   # rotate right

    # get to first roundabout
    (CUSTOM, 600, 600, 55),   # forwards
    SPLASH_SCREEN, SPLASH_SCREEN,

    # get to second roundabout
    (CUSTOM, 1500, 1500, 45),  # forwards
    SPLASH_SCREEN, SPLASH_SCREEN,

    # return home
    (CUSTOM, 220, -220, 55),   # rotate right
    (CUSTOM, 2100, 2100, 45),  # forwards

    # reverse in
    (CUSTOM, 110, -110, 65),   # rotate right
    (CUSTOM, -450, -450, 55),  # backwards
    STOP
]

# default_track_states = [
#     ROTATE_RIGHT,
#     ROTATE_LEFT,
#     STOP
# ]


# - - - - - - - - - - - - - - - - - - - - - STATE MACHINE - - - - - - - - - - - - - - - - - - - - - - - #
class StateMachine:
    def __init__(self, initial_state):
        self.prev_state = NULL  # Previous state
        self.state = initial_state  # Current state
        self.is_transition = True  # Transition flag telling us if we just switched states
        self.update_is_transition = False  # Flag tells us to update is_transition to false
        self.t0 = ticks_ms()  # For calculating time spent in a state

        # Special state variables for the path state
        self.default_track_counter = 0
        self.custom_target_left, self.custom_target_right = 0, 0
        self.max_speed = 65

        # Other initialisation
        self.vehicle = Vehicle(motor=True, enc=True, screen=True, ir_f=False, ir_b=False)
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
            elif self.state == PRINT_ROAD_INFO:
                self.screen.print("State: Road Info")
            elif self.state == STOP:
                self.screen.print("State: Stopped\n\nMy job is done!")
            elif self.state == HAZARD:
                self.screen.print("State: Hazard\n\nSomething got in\n my way!")
            elif self.state == FORWARDS:
                self.screen.print("State: Travel\n-ling Forwards")
            elif self.state == BACKWARDS:
                self.screen.print("State: Travel\n-ling Backwards")
            elif self.state == ROTATE_LEFT:
                self.screen.print("State: Rotate\nLeft")
            elif self.state == ROTATE_RIGHT:
                self.screen.print("State: Rotate\nRight")
            elif self.state == CUSTOM:
                self.screen.print("State: Custom")
            else:
                self.screen.print("State: Not Found")

    def elapsed_ms(self):
        """Calculates the elapsed ms in the current state, relies on update_state_variables()"""
        return ticks_diff(ticks_ms(), self.t0)

    def default_track_next_state(self):
        next_state = default_track_states[self.default_track_counter]

        if type(next_state) is tuple:
            if next_state[0] == CUSTOM:
                self.custom_target_left = next_state[1]
                self.custom_target_right = next_state[2]
                self.max_speed = next_state[3]
            next_state = next_state[0]

        self.default_track_counter += 1
        sleep_ms(200)
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
            hazard_forwards = False
            hazard_backwards = False

            # - - - - - - - - - - - - - - - - - - - - GLOBAL TRANSITIONS - - - - - - - - - - - - - - - - - - - - #
            if hazard_forwards or hazard_backwards:  # Something is in front so lets stop
                self.update_state(HAZARD)

            # - - - - - - - - - - - - - - - - - - - - STATE MACHINE HEADER - - - - - - - - - - - - - - - - - - - #
            self.update_state_variables()  # Updates prev_state and is_transition flag
            self.print_state()             # Prints current state information if we just transitioned

            # - - - - - - - - - - - - - - - - - - - - STATE MACHINE BODY - - - - - - - - - - - - - - - - - - - - #
            # - SPLASH_SCREEN -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Prints a cat to the screen for a second
            if self.state == SPLASH_SCREEN:
                if self.is_transition:
                    self.controller.set_target(0, 0)

                if self.elapsed_ms() > 1000:
                    self.update_state(PRINT_ROAD_INFO)

            # - PRINT_ROAD_INFO -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Displays what our IR sensors are saying
            elif self.state == PRINT_ROAD_INFO:
                if self.is_transition:
                    self.controller.set_target(0, 0)

                if self.elapsed_ms() > 100:
                    self.update_state(self.default_track_next_state())

            # - STOP -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Stop once we have finished our task
            elif self.state == STOP:  # TODO: How does user ask vehicle to go again after finishing track?
                if self.is_transition:
                    self.controller.set_target(0, 0)

            # - HAZARD -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            # Stop if we encounter a hazard on the road
            elif self.state == HAZARD:  # TODO: How do we react to a hazard? Stop? Go Around?
                if self.is_transition:
                    self.controller.set_target(-100, -100)

                if self.controller.target_met():
                    self.update_state(STOP)

            # - FORWARDS -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == FORWARDS:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(300, 300)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.default_track_next_state())

            # - BACKWARDS -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == BACKWARDS:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(-300, -300)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.default_track_next_state())

            # - ROTATE RIGHT -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == ROTATE_RIGHT:
                # set target for travel
                if self.is_transition:
                    print("run")
                    self.controller.set_target(110, -110)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.default_track_next_state())

            # - ROTATE LEFT -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == ROTATE_LEFT:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(-110, 110)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.default_track_next_state())

            # - CUSTOM -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            elif self.state == CUSTOM:
                # set target for travel
                if self.is_transition:
                    self.controller.set_target(self.custom_target_left, self.custom_target_right)
                # state transition
                if self.controller.target_met():
                    self.update_state(self.default_track_next_state())
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


if __name__ == "__main__":
    # sleep_ms(1000)
    # complete_basic_track()
    statemachine = StateMachine(SPLASH_SCREEN)
    statemachine.main()
