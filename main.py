from vehicle_components import Vehicle
from time import sleep_ms, ticks_ms, ticks_diff

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
IDLE = 2
STOP = 3
HAZARD = 4
LF_FWD = 5
LF_TURN_LEFT = 6
LF_TURN_RIGHT = 7


# - - - - - - - - - - - - - - - - - - - - - - - STATE VARIABLES - - - - - - - - - - - - - - - - - - - - - - - - #
prev_state = NULL        # Previous state
state = NULL             # Current state
is_transition = False    # Transition flag telling us if we just switched states
t0 = ticks_ms()          # For calculating time spent in a state


def update_state_variables():
    """Updates our previous state (prev_state) variable and transition flag (is_transition).
    The transition flag tells us if we have just transitioned to a new state. This is helpful
    in running a piece of code in a state one and once only."""
    global prev_state, state, is_transition, t0
    is_transition = prev_state != state
    prev_state = state
    if is_transition:
        t0 = ticks_ms()


def print_state(screen):
    """Print out what state we are in"""
    global state, is_transition
    if is_transition:  # Run this ONCE when we have just entered a new state -> avoids flickering
        if state == NULL:
            screen.print("State: NULL\n\nNo initial state\nwas specified!")
        elif state == SPLASH_SCREEN:
            screen.print_art(ascii_cat)
        elif state == PRINT_ROAD_INFO:
            screen.print("State: Road Info")
        elif state == IDLE:
            screen.print("State: Idling\n\nI am lost!")
        elif state == STOP:
            screen.print("State: Stopped\n\nMy job is done!")
        elif state == HAZARD:
            screen.print("State: Hazard\n\nSomething got in\n my way!")
        elif state == LF_FWD:
            screen.print("State: Line Foll\n-owing")
        elif state == LF_TURN_LEFT:
            screen.print("State: Line Foll\n-owing LEFT")
        elif state == LF_TURN_RIGHT:
            screen.print("State: Line Foll\n-owing RIGHT")
        else:
            screen.print("State: Not Found")


def set_initial_targets(controller):
    """Sets the initial targets for each state"""
    global state, is_transition
    if is_transition:  # Run this ONCE when we have just entered a new state -> otherwise motor controller will break
        # if state == NULL:
        #     controller.set_target(0, 0)
        # elif state == SPLASH_SCREEN:
        #     controller.set_target(0, 0)
        # elif state == PRINT_ROAD_INFO:
        #     controller.set_target(0, 0)
        # elif state == IDLE:
        #     controller.set_target(0, 0)
        # elif state == STOP:
        #     controller.set_target(0, 0)
        # elif state == HAZARD:
        #     controller.set_target(0, 0)
        if state == LF_FWD:
            controller.set_target(50, 50)
        elif state == LF_TURN_LEFT:
            controller.set_target(50, 100)
        elif state == LF_TURN_RIGHT:
            controller.set_target(100, 50)
        else:
            controller.set_target(0, 0)


def elapsed_ms():
    """Calculates the elapsed ms in the current state, relies on update_state_variables()"""
    global t0
    return ticks_diff(ticks_ms(), t0)


def main(initial_state=NULL):
    """This is our main state machine: a big loop that performs actions based on the current state!

    Initialisation: initialise our Vehicle object which initialises objects for each sensor, controller, etc.

    Sensor Data Collection: collect data from all our sensors

    Global Transitions: these are IMPORTANT TRANSITIONS which can overwrite anything. They are important as they
    are reacting to things like obstacles on the road, in which case we want to stop ASAP!

    State Machine Body: goes through all the nitty-gritty details """

    global prev_state, state, is_transition, ascii_cat, t0

    # - - - - - - - - - - - - - - - - - - - - - - - INITIALISATION - - - - - - - - - - - - - - - - - - - - - - - #
    vehicle = Vehicle(motor=True, enc=True, screen=True, rgb=True, ir_l=True, ir_r=True, us_l=True, us_r=True)
    controller = vehicle.controller          # Get motor control object -> can set duties to achieve desired targets
    screen = vehicle.screen    # Get OLED screen object -> can print useful information
    state = initial_state      # Set the requested initial state

    while True:
        # - - - - - - - - - - - - - - - - - - - - SENSOR DATA COLLECTION - - - - - - - - - - - - - - - - - - #
        ir_l_onroad = vehicle.ir_l.is_on_road()
        ir_r_onroad = vehicle.ir_r.is_on_road()
        rgb_onroad = vehicle.rgb.is_on_road()  # rgb_onroad is more vague than rgb_directly_onroad
        rgb_directly_onroad = vehicle.rgb.is_on_road_by_prox()  # more like an IR sensor reading
        ambient = vehicle.rgb.ambient()
        rgb_hue = vehicle.rgb.hue()
        rgb_prox = vehicle.rgb.proximity_mm()
        us_l = vehicle.us_l.proximity()
        us_r = vehicle.us_r.proximity()

        # - - - - - - - - - - - - - - - - - - - - GLOBAL TRANSITIONS - - - - - - - - - - - - - - - - - - - - #
        if rgb_prox < 35:         # Something is on the road or obstructing the sensor -> so lets stop
            state = HAZARD

        # - - - - - - - - - - - - - - - - - - - - STATE MACHINE HEADER - - - - - - - - - - - - - - - - - - - #
        update_state_variables()  # Updates prev_state and is_transition flag
        print_state(screen)       # Prints current state information if we just transitioned
        set_initial_targets(controller)  # Sets our controller targets if we just transitioned

        # - - - - - - - - - - - - - - - - - - - - STATE MACHINE BODY - - - - - - - - - - - - - - - - - - - - #
        # - SPLASH_SCREEN -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Prints a cat to the screen for a second
        if state == SPLASH_SCREEN:
            if elapsed_ms() > 1000:
                state = PRINT_ROAD_INFO

        # - PRINT_ROAD_INFO -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Displays what our IR/RGB sensors are saying about the road
        elif state == PRINT_ROAD_INFO:
            screen.print_variable("IR-L Road{!s:>7}\n"
                                  "IR-R Road{!s:>7}\n"
                                  "RGB Road{!s:>8}\n"
                                  "RGB Amb{:9d}\n"
                                  "RGB Hue{:9d}\n"
                                  "RGB Prox{:8d}".format(ir_l_onroad, ir_r_onroad, rgb_directly_onroad,
                                                         ambient, rgb_hue, rgb_prox),
                                  0, 2)

        # - IDLE -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # If we are lost, we go into idle and wander around
        elif state == IDLE:  # TODO: Smarter idle wandering
            if controller.target_met():
                controller.set_target(50, 50)

        # - STOP -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Stop once we have finished our task
        elif state == STOP:  # TODO: How does user ask vehicle to go again after finishing track?
            if is_transition:
                controller.set_target(0, 0)

        # - HAZARD -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Stop if we encounter a hazard on the road
        elif state == HAZARD:  # TODO: How do we react to a hazard? Stop? Go Around?
            pass

        # - LF_FWD -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Line-Follow-Forward attempts to follow straight or slightly bendy lines
        elif state == LF_FWD:  # TODO: Fix Line Following
            # If we finished our target, just go again
            if controller.target_met():
                controller.set_target(50, 50)

            # Adjust for slight veers rightwards off the road -> by veering left
            if ir_l_onroad and not ir_r_onroad:
                screen.print("State: LF_FWD\nveering right")
                controller.add_target(-15, 15)

            # Adjust for slight veers leftwards off the road -> by veering right
            if not ir_l_onroad and ir_r_onroad:
                screen.print("State: LF_FWD\nveering left")
                controller.add_target(15, -15)

        # - - - - - - - - - - - - - - - - - - - - CONTROL MOTORS - - - - - - - - - - - - - - - - - - - - #
        vehicle.set_motor(*controller.run())


def test_controller(target_mm_l, target_mm_r, amplitude, offset, base_duty, bias=3, loops=30, sleep=50):
    """Test different proportionality constants for controller"""
    vehicle = Vehicle(screen=True, enc=True, motor=True)
    vehicle.controller.reset(target_mm_l, target_mm_r, amplitude, offset, base_duty, bias)
    for i in range(0, loops):
        vehicle.set_motor(*vehicle.controller.run())
        sleep_ms(sleep)
    vehicle.set_motor(0, 0)


def run(vehicle, left_target, right_target, n=1):
    # Set our target
    vehicle.controller.set_target(left_target, right_target)

    # Loop until we are done!
    while not vehicle.controller.target_met():
        vehicle.set_motor(*vehicle.controller.run())

    # Done, so stop motors
    vehicle.set_motor(0, 0)


def gentle_curve(vehicle, turn_left=False, turn_right=False):
    """ Travel around the gentle curve track piece, in either the left or right direction"""
    if turn_left:
        vehicle.screen.print("Gentle Curve\n\nTurning left")
        run(vehicle, 300, 500)

    if turn_right:
        vehicle.screen.print("Gentle Curve\n\nTurning right")
        run(vehicle, 500, 300)


def roundabout(vehicle, exit_):
    """ Take an any exit on the roundabout track piece! We must first ensure 1 <= exit_ <= 4, then:
            1 -> turn left
            2 -> travel forward
            3 -> turn right
            4 -> U-turn """

    # Ensure that exit_ is a valid exit
    exit_ = (exit_ % 4)
    if exit_ == 0:
        exit_ = 4

    # Travel onto the roundabout
    vehicle.screen.print("Round About\n\nGetting on")
    run(vehicle, 100, 100)
    run(vehicle, -130, 130)

    for i in range(0, exit_, 1):
        # Complete a quarter turn
        vehicle.screen.print("Round About\n\nTravelling around {}/{}".format(i, exit_))
        run(vehicle, 267, 31)

    # Travel out of the roundabout
    vehicle.screen.print("Round About\n\nGetting off")
    run(vehicle, -130, 130)


if __name__ == "__main__":
    v = Vehicle(motor=True, enc=True, screen=True)

    sleep_ms(1000)
    roundabout(v, exit_=2)

    sleep_ms(1000)
    gentle_curve(v, turn_right=True)

    del v

    sleep_ms(1000)
    main(SPLASH_SCREEN)
