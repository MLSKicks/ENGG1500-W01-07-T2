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


def run(vehicle, left_target, right_target):
    """Complete a target using motor_control"""
    # Set our target
    vehicle.controller.set_target(left_target, right_target)

    # Loop until we are done!
    while not vehicle.controller.target_met():
        vehicle.set_motor(*vehicle.controller.run())

    # Done, so stop motors
    vehicle.set_motor(0, 0)


def gentle_curve(vehicle, turn_left=True):
    """Travel around the gentle curve track piece, in either the left or right direction"""
    if turn_left:  # Do the turn
        run(vehicle, 309, 529)
    else:
        run(vehicle, 529, 309)


def roundabout(vehicle, exit_):
    """Take an any exit on the roundabout track piece! We must first ensure 1 <= exit_ <= 4, then:
            1 -> turn left
            2 -> travel forward
            3 -> turn right
            4 -> U-turn """

    exit_ = (exit_ % 4)  # Ensure that exit_ is a valid exit
    if exit_ == 0:
        exit_ = 4

    run(vehicle, 45, 45)           # Lead in
    run(vehicle, -110, 110)        # Turn onto the roundabout
    for i in range(0, exit_, 1):   # Complete a quarter turn exit_ times
        run(vehicle, 271, 52)
    run(vehicle, -110, 110)        # Travel out of the roundabout
    run(vehicle, 45, 45)           # Lead out


def distracting_line(vehicle):
    """Complete the distracting line track piece"""
    run(vehicle, 297, 297)


def fork(vehicle, turn_left=True):
    """Complete the fork, turning left if turn_left=True or right if turn_left=False"""
    run(vehicle, 35, 35)  # Lead in
    if turn_left:
        run(vehicle, 70, 290)
    else:
        run(vehicle, 290, 70)
    run(vehicle, 35, 35)  # Lead out


def corner(vehicle, turn_left=True):
    """Complete the corner, turning left if turn_left=True or right if turn_left=False"""
    run(vehicle, 148, 148)
    if turn_left:
        run(vehicle, -110, 110)  # Rotate 90 degrees left
    else:
        run(vehicle, 110, -110)  # Rotate 90 degrees right
    run(vehicle, 148, 148)


def straight(vehicle):
    """Complete the straight track piece"""
    run(vehicle, 297, 297)


def hallway(vehicle):
    """Complete the converging or parallel hallway"""
    run(vehicle, 594, 594)


def curve_road(vehicle, reverse_direction=False):
    """Complete the special bendy piece"""
    if reverse_direction:
        run(vehicle, 297, 52)  # Travel in a big circle clockwise 100 degrees
        run(vehicle, 39, 479)  # Travel in a big circle anti-clockwise 180 degrees
        run(vehicle, 180, 180)  # Go straight for a bit
        run(vehicle, 55, -55)  # Rotate -45 degrees
    else:
        run(vehicle, -55, 55)  # Rotate 45 degrees
        run(vehicle, 180, 180)  # Go straight for a bit
        run(vehicle, 479, 39)   # Travel in a big circle clockwise 180 degrees
        run(vehicle, 52, 297)   # Travel in a big circle anti-clockwise 100 degrees


def dead_end(vehicle, facing_dead_end=True):
    """Complete the dead end piece"""
    # TODO dead_end
    if facing_dead_end:
        pass
    else:
        pass


def sharp_bend(vehicle, reverse_direction=False):
    """Complete the sharp bend piece"""
    # TODO sharp_bend
    if reverse_direction:
        pass
    else:
        pass


def complete_basic_track():
    """Complete the basic track, let's get at least a pass!"""
    vehicle = Vehicle(motor=True, enc=True, screen=True)

    # Get to 2 roundabouts
    gentle_curve(vehicle, turn_left=True)
    straight(vehicle)
    roundabout(vehicle, 2)
    straight(vehicle)
    fork(vehicle, turn_left=True)
    gentle_curve(vehicle, turn_left=False)
    gentle_curve(vehicle, turn_left=False)
    fork(vehicle, turn_left=True)
    distracting_line(vehicle)
    roundabout(vehicle, 4)

    # Get back home
    distracting_line(vehicle)
    fork(vehicle, turn_left=True)
    dead_end(vehicle, facing_dead_end=False)
    distracting_line(vehicle)
    curve_road(vehicle, reverse_direction=False)
    fork(vehicle, turn_left=True)
    straight(vehicle)
    roundabout(vehicle, 2)
    straight(vehicle)
    gentle_curve(vehicle, turn_left=False)


if __name__ == "__main__":
    v = Vehicle(motor=True, enc=True, screen=True)

    sleep_ms(1000)
    complete_basic_track()

    sleep_ms(1000)
    main(SPLASH_SCREEN)
