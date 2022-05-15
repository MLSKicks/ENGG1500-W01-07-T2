from vehicle_components import Vehicle
from time import sleep_ms, ticks_ms, ticks_diff

# TODO: TRACK DISTORTED BY 1.33x ????
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
PATH_STRAIGHT = 100
PATH_GENTLE_CURVE = 101
PATH_ROUNDABOUT = 102
PATH_DISTRACTING_LINE = 103
PATH_FORK = 104
PATH_CORNER = 105
PATH_HALLWAY = 106
PATH_CURVEY_ROAD = 107
PATH_DEAD_END = 108
PATH_SHARP_BEND = 109


# - - - - - - - - - - - - - - - - - - - - - - - STATE VARIABLES - - - - - - - - - - - - - - - - - - - - - - - - #
prev_state = NULL        # Previous state
state = NULL             # Current state
is_transition = False    # Transition flag telling us if we just switched states
t0 = ticks_ms()          # For calculating time spent in a state

# Special state variables for the path state
LEFT = False
RIGHT = True
path_state_turn = LEFT
path_state_exit = 0
path_state_phase = 0
default_track_counter = 0


# - - - - - - - - - - - - - - - - - - - - - STATE MACHINE FUNCTIONS - - - - - - - - - - - - - - - - - - - - - - - #
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
        elif state == PATH_STRAIGHT:
            screen.print("State: Path Stra\n-ight")
        elif state == PATH_GENTLE_CURVE:
            screen.print("State: Path Gent\n-le Curve")
        elif state == PATH_ROUNDABOUT:
            screen.print("State: Path Roun\n-about")
        elif state == PATH_DISTRACTING_LINE:
            screen.print("State: Distracti\n-ng Lines")
        elif state == PATH_FORK:
            screen.print("State: Fork")
        elif state == PATH_CORNER:
            screen.print("State: Corner")
        elif state == PATH_HALLWAY:
            screen.print("State: Hallway")
        elif state == PATH_CURVEY_ROAD:
            screen.print("State: Curvey Ro\nad")
        elif state == PATH_DEAD_END:
            screen.print("State: Dead End")
        elif state == PATH_SHARP_BEND:
            screen.print("State: Sharp Ben\n-d")
        else:
            screen.print("State: Not Found")


def elapsed_ms():
    """Calculates the elapsed ms in the current state, relies on update_state_variables()"""
    global t0
    return ticks_diff(ticks_ms(), t0)


def default_track_next_state():
    global path_state_turn, path_state_exit, path_state_phase, default_track_counter, LEFT, RIGHT

    path_state_turn = LEFT
    path_state_exit = 0
    path_state_phase = 0

    track_states = [
        (PATH_GENTLE_CURVE, LEFT),
        PATH_STRAIGHT,
        (PATH_ROUNDABOUT, 2),
        PATH_STRAIGHT,
        (PATH_FORK, LEFT),
        (PATH_GENTLE_CURVE, RIGHT),
        (PATH_GENTLE_CURVE, RIGHT),
        (PATH_FORK, LEFT),
        PATH_DISTRACTING_LINE,
        (PATH_ROUNDABOUT, 4),
        PATH_DISTRACTING_LINE,
        (PATH_FORK, LEFT),
        (PATH_DEAD_END, False),
        PATH_DISTRACTING_LINE,
        (PATH_CURVEY_ROAD, False),
        (PATH_FORK, LEFT),
        PATH_STRAIGHT,
        (PATH_ROUNDABOUT, 2),
        PATH_STRAIGHT,
        (PATH_GENTLE_CURVE, RIGHT)
    ]

    if type(track_states[default_track_counter]) is tuple:
        next_state = track_states[default_track_counter][0]
        if next_state == PATH_GENTLE_CURVE or next_state == PATH_FORK:
            path_state_turn = track_states[default_track_counter][1]
        elif next_state == PATH_ROUNDABOUT:
            path_state_exit = track_states[default_track_counter][1]
    else:
        next_state = track_states[default_track_counter]

    default_track_counter += 1
    sleep_ms(250)
    return next_state


# - - - - - - - - - - - - - - - - - - - - - - - STATE MACHINE - - - - - - - - - - - - - - - - - - - - - - - - #
def main(initial_state=NULL):
    """This is our main state machine: a big loop that performs actions based on the current state!

    Initialisation: initialise our Vehicle object which initialises objects for each sensor, controller, etc.

    Sensor Data Collection: collect data from all our sensors

    Global Transitions: these are IMPORTANT TRANSITIONS which can overwrite anything. They are important as they
    are reacting to things like obstacles on the road, in which case we want to stop ASAP!

    State Machine Body: goes through all the nitty-gritty details """

    global prev_state, state, is_transition, ascii_cat, t0, LEFT, RIGHT, path_state_turn, \
        path_state_exit, path_state_phase

    # - - - - - - - - - - - - - - - - - - - - - - - INITIALISATION - - - - - - - - - - - - - - - - - - - - - - - #
    vehicle = Vehicle(motor=True, enc=True, screen=True, rgb=True, ir_l=True, ir_r=True, us_l=True, us_r=True)
    controller = vehicle.controller          # Get motor control object -> can set duties to achieve desired targets
    screen = vehicle.screen    # Get OLED screen object -> can print useful information
    state = initial_state      # Set the requested initial state
    sf = 1                     # Scale Factor for road stretching

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

        # - - - - - - - - - - - - - - - - - - - - STATE MACHINE BODY - - - - - - - - - - - - - - - - - - - - #
        # - SPLASH_SCREEN -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Prints a cat to the screen for a second
        if state == SPLASH_SCREEN:
            if is_transition:
                controller.set_target(0, 0)

            if elapsed_ms() > 1000:
                state = PRINT_ROAD_INFO

        # - PRINT_ROAD_INFO -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Displays what our IR/RGB sensors are saying about the road
        elif state == PRINT_ROAD_INFO:
            if is_transition:
                controller.set_target(0, 0)

            screen.print_variable("IR-L Road{!s:>7}\n"
                                  "IR-R Road{!s:>7}\n"
                                  "RGB Road{!s:>8}\n"
                                  "RGB Amb{:9d}\n"
                                  "RGB Hue{:9d}\n"
                                  "RGB Prox{:8d}".format(ir_l_onroad, ir_r_onroad, rgb_directly_onroad,
                                                         ambient, rgb_hue, rgb_prox),
                                  0, 2)

            if elapsed_ms() > 1000:
                state = default_track_next_state()

        # - IDLE -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # If we are lost, we go into idle and wander around
        elif state == IDLE:  # TODO: Smarter idle wandering
            if is_transition or controller.target_met():
                controller.set_target(50, 50)

        # - STOP -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Stop once we have finished our task
        elif state == STOP:  # TODO: How does user ask vehicle to go again after finishing track?
            if is_transition:
                controller.set_target(0, 0)

        # - HAZARD -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Stop if we encounter a hazard on the road
        elif state == HAZARD:  # TODO: How do we react to a hazard? Stop? Go Around?
            if is_transition:
                controller.set_target(0, 0)

        # - LF_FWD -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        # Line-Follow-Forward attempts to follow straight or slightly bendy lines
        elif state == LF_FWD:  # TODO: Fix Line Following
            # If we finished our target, just go again
            if is_transition or controller.target_met():
                controller.set_target(50, 50)

            # Adjust for slight veers rightwards off the road -> by veering left
            if ir_l_onroad and not ir_r_onroad:
                screen.print("State: LF_FWD\nveering right")
                controller.add_target(-15, 15)

            # Adjust for slight veers leftwards off the road -> by veering right
            if not ir_l_onroad and ir_r_onroad:
                screen.print("State: LF_FWD\nveering left")
                controller.add_target(15, -15)

        # - PATH -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        elif state == PATH_STRAIGHT:
            # set target for travel
            if is_transition:
                controller.set_target(297*sf, 297*sf)
            # state transition
            if controller.target_met():
                state = default_track_next_state()

        elif state == PATH_GENTLE_CURVE:
            # set target for travel
            if is_transition:
                if path_state_turn == LEFT:
                    controller.set_target(309, 525)
                else:
                    controller.set_target(525, 309)
            # state transition
            if controller.target_met():
                state = default_track_next_state()

        elif state == PATH_ROUNDABOUT:
            if is_transition or controller.target_met():
                if path_state_phase == 0:  # turn onto roundabout
                    controller.set_target(-110, 110)
                elif path_state_phase == 1:  # travel around roundabout
                    if path_state_exit > 0:
                        controller.set_target(260, 40)  # perform a quarter turn
                        path_state_exit -= 1  # count off each quarter turn
                        path_state_phase -= 1  # stop state phase from incrementing until finished
                elif path_state_phase == 2:  # turn off roundabout
                    controller.set_target(-110, 110)
                else:  # state transition
                    state = default_track_next_state()
                # increment roundabout phase
                path_state_phase += 1

        elif state == PATH_DISTRACTING_LINE:
            # set target
            if is_transition:
                controller.set_target(297, 297)
            # state transition
            if controller.target_met():
                state = default_track_next_state()

        elif state == PATH_FORK:
            # set target
            if is_transition:
                if path_state_turn == LEFT:
                    controller.set_target(70, 290)
                else:
                    controller.set_target(290, 70)
            # state transition
            if controller.target_met():
                state = default_track_next_state()

        elif state == PATH_CORNER:
            if is_transition or controller.target_met():
                if path_state_phase == 0:  # travel forwards
                    controller.set_target(148, 148)
                elif path_state_phase == 1 and path_state_turn == LEFT:  # make a corner turn left
                    controller.set_target(-100, 100)
                elif path_state_phase == 1 and path_state_turn == RIGHT:  # make a corner turn right
                    controller.set_target(100, -100)
                elif path_state_phase == 2:  # travel forwards
                    controller.set_target(148, 148)
                else:  # state transition
                    state = default_track_next_state()
                # increment corner phase
                path_state_phase += 1

        elif state == PATH_HALLWAY:
            # set target
            if is_transition:
                controller.set_target(594, 594)
            # state transition
            if controller.target_met():
                state = default_track_next_state()

        elif state == PATH_CURVEY_ROAD:
            pass

        elif state == PATH_DEAD_END:
            pass

        elif state == PATH_SHARP_BEND:
            pass

        # - - - - - - - - - - - - - - - - - - - - CONTROL MOTORS - - - - - - - - - - - - - - - - - - - - #
        vehicle.set_motor(*controller.run())


# - - - - - - - - - - - - - - - - - - - - MOTOR CONTROLLER METHODS - - - - - - - - - - - - - - - - - - - - - - #

def test_controller(target_mm_l, target_mm_r, amplitude, offset, base_duty, bias=3, loops=30, sleep=50):
    """Test different proportionality constants for controller"""
    vehicle = Vehicle(screen=True, enc=True, motor=True)
    vehicle.controller.reset(target_mm_l, target_mm_r, amplitude, offset, base_duty, bias)
    while not vehicle.controller.target_met():
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


def run2(vehicle, left_target, right_target, divisions):
    # Divide the targets into n steps or segments
    left_target_step = left_target / divisions
    right_target_step = right_target / divisions

    # Set our initial target
    vehicle.controller.set_target(left_target_step, right_target_step)

    # Once each step is done, add the next step. Loop until we are done!
    for i in range(0, divisions, 1):
        while not vehicle.controller.target_met():
            vehicle.set_motor(*vehicle.controller.run())

        # vehicle.pid.add_target(left_target_step, right_target_step)
        vehicle.controller.set_target(left_target_step, right_target_step)

    # Done, so stop motors
    vehicle.set_motor(0, 0)


def gentle_curve(vehicle, turn_left=True):
    """Travel around the gentle curve track piece, in either the left or right direction"""
    global sf
    vehicle.screen.print("State*: Gentle C-urve")
    if turn_left:  # Do the turn
        run2(vehicle, 309, 525, 2)
    else:
        run2(vehicle, 525, 309, 2)


def roundabout(vehicle, exit_):
    """Take an any exit on the roundabout track piece! We must first ensure 1 <= exit_ <= 4, then:
            1 -> turn left
            2 -> travel forward
            3 -> turn right
            4 -> U-turn """
    vehicle.screen.print("State*: Round Ab-out")
    exit_ = (exit_ % 4)  # Ensure that exit_ is a valid exit
    if exit_ == 0:
        exit_ = 4

    # run(vehicle, 45, 45)           # Lead in
    run(vehicle, -110, 110)        # Turn onto the roundabout
    sleep_ms(200)
    for i in range(0, exit_, 1):   # Complete a quarter turn exit_ times
        run(vehicle, 260, 40)
        sleep_ms(200)
    run(vehicle, -110, 110)        # Travel out of the roundabout
    # run(vehicle, 45, 45)           # Lead out


def distracting_line(vehicle):
    """Complete the distracting line track piece"""
    vehicle.screen.print("State*: Distract-ing Lines")
    run(vehicle, 297, 297)


def fork(vehicle, turn_left=True):
    """Complete the fork, turning left if turn_left=True or right if turn_left=False"""
    vehicle.screen.print("State*: Fork")
    run(vehicle, 35, 35)  # Lead in
    if turn_left:
        run2(vehicle, 70, 290, 2)
    else:
        run2(vehicle, 290, 70, 2)
    run(vehicle, 35, 35)  # Lead out


def corner(vehicle, turn_left=True):
    """Complete the corner, turning left if turn_left=True or right if turn_left=False"""
    vehicle.screen.print("State*: Corner")
    run(vehicle, 148, 148)
    if turn_left:
        run(vehicle, -100, 100)  # Rotate 90 degrees left
    else:
        run(vehicle, 100, -100)  # Rotate 90 degrees right
    run(vehicle, 148, 148)


def straight(vehicle):
    vehicle.screen.print("State*: Straight")
    """Complete the straight track piece"""
    run(vehicle, 297, 297)
    # run(vehicle, 367, 367)


def hallway(vehicle):
    """Complete the converging or parallel hallway"""
    vehicle.screen.print("State*: Hallway")
    run(vehicle, 594, 594)


def curve_road(vehicle, reverse_direction=False):
    """Complete the special bendy piece"""
    vehicle.screen.print("State*: Curvey")
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
    vehicle.screen.print("State*: Dead End")
    # TODO dead_end
    if facing_dead_end:
        pass
    else:
        pass


def sharp_bend(vehicle, reverse_direction=False):
    """Complete the sharp bend piece"""
    vehicle.screen.print("State*: Sharp Be-nd")
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
    sleep_ms(200)
    straight(vehicle)
    sleep_ms(200)
    roundabout(vehicle, 2)
    sleep_ms(200)
    straight(vehicle)
    sleep_ms(200)
    fork(vehicle, turn_left=True)
    sleep_ms(200)
    gentle_curve(vehicle, turn_left=False)
    sleep_ms(200)
    gentle_curve(vehicle, turn_left=False)
    sleep_ms(200)
    fork(vehicle, turn_left=True)
    sleep_ms(200)
    distracting_line(vehicle)
    sleep_ms(200)
    roundabout(vehicle, 4)

    # Get back home
    distracting_line(vehicle)
    sleep_ms(200)
    fork(vehicle, turn_left=True)
    sleep_ms(200)
    dead_end(vehicle, facing_dead_end=False)
    sleep_ms(200)
    distracting_line(vehicle)
    sleep_ms(200)
    curve_road(vehicle, reverse_direction=False)
    sleep_ms(200)
    fork(vehicle, turn_left=True)
    sleep_ms(200)
    straight(vehicle)
    sleep_ms(200)
    roundabout(vehicle, 2)
    sleep_ms(200)
    straight(vehicle)
    sleep_ms(200)
    gentle_curve(vehicle, turn_left=False)
    sleep_ms(200)


if __name__ == "__main__":
    # sleep_ms(1000)
    # complete_basic_track()

    main(SPLASH_SCREEN)
