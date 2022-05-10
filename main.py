from vehicle_components import Vehicle
from time import sleep_ms

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
prev_state = NULL
state = NULL
is_transition = False


def update_state_variables():
    """Updates our previous state (prev_state) variable and transition flag (is_transition).
    The transition flag tells us if we have just transitioned to a new state. This is helpful
    in running a piece of code in a state one and once only."""
    global prev_state, state, is_transition

    is_transition = prev_state != state
    prev_state = state


def print_state(screen):
    """Print out what state we are in"""
    global state

    if state == NULL:
        screen.print("State: NULL\nNo initial state\nwas specified!")
    elif state == SPLASH_SCREEN:
        screen.print_art(ascii_cat)
    elif state == PRINT_ROAD_INFO:
        screen.print("State: Road Info\n")
    elif state == IDLE:
        screen.print("State: Idling\nI am lost!")
    elif state == STOP:
        screen.print("State: Stopped\nMy job is done!")
    elif state == HAZARD:
        screen.print("State: Hazard\nSomething got in\n my way!")
    elif state == LF_FWD:
        screen.print("State: Line Foll\n-owing")
    elif state == LF_TURN_LEFT:
        screen.print("State: Line Foll\n-owing LEFT")
    elif state == LF_TURN_RIGHT:
        screen.print("State: Line Foll\n-owing RIGHT")


def main(initial_state=STATE_NULL):
    """This is our main state machine: a big loop that performs actions based on the current state!

    Initialisation: initialise our Vehicle object which initialises objects for each sensor, controller, etc.

    Sensor Data Collection: collect data from all our sensors

    Global Transitions: these are IMPORTANT TRANSITIONS which can overwrite anything. They are important as they
    are reacting to things like obstacles on the road, in which case we want to stop ASAP!

    State Machine Body: goes through all the nitty-gritty details """
    global prev_state, state, is_transition, ascii_cat

    # - - - - - - - - - - - - - - - - - - - - - - - INITIALISATION - - - - - - - - - - - - - - - - - - - - - - - #
    vehicle = Vehicle(motor=True, enc=True, screen=True, rgb=True, ir_l=True, ir_r=True, us_l=True, us_r=True)
    pid = vehicle.pid          # PID-control object -> sets motor duties to achieve desired targets
    screen = vehicle.screen    # OLED screen object -> print useful information
    state = initial_state      # Set the requested initial state

    while True:
        # - - - - - - - - - - - - - - - - - - - - SENSOR DATA COLLECTION - - - - - - - - - - - - - - - - - - #
        ir_l_onroad = vehicle.ir_l.is_on_road()
        ir_r_onroad = vehicle.ir_r.is_on_road()
        rgb_onroad = vehicle.rgb.is_on_road_by_prox()
        ambient = vehicle.rgb.ambient()
        rgb_hue = vehicle.rgb.hue()
        rgb_prox = vehicle.rgb.proximity_mm()
        us_l = vehicle.us_l.proximity()
        us_r = vehicle.us_r.proximity()

        # - - - - - - - - - - - - - - - - - - - - GLOBAL TRANSITIONS - - - - - - - - - - - - - - - - - - - - #
        if rgb_prox < 35:  # Something is on the road or obstructing the sensor... so lets stop
            state = HAZARD

        # - - - - - - - - - - - - - - - - - - - - STATE MACHINE HEADER - - - - - - - - - - - - - - - - - - - #
        update_state_variables()
        print_state(screen)

        # - - - - - - - - - - - - - - - - - - - - STATE MACHINE BODY - - - - - - - - - - - - - - - - - - - - #
        if state == SPLASH_SCREEN:
            pid.set_target(0, 0)
            # state transition
            state = STATE_DRIVE_FWD

        elif state == PRINT_ROAD_INFO:
            # state actions
            screen.print_unformatted("IR-L_Road={}\nIR-R_Road={}\nRGB_Road={}\nAmb={}\nHue={}\nProx={}".format(
                ir_l_onroad, ir_r_onroad, rgb_onroad, ambient, rgb_hue, rgb_prox))
            lduty, rduty = 0, 0
            sleep_ms(250)

        elif state == STATE_IDLE:
            # state actions
            screen.print("State: IDLE")
            if transition_flag:
                pid.reset_target(50, 50)
                idle_timeout = 200
            if pid.target_met:
                pid.reset_target(50, 50)
            lduty, rduty = pid.run()

            # state transition
            if ir_l_onroad():
                state = STATE_PRINT_ART
            elif idle_timeout <= 0:
                state = STATE_STOP
            else:
                idle_timeout -= 1

        elif state == STATE_STOP:
            screen.print("State: Stopped!")
            lduty, rduty = 0, 0

        elif state == STATE_LF_FWD:
            # state actions
            screen.print("State: LF_FWD")
            if transition_flag or pid.target_met:
                pid.reset_target(50, 50)

            # if we have veered off the road slightly, lets fix it
            if ir_l_onroad and not ir_r_onroad:  # we have veered right
                screen.print("State: LF_FWD\nveering right")
                pid.add_target(-15, 15)
            if not ir_l_onroad and ir_r_onroad:  # we have veered left
                screen.print("State: LF_FWD\nveering left")
                pid.add_target(15, -15)

            lduty, rduty = pid.run()

        # - - - - - - - - - - - - - - - - - - - - CONTROL MOTORS - - - - - - - - - - - - - - - - - - - - #
        vehicle.set_motor(lduty, rduty)


def test_pid(target_mm_l, target_mm_r, kp, ki, kd, loops=30, sleep=50):
    """Test different proportionality constants for pid"""
    vehicle = Vehicle(screen=True, ir_l=True, enc=True, motor=True)
    vehicle.pid.reset(target_mm_l, target_mm_r, kp, ki, kd)
    for i in range(0, loops):
        vehicle.set_motor(*vehicle.pid.run())
        sleep_ms(sleep)
    vehicle.set_motor(0, 0)


def run_pid(vehicle, left_target, right_target, n=1):
    # divide the targets into n steps or segments
    left_target_step = left_target/n
    right_target_step = right_target/n

    # set our initial target
    vehicle.pid.reset_target(left_target_step, right_target_step)

    # once each step is done, add the next step, until we are done!
    for i in range(0, n, 1):
        while not vehicle.pid.target_met():
            lduty, rduty = vehicle.pid.run()
            vehicle.set_motor(lduty, rduty)
            sleep_ms(50)
        vehicle.pid.add_target(left_target_step, right_target_step)

    # done, so stop motors
    vehicle.set_motor(0, 0)


def gentle_curve(turn_left=False, turn_right=False):
    vehicle = Vehicle(motor=True, enc=True, screen=True)
    # direction True = turn left
    if turn_left:
        vehicle.screen.print("Gentle Curve\n\nTurning left")
        run_pid(vehicle, 300, 500, 3)

    # direction False = turn right
    if turn_right:
        vehicle.screen.print("Gentle Curve\n\nTurning right")
        run_pid(vehicle, 500, 300, 3)

    vehicle.screen.print("Done :)")


def roundabout(about_exit):
    # exit must be between 0 - 3: 0 is uturn, 1 is left, 2 is fwd, 3 is right
    about_exit = (about_exit % 4)
    if about_exit == 0:
        about_exit = 4  # make it travel all the way around
    vehicle = Vehicle(motor=True, enc=True, screen=True)

    # travel onto roundabout
    vehicle.screen.print("Round About\n\nGetting on")
    run_pid(vehicle, 40, 40)
    run_pid(vehicle, -80, 80)

    for i in range(0, about_exit, 1):
        # complete the quarter turn
        vehicle.screen.print("Round About\n\nTravelling around {}/{}".format(i, about_exit))
        pid_run(vehicle, 245, 15)  # 434, 214 * 0.75 then hand tuned

    # travel off roundabout
    vehicle.screen.print("Round About\n\nGetting off")
    run_pid(vehicle, -90, 90)

    vehicle.screen.print("Done :)")


if __name__ == "__main__":
    sleep_ms(500)
    roundabout(2)

    sleep_ms(2000)
    gentle_curve(turn_right=True)

    sleep_ms(2000)
    main(STATE_LF_FWD)
