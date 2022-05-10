import vehicle_components
from time import sleep_ms


def run_pid_test(target_mm_l, target_mm_r, kp, ki, kd, loops=30, sleep=50):
    vehicle = vehicle_components.Vehicle(init_screen=True, init_ir_l=True, init_encoder=True, init_motor=True)
    pid = vehicle.pid
    pid.reset(target_mm_l, target_mm_r, kp, ki, kd)
    sleep_ms(65)
    for i in range(0, loops, 1):
        lduty, rduty = pid.run()
        vehicle.set_motor(lduty, rduty)
        vehicle.screen.print("i={}\nlduty={}\nrduty={}\nir_road={}".format(i, lduty, rduty,
                                                                           vehicle.ir_l.is_on_road()))
        sleep_ms(sleep)
    vehicle.set_motor(0, 0)


# enumeration of states
STATE_NULL = -1
STATE_PRINT_ART = 0
STATE_DRIVE_FWD = 1
STATE_DRIVE_BKWD = 2
STATE_TURN_LEFT = 3
STATE_TURN_RIGHT = 4
STATE_PRINT_ROAD = 5
STATE_IDLE = 6
STATE_STOP = 7
STATE_LF_FWD = 8
STATE_LF_TURN_LEFT = 9
STATE_LF_TURN_RIGHT = 10


def main(initial_state = -1):
    # - - - - - - - - - - - - - - - - - - - - - - - initialisation - - - - - - - - - - - - - - - - - - - - - - - #
    vehicle = vehicle_components.Vehicle(init_motor=True, init_encoder=True, init_screen=True, init_ir_l=True,
                                         init_ir_r=True, init_us_l=True, init_us_r=True, init_rgb=True)
    pid = vehicle.pid
    screen = vehicle.screen

    # - - - - - - - - - - - - - - - - - - - - - - - initial state - - - - - - - - - - - - - - - - - - - - - - - #
    prev_state = STATE_NULL
    state = initial_state
    lduty = 0
    rduty = 0
    idle_timeout = 0

    while True:
        # - - - - - - - - - - - - - - - - - - - - collect sensor data - - - - - - - - - - - - - - - - - - - - #
        ir_l_onroad = vehicle.ir_l.is_on_road()
        ir_r_onroad = vehicle.ir_r.is_on_road()
        rgb_onroad = vehicle.rgb.is_on_road_by_prox()
        ambient = vehicle.rgb.ambient()
        rgb_hue = vehicle.rgb.hue()
        rgb_prox = vehicle.rgb.proximity_mm()
        us_l = vehicle.us_l.proximity()
        us_r = vehicle.us_r.proximity()
        print("us-l = {}, us-r = {}".format(us_l, us_r))
        # - - - - - - - - - - - - - - - - - - - - global transitions - - - - - - - - - - - - - - - - - - - - #
        # if us_l < 15 and us_r < 15:
        #     state = STATE_STOP
        if rgb_prox < 35:  # something is probably on the road... so lets stop
            state = STATE_STOP

        # set transition flag and update prev state (this tells us if we just transitioned, don't worry about it...)
        transition_flag = prev_state != state
        prev_state = state

        # - - - - - - - - - - - - - - - - - - - - state machine body - - - - - - - - - - - - - - - - - - - - #
        if state == STATE_PRINT_ART:
            # state actions
            screen.print_ascii_art(
                "State: PRINT_ART\n\n    _,,/|\n    \\o o'\n    =_~_=\n    /   \\ (\\\n   (////_)//\n   ~~~")
            lduty, rduty = 0, 0
            sleep_ms(1000)
            # state transition
            state = STATE_DRIVE_FWD

        elif state == STATE_DRIVE_FWD:
            # state actions
            screen.print("State: DRIVE_FWD")
            if transition_flag:
                pid.reset_target(100, 100)
            lduty, rduty = pid.run()

            # state transition
            if pid.target_met():
                state = STATE_TURN_LEFT
            if not ir_l_onroad and not ir_r_onroad and rgb_onroad:
                state = STATE_LF_FWD

        elif state == STATE_DRIVE_BKWD:
            # state actions
            screen.print("State: DRIVE_BKWD")
            if transition_flag:
                pid.reset_target(-100, -100)
            lduty, rduty = pid.run()

            # state transition
            if pid.target_met():
                state = STATE_TURN_RIGHT

        elif state == STATE_PRINT_ROAD:
            # state actions
            screen.print("State: PRINT_IR\n\nIR-L_Road={}\nIR-R_Road={}\nRGB_Road={}\nAmb={}\nHue={}\nProx={}".format(
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
            #
            # # state transition
            # if ir_l_onroad and not ir_r_onroad and not rgb_onroad:  # veered even further right
            #     state = STATE_LF_TURN_LEFT
            # if not ir_l_onroad and ir_r_onroad and not rgb_onroad:  # veered even further left
            #     state = STATE_LF_TURN_RIGHT

        elif state == STATE_LF_TURN_LEFT:
            # state actions
            screen.print("State: LF_TURN_LEFT")
            lduty, rduty = 0, 0
            # if transition_flag:
            #     pid.reset_target(70, 100)
            # lduty, rduty = pid.run()
            #
            # # TODO: UPDATE TURNING AUTOMATICALLY???
            #
            # # state transition
            # if (not ir_l_onroad and not ir_r_onroad and rgb_onroad) or pid.target_met:
            #     state = STATE_LF_FWD
            # if not ir_l_onroad and ir_r_onroad and rgb_onroad:
            #     state = STATE_LF_TURN_RIGHT

        elif state == STATE_LF_TURN_RIGHT:
            # state actions
            screen.print("State: LF_TURN_RIGHT")
            lduty, rduty = 0, 0
            # if transition_flag:
            #     pid.reset_target(100, 70)
            # lduty, rduty = pid.run()
            #
            # # TODO: UPDATE TURNING AUTOMATICALLY???
            #
            # # state transition
            # if (not ir_l_onroad and not ir_r_onroad and rgb_onroad) or pid.target_met:
            #     state = STATE_LF_FWD
            # if ir_l_onroad and not ir_r_onroad and rgb_onroad:
            #     state = STATE_LF_TURN_LEFT

        # - - - - - - - - - - - - - - - - - - - - control motors - - - - - - - - - - - - - - - - - - - - #
        vehicle.set_motor(lduty, rduty)


def pid_run(vehicle, left_target, right_target, n=1):
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


def pid_gentle_curve(direction_is_left):
    vehicle = vehicle_components.Vehicle(init_motor=True, init_encoder=True, init_screen=True)
    # direction True = turn left
    if direction_is_left:
        vehicle.screen.print("Gentle Curve\n\nTurning left")
        pid_run(vehicle, 300, 500, 3)

    # direction False = turn right
    else:
        vehicle.screen.print("Gentle Curve\n\nTurning right")
        pid_run(vehicle, 500, 300, 3)

    vehicle.screen.print("Done :)")


def pid_roundabout(about_exit):
    # exit must be between 0 - 3: 0 is uturn, 1 is left, 2 is fwd, 3 is right
    about_exit = (about_exit % 4)
    if about_exit == 0:
        about_exit = 4  # make it travel all the way around
    vehicle = vehicle_components.Vehicle(init_motor=True, init_encoder=True, init_screen=True)

    # travel onto roundabout
    vehicle.screen.print("Round About\n\nGetting on")
    pid_run(vehicle, 40, 40)
    pid_run(vehicle, -80, 80)

    for i in range(0, about_exit, 1):
        # complete the quarter turn
        vehicle.screen.print("Round About\n\nTravelling around {}/{}".format(i, about_exit))
        pid_run(vehicle, 245, 15)  # 434, 214 * 0.75 then hand tuned

    # travel off roundabout
    vehicle.screen.print("Round About\n\nGetting off")
    pid_run(vehicle, -90, 90)

    vehicle.screen.print("Done :)")


if __name__ == "__main__":
    sleep_ms(500)
    pid_roundabout(2)

    sleep_ms(2000)
    pid_gentle_curve(False)

    sleep_ms(2000)
    main(STATE_LF_FWD)
