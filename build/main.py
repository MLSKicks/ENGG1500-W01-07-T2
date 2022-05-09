import vehicle_components
from time import sleep_ms, ticks_ms, ticks_diff


def ir_sensor_demo():
    # initialise
    vehicle = vehicle_components.Vehicle(init_ir_l=True, init_screen=True)
    screen = vehicle.screen
    ir = vehicle.ir_l

    # splash screen
    screen.print_ascii_art("ir sensor demo\n\n    _,,/|\n    \\o o'\n    =_~_=\n    /   \\ (\\\n   (////_)//\n   ~~~")
    sleep_ms(1000)

    # calibrate
    vehicle.calibrate_ir(ir, 1)
    vehicle

    # print whether ir_sensor detects road or off-road
    while True:
        ir_reading = str(ir.reading())
        road_str = "detecting OFF-road"
        if ir.is_on_road():
            road_str = "detecting road"
        screen.print("ir: " + ir_reading + "\n\n" + road_str)
        vehicle.update_sleep(100)


def rgb_sensor_demo():
    # initialise
    vehicle = vehicle_components.Vehicle(init_screen=True, init_rgb=True, init_us_l=True)
    screen = vehicle.screen
    rgb = vehicle.rgb
    us = vehicle.us_l

    # splash screen
    screen.print_ascii_art("rgb sensor demo\n\n    _,,/|\n    \\o o'\n    =_~_=\n    /   \\ (\\\n   (////_)//\n   ~~~")
    sleep_ms(1000)

    # run calibration? vehicle.calibrate_rgb()

    # print rgb_sensor readings vs us_sensor
    while True:
        if rgb.is_color(0, threshold=20):
            screen.print("RED")
        else:
            screen.print("rgb prox: ", rgb.proximity(),
                         "\nrgb mm prox: {:.0f}".format(rgb.proximity_mm()),
                         "\nus prox: {:.0f}".format(us.proximity()),
                         "\nclr: ", rgb.ambient(), " r: " + rgb.red(),
                         "\ng: ", rgb.green(), " b: " + rgb.blue(),
                         "\nhue: ", rgb.hue())
        vehicle.update_sleep(300)


def run_pid(target_mm_l, target_mm_r, kp, ki, kd, loops=30, sleep=50):
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


def stop_motor():
    vehicle = vehicle_components.Vehicle(init_motor=True)
    vehicle.set_motor(0, 0)


def oled_demo():
    vehicle = vehicle_components.Vehicle(init_screen=True)
    screen = vehicle.screen
    screen.print("hi")


# noinspection PyPep8Naming
def main():
    vehicle = vehicle_components.Vehicle(init_motor=True, init_encoder=True, init_screen=True, init_ir_l=True,
                                         init_ir_r=True, init_us_l=True, init_us_r=True, init_rgb=True)
    pid = vehicle.pid
    screen = vehicle.screen

    # enumeration of states
    STATE_NULL = -1
    STATE_PRINT_ART = 0
    STATE_DRIVE_FWD = 1
    STATE_DRIVE_BKWD = 2
    STATE_TURN_LEFT = 3
    STATE_TURN_RIGHT = 4
    STATE_PRINT_IR = 5
    STATE_IDLE = 6
    STATE_HELP = 7
    STATE_LF = 8

    # initial state
    prev_state = STATE_NULL
    state = STATE_PRINT_ART
    lduty = 0
    rduty = 0
    idle_timeout = 0

    while True:
        # collect sensor data
        t0 = ticks_ms()
        ir_l_onroad = vehicle.ir_l.is_on_road()
        ir_r_onroad = vehicle.ir_r.is_on_road()
        rgb_onroad = vehicle.rgb.is_on_road()
        rgb_hue = vehicle.rgb.hue()
        rgb_prox = vehicle.rgb.proximity_mm()
        us_l = vehicle.us_l.proximity()
        us_r = vehicle.us_r.proximity()

        # set transition flag and update prev state
        transition_flag = prev_state != state
        prev_state = state

        # state machine body
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
            if (not ir_l_onroad and not ir_r_onroad and rgb_onroad):
                state = STATE_LF

        elif state == STATE_DRIVE_BKWD:
            # state actions
            screen.print("State: DRIVE_BKWD")
            if transition_flag:
                pid.reset_target(-100, -100)
            lduty, rduty = pid.run()

            # state transition
            if pid.target_met():
                state = STATE_TURN_RIGHT

        elif state == STATE_PRINT_IR:
            # state actions
            screen.print("State: PRINT_IR\n\nIR-L_Road={}\nIR-R_Road={}".format(ir_l_onroad, ir_r_onroad))
            lduty, rduty = 0, 0
            sleep_ms(1000)
            # state transition
            if ir_l_onroad:
                state = STATE_PRINT_ART
            else:
                state = STATE_IDLE

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
                state = STATE_HELP
            else:
                idle_timeout -= 1

        elif state == STATE_HELP:
            screen.print("State: HELP!")
            lduty, rduty = 0, 0
            if ir_l_onroad:
                state = STATE_PRINT_ART
                
        #TODO: NICH TEST PLS
        elif state == STATE_LF:
            # state actions
            screen.print("State: IR FORWARD")
            if transition_flag or pid.target_met:
                if not ir_l_onroad and not ir_r_onroad:
                    pid.reset_target(100,100)
                elif ir_l_onroad and not ir_r_onroad:
                    pid.reset_target(90,60)
                elif not ir_l_onroad and ir_r_onroad:
                    pid.reset_target(60,90)       
            lduty, rduty = pid.run()

            # state transition
            if ir_l_onroad and ir_r_onroad:
                state = STATE_IDLE
            

        # control motors
        vehicle.set_motor(lduty, rduty)
        print("time diff is {}".format(ticks_diff(ticks_ms(), t0)))


if __name__ == "__main__":
    sleep_ms(1000)
    main()








# elif state == STATE_TURN_LEFT:
#             # state actions
#             screen.print("State: TURN_LEFT")
#             if transition_flag:
#                 pid.reset_target(157, 377)
#             lduty, rduty = pid.run()

#             # state transition
#             if pid.target_met():
#                 state = STATE_DRIVE_BKWD

#         elif state == STATE_TURN_RIGHT:
#             # state actions
#             screen.print("State: TURN_RIGHT")
#             if transition_flag:
#                 pid.reset_target(377, 157)
#             lduty, rduty = pid.run()

#             # state transition
#             if pid.target_met():
#                 state = STATE_PRINT_IR


# mark code
# elif state == STATE_LF_TURN_LEFT:
#             # state actions
#             screen.print("State: IR TURN LEFT")
#             if transition_flag:
#                 pid.reset_target(70,100)
#             lduty, rduty = pid.run() 

#             #TODO: UPDATE TURNING AUTOMATICALLY???

#             # state transition
#             if (not ir_l_onroad and not ir_r_onroad and rgb_onroad) or pid.target_met:
#                 state = STATE_LF_FWD
#             if not ir_l_onroad and ir_r_onroad and rgb_onroad:
#                 state = STATE_LF_TURN_RIGHT

#         elif state == STATE_LF_TURN_RIGHT:
#             # state actions
#             screen.print("State: IR TURN RIGHT")
#             if transition_flag:
#                 pid.reset_target(100,70)
#             lduty, rduty = pid.run() 
            
#             #TODO: UPDATE TURNING AUTOMATICALLY???

#             # state transition
#             if (not ir_l_onroad and not ir_r_onroad and rgb_onroad) or pid.target_met:
#                 state = STATE_LF_FWD
#             if ir_l_onroad and not ir_r_onroad and rgb_onroad:
#                 state = STATE_LF_TURN_LEFT