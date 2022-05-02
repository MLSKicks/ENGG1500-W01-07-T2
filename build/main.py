import vehicle_components
from time import sleep_ms, time
from PID import PID


def ir_sensor_demo():
    # initialise
    vehicle = vehicle_components.Vehicle(init_ir=True, init_screen=True)
    screen = vehicle.screen
    ir = vehicle.ir_sensor

    # splash screen
    screen.print_ascii_art("ir sensor demo\n\n    _,,/|\n    \\o o'\n    =_~_=\n    /   \\ (\\\n   (////_)//\n   ~~~")
    sleep_ms(1000)

    # calibrate
    vehicle.calibrate_ir(ir, 1)

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
    vehicle = vehicle_components.Vehicle(init_screen=True, init_rgb=True, init_us=True)
    screen = vehicle.screen
    rgb = vehicle.rgb_sensor
    us = vehicle.us_sensor

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
    vehicle = vehicle_components.Vehicle(init_screen=True, init_ir=True, init_encoder=True, init_motor=True)
    pid = vehicle.pid
    pid.reset(target_mm_l, target_mm_r, kp, ki, kd)
    sleep_ms(65)
    for i in range(0, loops, 1):
        lduty, rduty = pid.run()
        vehicle.set_motor(lduty, rduty)
        # pid.update_encoder()
        # print("i = {}, lduty = {}, rduty = {}\nenc_l = {}, enc_r = {}".format(i, lduty, rduty, vehicle.encoder.get_left(), vehicle.encoder.get_right()))
        vehicle.screen.print("i={}\nlduty={}\nrduty={}\nir_road={}".format(i, lduty, rduty,
                                                                           vehicle.ir_sensor.is_on_road()))
        sleep_ms(sleep)
    vehicle.set_motor(0, 0)


def stop_motor():
    vehicle = vehicle_components.Vehicle(init_motor=True)
    vehicle.set_motor(0, 0)


def oled_demo():
    vehicle = vehicle_components.Vehicle(init_screen=True)
    screen = vehicle.screen
    screen.print("hi")


def main():
    vehicle = vehicle_components.Vehicle(init_motor=True, init_encoder=True, init_screen=True, init_ir=True)
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
    transition_flag = True

    # initial state
    prev_state = STATE_NULL
    state = STATE_PRINT_ART
    lduty = 0
    rduty = 0

    while True:
        # collect sensor data

        # set transition flag
        transition_flag = False
        if prev_state != state:
            transition_flag = True

        # update prev_state
        prev_state = state

        # state machine
        if state == STATE_PRINT_ART:
            # state actions
            screen.print_ascii_art(
                "State: PRINT_ART\n\n    _,,/|\n    \\o o'\n    =_~_=\n    /   \\ (\\\n   (////_)//\n   ~~~")
            lduty, rduty = 0, 0

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

        elif state == STATE_DRIVE_BKWD:
            # state actions
            screen.print("State: DRIVE_BKWD")
            if transition_flag:
                pid.reset_target(-100, -100)
            lduty, rduty = pid.run()

            # state transition
            if pid.target_met():
                state = STATE_TURN_RIGHT

        elif state == STATE_TURN_LEFT:
            # state actions
            screen.print("State: TURN_LEFT")
            if transition_flag:
                pid.reset_target(157, 377)
            lduty, rduty = pid.run()

            # state transition
            if pid.target_met():
                state = STATE_DRIVE_BKWD

        elif state == STATE_TURN_RIGHT:
            # state actions
            screen.print("State: TURN_RIGHT")
            if transition_flag:
                pid.reset_target(377, 157)
            lduty, rduty = pid.run()

            # state transition
            if pid.target_met():
                state = STATE_PRINT_IR

        elif state == STATE_PRINT_IR:
            # state actions
            is_on_road = vehicle.ir_sensor.is_on_road()
            screen.print("State: PRINT_IR\n\nIR_Road={}".format(is_on_road))
            lduty, rduty = 0, 0
            # state transition
            if is_on_road:
                state = STATE_PRINT_ART
            else:
                state = STATE_IDLE

        elif state == STATE_IDLE:
            # state actions
            screen.print("State: IDLE")
            if transition_flag or pid.target_met():
                pid.reset_target(150, 150)
            counter = 0
            while counter < 60:
                lduty, rduty =  pid.run()
                counter += 1
            if counter >= 60:
                lduty, rduty = 0, 0
                while True: print("Help Me!") # Requires Human Intervention

            # state transition
            if vehicle.ir_sensor.is_on_road():
                state = STATE_PRINT_ART
            


        # control motors
        vehicle.set_motor(lduty, rduty)
        sleep_ms(1000)


if __name__ == "__main__":
    sleep_ms(1000)
    main()
