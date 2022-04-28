import vehicle_components
from time import sleep_ms


def ir_sensor_demo():
    # initialise
    vehicle = vehicle_components.Vehicle(init_rgb=False, init_encoder=False, init_motor=False)
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
    vehicle = vehicle_components.Vehicle(init_ir=False)
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


def pid_run(loops, sleep, kp, ki, kd, target_mm):
    vehicle = vehicle_components.Vehicle(init_screen=True, init_rgb=False, init_ir=True, init_us=False)
    pid = vehicle.pid
    pid.reset(kp, ki, kd)
    pid.set_target(target_mm)
    sleep_ms(65)
    for i in range(0, loops, 1):
        lduty, rduty = pid.run()
        vehicle.set_motor(lduty, rduty)
        pid.update_encoder(lduty, rduty)
        print("i = {}, lduty = {}, rduty = {}\nenc_l = {}, enc_r = {}".format(i, lduty, rduty, vehicle.encoder.get_left(), vehicle.encoder.get_right()))
        vehicle.screen.print("i={}\nlduty={}\nrduty={}\nir_road={}".format(i, lduty, rduty,
                                                                           vehicle.ir_sensor.is_on_road()))
        sleep_ms(sleep)
    stop_motor()


def pid_demo():
    vehicle = vehicle_components.Vehicle(init_screen=True, init_rgb=False, init_ir=True, init_us=False)
    pid = vehicle.pid
    sleep_ms(65)
    for i in range(0, 100, 1):
        lduty, rduty = pid.run()
        vehicle.set_motor(lduty, rduty)
        pid.update_encoder(lduty, rduty)
        print("i = {}, lduty = {}, rduty = {}".format(i, lduty, rduty))
        vehicle.screen.print("i={}\nlduty={}\nrduty={}\nir_road={}".format(i, lduty, rduty,
                                                                           vehicle.ir_sensor.is_on_road()))
        sleep_ms(150)
    stop_motor()


def stop_motor():
    vehicle = vehicle_components.Vehicle(init_screen=True, init_rgb=False, init_ir=True, init_us=False)
    vehicle.set_motor(0,0)


def oled_demo():
    vehicle = vehicle_components.Vehicle(init_rgb=False, init_us=False, init_ir=False, init_encoder=False, init_motor=False)
    screen = vehicle.screen
    screen.print("hi")


def main():
    vehicle = vehicle_components.Vehicle(init_rgb=False, init_us=False)

    # enumeration of states
    STATE_PRINT_ART = 0
    STATE_DRIVE_FWD = 1
    STATE_DRIVE_BKWD = 2
    STATE_PRINT_IR = 3

    # initial state
    state = STATE_PRINT_ART
    lduty = 0
    rduty = 0

    while True:
        # collect sensor data

        # make decisions
        if state == STATE_PRINT_ART:
            vehicle.screen.print_ascii_art(
                "State: PRINT_ART\n\n    _,,/|\n    \\o o'\n    =_~_=\n    /   \\ (\\\n   (////_)//\n   ~~~")
            lduty, rduty = 0, 0
            # state transition
            state += 1

        elif state == STATE_DRIVE_FWD:
            vehicle.screen.print("State: DRIVE_FWD")
            lduty, rduty = 50, 50
            # state transition
            state += 1

        elif state == STATE_DRIVE_BKWD:
            vehicle.screen.print("State: DRIVE_BKWD")
            lduty, rduty = -50, -50
            # state transition
            state += 1

        elif state == STATE_PRINT_IR:
            vehicle.screen.print("State: PRINT_IR\n\nIR_Road={}".format(vehicle.ir_sensor.is_on_road()))
            lduty, rduty = 0, 0
            # state transition
            state = STATE_PRINT_ART

        # control motors
        vehicle.set_motor(lduty, rduty)
        sleep_ms(1000)


if __name__ == "__main__":
    sleep_ms(1000)
    pid_run(100, 50, 1.425, 0.00097, 33, 2000)
