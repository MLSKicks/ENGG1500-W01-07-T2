import vehicle_components
from time import sleep


def ir_sensor_demo():
    # initialise
    vehicle = vehicle_components.Vehicle(init_rgb=False, init_encoder=False, init_motor=False)
    screen = vehicle.screen
    ir = vehicle.ir_sensor

    # splash screen
    screen.print_ascii_art("ir sensor demo\n\n    _,,/|\n    \\o o'\n    =_~_=\n    /   \\ (\\\n   (////_)//\n   ~~~")
    sleep(1)

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
    sleep(1)

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


def pid_demo():
    vehicle = vehicle_components.Vehicle(init_screen=False, init_rgb=False, init_ir=False, init_us=False)
    pid = vehicle.pid
    for i in range(0, 1000, 1):
        lduty, rduty = pid.run()
        vehicle.set_motor(lduty, rduty)
        print("i = {}, lduty = {}, rduty = {}".format(i, lduty, rduty))
        sleep(0.08)


if __name__ == "__main__":
    pid_demo()
