import peripherals
from time import sleep

io = peripherals.Peripherals()
screen = io.screen
rgb_sensor = io.rbg_sensor

screen.print_ascii_art("rgb sensor demo\n\n    _,,/|\n    \\o o'\n    =_~_=\n    /   \\ (\\\n   (////_)//\n   ~~~")
sleep(5)
while True:
    proximity = str(rgb_sensor.proximity())
    clear = str(rgb_sensor.ambient())
    red = str(rgb_sensor.red())
    green = str(rgb_sensor.green())
    blue = str(rgb_sensor.blue())
    screen.print("proximity: " + proximity + "\nclear: " + clear + "\nred: " + red +
                 "\ngreen: " + green + "\nblue: " + blue)
    sleep(0.5)
