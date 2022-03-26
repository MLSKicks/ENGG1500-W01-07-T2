from APDS9960LITE import APDS9960LITE


# example use of this module:
#   import rgb_sensor
#   from machine import I2C, Pin
#   bus = I2C(0, sda=Pin(), scl=Pin())
#   rgb_sensor = rgb_sensor.Sensor(bus)
#   print("proximity: ", rgb_sensor.proximity())


class Sensor:
    def __init__(self, bus):
        """Initialise RGB Sensor proximity and light sensing"""
        # initialise RBG sensor library
        self.apds9960 = APDS9960LITE(bus)

        # enable proximity sensing
        self.apds9960.prox.enableSensor()

        # enable light sensing
        self.apds9960.als.enableSensor()
        self.apds9960.als.eLightGain = 3  # x64 gain

    def proximity(self):
        """return proximity level"""
        return self.apds9960.prox.proximityLevel

    def ambient(self):
        """return ambient light level"""
        return self.apds9960.als.ambientLightLevel

    def red(self):
        """return red light level"""
        return self.apds9960.als.redLightLevel

    def green(self):
        """return green light level"""
        return self.apds9960.als.greenLightLevel

    def blue(self):
        """return blue light level"""
        return self.apds9960.als.blueLightLevel
