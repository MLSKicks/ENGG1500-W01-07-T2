from APDS9960LITE import APDS9960LITE
from math import log

# example use of this module:
#   import rgb_sensor
#   from machine import I2C, Pin
#   bus = I2C(0, sda=Pin(), scl=Pin())
#   rgb_sensor = rgb_sensor.Sensor(bus)
#   print("proximity: ", rgb_sensor.proximity_mm())


def rgb_to_hue(r, g, b):
    """calculates a hue from a rgb values ranging between 0 and 255. Algorithm courtesy of shrikanth13 at
    https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/"""
    h = 0
    r /= 255
    g /= 255
    b /= 255

    rgb_min = min(r, g, b)
    rgb_max = max(r, g, b)
    diff = rgb_max - rgb_min

    if diff == 0:
        return 0
    elif rgb_max == r:
        h = 6 + (g - b)/diff
    elif rgb_max == g:
        h = 2 + (b - r)/diff
    elif rgb_max == b:
        h = 4 + (r - g)/diff
    h *= 60
    h %= 360

    return int(h)


class RGB:
    def __init__(self, bus):
        """Initialise RGB Sensor proximity and light sensing"""
        # initialise RBG sensor library
        self.apds9960 = APDS9960LITE(bus)

        # enable proximity sensing
        self.apds9960.prox.enableSensor()

        # enable light sensing
        self.apds9960.als.enableSensor()
        self.apds9960.als.eLightGain = 3  # x64 gain

        # constants
        self.R_ADJUSTMENT = 0
        self.G_ADJUSTMENT = 0
        self.B_ADJUSTMENT = 0
        self.road_sensitivity = 100 # below this value is road

    def proximity_mm(self):
        """WARNING: only reliable for distances <= 50"""
        prox = self.proximity()
        if prox <= 1:
            return 500
        else:
            return log((prox - 1)/255)/(-0.062)

    def proximity(self):
        """return proximity level"""
        return self.apds9960.prox.proximityLevel

    def is_color(self, hue, threshold=10):
        """returns true if the ambient hue matches the given hue"""
        # NOTE: hues lie on a wheel with 360 degrees, therefore computing the smallest difference
        #       between two values involves testing a clockwise and anticlockwise case
        h = self.hue()
        h2 = hue
        dist_a = h - h2  # anticlockwise distance
        dist_b = h2 - h  # clockwise distance

        # if one of the distances is negative, we have crossed the boundary so convert it to a positive distance
        if dist_a < 0:
            dist_a += 360
        if dist_b < 0:
            dist_b += 360

        if min(dist_a, dist_b) < threshold:
            return True

        return False

    def is_color_rgb(self, r, g, b, threshold=10):
        """returns true if the ambient hue matches the hue of r, g, and b (between 0 - 255)"""
        return self.is_color(rgb_to_hue(r, g, b), threshold=threshold)

    def hue(self):
        """returns the hue calculated from ambient light levels"""
        return rgb_to_hue(self.red(), self.green(), self.blue())

    def ambient(self):
        """return ambient light level"""
        return self.apds9960.als.ambientLightLevel

    def red(self):
        """return red light level"""
        return self.apds9960.als.redLightLevel + self.R_ADJUSTMENT

    def green(self):
        """return green light level"""
        return self.apds9960.als.greenLightLevel + self.G_ADJUSTMENT

    def blue(self):
        """return blue light level"""
        return self.apds9960.als.blueLightLevel + self.B_ADJUSTMENT

    def set_road_sensitivity(self, sensitivity):
        self.road_sensitivity = sensitivity

    def get_road_sensitivity(self):
        return self.road_sensitivity

    def is_on_road(self):
        if self.ambient() < self.road_sensitivity:
            return True
        return False

    def is_on_road_by_prox(self):
        if self.proximity() <= 1:
            return True
        return False
