from machine import Pin, time_pulse_us
from time import sleep_us, ticks_ms, ticks_diff
from array import array


class UltraSonic:
    """
    A ``us_sensor`` object is used to control a HCSR04 ultrasonic sensor.
    """

    def __init__(self, trig, echo):
        """
        - trigger_pin: Output pin to send pulses
        - echo_pin: Input pin listens for reflection to estimate time of flight.
        """
        # Initialise constants
        self.NUM_READINGS = 6  # CONST: number of readings to average over (denoise sensor data)
        self.MAX_TIMEDIFF_MS = 1000  # CONST: max elapsed time (ms) before self.readings is too old (therefore invalid)
        self.ECHO_TIMEOUT_US = int(4000*2/(340.29*1e-3))  # CONST: note 4000mm is the max reasonable range of sensor
        self.SPEED_SOUND = 340.29  # CONST: m/s, for calculating distances

        # Initialise variables
        self.readings = array('d', (0 for _ in range(self.NUM_READINGS)))  # array holding previous readings
        self.t0 = ticks_ms()  # records reference time
        self.total = 0  # records sum of readings array
        self.read_index = 0  # records the read index for the readings array

        # Initialise the TRIG output pin, and ECHO input pin
        self.trigger = Pin(trig, mode=Pin.OUT)
        self.trigger.value(0)
        self.echo = Pin(echo, mode=Pin.IN)

        # Initialise readings array
        self.reset_sensor()

    def distance_mm(self):
        """
        Estimate distance to obstacle in front of the ultrasonic sensor in mm.
        Sends a 10us pulse to 'trigger' pin and listens on 'echo' pin.
        We use the method `machine.time_pulse_us()` to count the microseconds
        passed before the echo is received.
        The time of flight is used to calculate the estimated distance.
        """
        # Send a 10us HIGH pulse to trigger the ultrasonic burst
        self.trigger.value(1)
        sleep_us(10)
        self.trigger.value(0)
        # Read length of time pulse
        duration = time_pulse_us(self.echo, 1, self.ECHO_TIMEOUT_US)
        # Calculate the distance in mm, based on the delay before we hear
        # an echo and the constant speed of sound.
        # Note: duration is halved as the audio wave must travel there and back.
        mm = 0.5 * duration * 1e-6 * self.SPEED_SOUND * 1e3
        return mm

    def reset_sensor(self):
        """reset the sensor reading array with new values. Warning: this could take
        up to ~250ms with default ECHO_TIMEOUT_US, NUM_READINGS, and MAX_TIMEDIFF_MS"""
        self.total = 0
        self.read_index = 0
        for i in range(0, self.NUM_READINGS, 1):
            self.readings[i] = self.distance_mm()
            self.total += self.readings[i]
        self.t0 = ticks_ms()

    def proximity(self):
        """uses smoothing algorithm by David A. Mellis and Tom Igoe https://www.arduino.cc/en/Tutorial/Smoothing"""
        # if our most recent reading is too old, redo the whole array
        # NOTE: that the oldest reading can be MAX_TIMEDIFF_MS*NUM_READINGS = 1 second old
        if ticks_diff(ticks_ms(), self.t0) >= self.MAX_TIMEDIFF_MS:
            print("ultrasonic sensor: resetting old values in readings array!")
            self.reset_sensor()

        # remove the oldest reading
        self.total -= self.readings[self.read_index]

        # get a new reading, taking note to update the reference time
        self.t0 = ticks_ms()
        self.readings[self.read_index] = self.distance_mm()
        self.total += self.readings[self.read_index]

        # update read_index and ensure it loops back to zero
        self.read_index = (self.read_index + 1) % self.NUM_READINGS

        # return the average of all the readings!
        return self.total / self.NUM_READINGS
