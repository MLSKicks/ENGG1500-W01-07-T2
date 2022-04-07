# program to drive a robot that finds a black square in a box
from machine import Pin
from time import sleep
from motor import Motor
from ultrasonic import sonic

# settings
US_SENSOR_READINGS = 3 # ultrasonic sensor is averaged over this many readings
PAUSE_LENGTH = 0.5 # sleep length for transitions between different tasks

SCOUT_ROTATION_DUTY = 50 # motor duty for scout rotations
SCOUT_TOLERANCE = 5 # tolerance for finding correct distance
SCOUT_ROTATION_TIME = 0.030
SCOUT_ROTATION_SLEEP = 0
WALL_FOLLOW_TOLERANCE = 15
WALL_STOP_DISTANCE = 90 # distance bot will stop from a wall
WALL_FOLLOW_DISTANCE = WALL_STOP_DISTANCE

# initialise sensors and motors
IRSensor = Pin(26, Pin.IN)
USSensor = sonic(3, 2)
RightUSSensor = sonic(5, 4)
leftMotor = Motor("left", 8, 9, 6)
rightMotor = Motor("right", 10, 11, 7)


def stop_motor():
    leftMotor.duty(0)
    rightMotor.duty(0)


# function for quickly setting both motors
def set_motor(lduty, rduty):
    # sanitise input (0 <= left duty & right duty <= 100)
    if lduty > 100:
        lduty = 100
    elif lduty < -100:
        lduty = -100
    if rduty > 100:
        rduty = 100
    elif rduty < -100:
        rduty = -100
    # a negative duty means rotate backwards
    if lduty > 0:
        leftMotor.set_forwards()
        leftMotor.duty(lduty)
    else:
        leftMotor.set_backwards()
        leftMotor.duty(lduty*-1)
    if rduty > 0:
        rightMotor.set_forwards()
        rightMotor.duty(rduty)
    else:
        rightMotor.set_backwards()
        rightMotor.duty(rduty*-1)


# function for getting ultrasonic sensor reading (averaged over 3 values)
def get_distance(us_sensor):
    reading_sum = 0
    for x in range(0, US_SENSOR_READINGS):
        reading_sum += us_sensor.distance_mm()
    return reading_sum/US_SENSOR_READINGS


# estimate a 90 degree right turn
def rotate_right():
    set_motor(-50, 50)
    sleep(0.45)
    set_motor(0, 0)


# sequence scouts then orients the bot towards the closest wall
def rotate_to_closest_wall():
    # scout out distances (rotate alot) and record furthest
    success = False
    sleep(PAUSE_LENGTH)
    distances = []
    distances.clear() # in case?? idk why
    for i in range(1, 80):
        if IRSensor.value() == 1:
            halt()
        set_motor(-SCOUT_ROTATION_DUTY, SCOUT_ROTATION_DUTY)
        sleep(SCOUT_ROTATION_TIME)
        set_motor(0, 0)
        sleep(SCOUT_ROTATION_SLEEP)
        distances.append( get_distance(USSensor) )

    # find the largest distance
    sleep(PAUSE_LENGTH)
    farDist = distances[0]
    closeDist = distances[0]
    for dist in distances:
        if dist < closeDist:
            closeDist = dist
        if dist > farDist:
            farDist = dist

    # rotate until looking at furthest within a tolerance
    for i in range(1, 150):
        if IRSensor.value() == 1:
            halt()
        set_motor(-SCOUT_ROTATION_DUTY, SCOUT_ROTATION_DUTY)
        sleep(SCOUT_ROTATION_TIME)
        set_motor(0, 0)
        sleep(SCOUT_ROTATION_SLEEP)
        dist = get_distance(USSensor)
        if dist < closeDist+SCOUT_TOLERANCE:
            success = True
            break
    set_motor(0, 0)
    sleep(PAUSE_LENGTH)
    return success

"""def straight_travel():
    # travel to the wall staying parallel
    side_wall = get_distance(RightUSSensor)
    while get_distance(USSensor) > WALL_STOP_DISTANCE:
        if IRSensor.value() == 1:
            halt()
        set_motor(50, 50)
        sleep(0.15)
        set_motor(0, 0)
        #wallDistance = get_distance(RightUSSensor)
        #if wallDistance < side_wall - WALL_FOLLOW_TOLERANCE:
        #    veer_right()
        #elif wallDistance > side_wall + WALL_FOLLOW_TOLERANCE:
        #    veer_left()
        sleep(0.08)
    set_motor(0, 0)
    sleep(PAUSE_LENGTH)"""

def straight_travel():
    lastWallDistance = get_distance(RightUSSensor)
    while get_distance(USSensor) > WALL_STOP_DISTANCE:
        if IRSensor.value() == 1:
            halt()
        set_motor(50, 50)
        sleep(0.05)
        set_motor(0, 0)
        wallDistance = get_distance(RightUSSensor)
        if (lastWallDistance - wallDistance) > 25:
            rotate_veer_right()
        elif (lastWallDistance - wallDistance) < -25:
            rotate_veer_left()
        lastWallDistance = wallDistance
        sleep(0.01)
    set_motor(0, 0)


def veer_right():
    set_motor(-50,50)
    sleep(0.10)
    set_motor(50,50)
    sleep(0.1)
    set_motor(50,-50)
    sleep(0.12)
    set_motor(0,0)
    sleep(0.08)

def veer_left():
    set_motor(50,-50)
    sleep(0.10)
    set_motor(50,50)
    sleep(0.05)
    set_motor(-50,50)
    sleep(0.12)
    set_motor(0,0)
    sleep(0.08)


def rotate_veer_right():
    set_motor(-50, 50)
    sleep(0.085)
    set_motor(0, 0)


def rotate_veer_left():
    set_motor(50, -50)
    sleep(0.085)
    set_motor(0, 0)


def follow_wall():
    lastWallDistance = get_distance(RightUSSensor)
    while get_distance(USSensor) > WALL_STOP_DISTANCE:
        if IRSensor.value() == 1:
            halt()
        set_motor(50, 50)
        # sleep(0.15)
        sleep(0.05)
        set_motor(0, 0)
        wallDistance = get_distance(RightUSSensor)
        if (lastWallDistance - wallDistance) > 8:
            rotate_veer_right()
        elif (lastWallDistance - wallDistance) < -10: # kinda want to veer left less
            rotate_veer_left()
        elif wallDistance > WALL_FOLLOW_DISTANCE + WALL_FOLLOW_TOLERANCE:
            veer_left()
        # important to not crash into the wall
        if wallDistance < WALL_FOLLOW_DISTANCE - WALL_FOLLOW_TOLERANCE:
            veer_right()

        lastWallDistance = wallDistance
        # sleep(0.08)
        sleep(0.006)
    set_motor(0, 0)


def rewind():
    while get_distance(USSensor) < WALL_FOLLOW_DISTANCE:
        if IRSensor.value() == 1:
            halt()
        set_motor(-45, -45)
    set_motor(0,0)


def rotate_right_2():
    while get_distance(USSensor) < WALL_FOLLOW_DISTANCE:
        if IRSensor.value() == 1:
            halt()
        set_motor(-30, -60)
    sleep(0.09)
    set_motor(-60,-30)
    sleep(0.1)
    set_motor(0,0)
    sleep(0.01)
    set_motor(-50, 50)
    sleep(0.15)
    set_motor(45, 45)
    sleep(0.07)
    rotate_right()


def halt():
    set_motor(0, 0)
    while True:
        sleep(0.5)


while not rotate_to_closest_wall():
    SCOUT_TOLERANCE += 1 # increase tolerance so we eventually do something
    sleep(0)
straight_travel()
sleep(0.1)
rotate_right()
while True:
    follow_wall()
    sleep(0.5)
    rotate_right_2()
    #rewind()
    #sleep(0.5)
    #rotate_right()
    sleep(0.5)
