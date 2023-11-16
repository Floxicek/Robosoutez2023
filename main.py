#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    UltrasonicSensor,
    GyroSensor,
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase


MAX_SPEED = 1500
WHEEL_DIAMETER = 42
AXLE_TRACK = 165


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
belt_motor = Motor(Port.D)

distance_sensor = UltrasonicSensor(Port.S3)

ev3.screen.set_font(Font(size=18))

ev3.screen.print("Reset belt motor rotation")
belt_motor.run_until_stalled(-MAX_SPEED, Stop.BRAKE)
reset_angle(0)

ev3.screen.print("Reset completed")


def liftCube():
    belt_motor.run_angle(max_speed, 10000, wait=False)


def liftDown():
    belt_motor.run_target(MAX_SPEED, 0)
    # without smooth acceleration track_target(target_angle)

    # belt_motor.run_angle(-max_speed, 10000, wait=False)


def dropCube():
    belt_motor.run_time(1000, MAX_SPEED)
    belt_motor.brake()


def measureDistance():
    ev3.screen.print(distance_sensor.distance())


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

robot.settings(max_speed, 1000000, 90, 90)
# Go forward and backwards for one meter.
robot.straight(1000)
robot.straight(-1000)


# Turn clockwise by 360 degrees and back again.
robot.turn(360)
robot.turn(-360)


ev3.speaker.beep()
