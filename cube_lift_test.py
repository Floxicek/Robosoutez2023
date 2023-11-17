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
from pybricks.media.ev3dev import Font


max_wheel_speed = 300
wheel_diameter = 53.9
axle_track = 193

max_lift_speed = 1500
LIFT_ROT = 900


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)

us = UltrasonicSensor(Port.S2)


ev3.screen.set_font(Font(size=18))

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

robot.settings(max_wheel_speed, 1000000, 90, 90)


ev3.screen.set_font(Font(size=15))
ev3.screen.print("Control motor with\n arrow up or down")

lift_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
# gyro_sensor = GyroSensor(Port.S3)


## Lift motor INIT
ev3.screen.print("Reset belt motor rotation")
lift_motor.run_until_stalled(max_lift_speed, Stop.BRAKE)
lift_motor.run_until_stalled(-max_lift_speed, Stop.BRAKE)

wait(300)

lift_motor.reset_angle(0)


# Initialize functions
def elevatorUp():
    lift_motor.run_target(max_lift_speed, LIFT_ROT)


def elevatorDown():
    lift_motor.run_target(max_lift_speed, 0)
    ev3.screen.print("Lift motor down")
    ev3.screen.print("{angle} / 0".format(angle=lift_motor.angle()))


def dropCube():
    lift_motor.run_until_stalled(max_lift_speed, Stop.BRAKE, 50)


robot.straight(-500)
elevatorUp()
robot.drive(max_wheel_speed, 0)
wait(3000)
robot.stop()
elevatorDown()
robot.drive(max_wheel_speed, 0)
wait(100)
elevatorUp()
robot.stop()
wait(300)
