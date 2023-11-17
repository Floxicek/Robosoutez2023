#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    ColorSensor,
    UltrasonicSensor,
)
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import Font


max_wheel_speed = 300
wheel_diameter = 53.9
axle_track = 193

max_lift_speed = 1500
lift_max_height = 1100
distance_from_wall = 100


# Initialize ports
ev3 = EV3Brick()
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
lift_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
distance_sensor = UltrasonicSensor(Port.S2)
color_sensor = ColorSensor(Port.S3)


ev3.screen.set_font(Font(size=15))

## Lift motor INIT
ev3.screen.print("Reset belt motor rotation")
lift_motor.dc(-80)
wait(2000)
lift_motor.dc(0)
wait(200)
lift_motor.reset_angle(0)
wait(500)


# Initialize functions
def elevatorUp():
    lift_motor.run_target(max_lift_speed, lift_max_height, Stop.HOLD, False)


def elevatorDown():
    lift_motor.run_target(max_lift_speed, 0, Stop.HOLD, False)
    # ev3.screen.print("Lift motor down")
    # ev3.screen.print("{angle} / 0".format(angle=lift_motor.angle()))


def dropCube():
    lift_motor.dc(80)


def move():
    move_err = distance_from_wall - distance_sensor.distance()

    if abs(move_err) > 2 and abs(move_err < 200):
        corr_speed = 2

        left_motor.run(max_wheel_speed - move_err * corr_speed)
        right_motor.run(max_wheel_speed + move_err * corr_speed)


def check_color():
    if cube_detected:
        return False

    measure = color_sensor.rgb()

    if measure[0] > 8 or measure[1] > 8 or measure[2] > 8:
        ev3.light.on(Color.RED)
        ev3.speaker.beep(frequency=900, duration=50)
        cube_passed_clock.reset()
        elevatorUp()
        return True
    return False


cube_passed_clock = StopWatch()
cube_grabbed_clock = StopWatch()
cube_detected = False
cube_grabbed = False

elevatorUp()

while True:
    if check_color():
        cube_detected = True

    if cube_passed_clock.time() > 1500 and cube_detected:
        ev3.light.on(Color.GREEN)
        elevatorDown()
        cube_detected = False
        cube_grabbed = True
        cube_grabbed_clock.reset()
    

    if cube_grabbed_clock > 300 and cube_grabbed:
        elevatorUp
