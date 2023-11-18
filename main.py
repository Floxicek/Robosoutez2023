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


max_wheel_speed = 200
wheel_diameter = 53.9
axle_track = 193

max_lift_speed = 20000
lift_max_height = 1200
distance_from_wall = 200

deposit_rotate_speed = 100


cube_passed_clock = StopWatch()
cube_passed_clock.pause()
cube_passed_clock.reset()

block_wait_time = 500

number_of_cubes = 0

# Initialize ports
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
lift_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
distance_sensor = UltrasonicSensor(Port.S2)
color_sensor = ColorSensor(Port.S3)
ev3.screen.set_font(Font(size=12))
ev3.screen.print("Reset belt motor rotation")
lift_motor.dc(-80)
wait(2000)
lift_motor.dc(0)
wait(500)
lift_motor.reset_angle(0)
wait(100)


def liftUp():
    lift_motor.run_target(max_lift_speed, lift_max_height, Stop.HOLD, False)


def liftDown(wait=False):
    lift_motor.run_target(max_lift_speed, 0, Stop.HOLD, wait)
    # ev3.screen.print("Lift motor down")
    # ev3.screen.print("{angle} / 0".format(angle=lift_motor.angle()))


def dropCube():
    lift_motor.run_target(max_lift_speed, 1500, Stop.HOLD, False)


def move():
    move_err = distance_from_wall - distance_sensor.distance()

    if abs(move_err) > 3 and abs(move_err < 200):
        corr_speed = 1.2

        left_motor.run(max_wheel_speed + move_err * corr_speed)
        right_motor.run(max_wheel_speed - move_err * corr_speed)


def check_color():
    global number_of_cubes
    if cube_passed_clock.time() <= 0:
        measure = color_sensor.rgb()

        if (
            measure[0] > 8 or measure[1] > 8 or measure[2] > 8
        ) and lift_motor.speed() == 0:
            ev3.light.on(Color.RED)
            ev3.speaker.beep(frequency=900, duration=300)
            cube_passed_clock.reset()
            cube_passed_clock.resume()
            number_of_cubes = number_of_cubes + 1
            print(number_of_cubes)
            return True
    else:
        return False


liftUp()

while True:
    if number_of_cubes == 4:
        dropCube()
        wait(200)
        left_motor.run(-deposit_rotate_speed)
        right_motor.run(deposit_rotate_speed)
        wait(7000)
        right_motor.run(-deposit_rotate_speed)
        wait(7000)
        left_motor.stop()
        right_motor.stop()
        liftDown(True)
        break

    else:
        # move()
        check_color()

        first_cube = number_of_cubes == 1

        if cube_passed_clock.time() > block_wait_time:
            ev3.light.on(Color.GREEN)
            liftDown()

            cube_passed_clock.pause()
            cube_passed_clock.reset()
            ev3.light.on(color=Color.BLACK)
        elif lift_motor.angle() <= 5:
            liftUp()


ev3.speaker.beep(300, 1000)
