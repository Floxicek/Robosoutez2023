#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import Font
from pybricks.robotics import DriveBase


max_wheel_speed = [110, 90]
wheel_diameter = 53.9
axle_track = 193

max_lift_speed = 1000
lift_max_height = 1250
drop_lift_height = 1490
distance_from_wall = [
    40,
    189,
]
distance_index = 0

deposit_rotate_speed = 100
PROPORTIONAL_GAIN = 0.5

cube_passed_clock = StopWatch()
cube_passed_clock.pause()
cube_passed_clock.reset()

last_block_color = 0


block_wait_time = 0

number_of_cubes = 0

has_turned = False

# Initialize ports
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
lift_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)

distance_sensor = UltrasonicSensor(Port.S2)
color_sensor = ColorSensor(Port.S3)
# gyro_sensor = GyroSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)


lift_motor.control.limits(1000, 4000, 90)
lift_motor.control.stall_tolerances(10, 200)


# left_motor.control.stall_tolerances(0, 15)

ev3.screen.set_font(Font(size=20))
ev3.screen.print("Reset belt motor rotation")
lift_motor.run_until_stalled(-max_lift_speed, Stop.BRAKE, 50)
wait(500)
lift_motor.reset_angle(0)
wait(100)


# left_motor.control.pid(0, 0, 0, 0, 0)
# right_motor.control.pid(0, 0, 0, 0, 0)


def liftUp(wait=False):
    lift_motor.run_target(max_lift_speed, lift_max_height, Stop.HOLD, wait)


def liftDown(wait=False):
    lift_motor.run_target(max_lift_speed, 1, Stop.HOLD, wait)
    # ev3.screen.print("Lift motor down")
    # ev3.screen.print("{angle} / 0".format(angle=lift_motor.angle()))


def dropCube():
    lift_motor.run_until_stalled(max_lift_speed, Stop.BRAKE, 80)


def move():
    measured_distance = distance_sensor.distance()
    if measured_distance > 800:
        robot.drive(max_wheel_speed[distance_index] / 5, 0)
        print("Pomalu Rovně")
        print(measured_distance)

    else:
        move_err = (
            distance_from_wall[distance_index] - measured_distance
        ) * PROPORTIONAL_GAIN
        ev3.screen.draw_text(50, 20, move_err)
        ev3.screen.draw_text(50, 60, measured_distance)
        robot.drive(max_wheel_speed[distance_index], move_err)


def show_color(rgb):
    red, green, blue = rgb
    # print("Color {c}".format(c=rgb))
    threshold = 5

    if green > red + threshold and green > blue + threshold:
        ev3.light.on(Color.GREEN)
        return 1
    elif red > green + threshold and red > blue + threshold:
        ev3.light.on(Color.RED)
        return 3
    elif blue > red + threshold and blue > green + threshold:
        ev3.light.on(Color.BLUE)
        return 4
    else:
        ev3.light.on(Color.YELLOW)
        return 2


def check_color():
    global number_of_cubes
    global last_block_color
    if cube_passed_clock.time() <= 0:
        measure = color_sensor.rgb()

        if (measure[0] > 20 or measure[1] > 20 or measure[2] > 20) and abs(
            lift_max_height - lift_motor.angle()
        ) < 150:
            new_color = show_color(measure)
            if new_color == last_block_color:
                return
            else:
                last_block_color = new_color
                print("Cube {i}".format(i=last_block_color))

                ev3.speaker.beep(frequency=900, duration=300)
                cube_passed_clock.reset()
                cube_passed_clock.resume()
                number_of_cubes = number_of_cubes + 1
                ev3.screen.draw_text(0, 0, number_of_cubes)
                print("Number of cubes: {i}".format(i=number_of_cubes))


liftUp(True)

robot.reset()

print(robot.distance())

# ev3.screen.print("Press button to continue")
# while True:
#    if Button.CENTER in ev3.buttons.pressed():
#        break
#
# ev3.screen.clear()

# number_of_cubes = 1
# last_block_color = 1
# liftDown(True)
# wait(200)

while True:
    ev3.screen.clear()
    move()
    check_color()

    if number_of_cubes == 4 and not has_turned:
        liftDown()
        robot.straight(180)  # 180
        robot.drive(max_wheel_speed[distance_index], 28)
        wait(500)
        liftUp()
        wait(2300)
        robot.stop()
        distance_index = 1
        print(robot.distance())
        has_turned = True

    if cube_passed_clock.time() > block_wait_time:
        liftDown()

        cube_passed_clock.pause()
        cube_passed_clock.reset()
        # ev3.light.on(color=Color.BLACK)
    elif lift_motor.angle() <= 10 and lift_motor.control.done():
        liftUp()

    if number_of_cubes == 8:
        # robot.straight(1000)

        print(robot.state())
        dropCube()
        ev3.speaker.beep(200, 100)
        wait(100)
        ev3.speaker.beep(200, 100)
        wait(100)
        ev3.speaker.beep(200, 100)
        wait(200)
        robot.turn(120)
        robot.straight(800)
        robot.turn(-360)
        robot.straight(-400)
        liftDown(True)
        break