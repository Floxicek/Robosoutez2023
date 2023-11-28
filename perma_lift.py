#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait
from pybricks.media.ev3dev import Font

max_lift_speed = 1000
lift_max_height = 1250  # TODO tohle možná snížit
drop_lift_height = 1490


last_block_color = 0
cube_count = 0

lifting_cube = False


ev3 = EV3Brick()
lift_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
color_sensor = ColorSensor(Port.S3)


lift_motor.control.limits(1000, 4000, 80)
lift_motor.control.stall_tolerances(10, 200)

ev3.screen.set_font(Font(size=20))


def liftUp(wait=False, start_pos=False):
    if start_pos:
        lift_motor.run_target(max_lift_speed, lift_max_height * 0.8, Stop.HOLD, wait)
    else:
        lift_motor.run_target(max_lift_speed, lift_max_height, Stop.HOLD, wait)


def liftDown(wait=False):
    lift_motor.run_target(max_lift_speed, 1, Stop.HOLD, wait)


def dropCube():
    lift_motor.run_until_stalled(max_lift_speed, Stop.BRAKE, 80)


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
    global cube_count
    global last_block_color
    global lifting_cube
    if not lifting_cube:
        measure = color_sensor.rgb()

        if (measure[0] > 20 or measure[1] > 20 or measure[2] > 20) and (
            abs(lift_max_height - lift_motor.angle()) < 150
        ):
            new_color = show_color(measure)
            if new_color == last_block_color:
                return
            else:
                last_block_color = new_color
                print("Cube color {i}".format(i=last_block_color))

                ev3.speaker.beep(frequency=900, duration=50)
                lifting_cube = True
                cube_count = cube_count + 1
                ev3.screen.draw_text(10, 10, cube_count)
                print("Number of cubes: {i}".format(i=cube_count))


ev3.screen.print("Reset belt motor rotation")
lift_motor.run_until_stalled(-max_lift_speed, Stop.BRAKE, 50)
ev3.speaker.beep(300, 300)
wait(800)
lift_motor.reset_angle(0)
wait(100)

ev3.speaker.set_volume(100)


liftUp(True, True)


while True:
    check_color()

    if lifting_cube:
        liftDown()
    if lift_motor.angle() <= 10:
        lifting_cube = False
    if lift_motor.angle() <= 10 and lift_motor.control.done():
        liftUp()
