#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import Font
from pybricks.robotics import DriveBase


max_wheel_speed = [110, 80]
wheel_diameter = 53.9
axle_track = 193

max_lift_speed = 1000
lift_max_height = 1250  # TODO tohle možná snížit
drop_lift_height = 1490
distance_from_wall = [
    44,
    189,
]
wall_index = 0

correction = [0.5, 0.2]

time_passed = StopWatch()
time_passed.pause()
time_passed.reset()

last_block_color = 0
cube_count = 0

lifting_cube = False
has_turned = False


ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
lift_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
distance_sensor = UltrasonicSensor(Port.S2)
color_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)


# Kdyztak koment tady kdyz motor nefunguje
lift_motor.control.limits(1000, 4000, 80)
lift_motor.control.stall_tolerances(10, 200)


# left_motor.control.stall_tolerances(0, 15)

ev3.screen.set_font(Font(size=20))

# left_motor.control.pid(0, 0, 0, 0, 0)
# right_motor.control.pid(0, 0, 0, 0, 0)


def liftUp(wait=False, start_pos=False):
    if start_pos:
        lift_motor.run_target(max_lift_speed, lift_max_height * 0.8, Stop.HOLD, wait)
    else:
        lift_motor.run_target(max_lift_speed, lift_max_height, Stop.HOLD, wait)


def liftDown(wait=False):
    lift_motor.run_target(max_lift_speed, 1, Stop.HOLD, wait)


def dropCube():
    lift_motor.run_until_stalled(max_lift_speed, Stop.BRAKE, 80)


def move():
    measured_distance = distance_sensor.distance()
    if measured_distance > 2500:
        robot.drive(max_wheel_speed[wall_index] / 5, 60)
        print("Pomalu Rovně")
        print(measured_distance)
    else:
        move_err = (distance_from_wall[wall_index] - measured_distance) * correction[
            wall_index
        ]
        ev3.screen.draw_text(50, 20, move_err)
        ev3.screen.draw_text(50, 40, measured_distance)
        ev3.screen.draw_text(50, 60, wall_index)
        robot.drive(max_wheel_speed[wall_index], move_err)


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
            abs(lift_max_height - lift_motor.angle()) < 150 or cube_count == 0
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

                if cube_count == 9:
                    robot.reset()
                elif cube_count == 2:
                    print(robot.distance())
                    robot.reset()


def turn():
    global has_turned
    global cube_count
    global wall_index
    global last_block_color
    cube_count = cube_count + 1
    last_block_color = 0
    print("Driven distance {dis}".format(dis=robot.distance()))
    liftDown()
    robot.straight(120)  # 180
    robot.drive(max_wheel_speed[wall_index], 28)
    # robot.curve()
    wait(500)
    liftUp()
    wait(2300)
    robot.stop()
    wall_index = 1
    print("Distance: {d}".format(d=robot.distance()))
    has_turned = True


def turn2():
    global has_turned
    global cube_count
    global wall_index
    global last_block_color
    cube_count = cube_count + 1
    last_block_color = 0
    liftDown()
    print("Driven distance {dis}".format(dis=robot.distance()))
    robot.straight(120)  # 180
    robot.drive(max_wheel_speed[wall_index], 28)
    # robot.curve()
    wait(500)
    liftUp()
    wait(2300)
    robot.stop()
    wall_index = 1
    print("Distance: {d}".format(d=robot.distance()))
    has_turned = True


ev3.screen.print("Reset belt motor rotation")
lift_motor.run_until_stalled(-max_lift_speed, Stop.BRAKE, 50)
ev3.speaker.beep(300, 300)
wait(800)
lift_motor.reset_angle(0)
wait(100)

ev3.speaker.set_volume(100)


liftUp(True, True)
robot.reset()
print("Settings: {s}".format(s=robot.settings()))

time_passed.resume()

# ev3.screen.print("Press button to continue")
# while True:
#     if Button.CENTER in ev3.buttons.pressed():
#         break

while True:
    if time_passed.time() > 88000:
        break

    ev3.screen.clear()
    move()
    check_color()

    if robot.distance() > 800 and not has_turned:
        turn()

    if lifting_cube:
        liftDown()
    if lift_motor.angle() <= 10:
        lifting_cube = False
    if lift_motor.angle() <= 10 and lift_motor.control.done():
        if cube_count == 7:
            robot.straight(30)
            liftUp()
        else:
            liftUp()

    if cube_count == 8:  # or distance > blabla  # TODO add failsafe
        # print("Return distance from start {dist}".format(dist=robot.distance()))
        liftUp()
        wait(400)
        dropCube()
        # ev3.speaker.beep(200, 100)
        # wait(100)
        # ev3.speaker.beep(200, 100)
        # wait(100)
        # ev3.speaker.beep(200, 100)
        robot.stop()
        robot.settings(100000, 1000, 80, 10000)  # max 1020 prev 400
        robot.turn(125)
        robot.straight(700)
        robot.turn(40)
        robot.turn(-40)
        robot.straight(-800)  # TODO Improve this value
        liftUp(False, True)
        robot.turn(-70)  # TODO Improve this value
        robot.straight(290)
        robot.turn(30)
        break
wait(200)
wall_index = 0
has_turned = False
robot.stop()
robot.settings(125, 501, 74, 298)


while True:
    if time_passed.time() > 88000:
        print("Overtime")
        break

    ev3.screen.clear()
    move()
    check_color()

    # if  and not has_turned:
    if robot.distance() > 1105 and cube_count >= 9 and not has_turned:
        print(robot.distance())
        turn2()

    if lifting_cube:
        liftDown()
    if lift_motor.angle() <= 10:
        lifting_cube = False
    if lift_motor.angle() <= 10 and lift_motor.control.done():
        if cube_count == 15:
            robot.straight(30)
            liftUp()
        else:
            liftUp()

    if cube_count == 16:
        # TODO pick up last cube
        # print("Return distance from start {dist}".format(dist=robot.distance()))
        liftUp()
        wait(400)
        dropCube()
        robot.stop()
        robot.settings(100000, 1000, 80, 10000)  # max 1020 prev 400
        robot.turn(125)
        robot.straight(700)
        robot.turn(-40)  # 125
        robot.straight(-600)
        break


robot.stop()
time_passed.pause()
print(time_passed.time() / 1000)
ev3.speaker.beep(200, 3000)


beep_duration = 200
beep_wait = 0

wait(200)

ev3.speaker.beep(200, beep_duration)
wait(beep_wait)
ev3.speaker.beep(500, beep_duration)
wait(beep_wait)
ev3.speaker.beep(300, beep_duration)
wait(beep_wait)
ev3.speaker.beep(500, beep_duration)
wait(beep_wait)
ev3.speaker.beep(800, beep_duration)

ev3.screen.print("Press button to continue")
while True:
    if Button.CENTER in ev3.buttons.pressed():
        break

liftDown(True)
